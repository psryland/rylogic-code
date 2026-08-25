//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/forward.h"
#include "pr/common/cast.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Root-register assignments shared with articulation_impulse_aba.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto MobilityRanges = ESRVReg::t0;
			inline static constexpr auto Articulations = ESRVReg::t1;
			inline static constexpr auto Links = ESRVReg::t2;
			inline static constexpr auto Dofs = ESRVReg::t3;
			inline static constexpr auto Positions = ESRVReg::t4;
			inline static constexpr auto Forces = ESRVReg::t5;
			inline static constexpr auto ExternalForces = ESRVReg::t6;
			inline static constexpr auto Children = ESRVReg::t7;
			inline static constexpr auto LinkMobilities = ESRVReg::t8;
			inline static constexpr auto LinkImpulses = ESRVReg::t9;
			inline static constexpr auto Selection = ESRVReg::t10;
			inline static constexpr auto Velocities = EUAVReg::u0;
			inline static constexpr auto Accelerations = EUAVReg::u1;
			inline static constexpr auto AbaScratch = EUAVReg::u2;
			inline static constexpr auto AbaDofScratch = EUAVReg::u3;
			inline static constexpr auto InverseJointInertia = EUAVReg::u4;
			inline static constexpr auto ImpulseWork = EUAVReg::u5;
			inline static constexpr auto Results = EUAVReg::u6;
		};

		// Match the HLSL participation and packed-buffer bounds constant buffer.
		struct alignas(16) cbArticulationImpulseAba
		{
			int m_participating_articulation_count;
			int m_articulation_count;
			int m_link_count;
			int m_mobility_count;
		};
		static_assert(sizeof(cbArticulationImpulseAba) == 16);

		// Create or retain a typed UAV buffer with geometric high-water growth.
		template <typename Type>
		void EnsureBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int required_count, int& capacity, char const* name)
		{
			if (required_count <= capacity)
				return;

			auto const grown_capacity = std::max(required_count, std::max(1, capacity * 2));
			resource = gpu.CreateResource(ResDesc::Buf<Type>(grown_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, name);
			capacity = grown_capacity;
		}

		// Return the exact group count for a non-empty participating-articulation range.
		int ImpulseThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}

		// Validate compact link impulses before any command-list mutation.
		void ValidateImpulseStream(std::span<GpuArticulationSpatialVector const> link_impulses, int expected_count)
		{
			if (isize(link_impulses) != expected_count)
				throw std::invalid_argument("GPU articulation impulse stream must contain one entry per participating link");
			for (auto const& impulse : link_impulses)
			{
				if (
					!std::isfinite(impulse.ang.x) || !std::isfinite(impulse.ang.y) || !std::isfinite(impulse.ang.z) ||
					!std::isfinite(impulse.lin.x) || !std::isfinite(impulse.lin.y) || !std::isfinite(impulse.lin.z))
					throw std::invalid_argument("GPU articulation impulses must be finite");
			}
		}
	}

	// True when every packed link retained a valid fixed-configuration factorization.
	bool GpuArticulationImpulseAbaResult::AllValid() const
	{
		return std::ranges::all_of(m_solve_valid, [](int valid) { return valid != 0; });
	}

	// Create the impulse-response pipeline without allocating optional feature buffers.
	GpuArticulationImpulseAba::GpuArticulationImpulseAba(Gpu& gpu, GpuArticulationForceAba& aba, GpuArticulationMobility& mobility)
		:m_gpu(gpu)
		,m_aba(aba)
		,m_mobility(mobility)
		,m_cs_apply_impulses()
		,m_cs_evaluate_impulses()
		,m_cs_commit_impulses()
		,m_r_link_impulses()
		,m_r_work()
		,m_impulse_count()
		,m_work_count()
		,m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationImpulseAba>(EReg::Params)
			.SRV(EReg::MobilityRanges)
			.SRV(EReg::Articulations)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::Positions)
			.SRV(EReg::Forces)
			.SRV(EReg::ExternalForces)
			.SRV(EReg::Children)
			.SRV(EReg::LinkMobilities)
			.SRV(EReg::LinkImpulses)
			.UAV(EReg::Velocities)
			.UAV(EReg::Accelerations)
			.UAV(EReg::AbaScratch)
			.UAV(EReg::AbaDofScratch)
			.UAV(EReg::InverseJointInertia)
			.UAV(EReg::ImpulseWork)
			.Create(m_gpu, "Physics:ArticulationImpulseAbaSig");
		m_cs_apply_impulses.m_sig = root_sig;
		m_cs_apply_impulses.m_pso = ComputePSO(root_sig.get(), shader_code::articulation_apply_impulses).Create(m_gpu, "Physics:ArticulationApplyImpulsesPSO");

		// Transactional passes add caller-owned selection and result streams while retaining identical ABA resource ordering.
		auto transactional_root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationImpulseAba>(EReg::Params)
			.SRV(EReg::MobilityRanges)
			.SRV(EReg::Articulations)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::Positions)
			.SRV(EReg::Forces)
			.SRV(EReg::ExternalForces)
			.SRV(EReg::Children)
			.SRV(EReg::LinkMobilities)
			.SRV(EReg::LinkImpulses)
			.SRV(EReg::Selection)
			.UAV(EReg::Velocities)
			.UAV(EReg::Accelerations)
			.UAV(EReg::AbaScratch)
			.UAV(EReg::AbaDofScratch)
			.UAV(EReg::InverseJointInertia)
			.UAV(EReg::ImpulseWork)
			.UAV(EReg::Results)
			.Create(m_gpu, "Physics:ArticulationImpulseTransactionalSig");
		m_cs_evaluate_impulses.m_sig = transactional_root_sig;
		m_cs_evaluate_impulses.m_pso = ComputePSO(transactional_root_sig.get(), shader_code::articulation_evaluate_impulses).Create(m_gpu, "Physics:ArticulationEvaluateImpulsesPSO");
		m_cs_commit_impulses.m_sig = transactional_root_sig;
		m_cs_commit_impulses.m_pso = ComputePSO(transactional_root_sig.get(), shader_code::articulation_commit_impulses).Create(m_gpu, "Physics:ArticulationCommitImpulsesPSO");
	}

	// Allocate compact participating-link impulse and work buffers without initializing the impulse stream.
	bool GpuArticulationImpulseAba::Prepare(GpuJob& job)
	{
		m_stats.m_dispatch_count = 0;
		if (m_mobility.m_ranges.empty())
		{
			ReleaseBuffers();
			return false;
		}
		if (m_mobility.m_mobility_count <= 0)
			throw std::runtime_error("GPU articulation impulse response requires prepared mobility storage");

		// Both impulse and work streams follow the compact canonical mobility ranges.
		ResizeBuffers(job.m_cmd_list, m_mobility.m_mobility_count, m_mobility.m_mobility_count);
		m_impulse_count = m_mobility.m_mobility_count;
		m_work_count = m_mobility.m_mobility_count;
		m_stats.m_logical_buffer_bytes =
			static_cast<size_t>(m_impulse_count + m_work_count) *
			sizeof(GpuArticulationSpatialVector);
		return true;
	}

	// Upload one link-coordinate spatial impulse per participating link without submitting or waiting.
	bool GpuArticulationImpulseAba::Upload(GpuJob& job, std::span<GpuArticulationSpatialVector const> link_impulses)
	{
		if (m_mobility.m_ranges.empty())
		{
			if (!link_impulses.empty())
				throw std::invalid_argument("Articulation impulses require at least one participating articulation");
			return Prepare(job);
		}
		ValidateImpulseStream(link_impulses, m_mobility.m_mobility_count);
		if (!Prepare(job))
			return false;

		job.m_barriers.Transition(m_r_link_impulses.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		auto allocation = job.m_upload.Alloc<GpuArticulationSpatialVector>(isize(link_impulses));
		memcpy(allocation.ptr<GpuArticulationSpatialVector>(), link_impulses.data(), sizeof(GpuArticulationSpatialVector) * link_impulses.size());
		job.m_cmd_list.CopyBufferRegion(m_r_link_impulses.get(), 0, allocation);
		return true;
	}

	// Apply every participating tree's gathered impulses through one fixed-configuration ABA response.
	void GpuArticulationImpulseAba::Run(GpuJob& job)
	{
		if (m_mobility.m_ranges.empty())
			return;
		if (m_impulse_count != m_mobility.m_mobility_count || m_work_count != m_mobility.m_mobility_count)
			throw std::runtime_error("GPU articulation impulse buffers do not match the prepared articulation factors");

		// Bind retained fixed-configuration factors and make all complete-tree mutations visible through UAV state.
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_mobilities.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_link_impulses.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbArticulationImpulseAba{
			.m_participating_articulation_count = isize(m_mobility.m_ranges),
			.m_articulation_count = m_aba.m_articulation_count,
			.m_link_count = m_aba.m_link_count,
			.m_mobility_count = m_work_count,
		};
		job.m_cmd_list.SetPipelineState(m_cs_apply_impulses.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_apply_impulses.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_link_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_work->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(ImpulseThreadGroupCount(isize(m_mobility.m_ranges)), 1, 1);
		++m_stats.m_dispatch_count;

		// Downstream residual evaluation and proxy refresh observe one complete simultaneous articulation update.
		if (m_aba.m_velocity_count != 0)
			job.m_barriers.UAV(m_aba.m_r_velocities.get());
		if (m_aba.m_acceleration_count != 0)
			job.m_barriers.UAV(m_aba.m_r_accelerations.get());
		job.m_barriers.UAV(m_aba.m_r_scratch.get());
		job.m_barriers.UAV(m_r_work.get());
		job.m_barriers.Commit();
	}

	// Evaluate selected tree responses into detached work buffers and one caller-owned validity result per range.
	void GpuArticulationImpulseAba::Evaluate(GpuJob& job, ID3D12Resource* selection, ID3D12Resource* results)
	{
		DispatchTransactional(job, m_cs_evaluate_impulses, selection, results);
	}

	// Commit previously evaluated responses for the caller-selected valid ranges.
	void GpuArticulationImpulseAba::Commit(GpuJob& job, ID3D12Resource* selection, ID3D12Resource* results)
	{
		DispatchTransactional(job, m_cs_commit_impulses, selection, results);
	}

	// Prepare factors, apply impulses, and read focused output through exactly one GPU submission.
	GpuArticulationImpulseAbaResult GpuArticulationImpulseAba::Apply(
		GpuJob& job,
		GpuArticulationUpload const& upload,
		std::span<int const> articulation_indices,
		std::span<GpuArticulationSpatialVector const> link_impulses)
	{
		auto result = GpuArticulationImpulseAbaResult{};
		if (articulation_indices.empty())
		{
			m_mobility.Upload(job, upload, {});
			Upload(job, {});
			return result;
		}
		auto const ranges = BuildGpuArticulationMobilityRanges(upload, articulation_indices);
		auto const expected_impulse_count = ranges.back().mobility_offset + ranges.back().link_count;
		ValidateImpulseStream(link_impulses, expected_impulse_count);
		if (!m_aba.Upload(job, upload))
			throw std::invalid_argument("GPU articulation impulses require a non-empty packed forest");
		if (!m_mobility.Upload(job, upload, articulation_indices))
			throw std::invalid_argument("GPU articulation impulses require participating mobility ranges");
		if (!Upload(job, link_impulses))
			return result;

		m_mobility.Run(job);
		Run(job);

		// Read generalized velocities, cached link velocities, and solve validity without a second submission.
		if (m_aba.m_velocity_count != 0)
			job.m_barriers.Transition(m_aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto velocity_readback = ReadbackAlloc{};
		if (m_aba.m_velocity_count != 0)
			velocity_readback = job.m_readback.Alloc<float>(m_aba.m_velocity_count);
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(m_aba.m_link_count);
		if (m_aba.m_velocity_count != 0)
			job.m_cmd_list.CopyBufferRegion(velocity_readback, m_aba.m_r_velocities.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, m_aba.m_r_scratch.get(), 0);
		if (m_aba.m_velocity_count != 0)
			job.m_barriers.Transition(m_aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
		job.Run();

		result.m_ranges = m_mobility.m_ranges;
		if (m_aba.m_velocity_count != 0)
			result.m_velocities.assign(velocity_readback.ptr<float>(), velocity_readback.ptr<float>() + m_aba.m_velocity_count);

		auto const* scratch = scratch_readback.ptr<GpuArticulationAbaScratch>();
		result.m_link_velocities.reserve(m_work_count);
		result.m_solve_valid.reserve(m_work_count);
		for (auto const& range : m_mobility.m_ranges)
		{
			auto const& articulation = upload.m_articulations[range.articulation_index];
			for (int local_link_index = 0; local_link_index != range.link_count; ++local_link_index)
			{
				auto const link_index = articulation.link_offset + local_link_index;
				result.m_link_velocities.push_back(scratch[link_index].link_velocity);
				result.m_solve_valid.push_back(scratch[link_index].solve_valid);
			}
		}
		return result;
	}

	// Return the GPU impulse stream so deterministic gather passes can populate it directly.
	ID3D12Resource* GpuArticulationImpulseAba::LinkImpulses()
	{
		return m_r_link_impulses.get();
	}

	// Return detached per-link velocity deltas from the most recent impulse evaluation.
	ID3D12Resource* GpuArticulationImpulseAba::Work()
	{
		return m_r_work.get();
	}

	// Return current logical usage, retained capacities, and the most recent dispatch count.
	GpuArticulationImpulseAbaStats const& GpuArticulationImpulseAba::Stats() const
	{
		return m_stats;
	}

	// Release every lazily allocated impulse-response resource when no tree participates.
	void GpuArticulationImpulseAba::ReleaseBuffers()
	{
		m_r_link_impulses = nullptr;
		m_r_work = nullptr;
		m_impulse_count = 0;
		m_work_count = 0;
		m_stats = {};
	}

	// Create or grow typed impulse and work buffers for the active packed forest.
	void GpuArticulationImpulseAba::ResizeBuffers(CmdList& cmd_list, int link_count, int work_count)
	{
		EnsureBuffer<GpuArticulationSpatialVector>(m_gpu, cmd_list, m_r_link_impulses, link_count, m_stats.m_link_impulse_capacity, "Articulation impulse ABA link impulses");
		EnsureBuffer<GpuArticulationSpatialVector>(m_gpu, cmd_list, m_r_work, work_count, m_stats.m_work_capacity, "Articulation impulse ABA work");
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_link_impulse_capacity + m_stats.m_work_capacity) *
			sizeof(GpuArticulationSpatialVector);
	}

	// Bind and dispatch one transactional evaluate or commit pass using caller-owned selection and result streams.
	void GpuArticulationImpulseAba::DispatchTransactional(GpuJob& job, ComputeStep& step, ID3D12Resource* selection, ID3D12Resource* results)
	{
		if (m_mobility.m_ranges.empty())
			return;
		if (selection == nullptr || results == nullptr)
			throw std::invalid_argument("Transactional articulation impulse response requires selection and result resources");
		if (m_impulse_count != m_mobility.m_mobility_count || m_work_count != m_mobility.m_mobility_count)
			throw std::runtime_error("GPU articulation impulse buffers do not match the prepared articulation factors");

		// Preserve resource identity across evaluation and commit while making each dependency explicit to D3D12.
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_mobilities.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_link_impulses.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(selection, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(results, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbArticulationImpulseAba{
			.m_participating_articulation_count = isize(m_mobility.m_ranges),
			.m_articulation_count = m_aba.m_articulation_count,
			.m_link_count = m_aba.m_link_count,
			.m_mobility_count = m_work_count,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_link_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_work->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(results->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(ImpulseThreadGroupCount(isize(m_mobility.m_ranges)), 1, 1);
		++m_stats.m_dispatch_count;

		// Either pass may feed another coupled kernel immediately, so order every writable transactional resource once.
		if (m_aba.m_velocity_count != 0)
			job.m_barriers.UAV(m_aba.m_r_velocities.get());
		if (m_aba.m_acceleration_count != 0)
			job.m_barriers.UAV(m_aba.m_r_accelerations.get());
		job.m_barriers.UAV(m_aba.m_r_scratch.get());
		job.m_barriers.UAV(m_r_work.get());
		job.m_barriers.UAV(results);
		job.m_barriers.Commit();
	}
}
