//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Per-dispatch timestep and active packed bounds shared with the fused shader.
		struct alignas(16) cbArticulationMidpoint
		{
			float dt;
			int articulation_count;
			int link_count;
			int velocity_count;
		};
		static_assert(sizeof(cbArticulationMidpoint) == 16);

		// Root register assignments are kept local because the fused lane intentionally uses writable primary state.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Links = ESRVReg::t0;
			inline static constexpr auto Dofs = ESRVReg::t1;
			inline static constexpr auto Forces = ESRVReg::t2;
			inline static constexpr auto ExternalForces = ESRVReg::t3;
			inline static constexpr auto Children = ESRVReg::t4;
			inline static constexpr auto Articulations = EUAVReg::u0;
			inline static constexpr auto Positions = EUAVReg::u1;
			inline static constexpr auto Velocities = EUAVReg::u2;
			inline static constexpr auto Accelerations = EUAVReg::u3;
			inline static constexpr auto Scratch = EUAVReg::u4;
			inline static constexpr auto DofScratch = EUAVReg::u5;
			inline static constexpr auto JointMatrixScratch = EUAVReg::u6;
			inline static constexpr auto PositionStart = EUAVReg::u7;
			inline static constexpr auto VelocityStart = EUAVReg::u8;
			inline static constexpr auto MidpointVelocity = EUAVReg::u9;
			inline static constexpr auto IntegrationState = EUAVReg::u10;
		};

		// Create or geometrically grow one integration-only typed buffer.
		template <typename T> void EnsureMidpointBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int count, int& capacity, std::string_view name)
		{
			if (count <= capacity)
				return;

			auto const doubled_capacity = capacity <= std::numeric_limits<int>::max() / 2 ? capacity * 2 : std::numeric_limits<int>::max();
			capacity = std::max(count, std::max(1, doubled_capacity));
			resource = gpu.CreateResource(ResDesc::Buf<T>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, name);
		}

		// Return the exact group count for a non-empty articulation range.
		int MidpointThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// Return true only when every articulation completed all recorded substeps successfully.
	bool GpuArticulationMidpointResult::AllSucceeded() const
	{
		return std::ranges::all_of(m_states, [](GpuArticulationIntegrationState const& state)
		{
			return state.status == GpuArticulationIntegrationStatus_Success;
		});
	}

	// Create the fused integration pipeline without allocating articulation-dependent buffers.
	GpuArticulationMidpoint::GpuArticulationMidpoint(GpuArticulationForceAba& aba)
		: m_aba(aba)
		, m_cs_midpoint()
		, m_r_position_start()
		, m_r_velocity_start()
		, m_r_midpoint_velocity()
		, m_r_integration_state()
		, m_stats()
	{
		// The integration root signature shares immutable topology/forces and exposes primary state plus compact ABA scratch as UAVs.
		auto sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationMidpoint>(EReg::Params)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::Forces)
			.SRV(EReg::ExternalForces)
			.SRV(EReg::Children)
			.UAV(EReg::Articulations)
			.UAV(EReg::Positions)
			.UAV(EReg::Velocities)
			.UAV(EReg::Accelerations)
			.UAV(EReg::Scratch)
			.UAV(EReg::DofScratch)
			.UAV(EReg::JointMatrixScratch)
			.UAV(EReg::PositionStart)
			.UAV(EReg::VelocityStart)
			.UAV(EReg::MidpointVelocity)
			.UAV(EReg::IntegrationState);
		m_cs_midpoint.m_sig = sig.Create(m_aba.m_gpu, "Physics:ArticulationMidpointSig");
		m_cs_midpoint.m_pso = ComputePSO(m_cs_midpoint.m_sig.get(), shader_code::articulation_midpoint).Create(m_aba.m_gpu, "Physics:ArticulationMidpointPSO");
	}

	// Release every integration-only feature resource when the optional forest becomes empty.
	void GpuArticulationMidpoint::ReleaseBuffers()
	{
		m_r_position_start = nullptr;
		m_r_velocity_start = nullptr;
		m_r_midpoint_velocity = nullptr;
		m_r_integration_state = nullptr;
		m_stats = {};
	}

	// Create or geometrically grow the exact P, V, V, and A integration scratch ranges.
	void GpuArticulationMidpoint::ResizeBuffers(CmdList& cmd_list)
	{
		EnsureMidpointBuffer<float>(m_aba.m_gpu, cmd_list, m_r_position_start, m_aba.m_position_count, m_stats.m_position_start_capacity, "Physics:ArticulationMidpointPositionStart");
		EnsureMidpointBuffer<float>(m_aba.m_gpu, cmd_list, m_r_velocity_start, m_aba.m_velocity_count, m_stats.m_velocity_start_capacity, "Physics:ArticulationMidpointVelocityStart");
		EnsureMidpointBuffer<float>(m_aba.m_gpu, cmd_list, m_r_midpoint_velocity, m_aba.m_velocity_count, m_stats.m_midpoint_velocity_capacity, "Physics:ArticulationMidpointVelocity");
		EnsureMidpointBuffer<GpuArticulationIntegrationState>(m_aba.m_gpu, cmd_list, m_r_integration_state, m_aba.m_articulation_count, m_stats.m_integration_state_capacity, "Physics:ArticulationMidpointState");
	}

	// Upload one validated forest, initialize diagnostics, and prepare shared state without submitting or waiting.
	bool GpuArticulationMidpoint::Upload(GpuJob& job, GpuArticulationUpload const& upload)
	{
		m_stats.m_dispatch_count = 0;
		if (!m_aba.Upload(job, upload))
		{
			ReleaseBuffers();
			return false;
		}

		ResizeBuffers(job.m_cmd_list);

		// Initialize one success record per articulation while preserving the uploaded root as the first accepted pose.
		auto states = std::vector<GpuArticulationIntegrationState>{};
		states.reserve(upload.m_articulations.size());
		for (auto const& articulation : upload.m_articulations)
		{
			states.push_back(GpuArticulationIntegrationState{
				.root_to_world_start = articulation.root_to_world,
				.status = GpuArticulationIntegrationStatus_Success,
				.iteration_count = 0,
				.residual = 0.0f,
				.pad = 0.0f,
			});
		}
		job.m_barriers.Transition(m_r_integration_state.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		auto allocation = job.m_upload.Alloc<GpuArticulationIntegrationState>(isize(states));
		memcpy(allocation.ptr<GpuArticulationIntegrationState>(), states.data(), states.size() * sizeof(states[0]));
		job.m_cmd_list.CopyBufferRegion(m_r_integration_state.get(), 0, allocation);

		// Primary state becomes writable for fused trials; empty generalized ranges bind the shared UAV sentinel.
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_r_position_start : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_r_velocity_start : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_r_midpoint_velocity : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_integration_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		m_stats.m_logical_scratch_bytes =
			static_cast<size_t>(m_aba.m_position_count) * sizeof(float) +
			static_cast<size_t>(m_aba.m_velocity_count) * 2 * sizeof(float) +
			static_cast<size_t>(m_aba.m_articulation_count) * sizeof(GpuArticulationIntegrationState);
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_position_start_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_velocity_start_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_midpoint_velocity_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_integration_state_capacity) * sizeof(GpuArticulationIntegrationState);
		return true;
	}

	// Bind shared ABA resources and integration-only scratch for one fused substep.
	void GpuArticulationMidpoint::Dispatch(GpuJob& job, float dt, ID3D12Resource* external_forces)
	{
		external_forces = external_forces != nullptr ? external_forces : m_aba.m_r_external_forces.get();
		auto const constants = cbArticulationMidpoint{
			.dt = dt,
			.articulation_count = m_aba.m_articulation_count,
			.link_count = m_aba.m_link_count,
			.velocity_count = m_aba.m_velocity_count,
		};
		job.m_barriers.Transition(external_forces, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		job.m_cmd_list.SetPipelineState(m_cs_midpoint.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_midpoint.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_position_count != 0 ? m_r_position_start : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_r_velocity_start : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_r_midpoint_velocity : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_integration_state->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(MidpointThreadGroupCount(m_aba.m_articulation_count), 1, 1);
		++m_stats.m_dispatch_count;
	}

	// Order all primary, ABA, and integration scratch writes before another substep or readback.
	void GpuArticulationMidpoint::CommitUavBarriers(GpuJob& job)
	{
		job.m_barriers.UAV(m_aba.m_r_articulations.get());
		if (m_aba.m_position_count != 0)
		{
			job.m_barriers.UAV(m_aba.m_r_positions.get());
			job.m_barriers.UAV(m_r_position_start.get());
		}
		if (m_aba.m_velocity_count != 0)
		{
			job.m_barriers.UAV(m_aba.m_r_velocities.get());
			job.m_barriers.UAV(m_r_velocity_start.get());
			job.m_barriers.UAV(m_r_midpoint_velocity.get());
		}
		if (m_aba.m_acceleration_count != 0)
			job.m_barriers.UAV(m_aba.m_r_accelerations.get());
		job.m_barriers.UAV(m_aba.m_r_scratch.get());
		if (m_aba.m_dof_count != 0)
			job.m_barriers.UAV(m_aba.m_r_dof_scratch.get());
		if (m_aba.m_joint_matrix_count != 0)
			job.m_barriers.UAV(m_aba.m_r_joint_matrix_scratch.get());
		job.m_barriers.UAV(m_r_integration_state.get());
		job.m_barriers.Commit();
	}

	// Record exactly one fused dispatch per requested internal substep with explicit inter-substep ordering.
	void GpuArticulationMidpoint::Run(GpuJob& job, float dt, int substep_count)
	{
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("GPU articulation midpoint timestep must be finite and non-negative");
		if (substep_count < 0)
			throw std::invalid_argument("GPU articulation midpoint substep count must be non-negative");

		m_stats.m_dispatch_count = 0;
		if (m_aba.m_articulation_count == 0)
			return;

		for (int substep_index = 0; substep_index != substep_count; ++substep_index)
			Integrate(job, dt);
	}

	// Record one fused internal substep while retaining sticky status and accumulated dispatch diagnostics.
	void GpuArticulationMidpoint::Integrate(GpuJob& job, float dt, ID3D12Resource* external_forces)
	{
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("GPU articulation midpoint timestep must be finite and non-negative");
		if (m_aba.m_articulation_count == 0)
			return;

		Dispatch(job, dt, external_forces);
		CommitUavBarriers(job);
	}

	// Upload, record all substeps, submit once, and read back focused integration diagnostics.
	GpuArticulationMidpointResult GpuArticulationMidpoint::Solve(GpuJob& job, GpuArticulationUpload const& upload, float dt, int substep_count)
	{
		auto result = GpuArticulationMidpointResult{};
		if (!Upload(job, upload))
			return result;

		Run(job, dt, substep_count);

		// Read back primary state, phase-reused outputs, and explicit diagnostics after one submission.
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (m_aba.m_position_count != 0)
			job.m_barriers.Transition(m_aba.m_r_positions.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (m_aba.m_velocity_count != 0)
			job.m_barriers.Transition(m_aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (m_aba.m_acceleration_count != 0)
			job.m_barriers.Transition(m_aba.m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_integration_state.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();

		auto articulation_readback = job.m_readback.Alloc<GpuArticulation>(m_aba.m_articulation_count);
		auto position_readback = m_aba.m_position_count != 0 ? job.m_readback.Alloc<float>(m_aba.m_position_count) : ReadbackAlloc{};
		auto velocity_readback = m_aba.m_velocity_count != 0 ? job.m_readback.Alloc<float>(m_aba.m_velocity_count) : ReadbackAlloc{};
		auto acceleration_readback = m_aba.m_acceleration_count != 0 ? job.m_readback.Alloc<float>(m_aba.m_acceleration_count) : ReadbackAlloc{};
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(m_aba.m_link_count);
		auto state_readback = job.m_readback.Alloc<GpuArticulationIntegrationState>(m_aba.m_articulation_count);
		job.m_cmd_list.CopyBufferRegion(articulation_readback, m_aba.m_r_articulations.get(), 0);
		if (m_aba.m_position_count != 0)
			job.m_cmd_list.CopyBufferRegion(position_readback, m_aba.m_r_positions.get(), 0);
		if (m_aba.m_velocity_count != 0)
			job.m_cmd_list.CopyBufferRegion(velocity_readback, m_aba.m_r_velocities.get(), 0);
		if (m_aba.m_acceleration_count != 0)
			job.m_cmd_list.CopyBufferRegion(acceleration_readback, m_aba.m_r_accelerations.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, m_aba.m_r_scratch.get(), 0);
		job.m_cmd_list.CopyBufferRegion(state_readback, m_r_integration_state.get(), 0);
		job.Run();

		// Convert all readback ranges into stable typed result vectors.
		result.m_articulations.assign(articulation_readback.ptr<GpuArticulation>(), articulation_readback.ptr<GpuArticulation>() + m_aba.m_articulation_count);
		if (m_aba.m_position_count != 0)
			result.m_positions.assign(position_readback.ptr<float>(), position_readback.ptr<float>() + m_aba.m_position_count);
		if (m_aba.m_velocity_count != 0)
			result.m_velocities.assign(velocity_readback.ptr<float>(), velocity_readback.ptr<float>() + m_aba.m_velocity_count);
		if (m_aba.m_acceleration_count != 0)
			result.m_accelerations.assign(acceleration_readback.ptr<float>(), acceleration_readback.ptr<float>() + m_aba.m_acceleration_count);
		result.m_states.assign(state_readback.ptr<GpuArticulationIntegrationState>(), state_readback.ptr<GpuArticulationIntegrationState>() + m_aba.m_articulation_count);
		result.m_link_accelerations.reserve(m_aba.m_link_count);
		auto const* scratch = scratch_readback.ptr<GpuArticulationAbaScratch>();
		for (int link_index = 0; link_index != m_aba.m_link_count; ++link_index)
			result.m_link_accelerations.push_back(scratch[link_index].articulated_bias_or_acceleration);
		result.m_logical_scratch_bytes = m_stats.m_logical_scratch_bytes;
		result.m_dispatch_count = m_stats.m_dispatch_count;
		return result;
	}

	// Return the current packed primary state and diagnostics for the final gathered frame output.
	GpuArticulationMidpointOutput GpuArticulationMidpoint::Output()
	{
		if (m_aba.m_articulation_count == 0)
			return {};

		return GpuArticulationMidpointOutput{
			.m_articulations = m_aba.m_r_articulations.get(),
			.m_positions = (m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel).get(),
			.m_velocities = (m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel).get(),
			.m_accelerations = (m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel).get(),
			.m_states = m_r_integration_state.get(),
			.m_articulation_count = m_aba.m_articulation_count,
			.m_position_count = m_aba.m_position_count,
			.m_velocity_count = m_aba.m_velocity_count,
			.m_pad0 = 0,
		};
	}

	// Return current integration-only capacities, logical usage, and most recent dispatch count.
	GpuArticulationMidpointStats const& GpuArticulationMidpoint::Stats() const
	{
		return m_stats;
	}

	// Destroy lazily owned midpoint resources where the implementation type is complete.
	void Deleter<GpuArticulationMidpoint>::operator()(GpuArticulationMidpoint* integrator) const
	{
		delete integrator;
	}
}
