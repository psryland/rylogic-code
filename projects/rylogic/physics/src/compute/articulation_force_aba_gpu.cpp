//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Per-dispatch breadth range and immutable packed-buffer bounds shared with HLSL.
		struct alignas(16) cbArticulationForceAba
		{
			int level_offset;
			int level_count;
			int articulation_count;
			int link_count;
		};
		static_assert(sizeof(cbArticulationForceAba) == 16);

		// Register assignments shared by every pure-tree force-ABA entry point.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Articulations = ESRVReg::t0;
			inline static constexpr auto Links = ESRVReg::t1;
			inline static constexpr auto Dofs = ESRVReg::t2;
			inline static constexpr auto Positions = ESRVReg::t3;
			inline static constexpr auto Velocities = ESRVReg::t4;
			inline static constexpr auto Forces = ESRVReg::t5;
			inline static constexpr auto ExternalForces = ESRVReg::t6;
			inline static constexpr auto Children = ESRVReg::t7;
			inline static constexpr auto LevelLinks = ESRVReg::t8;
			inline static constexpr auto Accelerations = EUAVReg::u0;
			inline static constexpr auto Scratch = EUAVReg::u1;
			inline static constexpr auto DofScratch = EUAVReg::u2;
			inline static constexpr auto JointMatrixScratch = EUAVReg::u3;
		};

		// Create or geometrically grow one typed feature buffer without allocating for an empty logical range.
		template <typename T> void EnsureArticulationBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int count, int& capacity, EUsage usage, std::string_view name)
		{
			if (count <= capacity)
				return;

			auto const doubled_capacity = capacity <= std::numeric_limits<int>::max() / 2 ? capacity * 2 : std::numeric_limits<int>::max();
			capacity = std::max(count, std::max(1, doubled_capacity));
			auto desc = ResDesc::Buf<T>(capacity, {}).usage(usage);
			resource = gpu.CreateResource(desc, cmd_list, name);
		}

		// Copy one non-empty CPU vector through the job-owned upload ring.
		template <typename T> void CopyArticulationUpload(GpuJob& job, ID3D12Resource* resource, std::vector<T> const& values)
		{
			if (values.empty())
				return;

			auto allocation = job.m_upload.Alloc<T>(static_cast<int>(values.size()));
			memcpy(allocation.ptr<T>(), values.data(), values.size() * sizeof(T));
			job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
		}

		// Return the exact number of thread groups for one non-empty breadth or root dispatch.
		int ArticulationThreadGroupCount(int item_count)
		{
			return (item_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// True when every packed link completed a non-singular joint and root solve.
	bool GpuArticulationForceAbaResult::AllValid() const
	{
		return std::ranges::all_of(m_solve_valid, [](int valid) { return valid != 0; });
	}

	// Create the common root signature and four pipeline states without allocating feature buffers.
	GpuArticulationForceAba::GpuArticulationForceAba(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_prepare()
		, m_cs_inward()
		, m_cs_root()
		, m_cs_outward()
		, m_r_articulations()
		, m_r_links()
		, m_r_dofs()
		, m_r_positions()
		, m_r_velocities()
		, m_r_forces()
		, m_r_external_forces()
		, m_r_children()
		, m_r_level_links()
		, m_r_accelerations()
		, m_r_scratch()
		, m_r_dof_scratch()
		, m_r_joint_matrix_scratch()
		, m_r_srv_sentinel()
		, m_r_uav_sentinel()
		, m_levels()
		, m_articulation_count()
		, m_link_count()
		, m_dof_count()
		, m_position_count()
		, m_velocity_count()
		, m_force_count()
		, m_child_count()
		, m_acceleration_count()
		, m_joint_matrix_count()
		, m_stats()
	{
		// Every phase uses one root layout so level changes replace only constants and pipeline state.
		auto sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationForceAba>(EReg::Params)
			.SRV(EReg::Articulations)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::Positions)
			.SRV(EReg::Velocities)
			.SRV(EReg::Forces)
			.SRV(EReg::ExternalForces)
			.SRV(EReg::Children)
			.SRV(EReg::LevelLinks)
			.UAV(EReg::Accelerations)
			.UAV(EReg::Scratch)
			.UAV(EReg::DofScratch)
			.UAV(EReg::JointMatrixScratch);
		auto const root_sig = sig.Create(m_gpu, "Physics:ArticulationForceAbaSig");

		auto compile_step = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			step.m_sig = root_sig;
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, FmtS("Physics:%sPSO", name));
		};
		compile_step(m_cs_prepare, shader_code::articulation_prepare, "ArticulationPrepare");
		compile_step(m_cs_inward, shader_code::articulation_inward_dynamics, "ArticulationInwardDynamics");
		compile_step(m_cs_root, shader_code::articulation_root_dynamics, "ArticulationRootDynamics");
		compile_step(m_cs_outward, shader_code::articulation_outward_dynamics, "ArticulationOutwardDynamics");
	}

	// Release every lazily allocated feature resource when the optional lane becomes empty.
	void GpuArticulationForceAba::ReleaseBuffers()
	{
		m_r_articulations = nullptr;
		m_r_links = nullptr;
		m_r_dofs = nullptr;
		m_r_positions = nullptr;
		m_r_velocities = nullptr;
		m_r_forces = nullptr;
		m_r_external_forces = nullptr;
		m_r_children = nullptr;
		m_r_level_links = nullptr;
		m_r_accelerations = nullptr;
		m_r_scratch = nullptr;
		m_r_dof_scratch = nullptr;
		m_r_joint_matrix_scratch = nullptr;
		m_r_srv_sentinel = nullptr;
		m_r_uav_sentinel = nullptr;
		m_levels.clear();
		m_articulation_count = 0;
		m_link_count = 0;
		m_dof_count = 0;
		m_position_count = 0;
		m_velocity_count = 0;
		m_force_count = 0;
		m_child_count = 0;
		m_acceleration_count = 0;
		m_joint_matrix_count = 0;
		m_stats = {};
	}

	// Create or grow all typed buffers required by the current packed forest.
	void GpuArticulationForceAba::ResizeBuffers(CmdList& cmd_list, GpuArticulationUpload const& upload)
	{
		EnsureArticulationBuffer<GpuArticulation>(m_gpu, cmd_list, m_r_articulations, isize(upload.m_articulations), m_stats.m_articulation_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaArticulations");
		EnsureArticulationBuffer<GpuArticulationLink>(m_gpu, cmd_list, m_r_links, isize(upload.m_links), m_stats.m_link_capacity, EUsage::Default, "Physics:ArticulationForceAbaLinks");
		EnsureArticulationBuffer<GpuArticulationDof>(m_gpu, cmd_list, m_r_dofs, isize(upload.m_dofs), m_stats.m_dof_capacity, EUsage::Default, "Physics:ArticulationForceAbaDofs");
		EnsureArticulationBuffer<float>(m_gpu, cmd_list, m_r_positions, isize(upload.m_positions), m_stats.m_position_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaPositions");
		EnsureArticulationBuffer<float>(m_gpu, cmd_list, m_r_velocities, isize(upload.m_velocities), m_stats.m_velocity_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaVelocities");
		EnsureArticulationBuffer<float>(m_gpu, cmd_list, m_r_forces, isize(upload.m_forces), m_stats.m_force_capacity, EUsage::Default, "Physics:ArticulationForceAbaForces");
		EnsureArticulationBuffer<GpuFrameForce>(m_gpu, cmd_list, m_r_external_forces, isize(upload.m_external_forces), m_stats.m_external_force_capacity, EUsage::Default, "Physics:ArticulationForceAbaExternalForces");
		EnsureArticulationBuffer<uint32_t>(m_gpu, cmd_list, m_r_children, isize(upload.m_children), m_stats.m_child_capacity, EUsage::Default, "Physics:ArticulationForceAbaChildren");
		EnsureArticulationBuffer<uint32_t>(m_gpu, cmd_list, m_r_level_links, isize(upload.m_level_links), m_stats.m_level_link_capacity, EUsage::Default, "Physics:ArticulationForceAbaLevelLinks");
		EnsureArticulationBuffer<float>(m_gpu, cmd_list, m_r_accelerations, isize(upload.m_accelerations), m_stats.m_acceleration_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaAccelerations");
		EnsureArticulationBuffer<GpuArticulationAbaScratch>(m_gpu, cmd_list, m_r_scratch, isize(upload.m_links), m_stats.m_scratch_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaScratch");
		EnsureArticulationBuffer<GpuArticulationAbaDofScratch>(m_gpu, cmd_list, m_r_dof_scratch, isize(upload.m_dofs), m_stats.m_dof_scratch_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaDofScratch");
		EnsureArticulationBuffer<float>(m_gpu, cmd_list, m_r_joint_matrix_scratch, upload.m_joint_matrix_scratch_count, m_stats.m_joint_matrix_capacity, EUsage::UnorderedAccess, "Physics:ArticulationForceAbaJointMatrices");

		// One 64-byte resource per access state safely backs every logically empty optional structured stream.
		auto const needs_srv_sentinel =
			upload.m_dofs.empty() ||
			upload.m_positions.empty() ||
			upload.m_velocities.empty() ||
			upload.m_forces.empty() ||
			upload.m_children.empty();
		auto const needs_uav_sentinel =
			upload.m_accelerations.empty() ||
			upload.m_dofs.empty() ||
			upload.m_joint_matrix_scratch_count == 0;
		if (needs_srv_sentinel && m_r_srv_sentinel == nullptr)
			m_r_srv_sentinel = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationAbaDofScratch>(1, {}), cmd_list, "Physics:ArticulationForceAbaSrvSentinel");
		if (needs_uav_sentinel && m_r_uav_sentinel == nullptr)
			m_r_uav_sentinel = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationAbaDofScratch>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ArticulationForceAbaUavSentinel");
	}

	// Upload one validated packed forest without submitting or waiting, returning whether work is active.
	bool GpuArticulationForceAba::Upload(GpuJob& job, GpuArticulationUpload const& upload)
	{
		ValidateGpuArticulationUpload(upload);
		m_stats.m_dispatch_count = 0;
		if (upload.m_articulations.empty())
		{
			ReleaseBuffers();
			return false;
		}

		ResizeBuffers(job.m_cmd_list, upload);
		m_levels = upload.m_levels;
		m_articulation_count = isize(upload.m_articulations);
		m_link_count = isize(upload.m_links);
		m_dof_count = isize(upload.m_dofs);
		m_position_count = isize(upload.m_positions);
		m_velocity_count = isize(upload.m_velocities);
		m_force_count = isize(upload.m_forces);
		m_child_count = isize(upload.m_children);
		m_acceleration_count = isize(upload.m_accelerations);
		m_joint_matrix_count = upload.m_joint_matrix_scratch_count;

		// Immutable streams and the phase-reused generalized output are copied together before kernel access.
		job.m_barriers.Transition(m_r_articulations.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_links.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_dofs.empty())
			job.m_barriers.Transition(m_r_dofs.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_positions.empty())
			job.m_barriers.Transition(m_r_positions.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_velocities.empty())
			job.m_barriers.Transition(m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_forces.empty())
			job.m_barriers.Transition(m_r_forces.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_external_forces.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_children.empty())
			job.m_barriers.Transition(m_r_children.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_level_links.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (!upload.m_accelerations.empty())
			job.m_barriers.Transition(m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();

		CopyArticulationUpload(job, m_r_articulations.get(), upload.m_articulations);
		CopyArticulationUpload(job, m_r_links.get(), upload.m_links);
		CopyArticulationUpload(job, m_r_dofs.get(), upload.m_dofs);
		CopyArticulationUpload(job, m_r_positions.get(), upload.m_positions);
		CopyArticulationUpload(job, m_r_velocities.get(), upload.m_velocities);
		CopyArticulationUpload(job, m_r_forces.get(), upload.m_forces);
		CopyArticulationUpload(job, m_r_external_forces.get(), upload.m_external_forces);
		CopyArticulationUpload(job, m_r_children.get(), upload.m_children);
		CopyArticulationUpload(job, m_r_level_links.get(), upload.m_level_links);
		CopyArticulationUpload(job, m_r_accelerations.get(), upload.m_accelerations);

		// Every declared root resource reaches its required state, including shared sentinels for empty streams.
		job.m_barriers.Transition(m_r_articulations.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_dof_count != 0 ? m_r_dofs : m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_position_count != 0 ? m_r_positions : m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_velocity_count != 0 ? m_r_velocities : m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_force_count != 0 ? m_r_forces : m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_child_count != 0 ? m_r_children : m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_level_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_acceleration_count != 0 ? m_r_accelerations : m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_dof_count != 0 ? m_r_dof_scratch : m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_joint_matrix_count != 0 ? m_r_joint_matrix_scratch : m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		// Logical scratch follows active dimensions even when retained capacities grow geometrically.
		m_stats.m_logical_scratch_bytes =
			static_cast<size_t>(m_link_count) * sizeof(GpuArticulationAbaScratch) +
			static_cast<size_t>(m_dof_count) * sizeof(GpuArticulationAbaDofScratch) +
			static_cast<size_t>(m_joint_matrix_count) * sizeof(float);
		m_stats.m_logical_feature_bytes =
			static_cast<size_t>(m_articulation_count) * sizeof(GpuArticulation) +
			static_cast<size_t>(m_link_count) * sizeof(GpuArticulationLink) +
			static_cast<size_t>(m_dof_count) * sizeof(GpuArticulationDof) +
			static_cast<size_t>(m_position_count) * sizeof(float) +
			static_cast<size_t>(m_velocity_count) * sizeof(float) +
			static_cast<size_t>(m_force_count) * sizeof(float) +
			static_cast<size_t>(m_acceleration_count) * sizeof(float) +
			upload.m_external_forces.size() * sizeof(GpuFrameForce) +
			static_cast<size_t>(m_child_count) * sizeof(uint32_t) +
			upload.m_level_links.size() * sizeof(uint32_t) +
			m_stats.m_logical_scratch_bytes;
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_articulation_capacity) * sizeof(GpuArticulation) +
			static_cast<size_t>(m_stats.m_link_capacity) * sizeof(GpuArticulationLink) +
			static_cast<size_t>(m_stats.m_dof_capacity) * sizeof(GpuArticulationDof) +
			static_cast<size_t>(m_stats.m_position_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_velocity_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_force_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_acceleration_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_joint_matrix_capacity) * sizeof(float) +
			static_cast<size_t>(m_stats.m_external_force_capacity) * sizeof(GpuFrameForce) +
			static_cast<size_t>(m_stats.m_child_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_level_link_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_scratch_capacity) * sizeof(GpuArticulationAbaScratch) +
			static_cast<size_t>(m_stats.m_dof_scratch_capacity) * sizeof(GpuArticulationAbaDofScratch) +
			(m_r_srv_sentinel != nullptr ? sizeof(GpuArticulationAbaDofScratch) : 0) +
			(m_r_uav_sentinel != nullptr ? sizeof(GpuArticulationAbaDofScratch) : 0);
		return true;
	}

	// Bind the shared root layout and dispatch one non-empty level or root range.
	void GpuArticulationForceAba::Dispatch(GpuJob& job, ComputeStep& step, GpuArticulationLevel const& level, int item_count)
	{
		auto const constants = cbArticulationForceAba{
			.level_offset = level.link_offset,
			.level_count = level.link_count,
			.articulation_count = m_articulation_count,
			.link_count = m_link_count,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_dof_count != 0 ? m_r_dofs : m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_position_count != 0 ? m_r_positions : m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_velocity_count != 0 ? m_r_velocities : m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_force_count != 0 ? m_r_forces : m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_child_count != 0 ? m_r_children : m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_level_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_acceleration_count != 0 ? m_r_accelerations : m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_dof_count != 0 ? m_r_dof_scratch : m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_joint_matrix_count != 0 ? m_r_joint_matrix_scratch : m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(ArticulationThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
	}

	// Order all writable ranges that can feed the next dependent force-ABA pass.
	void GpuArticulationForceAba::CommitUavBarriers(GpuJob& job)
	{
		job.m_barriers.UAV(m_r_scratch.get());
		if (m_acceleration_count != 0)
			job.m_barriers.UAV(m_r_accelerations.get());
		if (m_dof_count != 0)
			job.m_barriers.UAV(m_r_dof_scratch.get());
		if (m_joint_matrix_count != 0)
			job.m_barriers.UAV(m_r_joint_matrix_scratch.get());
		job.m_barriers.Commit();
	}

	// Record prepare, inward, root, and outward passes in deterministic breadth-level order.
	void GpuArticulationForceAba::Run(GpuJob& job)
	{
		m_stats.m_dispatch_count = 0;
		if (m_articulation_count == 0)
			return;

		// Prepare levels consume solved parent kinematics, so every level completes before its children begin.
		for (auto const& level : m_levels)
		{
			Dispatch(job, m_cs_prepare, level, level.link_count);
			CommitUavBarriers(job);
		}

		// Inward levels reduce completed direct children in stable parent-owned adjacency order.
		for (int level_index = isize(m_levels); level_index-- != 0;)
		{
			auto const& level = m_levels[level_index];
			Dispatch(job, m_cs_inward, level, level.link_count);
			CommitUavBarriers(job);
		}

		// Independent roots impose fixed acceleration or solve their complete six-dimensional inertia.
		Dispatch(job, m_cs_root, GpuArticulationLevel{}, m_articulation_count);
		CommitUavBarriers(job);

		// Non-root outward levels overwrite reduced forces with qdd and bias storage with link acceleration.
		for (int level_index = 1; level_index != isize(m_levels); ++level_index)
		{
			auto const& level = m_levels[level_index];
			Dispatch(job, m_cs_outward, level, level.link_count);
			CommitUavBarriers(job);
		}
	}

	// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
	GpuArticulationForceAbaResult GpuArticulationForceAba::Solve(GpuJob& job, GpuArticulationUpload const& upload)
	{
		auto result = GpuArticulationForceAbaResult{};
		if (!Upload(job, upload))
			return result;

		Run(job);

		// Read only generalized output and compact per-link cores; large per-DOF factors and inverses remain GPU-local.
		if (m_acceleration_count != 0)
			job.m_barriers.Transition(m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto acceleration_readback = ReadbackAlloc{};
		if (m_acceleration_count != 0)
		{
			acceleration_readback = job.m_readback.Alloc<float>(m_acceleration_count);
			job.m_cmd_list.CopyBufferRegion(acceleration_readback, m_r_accelerations.get(), 0);
		}
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(m_link_count);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, m_r_scratch.get(), 0);
		if (m_acceleration_count != 0)
			job.m_barriers.Transition(m_r_accelerations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
		job.Run();

		// Convert phase-reused core fields into a stable typed result after the single submission completes.
		result.m_accelerations.resize(m_acceleration_count);
		if (m_acceleration_count != 0)
			memcpy(result.m_accelerations.data(), acceleration_readback.ptr<float>(), result.m_accelerations.size() * sizeof(float));
		result.m_link_accelerations.reserve(m_link_count);
		result.m_solve_valid.reserve(m_link_count);
		auto const* scratch = scratch_readback.ptr<GpuArticulationAbaScratch>();
		for (int link_index = 0; link_index != m_link_count; ++link_index)
		{
			result.m_link_accelerations.push_back(scratch[link_index].articulated_bias_or_acceleration);
			result.m_solve_valid.push_back(scratch[link_index].solve_valid);
		}
		return result;
	}

	// Return current logical usage, retained capacities, and the most recent dispatch count.
	GpuArticulationForceAbaStats const& GpuArticulationForceAba::Stats() const
	{
		return m_stats;
	}

	// Destroy lazily owned force-ABA resources where the implementation type is complete.
	void Deleter<GpuArticulationForceAba>::operator()(GpuArticulationForceAba* solver) const
	{
		delete solver;
	}
}
