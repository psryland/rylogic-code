//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/cast.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Root-register assignments shared with coupled_constraint_velocity.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Endpoints = ESRVReg::t0;
			inline static constexpr auto CoupledEndpoints = ESRVReg::t1;
			inline static constexpr auto BlockTopology = ESRVReg::t2;
			inline static constexpr auto Targets = ESRVReg::t3;
			inline static constexpr auto Adjacency = ESRVReg::t4;
			inline static constexpr auto Preconditioners = ESRVReg::t5;
			inline static constexpr auto AbaScratch = ESRVReg::t6;
			inline static constexpr auto ArticulationIslands = ESRVReg::t7;
			inline static constexpr auto Islands = ESRVReg::t8;
			inline static constexpr auto IslandBlocks = ESRVReg::t9;
			inline static constexpr auto Blocks = EUAVReg::u1;
			inline static constexpr auto Rows = EUAVReg::u2;
			inline static constexpr auto Scratch = EUAVReg::u3;
			inline static constexpr auto Contributions = EUAVReg::u4;
			inline static constexpr auto TargetImpulses = EUAVReg::u5;
			inline static constexpr auto IslandStates = EUAVReg::u6;
			inline static constexpr auto LinkImpulses = EUAVReg::u7;
			inline static constexpr auto TreeSelection = EUAVReg::u8;
			inline static constexpr auto TreeResults = EUAVReg::u9;
			inline static constexpr auto IslandFailures = EUAVReg::u10;
			inline static constexpr auto ArticulationWork = EUAVReg::u11;
		};

		// Match the HLSL bounds and phase-mode constant buffer exactly.
		struct alignas(16) cbCoupledConstraintVelocity
		{
			int m_slot_count;
			int m_body_count;
			int m_target_count;
			int m_adjacency_count;
			int m_island_count;
			int m_articulation_range_count;
			int m_mobility_count;
			int m_work_count;
			int m_island_block_count;
			int m_selection_mode;
			int m_attempt_index;
			int m_backtrack_limit;
			float m_relaxation;
			float m_pad0;
			float m_pad1;
			float m_pad2;
		};
		static_assert(sizeof(cbCoupledConstraintVelocity) == 64);

		// Create or retain one typed resource using geometric high-water growth.
		template <typename Type>
		void EnsureCoupledVelocityBuffer(
			Gpu& gpu,
			CmdList& cmd_list,
			D3DPtr<ID3D12Resource>& resource,
			int required_count,
			int& capacity,
			EUsage usage,
			char const* name)
		{
			if (required_count <= capacity)
				return;

			auto const grown_capacity = std::max(required_count, std::max(1, 2 * capacity));
			resource = gpu.CreateResource(ResDesc::Buf<Type>(grown_capacity, {}).usage(usage), cmd_list, name);
			capacity = grown_capacity;
		}

		// Upload one non-empty immutable typed stream and leave it ready for shader reads.
		template <typename Type>
		void UploadCoupledVelocityStream(GpuJob& job, ID3D12Resource* resource, std::span<Type const> values)
		{
			if (resource == nullptr || values.empty())
				throw std::invalid_argument("Coupled velocity topology streams must be non-empty");

			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<Type>(isize(values));
			memcpy(allocation.ptr<Type>(), values.data(), values.size_bytes());
			job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		}

		// Create and initialize one diagnostic input resource, retaining a one-element sentinel for empty body input.
		template <typename Type>
		D3DPtr<ID3D12Resource> CreateCoupledVelocityInput(
			Gpu& gpu,
			GpuJob& job,
			std::span<Type const> values,
			EUsage usage,
			D3D12_RESOURCE_STATES final_state,
			std::string_view name)
		{
			auto const count = std::max(1, isize(values));
			auto resource = gpu.CreateResource(ResDesc::Buf<Type>(count, {}).usage(usage), job.m_cmd_list, name);
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<Type>(count);
			std::fill_n(allocation.ptr<Type>(), count, Type{});
			if (!values.empty())
				memcpy(allocation.ptr<Type>(), values.data(), values.size_bytes());
			job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
			job.m_barriers.Transition(resource.get(), final_state).Commit();
			return resource;
		}

		// Return the exact dispatch width for a non-empty coupled work range.
		int CoupledVelocityThreadGroupCount(int item_count)
		{
			return (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount;
		}
	}

	// Create all fixed pipeline state without allocating coupled-feature resources.
	GpuCoupledConstraintVelocity::GpuCoupledConstraintVelocity(
		GpuCoupledConstraintPrepare& prepare,
		GpuArticulationImpulseAba& impulse_aba,
		EngineConfig const& config)
		:m_gpu(prepare.m_gpu)
		,m_config(config)
		,m_prepare(prepare)
		,m_impulse_aba(impulse_aba)
		,m_cs_begin()
		,m_cs_candidates()
		,m_cs_gather()
		,m_cs_select_trees()
		,m_cs_validate_trees()
		,m_cs_evaluate_merit()
		,m_cs_commit()
		,m_cs_finalize_islands()
		,m_r_block_topology()
		,m_r_targets()
		,m_r_adjacency()
		,m_r_articulation_islands()
		,m_r_islands()
		,m_r_island_blocks()
		,m_r_scratch()
		,m_r_contributions()
		,m_r_target_impulses()
		,m_r_island_states()
		,m_r_island_failures()
		,m_r_tree_selection()
		,m_r_tree_results()
		,m_source()
		,m_slot_count()
		,m_target_count()
		,m_adjacency_count()
		,m_island_count()
		,m_island_block_count()
		,m_articulation_range_count()
		,m_mobility_count()
		,m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintVelocity>(EReg::Params)
			.UAV(EReg::Bodies)
			.SRV(EReg::Endpoints)
			.SRV(EReg::CoupledEndpoints)
			.SRV(EReg::BlockTopology)
			.SRV(EReg::Targets)
			.SRV(EReg::Adjacency)
			.SRV(EReg::Preconditioners)
			.SRV(EReg::AbaScratch)
			.SRV(EReg::ArticulationIslands)
			.SRV(EReg::Islands)
			.SRV(EReg::IslandBlocks)
			.UAV(EReg::Blocks)
			.UAV(EReg::Rows)
			.UAV(EReg::Scratch)
			.UAV(EReg::Contributions)
			.UAV(EReg::TargetImpulses)
			.UAV(EReg::IslandStates)
			.UAV(EReg::LinkImpulses)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::IslandFailures)
			.UAV(EReg::ArticulationWork)
			.Create(m_gpu, "Physics:CoupledConstraintVelocitySig");
		m_cs_begin.m_sig = root_sig;
		m_cs_begin.m_pso = ComputePSO(root_sig.get(), shader_code::begin_coupled_velocity).Create(m_gpu, "Physics:BeginCoupledVelocityPSO");
		m_cs_candidates.m_sig = root_sig;
		m_cs_candidates.m_pso = ComputePSO(root_sig.get(), shader_code::build_coupled_velocity_candidates).Create(m_gpu, "Physics:BuildCoupledVelocityCandidatesPSO");
		m_cs_gather.m_sig = root_sig;
		m_cs_gather.m_pso = ComputePSO(root_sig.get(), shader_code::gather_coupled_velocity_targets).Create(m_gpu, "Physics:GatherCoupledVelocityTargetsPSO");
		m_cs_select_trees.m_sig = root_sig;
		m_cs_select_trees.m_pso = ComputePSO(root_sig.get(), shader_code::select_coupled_velocity_trees).Create(m_gpu, "Physics:SelectCoupledVelocityTreesPSO");
		m_cs_validate_trees.m_sig = root_sig;
		m_cs_validate_trees.m_pso = ComputePSO(root_sig.get(), shader_code::validate_coupled_velocity_trees).Create(m_gpu, "Physics:ValidateCoupledVelocityTreesPSO");
		m_cs_evaluate_merit.m_sig = root_sig;
		m_cs_evaluate_merit.m_pso = ComputePSO(root_sig.get(), shader_code::evaluate_coupled_velocity_merit).Create(m_gpu, "Physics:EvaluateCoupledVelocityMeritPSO");
		m_cs_commit.m_sig = root_sig;
		m_cs_commit.m_pso = ComputePSO(root_sig.get(), shader_code::commit_coupled_velocity).Create(m_gpu, "Physics:CommitCoupledVelocityPSO");
		m_cs_finalize_islands.m_sig = root_sig;
		m_cs_finalize_islands.m_pso = ComputePSO(root_sig.get(), shader_code::finalize_coupled_velocity_islands).Create(m_gpu, "Physics:FinalizeCoupledVelocityIslandsPSO");
	}

	// Upload deterministic persistent topology after matching constraint and articulation participation streams.
	bool GpuCoupledConstraintVelocity::Upload(GpuJob& job, GpuConstraintUpload const& upload)
	{
		m_stats.m_dispatch_count = 0;
		if (upload.m_coupled_active_count == 0)
		{
			ReleaseBuffers();
			return false;
		}
		if (
			m_prepare.m_source != upload.m_source ||
			m_prepare.m_slot_count != isize(upload.m_endpoints) ||
			m_prepare.m_active_count != s_cast<int>(upload.m_coupled_active_count))
			throw std::logic_error("Coupled velocity topology must follow matching coupled preparation upload");
		if (
			upload.m_coupled_block_topology.size() != upload.m_endpoints.size() ||
			upload.m_coupled_targets.empty() ||
			upload.m_coupled_target_adjacency.empty() ||
			upload.m_coupled_islands.empty() ||
			upload.m_coupled_island_blocks.empty() ||
			upload.m_coupled_articulation_islands.size() != upload.m_coupled_articulation_indices.size())
			throw std::invalid_argument("Coupled velocity upload requires complete deterministic topology");
		if (m_impulse_aba.m_mobility.Ranges().size() != upload.m_coupled_articulation_indices.size())
			throw std::logic_error("Coupled velocity topology must match prepared articulation participation");

		m_source = upload.m_source;
		m_slot_count = isize(upload.m_endpoints);
		m_target_count = isize(upload.m_coupled_targets);
		m_adjacency_count = isize(upload.m_coupled_target_adjacency);
		m_island_count = isize(upload.m_coupled_islands);
		m_island_block_count = isize(upload.m_coupled_island_blocks);
		m_articulation_range_count = isize(upload.m_coupled_articulation_indices);
		auto const ranges = m_impulse_aba.m_mobility.Ranges();
		m_mobility_count = ranges.empty() ? 0 : ranges.back().mobility_offset + ranges.back().link_count;
		if (m_mobility_count <= 0 || !m_impulse_aba.Prepare(job))
			throw std::logic_error("Coupled velocity upload requires prepared articulation impulse storage");
		ResizeBuffers(job.m_cmd_list);

		// Immutable topology is rebuilt frame-locally because rigid and articulation packing indices may change.
		UploadCoupledVelocityStream(job, m_r_block_topology.get(), std::span{upload.m_coupled_block_topology});
		UploadCoupledVelocityStream(job, m_r_targets.get(), std::span{upload.m_coupled_targets});
		UploadCoupledVelocityStream(job, m_r_adjacency.get(), std::span{upload.m_coupled_target_adjacency});
		UploadCoupledVelocityStream(job, m_r_articulation_islands.get(), std::span{upload.m_coupled_articulation_islands});
		UploadCoupledVelocityStream(job, m_r_islands.get(), std::span{upload.m_coupled_islands});
		UploadCoupledVelocityStream(job, m_r_island_blocks.get(), std::span{upload.m_coupled_island_blocks});

		m_stats.m_active_count = s_cast<int>(upload.m_coupled_active_count);
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_slot_count) *
				(sizeof(GpuCoupledConstraintBlockTopology) + sizeof(GpuCoupledConstraintSolveScratch) + 2 * sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_target_count) *
				(sizeof(GpuCoupledConstraintTarget) + sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_adjacency_count) * sizeof(uint32_t) +
			static_cast<size_t>(m_island_count) *
				(sizeof(GpuCoupledConstraintIsland) + sizeof(GpuCoupledConstraintIslandState) + sizeof(uint32_t)) +
			static_cast<size_t>(m_island_block_count) * sizeof(uint32_t) +
			static_cast<size_t>(m_articulation_range_count) * 3 * sizeof(uint32_t);
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_slot_capacity) *
				(sizeof(GpuCoupledConstraintBlockTopology) + sizeof(GpuCoupledConstraintSolveScratch) + 2 * sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_stats.m_target_capacity) *
				(sizeof(GpuCoupledConstraintTarget) + sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_stats.m_adjacency_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_island_capacity) *
				(sizeof(GpuCoupledConstraintIsland) + sizeof(GpuCoupledConstraintIslandState) + sizeof(uint32_t)) +
			static_cast<size_t>(m_stats.m_island_block_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_articulation_range_capacity) * 3 * sizeof(uint32_t);
		return true;
	}

	// Execute one fixed-relaxation transactional simultaneous coupled velocity sweep.
	void GpuCoupledConstraintVelocity::Run(GpuJob& job, int body_count, ID3D12Resource* bodies)
	{
		m_stats.m_dispatch_count = 0;
		if (m_source == nullptr)
			return;
		if (body_count < 0 || bodies == nullptr)
			throw std::invalid_argument("Coupled velocity sweep requires a valid rigid-body stream");
		if (!(m_config.constraint_coupled_relaxation > 0.0f) || m_config.constraint_coupled_relaxation > 1.0f || !std::isfinite(m_config.constraint_coupled_relaxation))
			throw std::invalid_argument("Coupled constraint relaxation must be finite and in (0,1]");
		if (m_config.constraint_coupled_backtrack_limit < 0 || m_config.constraint_coupled_backtrack_limit > 8)
			throw std::invalid_argument("Coupled constraint backtracking must be between zero and eight retries");
		if (m_impulse_aba.LinkImpulses() == nullptr || m_impulse_aba.Work() == nullptr)
			throw std::logic_error("Coupled velocity sweep requires prepared articulation impulse resources");

		auto const work_count = std::max({m_slot_count, m_target_count, m_island_count, m_articulation_range_count, m_mobility_count});

		// Every bounded attempt remains detached; accepted islands become no-ops while pending islands halve relaxation and retry.
		Dispatch(job, m_cs_begin, 0, 0, work_count, body_count, bodies);
		for (int attempt_index = 0; attempt_index != m_config.constraint_coupled_backtrack_limit + 1; ++attempt_index)
		{
			Dispatch(job, m_cs_candidates, 0, attempt_index, m_slot_count, body_count, bodies);
			Dispatch(job, m_cs_gather, 0, attempt_index, m_target_count, body_count, bodies);
			Dispatch(job, m_cs_select_trees, 0, attempt_index, m_articulation_range_count, body_count, bodies);
			m_impulse_aba.Evaluate(job, m_r_tree_selection.get(), m_r_tree_results.get());
			Dispatch(job, m_cs_validate_trees, 0, attempt_index, m_articulation_range_count, body_count, bodies);
			Dispatch(job, m_cs_evaluate_merit, 0, attempt_index, m_island_count, body_count, bodies);
		}
		Dispatch(job, m_cs_select_trees, 1, m_config.constraint_coupled_backtrack_limit, m_articulation_range_count, body_count, bodies);

		// Rigid, row, and articulation state commit under the same accepted island selection.
		Dispatch(job, m_cs_commit, 1, m_config.constraint_coupled_backtrack_limit, work_count, body_count, bodies);
		m_impulse_aba.Commit(job, m_r_tree_selection.get(), m_r_tree_results.get());
		Dispatch(job, m_cs_finalize_islands, 1, m_config.constraint_coupled_backtrack_limit, m_island_count, body_count, bodies);
	}

	// Prepare all dependent production lanes and read one coupled sweep through exactly one GPU submission.
	GpuCoupledConstraintVelocityResult GpuCoupledConstraintVelocity::Solve(
		GpuJob& job,
		float timestep,
		GpuConstraintUpload const& constraint_upload,
		GpuArticulationUpload const& articulation_upload,
		std::span<GpuRigidBody const> bodies,
		std::span<GpuConstraintFrame const> link_to_world,
		std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override)
	{
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled velocity diagnostics require a finite positive timestep");
		if (link_to_world.size() != articulation_upload.m_links.size())
			throw std::invalid_argument("Coupled velocity link-frame count must match the packed articulation forest");

		auto& constraints = m_prepare.m_constraints;
		auto& mobility = m_impulse_aba.m_mobility;
		auto& aba = m_impulse_aba.m_aba;
		constraints.Upload(job, constraint_upload);
		if (!aba.Upload(job, articulation_upload))
			throw std::invalid_argument("Coupled velocity diagnostics require a non-empty articulation forest");
		if (!mobility.Upload(job, articulation_upload, constraint_upload.m_coupled_articulation_indices))
			throw std::invalid_argument("Coupled velocity diagnostics require participating articulation ranges");
		if (!m_prepare.Upload(job, constraint_upload))
			return {};
		if (!Upload(job, constraint_upload))
			return {};

		auto r_bodies = CreateCoupledVelocityInput(m_gpu, job, bodies, EUsage::UnorderedAccess, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, "Physics:CoupledVelocityTestBodies");
		auto r_link_to_world = CreateCoupledVelocityInput(m_gpu, job, link_to_world, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledVelocityTestLinkFrames");
		mobility.Run(job);
		m_prepare.Run(job, timestep, isize(bodies), r_bodies.get(), r_link_to_world.get(), mobility);
		if (!preconditioner_override.empty())
		{
			if (preconditioner_override.size() != constraint_upload.m_endpoints.size())
				throw std::invalid_argument("Coupled velocity preconditioner override must contain one stable-slot value");

			job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<GpuCoupledConstraintPreconditioner>(isize(preconditioner_override));
			memcpy(allocation.ptr<GpuCoupledConstraintPreconditioner>(), preconditioner_override.data(), preconditioner_override.size_bytes());
			job.m_cmd_list.CopyBufferRegion(m_prepare.m_r_preconditioners.get(), 0, allocation);
		}
		Run(job, isize(bodies), r_bodies.get());

		// Capture all authoritative outputs and transaction diagnostics before the caller's single submission.
		auto result = GpuCoupledConstraintVelocityResult{};
		result.m_bodies.resize(bodies.size());
		result.m_rows.resize(static_cast<size_t>(GpuConstraintRowsPerBlock) * m_slot_count);
		result.m_articulation_velocities.resize(aba.m_velocity_count);
		result.m_articulation_accelerations.resize(aba.m_acceleration_count);
		result.m_articulation_scratch.resize(aba.m_link_count);
		result.m_island_states.resize(m_island_count);
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_velocity_count != 0)
			job.m_barriers.Transition(aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_acceleration_count != 0)
			job.m_barriers.Transition(aba.m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();

		auto body_readback = ReadbackAlloc{};
		if (!bodies.empty())
			body_readback = job.m_readback.Alloc<GpuRigidBody>(isize(bodies));
		auto row_readback = job.m_readback.Alloc<GpuConstraintRow>(GpuConstraintRowsPerBlock * m_slot_count);
		auto velocity_readback = ReadbackAlloc{};
		if (aba.m_velocity_count != 0)
			velocity_readback = job.m_readback.Alloc<float>(aba.m_velocity_count);
		auto acceleration_readback = ReadbackAlloc{};
		if (aba.m_acceleration_count != 0)
			acceleration_readback = job.m_readback.Alloc<float>(aba.m_acceleration_count);
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(aba.m_link_count);
		auto island_readback = job.m_readback.Alloc<GpuCoupledConstraintIslandState>(m_island_count);
		if (!bodies.empty())
			job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
		job.m_cmd_list.CopyBufferRegion(row_readback, constraints.m_r_rows.get(), 0);
		if (aba.m_velocity_count != 0)
			job.m_cmd_list.CopyBufferRegion(velocity_readback, aba.m_r_velocities.get(), 0);
		if (aba.m_acceleration_count != 0)
			job.m_cmd_list.CopyBufferRegion(acceleration_readback, aba.m_r_accelerations.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, aba.m_r_scratch.get(), 0);
		job.m_cmd_list.CopyBufferRegion(island_readback, m_r_island_states.get(), 0);
		job.Run();

		if (!bodies.empty())
			memcpy(result.m_bodies.data(), body_readback.ptr<GpuRigidBody>(), result.m_bodies.size() * sizeof(result.m_bodies[0]));
		memcpy(result.m_rows.data(), row_readback.ptr<GpuConstraintRow>(), result.m_rows.size() * sizeof(result.m_rows[0]));
		if (aba.m_velocity_count != 0)
			memcpy(result.m_articulation_velocities.data(), velocity_readback.ptr<float>(), result.m_articulation_velocities.size() * sizeof(result.m_articulation_velocities[0]));
		if (aba.m_acceleration_count != 0)
			memcpy(result.m_articulation_accelerations.data(), acceleration_readback.ptr<float>(), result.m_articulation_accelerations.size() * sizeof(result.m_articulation_accelerations[0]));
		memcpy(result.m_articulation_scratch.data(), scratch_readback.ptr<GpuArticulationAbaScratch>(), result.m_articulation_scratch.size() * sizeof(result.m_articulation_scratch[0]));
		memcpy(result.m_island_states.data(), island_readback.ptr<GpuCoupledConstraintIslandState>(), result.m_island_states.size() * sizeof(result.m_island_states[0]));
		return result;
	}

	// Return current logical usage, retained capacities, and most recent coupled dispatch count.
	GpuCoupledConstraintVelocityStats const& GpuCoupledConstraintVelocity::Stats() const
	{
		return m_stats;
	}

	// Release every optional coupled velocity resource when no coupled work remains.
	void GpuCoupledConstraintVelocity::ReleaseBuffers()
	{
		m_r_block_topology = nullptr;
		m_r_targets = nullptr;
		m_r_adjacency = nullptr;
		m_r_articulation_islands = nullptr;
		m_r_islands = nullptr;
		m_r_island_blocks = nullptr;
		m_r_scratch = nullptr;
		m_r_contributions = nullptr;
		m_r_target_impulses = nullptr;
		m_r_island_states = nullptr;
		m_r_island_failures = nullptr;
		m_r_tree_selection = nullptr;
		m_r_tree_results = nullptr;
		m_source = nullptr;
		m_slot_count = 0;
		m_target_count = 0;
		m_adjacency_count = 0;
		m_island_count = 0;
		m_island_block_count = 0;
		m_articulation_range_count = 0;
		m_mobility_count = 0;
		m_stats = {};
	}

	// Create or geometrically grow topology and detached transaction buffers.
	void GpuCoupledConstraintVelocity::ResizeBuffers(CmdList& cmd_list)
	{
		// Resources sharing one logical index grow together so one capacity describes their retained cost exactly.
		if (m_slot_count > m_stats.m_slot_capacity)
		{
			m_stats.m_slot_capacity = std::max(m_slot_count, std::max(1, 2 * m_stats.m_slot_capacity));
			m_r_block_topology = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintBlockTopology>(m_stats.m_slot_capacity, {}).usage(EUsage::Default), cmd_list, "Physics:CoupledVelocityBlockTopology");
			m_r_scratch = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintSolveScratch>(m_stats.m_slot_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityScratch");
			m_r_contributions = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(2 * m_stats.m_slot_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityContributions");
		}

		if (m_target_count > m_stats.m_target_capacity)
		{
			m_stats.m_target_capacity = std::max(m_target_count, std::max(1, 2 * m_stats.m_target_capacity));
			m_r_targets = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintTarget>(m_stats.m_target_capacity, {}).usage(EUsage::Default), cmd_list, "Physics:CoupledVelocityTargets");
			m_r_target_impulses = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(m_stats.m_target_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityTargetImpulses");
		}

		EnsureCoupledVelocityBuffer<uint32_t>(m_gpu, cmd_list, m_r_adjacency, m_adjacency_count, m_stats.m_adjacency_capacity, EUsage::Default, "Physics:CoupledVelocityAdjacency");
		if (m_island_count > m_stats.m_island_capacity)
		{
			m_stats.m_island_capacity = std::max(m_island_count, std::max(1, 2 * m_stats.m_island_capacity));
			m_r_islands = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintIsland>(m_stats.m_island_capacity, {}).usage(EUsage::Default), cmd_list, "Physics:CoupledVelocityIslands");
			m_r_island_states = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintIslandState>(m_stats.m_island_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityIslandStates");
			m_r_island_failures = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(m_stats.m_island_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityIslandFailures");
		}
		EnsureCoupledVelocityBuffer<uint32_t>(m_gpu, cmd_list, m_r_island_blocks, m_island_block_count, m_stats.m_island_block_capacity, EUsage::Default, "Physics:CoupledVelocityIslandBlocks");

		if (m_articulation_range_count > m_stats.m_articulation_range_capacity)
		{
			m_stats.m_articulation_range_capacity = std::max(m_articulation_range_count, std::max(1, 2 * m_stats.m_articulation_range_capacity));
			m_r_articulation_islands = m_gpu.CreateResource(ResDesc::Buf<int>(m_stats.m_articulation_range_capacity, {}).usage(EUsage::Default), cmd_list, "Physics:CoupledVelocityArticulationIslands");
			m_r_tree_selection = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(m_stats.m_articulation_range_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityTreeSelection");
			m_r_tree_results = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(m_stats.m_articulation_range_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledVelocityTreeResults");
		}
	}

	// Bind one coupled phase against the complete stable resource layout.
	void GpuCoupledConstraintVelocity::Dispatch(
		GpuJob& job,
		ComputeStep& step,
		int selection_mode,
		int attempt_index,
		int item_count,
		int body_count,
		ID3D12Resource* bodies)
	{
		if (item_count <= 0)
			return;

		PrepareResources(job, bodies);
		auto const constants = cbCoupledConstraintVelocity{
			.m_slot_count = m_slot_count,
			.m_body_count = body_count,
			.m_target_count = m_target_count,
			.m_adjacency_count = m_adjacency_count,
			.m_island_count = m_island_count,
			.m_articulation_range_count = m_articulation_range_count,
			.m_mobility_count = m_mobility_count,
			.m_work_count = std::max({m_slot_count, m_target_count, m_island_count, m_articulation_range_count, m_mobility_count}),
			.m_island_block_count = m_island_block_count,
			.m_selection_mode = selection_mode,
			.m_attempt_index = attempt_index,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit,
			.m_relaxation = m_config.constraint_coupled_relaxation,
		};
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(constraints.m_r_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_coupled_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_block_topology->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_targets->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_adjacency->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_preconditioners->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_articulation_islands->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_islands->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_island_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_rows->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contributions->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_target_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_island_states->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.LinkImpulses()->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_island_failures->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.Work()->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledVelocityThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Transition the common coupled resources into the states required by a coupled shader phase.
	void GpuCoupledConstraintVelocity::PrepareResources(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_prepare.m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_block_topology.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_targets.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_adjacency.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_articulation_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_island_blocks.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contributions.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_target_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.LinkImpulses(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_island_failures.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.Work(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Order every mutable coupled resource before its next dependent phase.
	void GpuCoupledConstraintVelocity::CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		job.m_barriers.UAV(bodies);
		job.m_barriers.UAV(constraints.m_r_blocks.get());
		job.m_barriers.UAV(constraints.m_r_rows.get());
		job.m_barriers.UAV(m_r_scratch.get());
		job.m_barriers.UAV(m_r_contributions.get());
		job.m_barriers.UAV(m_r_target_impulses.get());
		job.m_barriers.UAV(m_r_island_states.get());
		job.m_barriers.UAV(m_impulse_aba.LinkImpulses());
		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_island_failures.get());
		job.m_barriers.UAV(m_impulse_aba.Work());
		job.m_barriers.Commit();
	}
}
