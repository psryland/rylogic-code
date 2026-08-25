//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/cast.h"
#include "src/compute/coupled_constraint_position_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Root-register assignments shared with coupled_constraint_position.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto LinkEndpoints = ESRVReg::t0;
			inline static constexpr auto BlockTopology = ESRVReg::t1;
			inline static constexpr auto Targets = ESRVReg::t2;
			inline static constexpr auto Adjacency = ESRVReg::t3;
			inline static constexpr auto Preconditioners = ESRVReg::t4;
			inline static constexpr auto ArticulationIslands = ESRVReg::t5;
			inline static constexpr auto Islands = ESRVReg::t6;
			inline static constexpr auto IslandBlocks = ESRVReg::t7;
			inline static constexpr auto Ranges = ESRVReg::t8;
			inline static constexpr auto Links = ESRVReg::t9;
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
			inline static constexpr auto VelocityDeltas = EUAVReg::u12;
			inline static constexpr auto RigidPseudo = EUAVReg::u13;
			inline static constexpr auto LinkPseudo = EUAVReg::u14;
			inline static constexpr auto GeneralizedPseudo = EUAVReg::u15;
			inline static constexpr auto Articulations = EUAVReg::u16;
			inline static constexpr auto Positions = EUAVReg::u17;
		};

		// Match the HLSL bounds, controls, and phase-mode constant buffer exactly.
		struct alignas(16) cbCoupledConstraintPosition
		{
			int m_slot_count;
			int m_body_count;
			int m_target_count;
			int m_island_count;
			int m_articulation_range_count;
			int m_mobility_count;
			int m_velocity_delta_count;
			int m_island_block_count;
			int m_phase;
			int m_attempt_index;
			int m_backtrack_limit;
			int m_pad0;
			float m_timestep;
			float m_relaxation;
			float m_position_beta;
			float m_max_position_speed;
		};
		static_assert(sizeof(cbCoupledConstraintPosition) == 64);

		// Return the exact dispatch width for one non-empty position work range.
		int CoupledPositionThreadGroupCount(int item_count)
		{
			return (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount;
		}

		// Create and initialize one diagnostic input, retaining a one-element sentinel for an empty stream.
		template <typename Type>
		D3DPtr<ID3D12Resource> CreateCoupledPositionInput(
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
	}

	// Create fixed pipeline state without allocating optional pseudo-state resources.
	GpuCoupledConstraintPosition::GpuCoupledConstraintPosition(GpuCoupledConstraintVelocity& velocity, EngineConfig const& config)
		: m_gpu(velocity.m_gpu)
		, m_config(config)
		, m_velocity(velocity)
		, m_prepare(velocity.m_prepare)
		, m_impulse_aba(velocity.m_impulse_aba)
		, m_cs_clear()
		, m_cs_begin()
		, m_cs_candidates()
		, m_cs_gather()
		, m_cs_select_trees()
		, m_cs_validate_trees()
		, m_cs_evaluate_merit()
		, m_cs_commit_state()
		, m_cs_commit_articulations()
		, m_cs_finalize_islands()
		, m_cs_apply()
		, m_r_link_pseudo()
		, m_r_generalized_pseudo()
		, m_source()
		, m_timestep()
		, m_body_count()
		, m_velocity_delta_count()
		, m_stats()
	{
		// The common layout deliberately occupies exactly 64 DWORDs: 16 constants, eight SRVs, and sixteen UAVs.
		auto common_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintPosition>(EReg::Params)
			.SRV(EReg::LinkEndpoints)
			.SRV(EReg::BlockTopology)
			.SRV(EReg::Targets)
			.SRV(EReg::Adjacency)
			.SRV(EReg::Preconditioners)
			.SRV(EReg::ArticulationIslands)
			.SRV(EReg::Islands)
			.SRV(EReg::IslandBlocks)
			.UAV(EReg::Bodies)
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
			.UAV(EReg::VelocityDeltas)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.Create(m_gpu, "Physics:CoupledConstraintPositionCommonSig");
		auto CompileCommon = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			step.m_sig = common_sig;
			step.m_pso = ComputePSO(common_sig.get(), bytecode).Create(m_gpu, name);
		};
		CompileCommon(m_cs_clear, shader_code::clear_coupled_position_state, "Physics:ClearCoupledPositionStatePSO");
		CompileCommon(m_cs_begin, shader_code::begin_coupled_position, "Physics:BeginCoupledPositionPSO");
		CompileCommon(m_cs_candidates, shader_code::build_coupled_position_candidates, "Physics:BuildCoupledPositionCandidatesPSO");
		CompileCommon(m_cs_gather, shader_code::gather_coupled_position_targets, "Physics:GatherCoupledPositionTargetsPSO");
		CompileCommon(m_cs_evaluate_merit, shader_code::evaluate_coupled_position_merit, "Physics:EvaluateCoupledPositionMeritPSO");
		CompileCommon(m_cs_commit_state, shader_code::commit_coupled_position_state, "Physics:CommitCoupledPositionStatePSO");
		CompileCommon(m_cs_finalize_islands, shader_code::finalize_coupled_position_islands, "Physics:FinalizeCoupledPositionIslandsPSO");

		// Complete-tree validation and commit need ranges and articulation headers but not the wider topology layout.
		auto articulation_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintPosition>(EReg::Params)
			.SRV(EReg::ArticulationIslands)
			.SRV(EReg::Ranges)
			.UAV(EReg::IslandStates)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::IslandFailures)
			.UAV(EReg::ArticulationWork)
			.UAV(EReg::VelocityDeltas)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.Create(m_gpu, "Physics:CoupledConstraintPositionArticulationSig");
		m_cs_select_trees.m_sig = articulation_sig;
		m_cs_select_trees.m_pso = ComputePSO(articulation_sig.get(), shader_code::select_coupled_position_trees).Create(m_gpu, "Physics:SelectCoupledPositionTreesPSO");
		m_cs_validate_trees.m_sig = articulation_sig;
		m_cs_validate_trees.m_pso = ComputePSO(articulation_sig.get(), shader_code::validate_coupled_position_trees).Create(m_gpu, "Physics:ValidateCoupledPositionTreesPSO");
		m_cs_commit_articulations.m_sig = articulation_sig;
		m_cs_commit_articulations.m_pso = ComputePSO(articulation_sig.get(), shader_code::commit_coupled_position_articulations).Create(m_gpu, "Physics:CommitCoupledPositionArticulationsPSO");

		// Final coordinate application has a small isolated layout and cannot accidentally bind physical velocity buffers.
		auto apply_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintPosition>(EReg::Params)
			.SRV(EReg::Ranges)
			.SRV(EReg::Links)
			.UAV(EReg::Bodies)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.UAV(EReg::Positions)
			.Create(m_gpu, "Physics:CoupledConstraintPositionApplySig");
		m_cs_apply.m_sig = apply_sig;
		m_cs_apply.m_pso = ComputePSO(apply_sig.get(), shader_code::apply_coupled_position).Create(m_gpu, "Physics:ApplyCoupledPositionPSO");
	}

	// Prepare position-only exact-self inverses and clear detached pseudo state once per substep.
	bool GpuCoupledConstraintPosition::Prepare(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, ID3D12Resource* link_to_world)
	{
		m_stats.m_dispatch_count = 0;
		if (m_config.push_out_iterations <= 0 || m_velocity.m_source == nullptr)
		{
			ReleaseBuffers();
			return false;
		}
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled position correction requires a finite positive timestep");
		if (body_count < 0 || bodies == nullptr || link_to_world == nullptr)
			throw std::invalid_argument("Coupled position correction requires valid rigid and link-frame streams");
		if (!(m_config.constraint_position_beta >= 0.0f) || !std::isfinite(m_config.constraint_position_beta))
			throw std::invalid_argument("Constraint position beta must be finite and non-negative");
		if (!(m_config.constraint_max_position_speed >= 0.0f) || !std::isfinite(m_config.constraint_max_position_speed))
			throw std::invalid_argument("Constraint maximum position speed must be finite and non-negative");

		auto const relaxation = std::min(m_config.constraint_position_relaxation, m_config.constraint_coupled_relaxation);
		if (!(relaxation > 0.0f) || relaxation > 1.0f || !std::isfinite(relaxation))
			throw std::invalid_argument("Coupled position relaxation must be finite and in (0,1]");
		if (m_config.constraint_coupled_backtrack_limit < 0 || m_config.constraint_coupled_backtrack_limit > 8)
			throw std::invalid_argument("Coupled position backtracking must be between zero and eight retries");

		auto& constraints = m_prepare.m_constraints;
		auto& mobility = m_impulse_aba.m_mobility;
		if (m_velocity.m_source != m_prepare.m_source || m_velocity.m_articulation_range_count != isize(mobility.m_ranges))
			throw std::logic_error("Coupled position correction must follow matching velocity topology and mobility preparation");
		if (m_impulse_aba.m_r_work == nullptr || m_impulse_aba.m_r_link_impulses == nullptr)
			throw std::logic_error("Coupled position correction requires detached articulation impulse storage");

		m_source = m_velocity.m_source;
		m_timestep = timestep;
		m_body_count = body_count;
		m_velocity_delta_count = mobility.m_velocity_delta_count;
		constraints.EnsurePseudoVelocityStorage(job.m_cmd_list, body_count);
		ResizeBuffers(job.m_cmd_list);
		m_prepare.PreparePositionPreconditioners(job, body_count, bodies, link_to_world, mobility);

		m_stats.m_active_count = m_velocity.m_stats.m_active_count;
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_velocity.m_mobility_count) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_velocity_delta_count) * sizeof(float);
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_link_pseudo_capacity) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_stats.m_generalized_pseudo_capacity) * sizeof(float);

		auto const work_count = std::max({
			m_velocity.m_slot_count,
			m_body_count,
			m_velocity.m_target_count,
			m_velocity.m_island_count,
			m_velocity.m_articulation_range_count,
			m_velocity.m_mobility_count,
			m_velocity_delta_count,
		});
		DispatchCommon(job, m_cs_clear, GpuCoupledConstraintPhase_Evaluate, 0, work_count, bodies);
		return true;
	}

	// Execute one bounded-backtracking simultaneous pseudo-position sweep.
	void GpuCoupledConstraintPosition::Run(GpuJob& job, ID3D12Resource* bodies)
	{
		if (m_source == nullptr)
			return;
		if (bodies == nullptr)
			throw std::invalid_argument("Coupled position sweep requires a valid rigid-body stream");

		auto const work_count = std::max({
			m_velocity.m_slot_count,
			m_velocity.m_target_count,
			m_velocity.m_island_count,
			m_velocity.m_articulation_range_count,
			m_velocity.m_mobility_count,
			m_velocity_delta_count,
		});
		DispatchCommon(job, m_cs_begin, GpuCoupledConstraintPhase_Evaluate, 0, work_count, bodies);
		for (int attempt_index = 0; attempt_index != m_config.constraint_coupled_backtrack_limit + 1; ++attempt_index)
		{
			DispatchCommon(job, m_cs_candidates, GpuCoupledConstraintPhase_Evaluate, attempt_index, m_velocity.m_slot_count, bodies);
			DispatchCommon(job, m_cs_gather, GpuCoupledConstraintPhase_Evaluate, attempt_index, m_velocity.m_target_count, bodies);
			DispatchArticulations(job, m_cs_select_trees, GpuCoupledConstraintPhase_Evaluate, attempt_index, m_velocity.m_articulation_range_count);
			m_impulse_aba.Evaluate(job, m_velocity.m_r_tree_selection.get(), m_velocity.m_r_tree_results.get());
			DispatchArticulations(job, m_cs_validate_trees, GpuCoupledConstraintPhase_Evaluate, attempt_index, m_velocity.m_articulation_range_count);
			DispatchCommon(job, m_cs_evaluate_merit, GpuCoupledConstraintPhase_Evaluate, attempt_index, m_velocity.m_island_count, bodies);
		}

		DispatchArticulations(job, m_cs_select_trees, GpuCoupledConstraintPhase_CommitPosition, m_config.constraint_coupled_backtrack_limit, m_velocity.m_articulation_range_count);
		DispatchCommon(job, m_cs_commit_state, GpuCoupledConstraintPhase_CommitPosition, m_config.constraint_coupled_backtrack_limit, std::max(m_velocity.m_slot_count, m_velocity.m_target_count), bodies);
		DispatchArticulations(job, m_cs_commit_articulations, GpuCoupledConstraintPhase_CommitPosition, m_config.constraint_coupled_backtrack_limit, m_velocity.m_articulation_range_count);
		DispatchCommon(job, m_cs_finalize_islands, GpuCoupledConstraintPhase_CommitPosition, m_config.constraint_coupled_backtrack_limit, m_velocity.m_island_count, bodies);
	}

	// Integrate converged rigid and articulation pseudo state exactly once into coordinates.
	void GpuCoupledConstraintPosition::Apply(GpuJob& job, ID3D12Resource* bodies)
	{
		if (m_source == nullptr)
			return;
		if (bodies == nullptr)
			throw std::invalid_argument("Coupled position application requires a valid rigid-body stream");

		DispatchApply(job, bodies);
	}

	// Prepare every dependency, solve configured position iterations, and read focused state through one submission.
	GpuCoupledConstraintPositionResult GpuCoupledConstraintPosition::Solve(
		GpuJob& job,
		float timestep,
		GpuConstraintUpload const& constraint_upload,
		GpuArticulationUpload const& articulation_upload,
		std::span<GpuRigidBody const> bodies,
		std::span<GpuConstraintFrame const> link_to_world,
		std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override,
		std::span<GpuConstraintRow const> prepared_row_override)
	{
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled position diagnostics require a finite positive timestep");
		if (link_to_world.size() != articulation_upload.m_links.size())
			throw std::invalid_argument("Coupled position link-frame count must match the packed articulation forest");

		auto& constraints = m_prepare.m_constraints;
		auto& mobility = m_impulse_aba.m_mobility;
		auto& aba = m_impulse_aba.m_aba;
		constraints.Upload(job, constraint_upload);
		if (!aba.Upload(job, articulation_upload))
			throw std::invalid_argument("Coupled position diagnostics require a non-empty articulation forest");
		if (!mobility.Upload(job, articulation_upload, constraint_upload.m_coupled_articulation_indices))
			throw std::invalid_argument("Coupled position diagnostics require participating articulation ranges");
		if (!m_prepare.Upload(job, constraint_upload))
			return {};
		if (!m_velocity.Upload(job, constraint_upload))
			return {};

		auto r_bodies = CreateCoupledPositionInput(m_gpu, job, bodies, EUsage::UnorderedAccess, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, "Physics:CoupledPositionTestBodies");
		auto r_link_to_world = CreateCoupledPositionInput(m_gpu, job, link_to_world, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledPositionTestLinkFrames");
		mobility.Run(job);
		m_prepare.Run(job, timestep, isize(bodies), r_bodies.get(), r_link_to_world.get(), mobility);
		if (!prepared_row_override.empty())
		{
			if (prepared_row_override.size() != static_cast<size_t>(GpuConstraintRowsPerBlock * m_velocity.m_slot_count))
				throw std::invalid_argument("Coupled position prepared-row override must contain six rows per stable slot");

			job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<GpuConstraintRow>(isize(prepared_row_override));
			memcpy(allocation.ptr<GpuConstraintRow>(), prepared_row_override.data(), prepared_row_override.size_bytes());
			job.m_cmd_list.CopyBufferRegion(constraints.m_r_rows.get(), 0, allocation);
		}
		if (!Prepare(job, timestep, isize(bodies), r_bodies.get(), r_link_to_world.get()))
			return {};
		if (!preconditioner_override.empty())
		{
			if (preconditioner_override.size() != constraint_upload.m_endpoints.size())
				throw std::invalid_argument("Coupled position preconditioner override must contain one stable-slot value");

			job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<GpuCoupledConstraintPreconditioner>(isize(preconditioner_override));
			memcpy(allocation.ptr<GpuCoupledConstraintPreconditioner>(), preconditioner_override.data(), preconditioner_override.size_bytes());
			job.m_cmd_list.CopyBufferRegion(m_prepare.m_r_preconditioners.get(), 0, allocation);
		}
		for (int iteration = 0; iteration != m_config.push_out_iterations; ++iteration)
			Run(job, r_bodies.get());
		Apply(job, r_bodies.get());

		// Capture coordinates, untouched physical state, and transaction diagnostics before the caller's single submission.
		auto result = GpuCoupledConstraintPositionResult{};
		result.m_bodies.resize(bodies.size());
		result.m_rows.resize(static_cast<size_t>(GpuConstraintRowsPerBlock) * m_velocity.m_slot_count);
		result.m_articulations.resize(aba.m_articulation_count);
		result.m_positions.resize(aba.m_position_count);
		result.m_articulation_velocities.resize(aba.m_velocity_count);
		result.m_articulation_accelerations.resize(aba.m_acceleration_count);
		result.m_articulation_scratch.resize(aba.m_link_count);
		result.m_island_states.resize(m_velocity.m_island_count);
		if (!bodies.empty())
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_position_count != 0)
			job.m_barriers.Transition(aba.m_r_positions.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_velocity_count != 0)
			job.m_barriers.Transition(aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_acceleration_count != 0)
			job.m_barriers.Transition(aba.m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_velocity.m_r_island_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();

		auto body_readback = ReadbackAlloc{};
		if (!bodies.empty())
			body_readback = job.m_readback.Alloc<GpuRigidBody>(isize(bodies));
		auto row_readback = job.m_readback.Alloc<GpuConstraintRow>(GpuConstraintRowsPerBlock * m_velocity.m_slot_count);
		auto articulation_readback = job.m_readback.Alloc<GpuArticulation>(aba.m_articulation_count);
		auto position_readback = ReadbackAlloc{};
		if (aba.m_position_count != 0)
			position_readback = job.m_readback.Alloc<float>(aba.m_position_count);
		auto velocity_readback = ReadbackAlloc{};
		if (aba.m_velocity_count != 0)
			velocity_readback = job.m_readback.Alloc<float>(aba.m_velocity_count);
		auto acceleration_readback = ReadbackAlloc{};
		if (aba.m_acceleration_count != 0)
			acceleration_readback = job.m_readback.Alloc<float>(aba.m_acceleration_count);
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(aba.m_link_count);
		auto island_readback = job.m_readback.Alloc<GpuCoupledConstraintIslandState>(m_velocity.m_island_count);
		if (!bodies.empty())
			job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
		job.m_cmd_list.CopyBufferRegion(row_readback, constraints.m_r_rows.get(), 0);
		job.m_cmd_list.CopyBufferRegion(articulation_readback, aba.m_r_articulations.get(), 0);
		if (aba.m_position_count != 0)
			job.m_cmd_list.CopyBufferRegion(position_readback, aba.m_r_positions.get(), 0);
		if (aba.m_velocity_count != 0)
			job.m_cmd_list.CopyBufferRegion(velocity_readback, aba.m_r_velocities.get(), 0);
		if (aba.m_acceleration_count != 0)
			job.m_cmd_list.CopyBufferRegion(acceleration_readback, aba.m_r_accelerations.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, aba.m_r_scratch.get(), 0);
		job.m_cmd_list.CopyBufferRegion(island_readback, m_velocity.m_r_island_states.get(), 0);
		job.Run();

		if (!bodies.empty())
			memcpy(result.m_bodies.data(), body_readback.ptr<GpuRigidBody>(), result.m_bodies.size() * sizeof(result.m_bodies[0]));
		memcpy(result.m_rows.data(), row_readback.ptr<GpuConstraintRow>(), result.m_rows.size() * sizeof(result.m_rows[0]));
		memcpy(result.m_articulations.data(), articulation_readback.ptr<GpuArticulation>(), result.m_articulations.size() * sizeof(result.m_articulations[0]));
		if (aba.m_position_count != 0)
			memcpy(result.m_positions.data(), position_readback.ptr<float>(), result.m_positions.size() * sizeof(result.m_positions[0]));
		if (aba.m_velocity_count != 0)
			memcpy(result.m_articulation_velocities.data(), velocity_readback.ptr<float>(), result.m_articulation_velocities.size() * sizeof(result.m_articulation_velocities[0]));
		if (aba.m_acceleration_count != 0)
			memcpy(result.m_articulation_accelerations.data(), acceleration_readback.ptr<float>(), result.m_articulation_accelerations.size() * sizeof(result.m_articulation_accelerations[0]));
		memcpy(result.m_articulation_scratch.data(), scratch_readback.ptr<GpuArticulationAbaScratch>(), result.m_articulation_scratch.size() * sizeof(result.m_articulation_scratch[0]));
		memcpy(result.m_island_states.data(), island_readback.ptr<GpuCoupledConstraintIslandState>(), result.m_island_states.size() * sizeof(result.m_island_states[0]));
		return result;
	}

	// Return current logical usage, retained capacities, and most recent dispatch count.
	GpuCoupledConstraintPositionStats const& GpuCoupledConstraintPosition::Stats() const
	{
		return m_stats;
	}

	// Release position-owned pseudo resources when no coupled push-out work remains.
	void GpuCoupledConstraintPosition::ReleaseBuffers()
	{
		m_r_link_pseudo = nullptr;
		m_r_generalized_pseudo = nullptr;
		m_source = nullptr;
		m_timestep = 0.0f;
		m_body_count = 0;
		m_velocity_delta_count = 0;
		m_stats = {};
	}

	// Create or geometrically grow compact pseudo link and generalized streams.
	void GpuCoupledConstraintPosition::ResizeBuffers(CmdList& cmd_list)
	{
		if (m_velocity.m_mobility_count > m_stats.m_link_pseudo_capacity)
		{
			m_stats.m_link_pseudo_capacity = std::max(m_velocity.m_mobility_count, std::max(1, 2 * m_stats.m_link_pseudo_capacity));
			m_r_link_pseudo = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(m_stats.m_link_pseudo_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledPositionLinkPseudo");
		}
		if (std::max(1, m_velocity_delta_count) > m_stats.m_generalized_pseudo_capacity)
		{
			m_stats.m_generalized_pseudo_capacity = std::max(std::max(1, m_velocity_delta_count), std::max(1, 2 * m_stats.m_generalized_pseudo_capacity));
			m_r_generalized_pseudo = m_gpu.CreateResource(ResDesc::Buf<float>(m_stats.m_generalized_pseudo_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledPositionGeneralizedPseudo");
		}
	}

	// Bind and dispatch one phase using the 64-DWORD common transaction layout.
	void GpuCoupledConstraintPosition::DispatchCommon(GpuJob& job, ComputeStep& step, int phase, int attempt_index, int item_count, ID3D12Resource* bodies)
	{
		if (item_count <= 0)
			return;

		PrepareCommonResources(job, bodies);
		auto const constants = cbCoupledConstraintPosition{
			.m_slot_count = m_velocity.m_slot_count,
			.m_body_count = m_body_count,
			.m_target_count = m_velocity.m_target_count,
			.m_island_count = m_velocity.m_island_count,
			.m_articulation_range_count = m_velocity.m_articulation_range_count,
			.m_mobility_count = m_velocity.m_mobility_count,
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_island_block_count = m_velocity.m_island_block_count,
			.m_phase = phase,
			.m_attempt_index = attempt_index,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit,
			.m_pad0 = 0,
			.m_timestep = m_timestep,
			.m_relaxation = std::min(m_config.constraint_position_relaxation, m_config.constraint_coupled_relaxation),
			.m_position_beta = m_config.constraint_position_beta,
			.m_max_position_speed = m_config.constraint_max_position_speed,
		};
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: aba.m_r_uav_sentinel.get();
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_coupled_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_block_topology->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_targets->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_adjacency->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_preconditioners->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_articulation_islands->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_islands->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_island_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_rows->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_contributions->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_target_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_island_states->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_link_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_tree_selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_tree_results->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_island_failures->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_work->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(velocity_deltas->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_pseudo_velocities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledPositionThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Bind and dispatch one complete-tree validation or commit phase.
	void GpuCoupledConstraintPosition::DispatchArticulations(GpuJob& job, ComputeStep& step, int phase, int attempt_index, int item_count)
	{
		if (item_count <= 0)
			return;

		PrepareArticulationResources(job);
		auto const constants = cbCoupledConstraintPosition{
			.m_slot_count = m_velocity.m_slot_count,
			.m_body_count = m_body_count,
			.m_target_count = m_velocity.m_target_count,
			.m_island_count = m_velocity.m_island_count,
			.m_articulation_range_count = m_velocity.m_articulation_range_count,
			.m_mobility_count = m_velocity.m_mobility_count,
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_island_block_count = m_velocity.m_island_block_count,
			.m_phase = phase,
			.m_attempt_index = attempt_index,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit,
			.m_pad0 = 0,
			.m_timestep = m_timestep,
			.m_relaxation = std::min(m_config.constraint_position_relaxation, m_config.constraint_coupled_relaxation),
			.m_position_beta = m_config.constraint_position_beta,
			.m_max_position_speed = m_config.constraint_max_position_speed,
		};
		auto& aba = m_impulse_aba.m_aba;
		auto& mobility = m_impulse_aba.m_mobility;
		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: aba.m_r_uav_sentinel.get();
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_velocity.m_r_articulation_islands->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_island_states->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_tree_selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_tree_results->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_velocity.m_r_island_failures->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_work->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(velocity_deltas->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledPositionThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
		job.m_barriers.UAV(m_velocity.m_r_island_states.get());
		job.m_barriers.UAV(m_velocity.m_r_tree_selection.get());
		job.m_barriers.UAV(m_velocity.m_r_tree_results.get());
		job.m_barriers.UAV(m_velocity.m_r_island_failures.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_work.get());
		job.m_barriers.UAV(velocity_deltas);
		job.m_barriers.UAV(m_r_link_pseudo.get());
		job.m_barriers.UAV(m_r_generalized_pseudo.get());
		job.m_barriers.UAV(aba.m_r_articulations.get());
		job.m_barriers.Commit();
	}

	// Bind and dispatch final rigid and articulation coordinate integration.
	void GpuCoupledConstraintPosition::DispatchApply(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		auto& mobility = m_impulse_aba.m_mobility;
		auto* positions = aba.m_r_positions != nullptr ? aba.m_r_positions.get() : aba.m_r_uav_sentinel.get();
		job.m_barriers.Transition(mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_pseudo_velocities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(positions, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbCoupledConstraintPosition{
			.m_slot_count = m_velocity.m_slot_count,
			.m_body_count = m_body_count,
			.m_target_count = m_velocity.m_target_count,
			.m_island_count = m_velocity.m_island_count,
			.m_articulation_range_count = m_velocity.m_articulation_range_count,
			.m_mobility_count = m_velocity.m_mobility_count,
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_island_block_count = m_velocity.m_island_block_count,
			.m_phase = GpuCoupledConstraintPhase_CommitPosition,
			.m_attempt_index = m_config.constraint_coupled_backtrack_limit,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit,
			.m_pad0 = 0,
			.m_timestep = m_timestep,
			.m_relaxation = std::min(m_config.constraint_position_relaxation, m_config.constraint_coupled_relaxation),
			.m_position_beta = m_config.constraint_position_beta,
			.m_max_position_speed = m_config.constraint_max_position_speed,
		};
		job.m_cmd_list.SetPipelineState(m_cs_apply.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_apply.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_pseudo_velocities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(positions->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledPositionThreadGroupCount(std::max(m_body_count, m_velocity.m_articulation_range_count)), 1, 1);
		++m_stats.m_dispatch_count;
		job.m_barriers.UAV(bodies);
		job.m_barriers.UAV(aba.m_r_articulations.get());
		job.m_barriers.UAV(positions);
		job.m_barriers.Commit();
	}

	// Transition all resources used by the common transaction root layout.
	void GpuCoupledConstraintPosition::PrepareCommonResources(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: aba.m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_prepare.m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_block_topology.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_targets.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_adjacency.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_articulation_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_island_blocks.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_contributions.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_target_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_island_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_link_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_island_failures.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(velocity_deltas, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_pseudo_velocities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Transition all resources used by complete-tree validation and pseudo commit.
	void GpuCoupledConstraintPosition::PrepareArticulationResources(GpuJob& job)
	{
		auto& aba = m_impulse_aba.m_aba;
		auto& mobility = m_impulse_aba.m_mobility;
		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: aba.m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_velocity.m_r_articulation_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_velocity.m_r_island_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_velocity.m_r_island_failures.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(velocity_deltas, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Insert ordering across every mutable position and reused transaction resource.
	void GpuCoupledConstraintPosition::CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: aba.m_r_uav_sentinel.get();
		job.m_barriers.UAV(bodies);
		job.m_barriers.UAV(constraints.m_r_blocks.get());
		job.m_barriers.UAV(constraints.m_r_rows.get());
		job.m_barriers.UAV(m_velocity.m_r_scratch.get());
		job.m_barriers.UAV(m_velocity.m_r_contributions.get());
		job.m_barriers.UAV(m_velocity.m_r_target_impulses.get());
		job.m_barriers.UAV(m_velocity.m_r_island_states.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_link_impulses.get());
		job.m_barriers.UAV(m_velocity.m_r_tree_selection.get());
		job.m_barriers.UAV(m_velocity.m_r_tree_results.get());
		job.m_barriers.UAV(m_velocity.m_r_island_failures.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_work.get());
		job.m_barriers.UAV(velocity_deltas);
		job.m_barriers.UAV(constraints.m_r_pseudo_velocities.get());
		job.m_barriers.UAV(m_r_link_pseudo.get());
		job.m_barriers.UAV(m_r_generalized_pseudo.get());
		job.m_barriers.Commit();
	}
}
