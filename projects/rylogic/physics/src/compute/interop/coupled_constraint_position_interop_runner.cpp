//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/coupled_constraint_position_runner.h"

#define g_coupled_position interop_coupled_position
#define g_coupled_position_bodies interop_coupled_position_bodies
#define g_coupled_position_link_endpoints interop_coupled_position_link_endpoints
#define g_coupled_position_block_topology interop_coupled_position_block_topology
#define g_coupled_position_targets interop_coupled_position_targets
#define g_coupled_position_adjacency interop_coupled_position_adjacency
#define g_coupled_position_preconditioners interop_coupled_position_preconditioners
#define g_coupled_position_articulation_islands interop_coupled_position_articulation_islands
#define g_coupled_position_islands interop_coupled_position_islands
#define g_coupled_position_island_blocks interop_coupled_position_island_blocks
#define g_coupled_position_ranges interop_coupled_position_ranges
#define g_coupled_position_links interop_coupled_position_links
#define g_coupled_position_blocks interop_coupled_position_blocks
#define g_coupled_position_rows interop_coupled_position_rows
#define g_coupled_position_scratch interop_coupled_position_scratch
#define g_coupled_position_contributions interop_coupled_position_contributions
#define g_coupled_position_target_impulses interop_coupled_position_target_impulses
#define g_coupled_position_island_states interop_coupled_position_island_states
#define g_coupled_position_link_impulses interop_coupled_position_link_impulses
#define g_coupled_position_tree_selection interop_coupled_position_tree_selection
#define g_coupled_position_tree_results interop_coupled_position_tree_results
#define g_coupled_position_island_failures interop_coupled_position_island_failures
#define g_coupled_position_articulation_work interop_coupled_position_articulation_work
#define g_coupled_position_velocity_deltas interop_coupled_position_velocity_deltas
#define g_coupled_position_rigid_pseudo interop_coupled_position_rigid_pseudo
#define g_coupled_position_link_pseudo interop_coupled_position_link_pseudo
#define g_coupled_position_generalized_pseudo interop_coupled_position_generalized_pseudo
#define g_coupled_position_articulations interop_coupled_position_articulations
#define g_coupled_position_positions interop_coupled_position_positions
#define g_coupled_position_frame_failures interop_coupled_position_frame_failures
#include "src/compute/coupled_constraint_position.hlsl"

namespace pr::physics
{
	namespace
	{
		// Convert mutable vectors to the emulator's mutable resource-view convention.
		template <typename Type>
		std::span<Type const> CoupledPositionSpanOf(std::vector<Type> const& values)
		{
			return std::span<Type const>{values.data(), values.size()};
		}

		// Convert one logical count to at least one emulated thread group.
		int CoupledPositionThreadGroupCount(int count)
		{
			return std::max(1, (count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		}
	}

	// Solve hard coupled drift in pseudo state and integrate accepted coordinates exactly once.
	void CoupledConstraintPositionInteropRunner::Run(
		float timestep,
		float relaxation,
		float position_beta,
		float max_position_speed,
		int position_iterations,
		GpuConstraintUpload const& constraint_upload,
		GpuArticulationUpload const& articulation_upload,
		std::span<GpuConstraintBlock const> blocks,
		std::span<GpuConstraintRow const> rows,
		std::span<GpuCoupledConstraintPreconditioner const> preconditioners,
		std::span<GpuRigidBody const> bodies,
		int backtrack_limit)
	{
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled position replay timestep must be finite and positive");
		if (!(relaxation > 0.0f) || !std::isfinite(relaxation))
			throw std::invalid_argument("Coupled position replay relaxation must be finite and positive");
		if (!(position_beta >= 0.0f) || !std::isfinite(position_beta))
			throw std::invalid_argument("Coupled position replay beta must be finite and non-negative");
		if (!(max_position_speed >= 0.0f) || !std::isfinite(max_position_speed))
			throw std::invalid_argument("Coupled position replay speed limit must be finite and non-negative");
		if (position_iterations < 0 || backtrack_limit < 0)
			throw std::invalid_argument("Coupled position replay iteration counts must be non-negative");

		auto const slot_count = isize(constraint_upload.m_endpoints);
		if (
			isize(blocks) != slot_count ||
			isize(rows) != GpuConstraintRowsPerBlock * slot_count ||
			isize(preconditioners) != slot_count ||
			isize(constraint_upload.m_coupled_block_topology) != slot_count)
			throw std::invalid_argument("Coupled position replay stable-slot streams do not match");
		if (constraint_upload.m_coupled_articulation_indices.size() != constraint_upload.m_coupled_articulation_islands.size())
			throw std::invalid_argument("Coupled position replay articulation-island mapping does not match participation");

		// Build retained fixed-configuration factors without committing any detached response to physical state.
		auto mobility_count = 0;
		for (auto const articulation_idx : constraint_upload.m_coupled_articulation_indices)
			mobility_count += articulation_upload.m_articulations[articulation_idx].link_count;
		auto const zero_impulses = std::vector<GpuArticulationSpatialVector>(mobility_count);
		m_impulse_aba.Run(articulation_upload, constraint_upload.m_coupled_articulation_indices, zero_impulses);
		auto const velocity_delta_count = isize(m_impulse_aba.VelocityDeltas());

		// Bind immutable topology and initialize only the detached streams owned by position correction.
		m_bodies.assign(bodies.begin(), bodies.end());
		m_blocks.assign(blocks.begin(), blocks.end());
		m_rows.assign(rows.begin(), rows.end());
		m_rigid_pseudo.resize(m_bodies.size());
		m_link_pseudo.resize(mobility_count);
		m_generalized_pseudo.resize(velocity_delta_count);
		m_articulations = articulation_upload.m_articulations;
		m_positions = articulation_upload.m_positions;
		m_scratch.resize(slot_count);
		m_contributions.resize(2 * slot_count);
		m_target_impulses.resize(constraint_upload.m_coupled_targets.size());
		m_island_states.resize(constraint_upload.m_coupled_islands.size());
		m_island_failures.resize(constraint_upload.m_coupled_islands.size());
		m_frame_failures.assign(constraint_upload.m_coupled_islands.size(), GpuCoupledConstraintFailureState{.substep_index = -1});
		m_link_impulses.resize(mobility_count);
		m_tree_selection.resize(constraint_upload.m_coupled_articulation_indices.size());
		m_tree_results.resize(constraint_upload.m_coupled_articulation_indices.size());
		auto const work_count = std::max({
			slot_count,
			isize(m_bodies),
			mobility_count,
			velocity_delta_count,
			isize(constraint_upload.m_coupled_targets),
			isize(constraint_upload.m_coupled_islands),
			isize(constraint_upload.m_coupled_articulation_indices),
		});
		g_coupled_position = cbCoupledConstraintPosition{
			.slot_count = slot_count,
			.body_count = isize(m_bodies),
			.target_count = isize(constraint_upload.m_coupled_targets),
			.island_count = isize(constraint_upload.m_coupled_islands),
			.articulation_range_count = isize(constraint_upload.m_coupled_articulation_indices),
			.mobility_count = mobility_count,
			.velocity_delta_count = velocity_delta_count,
			.island_block_count = isize(constraint_upload.m_coupled_island_blocks),
			.phase = GpuCoupledConstraintPhase_Evaluate,
			.attempt_index = 0,
			.backtrack_limit = backtrack_limit,
			.substep_index = 0,
			.timestep = timestep,
			.relaxation = relaxation,
			.position_beta = position_beta,
			.max_position_speed = max_position_speed,
		};
		g_coupled_position_bodies.assign(CoupledPositionSpanOf(m_bodies));
		g_coupled_position_link_endpoints.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_endpoints));
		g_coupled_position_block_topology.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_block_topology));
		g_coupled_position_targets.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_targets));
		g_coupled_position_adjacency.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_target_adjacency));
		g_coupled_position_preconditioners.assign(preconditioners);
		g_coupled_position_articulation_islands.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_articulation_islands));
		g_coupled_position_islands.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_islands));
		g_coupled_position_island_blocks.assign(CoupledPositionSpanOf(constraint_upload.m_coupled_island_blocks));
		g_coupled_position_ranges.assign(m_impulse_aba.Ranges());
		g_coupled_position_links.assign(CoupledPositionSpanOf(articulation_upload.m_links));
		g_coupled_position_blocks.assign(CoupledPositionSpanOf(m_blocks));
		g_coupled_position_rows.assign(CoupledPositionSpanOf(m_rows));
		g_coupled_position_scratch.assign(CoupledPositionSpanOf(m_scratch));
		g_coupled_position_contributions.assign(CoupledPositionSpanOf(m_contributions));
		g_coupled_position_target_impulses.assign(CoupledPositionSpanOf(m_target_impulses));
		g_coupled_position_island_states.assign(CoupledPositionSpanOf(m_island_states));
		g_coupled_position_link_impulses.assign(CoupledPositionSpanOf(m_link_impulses));
		g_coupled_position_tree_selection.assign(CoupledPositionSpanOf(m_tree_selection));
		g_coupled_position_tree_results.assign(CoupledPositionSpanOf(m_tree_results));
		g_coupled_position_island_failures.assign(CoupledPositionSpanOf(m_island_failures));
		g_coupled_position_articulation_work.assign(m_impulse_aba.Work());
		g_coupled_position_velocity_deltas.assign(m_impulse_aba.VelocityDeltas());
		g_coupled_position_rigid_pseudo.assign(CoupledPositionSpanOf(m_rigid_pseudo));
		g_coupled_position_link_pseudo.assign(CoupledPositionSpanOf(m_link_pseudo));
		g_coupled_position_generalized_pseudo.assign(CoupledPositionSpanOf(m_generalized_pseudo));
		g_coupled_position_articulations.assign(CoupledPositionSpanOf(m_articulations));
		g_coupled_position_positions.assign(CoupledPositionSpanOf(m_positions));
		g_coupled_position_frame_failures.assign(CoupledPositionSpanOf(m_frame_failures));

		hlsl::GpuEmulator clear(CSClearCoupledPositionState, CSClearCoupledPositionState_NumThreads);
		hlsl::GpuEmulator begin(CSBeginCoupledPosition, CSBeginCoupledPosition_NumThreads);
		hlsl::GpuEmulator candidates(CSBuildCoupledPositionCandidates, CSBuildCoupledPositionCandidates_NumThreads);
		hlsl::GpuEmulator gather(CSGatherCoupledPositionTargets, CSGatherCoupledPositionTargets_NumThreads);
		hlsl::GpuEmulator select(CSSelectCoupledPositionTrees, CSSelectCoupledPositionTrees_NumThreads);
		hlsl::GpuEmulator validate(CSValidateCoupledPositionTrees, CSValidateCoupledPositionTrees_NumThreads);
		hlsl::GpuEmulator merit(CSEvaluateCoupledPositionMerit, CSEvaluateCoupledPositionMerit_NumThreads);
		hlsl::GpuEmulator commit_state(CSCommitCoupledPositionState, CSCommitCoupledPositionState_NumThreads);
		hlsl::GpuEmulator commit_articulations(CSCommitCoupledPositionArticulations, CSCommitCoupledPositionArticulations_NumThreads);
		hlsl::GpuEmulator finalize(CSFinalizeCoupledPositionIslands, CSFinalizeCoupledPositionIslands_NumThreads);
		hlsl::GpuEmulator apply(CSApplyCoupledPosition, CSApplyCoupledPosition_NumThreads);
		clear.Dispatch({CoupledPositionThreadGroupCount(work_count), 1, 1});

		// Each iteration is an independently backtracked simultaneous island transaction over accumulated pseudo state.
		for (int iteration = 0; iteration != position_iterations; ++iteration)
		{
			g_coupled_position.phase = GpuCoupledConstraintPhase_Evaluate;
			begin.Dispatch({CoupledPositionThreadGroupCount(work_count), 1, 1});
			for (int attempt_index = 0; attempt_index != backtrack_limit + 1; ++attempt_index)
			{
				g_coupled_position.attempt_index = attempt_index;
				candidates.Dispatch({CoupledPositionThreadGroupCount(slot_count), 1, 1});
				gather.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_targets)), 1, 1});
				select.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});

				m_link_impulses.assign(g_coupled_position_link_impulses.begin(), g_coupled_position_link_impulses.end());
				m_tree_selection.assign(g_coupled_position_tree_selection.begin(), g_coupled_position_tree_selection.end());
				auto const results = m_impulse_aba.Evaluate(m_link_impulses, m_tree_selection);
				m_tree_results.assign(results.begin(), results.end());
				g_coupled_position_tree_results.assign(CoupledPositionSpanOf(m_tree_results));
				g_coupled_position_articulation_work.assign(m_impulse_aba.Work());
				g_coupled_position_velocity_deltas.assign(m_impulse_aba.VelocityDeltas());
				validate.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});
				merit.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_islands)), 1, 1});
			}

			g_coupled_position.phase = GpuCoupledConstraintPhase_CommitPosition;
			select.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});
			commit_state.Dispatch({CoupledPositionThreadGroupCount(std::max(slot_count, isize(constraint_upload.m_coupled_targets))), 1, 1});
			commit_articulations.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});
			finalize.Dispatch({CoupledPositionThreadGroupCount(isize(constraint_upload.m_coupled_islands)), 1, 1});
		}

		// Coordinate commit consumes converged pseudo state once and never touches physical momentum or ABA cache.
		apply.Dispatch({CoupledPositionThreadGroupCount(std::max(isize(m_bodies), isize(constraint_upload.m_coupled_articulation_indices))), 1, 1});
		m_bodies.assign(g_coupled_position_bodies.begin(), g_coupled_position_bodies.end());
		m_rows.assign(g_coupled_position_rows.begin(), g_coupled_position_rows.end());
		m_rigid_pseudo.assign(g_coupled_position_rigid_pseudo.begin(), g_coupled_position_rigid_pseudo.end());
		m_link_pseudo.assign(g_coupled_position_link_pseudo.begin(), g_coupled_position_link_pseudo.end());
		m_generalized_pseudo.assign(g_coupled_position_generalized_pseudo.begin(), g_coupled_position_generalized_pseudo.end());
		m_articulations.assign(g_coupled_position_articulations.begin(), g_coupled_position_articulations.end());
		m_positions.assign(g_coupled_position_positions.begin(), g_coupled_position_positions.end());
		m_island_states.assign(g_coupled_position_island_states.begin(), g_coupled_position_island_states.end());
	}

#undef g_coupled_position
#undef g_coupled_position_bodies
#undef g_coupled_position_link_endpoints
#undef g_coupled_position_block_topology
#undef g_coupled_position_targets
#undef g_coupled_position_adjacency
#undef g_coupled_position_preconditioners
#undef g_coupled_position_articulation_islands
#undef g_coupled_position_islands
#undef g_coupled_position_island_blocks
#undef g_coupled_position_ranges
#undef g_coupled_position_links
#undef g_coupled_position_blocks
#undef g_coupled_position_rows
#undef g_coupled_position_scratch
#undef g_coupled_position_contributions
#undef g_coupled_position_target_impulses
#undef g_coupled_position_island_states
#undef g_coupled_position_link_impulses
#undef g_coupled_position_tree_selection
#undef g_coupled_position_tree_results
#undef g_coupled_position_island_failures
#undef g_coupled_position_articulation_work
#undef g_coupled_position_velocity_deltas
#undef g_coupled_position_rigid_pseudo
#undef g_coupled_position_link_pseudo
#undef g_coupled_position_generalized_pseudo
#undef g_coupled_position_articulations
#undef g_coupled_position_positions

	// Return rigid state after coordinate-only pseudo correction.
	std::span<GpuRigidBody const> CoupledConstraintPositionInteropRunner::Bodies() const
	{
		return m_bodies;
	}

	// Return persistent scalar rows containing accumulated pseudo impulses in bounds.w.
	std::span<GpuConstraintRow const> CoupledConstraintPositionInteropRunner::Rows() const
	{
		return m_rows;
	}

	// Return articulation headers after coordinate-only root correction.
	std::span<GpuArticulation const> CoupledConstraintPositionInteropRunner::Articulations() const
	{
		return m_articulations;
	}

	// Return packed child joint positions after coordinate-only correction.
	std::span<float const> CoupledConstraintPositionInteropRunner::Positions() const
	{
		return m_positions;
	}

	// Return accumulated detached rigid pseudo twists.
	std::span<GpuConstraintPseudoVelocity const> CoupledConstraintPositionInteropRunner::RigidPseudoVelocities() const
	{
		return m_rigid_pseudo;
	}

	// Return accumulated detached compact generalized pseudo velocities.
	std::span<float const> CoupledConstraintPositionInteropRunner::GeneralizedPseudoVelocities() const
	{
		return m_generalized_pseudo;
	}

	// Return final per-island transaction states and failure diagnostics.
	std::span<GpuCoupledConstraintIslandState const> CoupledConstraintPositionInteropRunner::IslandStates() const
	{
		return m_island_states;
	}

	// Return physical generalized velocities preserved by detached position response.
	std::span<float const> CoupledConstraintPositionInteropRunner::PhysicalVelocities() const
	{
		return m_impulse_aba.Velocities();
	}

	// Return physical generalized accelerations preserved by detached position response.
	std::span<float const> CoupledConstraintPositionInteropRunner::PhysicalAccelerations() const
	{
		return m_impulse_aba.Accelerations();
	}

	// Return physical cached link state preserved by detached position response.
	std::span<GpuArticulationAbaScratch const> CoupledConstraintPositionInteropRunner::PhysicalScratch() const
	{
		return m_impulse_aba.Scratch();
	}
}
