//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/coupled_constraint_velocity_runner.h"

// Give this translation unit private emulated resources because replay shaders use production resource names.
#define g_coupled_velocity g_coupled_velocity_replay
#define g_coupled_velocity_bodies g_coupled_velocity_replay_bodies
#define g_coupled_velocity_endpoints g_coupled_velocity_replay_endpoints
#define g_coupled_velocity_link_endpoints g_coupled_velocity_replay_link_endpoints
#define g_coupled_velocity_block_topology g_coupled_velocity_replay_block_topology
#define g_coupled_velocity_targets g_coupled_velocity_replay_targets
#define g_coupled_velocity_adjacency g_coupled_velocity_replay_adjacency
#define g_coupled_velocity_preconditioners g_coupled_velocity_replay_preconditioners
#define g_coupled_velocity_aba_scratch g_coupled_velocity_replay_aba_scratch
#define g_coupled_velocity_articulation_islands g_coupled_velocity_replay_articulation_islands
#define g_coupled_velocity_islands g_coupled_velocity_replay_islands
#define g_coupled_velocity_island_blocks g_coupled_velocity_replay_island_blocks
#define g_coupled_velocity_blocks g_coupled_velocity_replay_blocks
#define g_coupled_velocity_rows g_coupled_velocity_replay_rows
#define g_coupled_velocity_scratch g_coupled_velocity_replay_scratch
#define g_coupled_velocity_contributions g_coupled_velocity_replay_contributions
#define g_coupled_velocity_target_impulses g_coupled_velocity_replay_target_impulses
#define g_coupled_velocity_island_states g_coupled_velocity_replay_island_states
#define g_coupled_velocity_link_impulses g_coupled_velocity_replay_link_impulses
#define g_coupled_velocity_tree_selection g_coupled_velocity_replay_tree_selection
#define g_coupled_velocity_tree_results g_coupled_velocity_replay_tree_results
#define g_coupled_velocity_island_failures g_coupled_velocity_replay_island_failures
#define g_coupled_velocity_articulation_work g_coupled_velocity_replay_articulation_work
#define g_coupled_velocity_rejections g_coupled_velocity_replay_rejections
#define g_coupled_velocity_results g_coupled_velocity_replay_results
#include "src/compute/coupled_constraint_velocity.hlsl"
#include "src/compute/coupled_constraint_trial.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename Type>
		std::span<Type const> CoupledVelocitySpanOf(std::vector<Type> const& values)
		{
			return std::span<Type const>{values.data(), values.size()};
		}

		// Return the exact dispatch width for one non-empty coupled work range.
		int CoupledVelocityThreadGroupCount(int item_count)
		{
			return (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount;
		}
	}

	// Apply one bounded-backtracking simultaneous sweep from prepared coupled rows and articulation factors.
	void CoupledConstraintVelocityInteropRunner::Run(
		float relaxation,
		GpuConstraintUpload const& constraint_upload,
		GpuArticulationUpload const& articulation_upload,
		std::span<GpuConstraintBlock const> blocks,
		std::span<GpuConstraintRow const> rows,
		std::span<GpuCoupledConstraintPreconditioner const> preconditioners,
		std::span<GpuRigidBody const> bodies,
		CoupledConstraintPrimaryReference reference,
		int backtrack_limit)
	{
		auto const slot_count = isize(constraint_upload.m_endpoints);
		if (!(relaxation > 0.0f) || !std::isfinite(relaxation))
			throw std::invalid_argument("Coupled velocity replay requires finite positive relaxation");
		if (backtrack_limit < 0 || backtrack_limit > 8)
			throw std::invalid_argument("Coupled velocity replay backtracking must be between zero and eight retries");
		if (constraint_upload.m_coupled_active_count == 0)
			throw std::invalid_argument("Coupled velocity replay requires active coupled constraints");
		if (
			isize(blocks) != slot_count ||
			isize(rows) != GpuConstraintRowsPerBlock * slot_count ||
			isize(preconditioners) != slot_count ||
			isize(constraint_upload.m_coupled_block_topology) != slot_count)
			throw std::invalid_argument("Coupled velocity replay stable-slot streams do not match");
		if (constraint_upload.m_coupled_articulation_indices.size() != constraint_upload.m_coupled_articulation_islands.size())
			throw std::invalid_argument("Coupled velocity replay articulation-island mapping does not match participation");
		if (reference.m_bodies.size() != bodies.size() || reference.m_velocities.size() != articulation_upload.m_velocities.size())
			throw std::invalid_argument("Coupled velocity replay requires the complete actual owner-free primary reference");

		// Build retained factors once and establish the current cached link velocities without changing state.
		auto mobility_count = 0;
		for (auto const articulation_idx : constraint_upload.m_coupled_articulation_indices)
			mobility_count += articulation_upload.m_articulations[articulation_idx].link_count;
		auto const zero_impulses = std::vector<GpuArticulationSpatialVector>(mobility_count);
		m_impulse_aba.Run(articulation_upload, constraint_upload.m_coupled_articulation_indices, zero_impulses);

		// Bind immutable topology and initialize all detached transactional streams.
		m_bodies.assign(bodies.begin(), bodies.end());
		m_blocks.assign(blocks.begin(), blocks.end());
		m_rows.assign(rows.begin(), rows.end());
		m_scratch.resize(slot_count);
		m_contributions.resize(2 * slot_count);
		m_target_impulses.resize(constraint_upload.m_coupled_targets.size());
		m_island_states.resize(constraint_upload.m_coupled_islands.size());
		m_island_failures.resize(constraint_upload.m_coupled_islands.size());
		m_link_impulses.resize(mobility_count);
		m_tree_selection.resize(constraint_upload.m_coupled_articulation_indices.size());
		m_tree_results.resize(constraint_upload.m_coupled_articulation_indices.size());
		auto const work_count = std::max({
			slot_count,
			isize(constraint_upload.m_coupled_targets),
			isize(constraint_upload.m_coupled_islands),
			isize(constraint_upload.m_coupled_articulation_indices),
			mobility_count,
		});
		g_coupled_velocity = cbCoupledConstraintVelocity{
			.slot_count = slot_count,
			.body_count = isize(m_bodies),
			.target_count = isize(constraint_upload.m_coupled_targets),
			.adjacency_count = isize(constraint_upload.m_coupled_target_adjacency),
			.island_count = isize(constraint_upload.m_coupled_islands),
			.articulation_range_count = isize(constraint_upload.m_coupled_articulation_indices),
			.mobility_count = mobility_count,
			.work_count = work_count,
			.island_block_count = isize(constraint_upload.m_coupled_island_blocks),
			.phase = GpuCoupledConstraintPhase_Evaluate,
			.attempt_index = 0,
			.backtrack_limit = backtrack_limit,
			.relaxation = relaxation,
			.substep_index = 0,
		};
		g_coupled_velocity_bodies.assign(CoupledVelocitySpanOf(m_bodies));
		g_coupled_velocity_endpoints.assign(CoupledVelocitySpanOf(constraint_upload.m_endpoints));
		g_coupled_velocity_link_endpoints.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_endpoints));
		g_coupled_velocity_block_topology.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_block_topology));
		g_coupled_velocity_targets.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_targets));
		g_coupled_velocity_adjacency.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_target_adjacency));
		g_coupled_velocity_preconditioners.assign(preconditioners);
		g_coupled_velocity_aba_scratch.assign(m_impulse_aba.Scratch());
		g_coupled_velocity_articulation_islands.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_articulation_islands));
		g_coupled_velocity_islands.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_islands));
		g_coupled_velocity_island_blocks.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_island_blocks));
		g_coupled_velocity_blocks.assign(CoupledVelocitySpanOf(m_blocks));
		g_coupled_velocity_rows.assign(CoupledVelocitySpanOf(m_rows));
		g_coupled_velocity_scratch.assign(CoupledVelocitySpanOf(m_scratch));
		g_coupled_velocity_contributions.assign(CoupledVelocitySpanOf(m_contributions));
		g_coupled_velocity_target_impulses.assign(CoupledVelocitySpanOf(m_target_impulses));
		g_coupled_velocity_island_states.assign(CoupledVelocitySpanOf(m_island_states));
		g_coupled_velocity_island_failures.assign(CoupledVelocitySpanOf(m_island_failures));
		g_coupled_velocity_link_impulses.assign(CoupledVelocitySpanOf(m_link_impulses));
		g_coupled_velocity_tree_selection.assign(CoupledVelocitySpanOf(m_tree_selection));
		g_coupled_velocity_tree_results.assign(CoupledVelocitySpanOf(m_tree_results));
		g_coupled_velocity_articulation_work.assign(m_impulse_aba.Work());

		// Diagnostic references are explicit primary inputs, not a reconstruction through coefficient history.
		auto spans = std::vector<GpuCoupledConstraintWorkSpan>(m_island_states.size());
		auto indices = std::vector<uint32_t>{};
		auto momentum_references = std::vector<GpuCoupledConstraintMomentumReference>{};
		for (int island = 0; island != isize(spans); ++island)
		{
			spans[island].rigid_offset = s_cast<uint32_t>(indices.size());
			for (int target_idx = 0; target_idx != isize(constraint_upload.m_coupled_targets); ++target_idx)
			{
				auto const& target = constraint_upload.m_coupled_targets[target_idx];
				if (target.island_idx != island || target.target_type != GpuCoupledConstraintTargetType_Rigid)
					continue;

				indices.push_back(s_cast<uint32_t>(target_idx));
				++spans[island].rigid_count;
				momentum_references.push_back({
					.free = {reference.m_bodies[target.target_idx].momentum_ang, reference.m_bodies[target.target_idx].momentum_lin},
					.initial = {bodies[target.target_idx].momentum_ang, bodies[target.target_idx].momentum_lin}});
			}
		}
		auto const rigid_count = isize(indices);
		auto const ranges = m_impulse_aba.Ranges();
		auto const last_range = ranges.back();
		auto const velocity_count = last_range.velocity_delta_offset + articulation_upload.m_articulations[last_range.articulation_index].velocity_count;
		auto speed_references = std::vector<GpuCoupledConstraintSpeedReference>(velocity_count);
		for (int island = 0; island != isize(spans); ++island)
		{
			spans[island].tree_offset = s_cast<uint32_t>(indices.size());
			for (int range_idx = 0; range_idx != isize(ranges); ++range_idx)
			{
				if (constraint_upload.m_coupled_articulation_islands[range_idx] != island)
					continue;

				indices.push_back(s_cast<uint32_t>(range_idx));
				++spans[island].tree_count;
				auto const& range = ranges[range_idx];
				auto const& articulation = articulation_upload.m_articulations[range.articulation_index];
				for (int component = 0; component != articulation.velocity_count; ++component)
				{
					auto offset = articulation.velocity_offset + component;
					speed_references[range.velocity_delta_offset + component] = {reference.m_velocities[offset], articulation_upload.m_velocities[offset]};
				}
			}
		}
		auto initial_coefficients = std::vector<float>(rows.size());
		for (size_t index = 0; index != rows.size(); ++index)
			initial_coefficients[index] = rows[index].bounds.z;

		auto results = std::vector<uint32_t>(m_island_states.size(), GpuConstraintStage_Accepted);
		g_coupled_velocity_results.assign(CoupledVelocitySpanOf(results));
		g_trial = cbCoupledConstraintTrial{
			.slot_count = slot_count, .body_count = isize(bodies), .island_count = isize(spans), .range_count = isize(ranges),
			.rigid_count = rigid_count, .velocity_count = velocity_count, .link_count = isize(articulation_upload.m_links),
			.mobility_count = mobility_count, .island_block_count = isize(constraint_upload.m_coupled_island_blocks),
			.backtrack_limit = backtrack_limit};
		g_trial_targets.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_targets));
		g_trial_topology.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_block_topology));
		g_trial_link_endpoints.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_endpoints));
		g_trial_spans.assign(CoupledVelocitySpanOf(spans));
		g_trial_indices.assign(CoupledVelocitySpanOf(indices));
		g_trial_islands.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_islands));
		g_trial_island_blocks.assign(CoupledVelocitySpanOf(constraint_upload.m_coupled_island_blocks));
		g_trial_ranges.assign(ranges);
		g_trial_articulations.assign(CoupledVelocitySpanOf(articulation_upload.m_articulations));
		g_trial_links.assign(CoupledVelocitySpanOf(articulation_upload.m_links));
		g_trial_live_bodies.assign(bodies);
		g_trial_dof_scratch.assign(m_impulse_aba.DofScratch());
		g_trial_blocks.assign(blocks);
		g_trial_momentum_references.assign(CoupledVelocitySpanOf(momentum_references));
		g_trial_speed_references.assign(CoupledVelocitySpanOf(speed_references));
		g_trial_initial_coefficients.assign(CoupledVelocitySpanOf(initial_coefficients));
		g_trial_work_motion.assign(CoupledVelocitySpanOf(std::vector<GpuCoupledConstraintWorkMotion>(mobility_count)));

		hlsl::GpuEmulator begin(CSBeginCoupledVelocity, CSBeginCoupledVelocity_NumThreads);
		hlsl::GpuEmulator candidates(CSBuildCoupledVelocityCandidates, CSBuildCoupledVelocityCandidates_NumThreads);
		hlsl::GpuEmulator gather(CSGatherCoupledVelocityTargets, CSGatherCoupledVelocityTargets_NumThreads);
		hlsl::GpuEmulator select_evaluation(CSSelectCoupledVelocityTrees, CSSelectCoupledVelocityTrees_NumThreads);
		hlsl::GpuEmulator validate(CSValidateCoupledVelocityTrees, CSValidateCoupledVelocityTrees_NumThreads);
		hlsl::GpuEmulator merit(CSEvaluateCoupledVelocityMerit, CSEvaluateCoupledVelocityMerit_NumThreads);
		hlsl::GpuEmulator commit(CSCommitCoupledVelocity, CSCommitCoupledVelocity_NumThreads);
		hlsl::GpuEmulator begin_trial(CSBeginCoupledVelocityTrial, CSBeginCoupledVelocityTrial_NumThreads);

		// Evaluate selected complete trees into detached work and publish their deterministic success flags.
		auto EvaluateSelectedTrees = [&]
		{
			m_link_impulses.assign(g_coupled_velocity_link_impulses.begin(), g_coupled_velocity_link_impulses.end());
			m_tree_selection.assign(g_coupled_velocity_tree_selection.begin(), g_coupled_velocity_tree_selection.end());
			auto const results = m_impulse_aba.Evaluate(m_link_impulses, m_tree_selection);
			m_tree_results.assign(results.begin(), results.end());
			g_coupled_velocity_tree_results.assign(CoupledVelocitySpanOf(m_tree_results));
			g_coupled_velocity_articulation_work.assign(m_impulse_aba.Work());
		};

		g_coupled_velocity.phase = GpuCoupledConstraintPhase_Evaluate;
		begin.Dispatch({CoupledVelocityThreadGroupCount(work_count), 1, 1});
		for (int attempt_index = 0; attempt_index != backtrack_limit + 1; ++attempt_index)
		{
			// Restore only pending islands to the same actual numerical initial state; accepted output remains untouched.
			for (auto const& target : constraint_upload.m_coupled_targets)
			{
				if (target.target_type == GpuCoupledConstraintTargetType_Rigid &&
					g_coupled_velocity_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Pending)
					g_coupled_velocity_bodies[target.target_idx] = bodies[target.target_idx];
			}
			for (int slot = 0; slot != slot_count; ++slot)
			{
				auto island = constraint_upload.m_coupled_block_topology[slot].island_idx;
				if (island < 0 || g_coupled_velocity_island_states[island].status != GpuCoupledConstraintIslandStatus_Pending)
					continue;

				for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
					g_coupled_velocity_rows[GpuConstraintRowsPerBlock * slot + axis] = rows[GpuConstraintRowsPerBlock * slot + axis];
			}
			select_evaluation.Dispatch({CoupledVelocityThreadGroupCount(isize(ranges)), 1, 1});
			m_tree_selection.assign(g_coupled_velocity_tree_selection.begin(), g_coupled_velocity_tree_selection.end());
			m_impulse_aba.RestoreVelocities(articulation_upload.m_velocities, m_tree_selection);
			g_coupled_velocity_aba_scratch.assign(m_impulse_aba.Scratch());
			g_trial_states.assign(std::span<GpuCoupledConstraintIslandState const>(g_coupled_velocity_island_states));
			g_trial_results.assign(std::span<uint32_t const>(g_coupled_velocity_results));
			begin_trial.Dispatch({CoupledVelocityThreadGroupCount(isize(spans)), 1, 1});
			g_coupled_velocity_island_states.assign(std::span<GpuCoupledConstraintIslandState const>(g_trial_states));
			g_coupled_velocity.attempt_index = attempt_index;
			candidates.Dispatch({CoupledVelocityThreadGroupCount(slot_count), 1, 1});
			gather.Dispatch({CoupledVelocityThreadGroupCount(isize(constraint_upload.m_coupled_targets)), 1, 1});
			select_evaluation.Dispatch({CoupledVelocityThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});
			EvaluateSelectedTrees();
			validate.Dispatch({CoupledVelocityThreadGroupCount(isize(constraint_upload.m_coupled_articulation_indices)), 1, 1});
			select_evaluation.Dispatch({CoupledVelocityThreadGroupCount(isize(ranges)), 1, 1});
			commit.Dispatch({CoupledVelocityThreadGroupCount(work_count), 1, 1});
			m_tree_selection.assign(g_coupled_velocity_tree_selection.begin(), g_coupled_velocity_tree_selection.end());
			m_impulse_aba.Commit(m_tree_selection);
			g_trial.attempt_index = attempt_index;
			g_trial_bodies.assign(std::span<GpuRigidBody const>(g_coupled_velocity_bodies));
			g_trial_velocities.assign(m_impulse_aba.Velocities());
			g_trial_scratch.assign(m_impulse_aba.Scratch());
			g_trial_rows.assign(std::span<GpuConstraintRow const>(g_coupled_velocity_rows));
			g_trial_states.assign(std::span<GpuCoupledConstraintIslandState const>(g_coupled_velocity_island_states));
			g_trial_failures.assign(std::span<uint32_t const>(g_coupled_velocity_island_failures));
			g_trial_results.assign(std::span<uint32_t const>(g_coupled_velocity_results));
			merit.Dispatch({CoupledVelocityThreadGroupCount(isize(constraint_upload.m_coupled_islands)), 1, 1});
			g_coupled_velocity_island_states.assign(std::span<GpuCoupledConstraintIslandState const>(g_trial_states));
		}

		// A diagnostic rejection returns its original actual state, not the last rejected private candidate.
		for (auto const& target : constraint_upload.m_coupled_targets)
		{
			if (target.target_type == GpuCoupledConstraintTargetType_Rigid &&
				g_coupled_velocity_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Rejected)
				g_coupled_velocity_bodies[target.target_idx] = bodies[target.target_idx];
		}
		for (int slot = 0; slot != slot_count; ++slot)
		{
			auto island = constraint_upload.m_coupled_block_topology[slot].island_idx;
			if (island < 0 || g_coupled_velocity_island_states[island].status != GpuCoupledConstraintIslandStatus_Rejected)
				continue;

			for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
				g_coupled_velocity_rows[GpuConstraintRowsPerBlock * slot + axis] = rows[GpuConstraintRowsPerBlock * slot + axis];
		}
		for (int range = 0; range != isize(ranges); ++range)
			m_tree_selection[range] = g_coupled_velocity_island_states[constraint_upload.m_coupled_articulation_islands[range]].status == GpuCoupledConstraintIslandStatus_Rejected ? 1u : 0u;

		m_impulse_aba.RestoreVelocities(articulation_upload.m_velocities, m_tree_selection);
		m_bodies.assign(g_coupled_velocity_bodies.begin(), g_coupled_velocity_bodies.end());
		m_blocks.assign(g_coupled_velocity_blocks.begin(), g_coupled_velocity_blocks.end());
		m_rows.assign(g_coupled_velocity_rows.begin(), g_coupled_velocity_rows.end());
		m_scratch.assign(g_coupled_velocity_scratch.begin(), g_coupled_velocity_scratch.end());
		m_contributions.assign(g_coupled_velocity_contributions.begin(), g_coupled_velocity_contributions.end());
		m_target_impulses.assign(g_coupled_velocity_target_impulses.begin(), g_coupled_velocity_target_impulses.end());
		m_island_states.assign(g_coupled_velocity_island_states.begin(), g_coupled_velocity_island_states.end());
	}

#undef g_coupled_velocity
#undef g_coupled_velocity_bodies
#undef g_coupled_velocity_endpoints
#undef g_coupled_velocity_link_endpoints
#undef g_coupled_velocity_block_topology
#undef g_coupled_velocity_targets
#undef g_coupled_velocity_adjacency
#undef g_coupled_velocity_preconditioners
#undef g_coupled_velocity_aba_scratch
#undef g_coupled_velocity_articulation_islands
#undef g_coupled_velocity_islands
#undef g_coupled_velocity_island_blocks
#undef g_coupled_velocity_blocks
#undef g_coupled_velocity_rows
#undef g_coupled_velocity_scratch
#undef g_coupled_velocity_contributions
#undef g_coupled_velocity_target_impulses
#undef g_coupled_velocity_island_states
#undef g_coupled_velocity_link_impulses
#undef g_coupled_velocity_tree_selection
#undef g_coupled_velocity_tree_results
#undef g_coupled_velocity_island_failures
#undef g_coupled_velocity_articulation_work

	// Return committed rigid-body state after accepted island updates.
	std::span<GpuRigidBody const> CoupledConstraintVelocityInteropRunner::Bodies() const
	{
		return m_bodies;
	}

	// Return persistent scalar rows containing accepted accumulated impulses.
	std::span<GpuConstraintRow const> CoupledConstraintVelocityInteropRunner::Rows() const
	{
		return m_rows;
	}

	// Return committed packed generalized articulation velocities.
	std::span<float const> CoupledConstraintVelocityInteropRunner::ArticulationVelocities() const
	{
		return m_impulse_aba.Velocities();
	}

	// Return authoritative generalized articulation accelerations preserved by detached response evaluation.
	std::span<float const> CoupledConstraintVelocityInteropRunner::ArticulationAccelerations() const
	{
		return m_impulse_aba.Accelerations();
	}

	// Return committed cached articulation link velocities.
	std::span<GpuArticulationAbaScratch const> CoupledConstraintVelocityInteropRunner::ArticulationScratch() const
	{
		return m_impulse_aba.Scratch();
	}

	// Return final per-island transaction states and failure diagnostics.
	std::span<GpuCoupledConstraintIslandState const> CoupledConstraintVelocityInteropRunner::IslandStates() const
	{
		return m_island_states;
	}
}
