//*********************************************
// Physics Engine - Coupled Constraint Transactions
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "constraint_transaction.hlsli"
#include "coupled_constraint_work.hlsli"

// Publication uses the actual private solve response, not coefficients reinterpreted through a later mobility.
cbuffer cbCoupledConstraintTransaction : register(b0)
{
	int slot_count;
	int body_count;
	int target_count;
	int island_count;
	int range_count;
	int articulation_count;
	int velocity_count;
	int link_count;
	GpuConstraintEpoch epoch;
};
StructuredBuffer<GpuCoupledConstraintTarget> resource(g_targets, t0);
StructuredBuffer<GpuCoupledConstraintBlockTopology> resource(g_topology, t1);
StructuredBuffer<GpuConstraintEndpoint> resource(g_endpoints, t2);
StructuredBuffer<GpuCoupledConstraintEndpoint> resource(g_coupled_endpoints, t3);
StructuredBuffer<GpuConstraintBlock> resource(g_blocks, t4);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_ranges, t5);
StructuredBuffer<GpuArticulation> resource(g_articulations, t6);
StructuredBuffer<int> resource(g_articulation_islands, t7);
RWStructuredBuffer<GpuRigidBody> resource(g_work_bodies, u0);
RWStructuredBuffer<float> resource(g_work_velocities, u1);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_work_scratch, u2);
RWStructuredBuffer<GpuConstraintRow> resource(g_rows, u3);
RWStructuredBuffer<GpuConstraintAcceptedRows> resource(g_accepted_rows, u4);
RWStructuredBuffer<GpuConstraintAcceptance> resource(g_acceptance, u5);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_rigid_response, u6);
RWStructuredBuffer<float> resource(g_velocity_response, u7);
RWStructuredBuffer<uint> resource(g_results, u8);
RWStructuredBuffer<GpuCoupledConstraintIslandState> resource(g_island_states, u9);
RWStructuredBuffer<uint> resource(g_tree_selection, u10);
RWStructuredBuffer<uint> resource(g_tree_results, u11);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_target_impulses, u12);
RWStructuredBuffer<float> resource(g_velocity_deltas, u13);
RWStructuredBuffer<GpuRigidBody> resource(g_live_bodies, u14);
RWStructuredBuffer<float> resource(g_live_velocities, u15);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_live_scratch, u16);
RWStructuredBuffer<GpuConstraintSolverState> resource(g_solver_state, u17);

// Admission rejection (1) is subordinate to invalid transaction context (2); existing numerical rejection is reported separately (3).
void RejectStage(int island, uint reason)
{
	// Context errors dominate both non-terminal outcomes, regardless of which row or tree reports first.
	if (reason == GpuConstraintStage_NumericalFailure)
	{
		// No later admission or existing-solve rejection may hide this context error.
		uint previous;
		InterlockedExchange(g_results[island], reason, previous);
		return;
	}
	[allow_uav_condition]
	for (;;)
	{
		// Compare-and-exchange preserves a concurrently reported context error.
		uint current = g_results[island];
		if (current == GpuConstraintStage_NumericalFailure || current >= reason)
			return;
		uint observed;
		InterlockedCompareExchange(g_results[island], current, reason, observed);
		if (observed == current)
			return;
	}
}

// Observe the existing shared rigid-body reset input without acknowledging it for any other block.
uint BodyResetMask(int slot, RWStructuredBuffer<GpuRigidBody> bodies)
{
	// Articulation proxies do not supply the coupled compiler's ordinary endpoint reset event.
	GpuConstraintEndpoint endpoint = g_endpoints[slot];
	GpuCoupledConstraintEndpoint coupled = g_coupled_endpoints[slot];
	uint mask = 0u;
	if (coupled.link_idx_a < 0 && endpoint.body_idx_a >= 0)
		mask |= AllSet(bodies[endpoint.body_idx_a].state_flags, ERigidBodyStateFlags_ResetWarmStart) ? 1u : 0u;
	if (coupled.link_idx_b < 0 && endpoint.body_idx_b >= 0)
		mask |= AllSet(bodies[endpoint.body_idx_b].state_flags, ERigidBodyStateFlags_ResetWarmStart) ? 2u : 0u;
	return mask;
}

// Check the physical twist represented by finite momentum before admitting a private seed or final result.
bool PhysicalTwistFinite(GpuRigidBody body)
{
	// Momentum alone is insufficient when current inverse inertia magnifies it beyond the represented domain.
	BodyTwist twist = BodyTwistWS(body);
	return all(isfinite(twist.ang)) && all(isfinite(twist.lin));
}

// Recover free primary state and provisional row guesses. The host copied live state and cleared results before this dispatch.
numthreads(CSRecoverCoupledTransaction, ConstraintThreadCount, 1, 1)
void CSRecoverCoupledTransaction(int3 DTID(dtid))
{
	// Each participating ordinary rigid index has one target in this owner, independently of ordinary-block ownership.
	int index = dtid.x;
	if (index < target_count)
	{
		// Only ordinary rigid targets carry world-COM momentum response.
		GpuCoupledConstraintTarget target = g_targets[index];
		if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
		{
			// Reconstruct with the recorded old response, never current inverse mass or inertia.
			GpuRigidBody body = g_live_bodies[target.target_idx];
			GpuArticulationSpatialVector previous = g_rigid_response[target.target_idx];
			bool valid = true;
			for (int component = 0; component != 3; ++component)
			{
				// Validate all components even if another one already rejected this private candidate.
				float angular;
				float translation;
				bool angular_valid = ConstraintStageFreeComponent(body.momentum_ang[component], previous.ang[component], g_acceptance[target.island_idx], epoch, angular);
				bool translation_valid = ConstraintStageFreeComponent(body.momentum_lin[component], previous.lin[component], g_acceptance[target.island_idx], epoch, translation);
				valid = valid && angular_valid && translation_valid;
				body.momentum_ang[component] = angular;
				body.momentum_lin[component] = translation;
			}
			if (!valid || !PhysicalTwistFinite(body))
				RejectStage(target.island_idx, GpuConstraintStage_AdmissionRejected);
			g_work_bodies[target.target_idx] = body;
		}
	}
	if (index < range_count)
	{
		// Generalized velocities retain their native component representation through configuration-only updates.
		GpuArticulationMobilityRange range = g_ranges[index];
		int island = g_articulation_islands[index];
		GpuArticulation articulation = g_articulations[range.articulation_index];
		for (int link = 0; link != articulation.link_count; ++link)
		{
			// Every participating factor must belong to a successful configuration solve before a response overflow is rejectable.
			if (g_work_scratch[articulation.link_offset + link].solve_valid == 0)
				RejectStage(island, GpuConstraintStage_NumericalFailure);
		}
		for (int component = 0; component != articulation.velocity_count; ++component)
		{
			// The velocity record is forest-indexed; the trial response buffer remains compact-range-indexed.
			int offset = articulation.velocity_offset + component;
			float value;
			if (!ConstraintStageFreeComponent(g_live_velocities[offset], g_velocity_response[offset], g_acceptance[island], epoch, value))
				RejectStage(island, GpuConstraintStage_AdmissionRejected);
			g_work_velocities[offset] = value;
		}
	}
	if (index < slot_count)
	{
		// Prepared rows supply geometry only; accepted owner records are the sole authority for guesses and accepted dt.
		int island = g_topology[index].island_idx;
		if (island < 0)
			return;
		float scale;
		if (!ConstraintStageGuessScale(g_acceptance[island], epoch, scale))
		{
			// A changed dt within an accepted epoch is invalid context, not an ordinary replacement rejection.
			RejectStage(island, GpuConstraintStage_NumericalFailure);
			return;
		}
		GpuConstraintBlock block = g_blocks[index];
		if (AllSet(block.flags, ConstraintBlockFlags_NumericalFailure))
		{
			// An existing compilation failure is not overflow of a new provisional physical guess.
			RejectStage(island, GpuConstraintStage_SolveRejected);
			InterlockedOr(g_island_states[island].failure_flags, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
		if (!isfinite(scale))
		{
			// A finite warm factor times a valid timestep ratio can overflow only the provisional seed.
			RejectStage(island, GpuConstraintStage_AdmissionRejected);
			return;
		}
		GpuConstraintAcceptedRows accepted = g_accepted_rows[index];
		uint resets = BodyResetMask(index, g_live_bodies);
		uint revision = g_endpoints[index].invalidation_revision;
		if (revision != (accepted.invalidation_receipt >> 2) ||
			(resets & ~(accepted.invalidation_receipt & 3u)) != 0u ||
			(resets != 0u && !ConstraintAcceptedInEpoch(g_acceptance[island], epoch)))
			scale = 0.0f;
		for (uint row_index = 0; row_index != GpuConstraintRowsPerBlock; ++row_index)
		{
			// Inactive rows are explicitly cleared, including null-response support retained from an earlier visit.
			int offset = GpuConstraintRowsPerBlock * index + row_index;
			GpuConstraintRow row = g_rows[offset];
			row.bounds.z = ConstraintRowGuess(accepted, row_index, block.velocity_mask, block.row_states, block.row_identities, row.bounds.xy, scale);
			g_rows[offset] = row;
		}
	}
}

// Select only physically admissible private trees for cache reconstruction; no live state is modified.
numthreads(CSSelectCoupledTransactionTrees, ConstraintThreadCount, 1, 1)
void CSSelectCoupledTransactionTrees(int3 DTID(dtid))
{
	// Rejected owners do not enter any later numerical writer.
	int index = dtid.x;
	if (index < island_count && g_results[index] != GpuConstraintStage_Accepted)
		g_island_states[index].status = GpuCoupledConstraintIslandStatus_Rejected;
	if (index < range_count)
		g_tree_selection[index] = g_results[g_articulation_islands[index]] == GpuConstraintStage_Accepted &&
			g_island_states[g_articulation_islands[index]].status == GpuCoupledConstraintIslandStatus_Pending ? 1u : 0u;
}

// Classify reconstructed private twists after the configuration itself has already passed recovery validation.
numthreads(CSValidateCoupledTransactionTrees, ConstraintThreadCount, 1, 1)
void CSValidateCoupledTransactionTrees(int3 DTID(dtid))
{
	// A finite generalized seed can still produce an unrepresentable derived link twist.
	int index = dtid.x;
	if (index < range_count && g_tree_selection[index] != 0u && g_tree_results[index] == 0u)
		RejectStage(g_articulation_islands[index], GpuConstraintStage_AdmissionRejected);
}

// Separate new seed representability rejection from topology/preconditioner failures before a numerical projection is attempted.
numthreads(CSAdmitCoupledTransactionSeed, ConstraintThreadCount, 1, 1)
void CSAdmitCoupledTransactionSeed(int3 DTID(dtid))
{
	// The existing gather/response validators have not published any physical or accepted state.
	int island = dtid.x;
	if (island >= island_count)
		return;
	GpuCoupledConstraintIslandState state = g_island_states[island];
	if (g_results[island] == GpuConstraintStage_Accepted && state.status == GpuCoupledConstraintIslandStatus_Rejected)
	{
		// Response overflow is rejectable; topology/preconditioning retains the existing numerical rejection and original flags.
		uint physical = GpuCoupledConstraintFailure_NonFinite | GpuCoupledConstraintFailure_Articulation;
		RejectStage(island, (state.failure_flags & ~physical) == 0u ? GpuConstraintStage_AdmissionRejected : GpuConstraintStage_SolveRejected);
	}
	g_island_states[island].status = g_results[island] == GpuConstraintStage_Accepted ? GpuCoupledConstraintIslandStatus_Pending : GpuCoupledConstraintIslandStatus_Rejected;
}

// Apply validated seed responses only to the recovered private primary state.
numthreads(CSSeedCoupledTransaction, ConstraintThreadCount, 1, 1)
void CSSeedCoupledTransaction(int3 DTID(dtid))
{
	// Ordinary targets and complete-tree generalized responses use separate storage and frames.
	int index = dtid.x;
	if (index < target_count)
	{
		// The existing gather checked the prospective rigid twist before producing this response.
		GpuCoupledConstraintTarget target = g_targets[index];
		if (target.target_type == GpuCoupledConstraintTargetType_Rigid && g_results[target.island_idx] == GpuConstraintStage_Accepted &&
			g_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Pending)
		{
			// Provisional momentum does not change sleep metadata.
			GpuRigidBody body = g_work_bodies[target.target_idx];
			body.momentum_ang.xyz += g_target_impulses[index].ang.xyz;
			body.momentum_lin.xyz += g_target_impulses[index].lin.xyz;
			if (!PhysicalTwistFinite(body))
				RejectStage(target.island_idx, GpuConstraintStage_AdmissionRejected);
			g_work_bodies[target.target_idx] = body;
		}
	}
	if (index < range_count)
	{
		// ABA's response-only operator never added these compact deltas to either physical or private state.
		int island = g_articulation_islands[index];
		if (g_results[island] != GpuConstraintStage_Accepted || g_island_states[island].status != GpuCoupledConstraintIslandStatus_Pending)
			return;
		GpuArticulationMobilityRange range = g_ranges[index];
		GpuArticulation articulation = g_articulations[range.articulation_index];
		for (int component = 0; component != articulation.velocity_count; ++component)
		{
			// Admission includes the sum, not just finite inputs.
			int offset = articulation.velocity_offset + component;
			float value = g_work_velocities[offset] + g_velocity_deltas[range.velocity_delta_offset + component];
			if (!isfinite(value))
				RejectStage(island, GpuConstraintStage_AdmissionRejected);
			g_work_velocities[offset] = value;
		}
	}
}

// Preserve the existing numerical rejection class rather than relabeling it as a new staged-admission rejection.
numthreads(CSCollectCoupledTransactionSolve, ConstraintThreadCount, 1, 1)
void CSCollectCoupledTransactionSolve(int3 DTID(dtid))
{
	// Three is the distinct existing-solve-rejected outcome; its original flags remain in island state.
	int island = dtid.x;
	if (island < island_count && g_results[island] == GpuConstraintStage_Accepted &&
		g_island_states[island].status != GpuCoupledConstraintIslandStatus_Accepted)
		g_results[island] = GpuConstraintStage_SolveRejected;
}

// Stage the actual private response difference and validate every accepted load before the publication boundary.
numthreads(CSStageCoupledTransaction, ConstraintThreadCount, 1, 1)
void CSStageCoupledTransaction(int3 DTID(dtid))
{
	// Live state is the unchanged baseline of this attempt, including all earlier interleaved owners.
	int index = dtid.x;
	if (index < target_count)
	{
		// The selected private apply has consumed trial impulses; accepted islands retain this proposed ledger across later retries.
		GpuCoupledConstraintTarget target = g_targets[index];
		if (target.target_type == GpuCoupledConstraintTargetType_Rigid && g_results[target.island_idx] == GpuConstraintStage_Accepted &&
			g_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Pending)
		{
			// Recover the identical free value again; no additional free-state or rollback snapshot is required.
			GpuRigidBody live = g_live_bodies[target.target_idx];
			GpuRigidBody work = g_work_bodies[target.target_idx];
			GpuArticulationSpatialVector old_response = g_rigid_response[target.target_idx];
			GpuArticulationSpatialVector replacement = (GpuArticulationSpatialVector)0;
			for (int component = 0; component != 3; ++component)
			{
				// These differences measure actual represented seed-plus-solve response, not a new interpretation of old coefficients.
				float angular;
				float translation;
				ConstraintStageFreeComponent(live.momentum_ang[component], old_response.ang[component], g_acceptance[target.island_idx], epoch, angular);
				ConstraintStageFreeComponent(live.momentum_lin[component], old_response.lin[component], g_acceptance[target.island_idx], epoch, translation);
				bool current = ConstraintAcceptedInEpoch(g_acceptance[target.island_idx], epoch);
				replacement.ang[component] = CoupledWorkSame(work.momentum_ang[component], live.momentum_ang[component])
					? (current ? old_response.ang[component] : 0.0f) : work.momentum_ang[component] - angular;
				replacement.lin[component] = CoupledWorkSame(work.momentum_lin[component], live.momentum_lin[component])
					? (current ? old_response.lin[component] : 0.0f) : work.momentum_lin[component] - translation;
			}
			GpuRigidBody staged;
			GpuArticulationSpatialVector response;
			bool valid = ConstraintStageRigidEndpoint(live, old_response, replacement, g_acceptance[target.island_idx], epoch, staged, response);
			if (!valid || !PhysicalTwistFinite(staged))
				RejectStage(target.island_idx, GpuConstraintStage_AdmissionRejected);
			g_work_bodies[target.target_idx] = staged;
			g_target_impulses[index] = response;
		}
	}
	if (index < range_count)
	{
		// Complete trees retain the response in the same native generalized components as their physical state.
		int island = g_articulation_islands[index];
		if (g_results[island] == GpuConstraintStage_Accepted && g_island_states[island].status == GpuCoupledConstraintIslandStatus_Pending)
		{
			// Reuse ABA's compact delta only after private application; final work reads the staged native primary state instead.
			GpuArticulationMobilityRange range = g_ranges[index];
			GpuArticulation articulation = g_articulations[range.articulation_index];
			for (int component = 0; component != articulation.velocity_count; ++component)
			{
				// The final candidate can round differently from the private solve and must be admitted in its actual published form.
				int offset = articulation.velocity_offset + component;
				float free_value;
				ConstraintStageFreeComponent(g_live_velocities[offset], g_velocity_response[offset], g_acceptance[island], epoch, free_value);
				float replacement = CoupledWorkSame(g_work_velocities[offset], g_live_velocities[offset])
					? (ConstraintAcceptedInEpoch(g_acceptance[island], epoch) ? g_velocity_response[offset] : 0.0f)
					: g_work_velocities[offset] - free_value;
				GpuConstraintStagedComponent staged;
				if (!ConstraintStageReplacement(g_live_velocities[offset], g_velocity_response[offset], replacement, g_acceptance[island], epoch, staged))
					RejectStage(island, GpuConstraintStage_AdmissionRejected);
				g_work_velocities[offset] = staged.value;
				g_velocity_deltas[range.velocity_delta_offset + component] = staged.response;
			}
		}
	}
	if (index < slot_count)
	{
		// Accepted loads use linear-row impulse norm and pure-angular resultant, not world-COM response torque.
		int island = g_topology[index].island_idx;
		if (island >= 0 && g_results[island] == GpuConstraintStage_Accepted &&
			g_island_states[island].status == GpuCoupledConstraintIslandStatus_Pending)
		{
			// Zero-row visits still retire old coefficients and loads when the island publishes.
			GpuConstraintRow rows[GpuConstraintRowsPerBlock];
			for (int row_index = 0; row_index != GpuConstraintRowsPerBlock; ++row_index)
				rows[row_index] = g_rows[GpuConstraintRowsPerBlock * index + row_index];
			GpuConstraintAcceptedRows accepted;
			if (!ConstraintStageAcceptedRows(g_blocks[index], rows, accepted))
				RejectStage(island, GpuConstraintStage_AdmissionRejected);
		}
	}
}

// Publish only completely admitted owners. All inputs to this dispatch are immutable until its final UAV barrier.
numthreads(CSPublishCoupledTransaction, ConstraintThreadCount, 1, 1)
void CSPublishCoupledTransaction(int3 DTID(dtid))
{
	// Ordinary and coupled writers can share a physical body across transactions, never during this atomic attempt.
	int index = dtid.x;
	if (index < target_count)
	{
		// Each ordinary target is written exactly once by this coupled owner.
		GpuCoupledConstraintTarget target = g_targets[index];
		if (target.target_type == GpuCoupledConstraintTargetType_Rigid && g_results[target.island_idx] == GpuConstraintStage_Accepted)
		{
			// The staged body already derives wake state from the accepted live difference.
			g_live_bodies[target.target_idx] = g_work_bodies[target.target_idx];
			g_rigid_response[target.target_idx] = g_target_impulses[index];
		}
	}
	if (index < range_count)
	{
		// Generalized state, represented response, and reconstructed link velocities publish under the same island decision.
		int island = g_articulation_islands[index];
		if (g_results[island] == GpuConstraintStage_Accepted)
		{
			// Other ABA factors, force results, and cache members remain owned by their existing stages.
			GpuArticulationMobilityRange range = g_ranges[index];
			GpuArticulation articulation = g_articulations[range.articulation_index];
			for (int component = 0; component != articulation.velocity_count; ++component)
			{
				// Physical and response arrays share forest indexing, independently of compact participation order.
				int offset = articulation.velocity_offset + component;
				g_live_velocities[offset] = g_work_velocities[offset];
				g_velocity_response[offset] = g_velocity_deltas[range.velocity_delta_offset + component];
			}
			for (int link = 0; link != articulation.link_count; ++link)
			{
				// Only the velocity cache was reconstructed by this transaction.
				int offset = articulation.link_offset + link;
				g_live_scratch[offset].link_velocity = g_work_scratch[offset].link_velocity;
			}
		}
	}
	if (index < slot_count)
	{
		// Context errors retain the fatal slot-tagged transport; ordinary staged-admission and existing solve rejection do not enter it.
		int island = g_topology[index].island_idx;
		if (island >= 0)
		{
			// No published record changes on either rejection class.
			if (g_results[island] == GpuConstraintStage_NumericalFailure)
				InterlockedMax(g_solver_state[0].failure_slot_plus_one, (uint)index + 1u);
			if (g_results[island] == GpuConstraintStage_Accepted)
			{
				// This deterministic load calculation already passed admission against these unchanged working rows.
				GpuConstraintRow rows[GpuConstraintRowsPerBlock];
				for (int row_index = 0; row_index != GpuConstraintRowsPerBlock; ++row_index)
					rows[row_index] = g_rows[GpuConstraintRowsPerBlock * index + row_index];
				GpuConstraintAcceptedRows accepted;
				ConstraintStageAcceptedRows(g_blocks[index], rows, accepted);
				accepted.invalidation_receipt = (g_endpoints[index].invalidation_revision << 2) | BodyResetMask(index, g_work_bodies);
				g_accepted_rows[index] = accepted;
			}
		}
	}
	if (index < island_count)
	{
		// No observer may consume this stamp or its payload until the complete publication dispatch has finished.
		if (g_results[index] == GpuConstraintStage_Accepted)
			g_acceptance[index] = ConstraintAcceptance(epoch);
		g_island_states[index].status = g_results[index] == GpuConstraintStage_Accepted ? GpuCoupledConstraintIslandStatus_Committed : GpuCoupledConstraintIslandStatus_Rejected;
	}
}
