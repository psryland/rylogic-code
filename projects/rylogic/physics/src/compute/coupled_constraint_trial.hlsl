//*********************************************
// Physics Engine - Complete Coupled Trials
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "constraint_transaction.hlsli"
#include "coupled_constraint_work.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Private reference/candidate phases; these values have no accepted-epoch or public ABI meaning.
#define CoupledReference_Free 0
#define CoupledReference_Initial 1
#define CoupledReference_Current 2
#define CoupledReference_Retirement 3
#define CoupledCapture_Current 0
#define CoupledCapture_Free 1
#define CoupledCapture_Initial 2
#define CoupledRestore_Selected 3
#define CoupledRestore_Initial 4

// Actual packed bounds and the current private reference operation.
struct cbCoupledConstraintTrial
{
	int slot_count;
	int body_count;
	int island_count;
	int range_count;
	int rigid_count;
	int velocity_count;
	int link_count;
	int mobility_count;
	int island_block_count;
	int attempt_index;
	int backtrack_limit;
	int reference_phase;
	GpuConstraintEpoch epoch;
};

ConstantBuffer<cbCoupledConstraintTrial> resource(g_trial, b0);
StructuredBuffer<GpuCoupledConstraintTarget> resource(g_trial_targets, t0);
StructuredBuffer<GpuCoupledConstraintBlockTopology> resource(g_trial_topology, t1);
StructuredBuffer<GpuCoupledConstraintEndpoint> resource(g_trial_link_endpoints, t2);
StructuredBuffer<GpuCoupledConstraintWorkSpan> resource(g_trial_spans, t3);
StructuredBuffer<uint> resource(g_trial_indices, t4);
StructuredBuffer<GpuCoupledConstraintIsland> resource(g_trial_islands, t5);
StructuredBuffer<uint> resource(g_trial_island_blocks, t6);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_trial_ranges, t7);
StructuredBuffer<GpuArticulation> resource(g_trial_articulations, t8);
StructuredBuffer<GpuArticulationLink> resource(g_trial_links, t9);
StructuredBuffer<GpuConstraintAcceptedRows> resource(g_trial_accepted_rows, t10);
StructuredBuffer<GpuConstraintAcceptance> resource(g_trial_acceptance, t11);
StructuredBuffer<GpuRigidBody> resource(g_trial_live_bodies, t12);
StructuredBuffer<float> resource(g_trial_live_velocities, t13);
StructuredBuffer<GpuArticulationAbaDofScratch> resource(g_trial_dof_scratch, t14);
StructuredBuffer<GpuConstraintBlock> resource(g_trial_blocks, t15);
RWStructuredBuffer<GpuRigidBody> resource(g_trial_bodies, u0);
RWStructuredBuffer<float> resource(g_trial_velocities, u1);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_trial_scratch, u2);
RWStructuredBuffer<GpuConstraintRow> resource(g_trial_rows, u3);
RWStructuredBuffer<GpuCoupledConstraintMomentumReference> resource(g_trial_momentum_references, u4);
RWStructuredBuffer<GpuCoupledConstraintSpeedReference> resource(g_trial_speed_references, u5);
RWStructuredBuffer<float> resource(g_trial_initial_coefficients, u6);
RWStructuredBuffer<uint> resource(g_trial_choices, u7);
RWStructuredBuffer<GpuCoupledConstraintIslandState> resource(g_trial_states, u8);
RWStructuredBuffer<uint> resource(g_trial_failures, u9);
RWStructuredBuffer<uint> resource(g_trial_results, u10);
RWStructuredBuffer<GpuCoupledConstraintWorkMotion> resource(g_trial_work_motion, u11);

// Return whether this island may still change private numerical state.
bool CoupledTrialPending(int island)
{
	return island >= 0 && island < g_trial.island_count &&
		g_trial_states[island].status == GpuCoupledConstraintIslandStatus_Pending &&
		g_trial_results[island] == GpuConstraintStage_Accepted;
}

// Preserve earlier context/solve errors while rejecting unavailable reference admission.
void CoupledTrialRejectAdmission(int island)
{
	if (g_trial_results[island] == GpuConstraintStage_Accepted)
		g_trial_results[island] = GpuConstraintStage_AdmissionRejected;
	g_trial_states[island].status = GpuCoupledConstraintIslandStatus_Rejected;
}

// Report invalid private work topology without publishing any candidate.
void CoupledTrialRejectTopology(int island)
{
	g_trial_states[island].failure_flags |= GpuCoupledConstraintFailure_Topology;
	g_trial_states[island].status = GpuCoupledConstraintIslandStatus_Rejected;
	if (g_trial_results[island] == GpuConstraintStage_Accepted)
		g_trial_results[island] = GpuConstraintStage_SolveRejected;
}

// Read a native primary component independently of the coefficient ledger.
float CoupledTrialSpeed(GpuArticulationMobilityRange range, GpuArticulation articulation, int component, int reference)
{
	switch (reference)
	{
		case CoupledReference_Free: { return g_trial_speed_references[range.velocity_delta_offset + component].free; }
		case CoupledReference_Initial: { return g_trial_speed_references[range.velocity_delta_offset + component].initial; }
		case CoupledReference_Current: { return g_trial_velocities[articulation.velocity_offset + component]; }
		default: { return asfloat(0x7fc00000u); }
	}
}

// Read current-row coefficients for one explicitly selected physical/reference pair.
float CoupledTrialCoefficient(int row, int reference)
{
	switch (reference)
	{
		case CoupledReference_Free: { return 0.0f; }
		case CoupledReference_Initial: { return g_trial_initial_coefficients[row]; }
		case CoupledReference_Current: { return g_trial_rows[row].bounds.z; }
		default: { return asfloat(0x7fc00000u); }
	}
}

// Read one rigid primary pair without inferring momentum from cached impulses.
GpuArticulationSpatialVector CoupledTrialMomentum(int ordinal, int body, int reference)
{
	switch (reference)
	{
		case CoupledReference_Free: { return g_trial_momentum_references[ordinal].free; }
		case CoupledReference_Initial: { return g_trial_momentum_references[ordinal].initial; }
		case CoupledReference_Current:
		{
			GpuArticulationSpatialVector value;
			value.ang = g_trial_bodies[body].momentum_ang;
			value.lin = g_trial_bodies[body].momentum_lin;
			return value;
		}
		default:
		{
			GpuArticulationSpatialVector value;
			value.ang = value.lin = float4(asfloat(0x7fc00000u), 0, 0, 0);
			return value;
		}
	}
}

// Evaluate one complete tree in packed parent-before-child order with evaluator-only transient motion/error scratch.
bool CoupledTrialTreeWork(int range_idx, int a, int b, out_(CoupledMeritValue) work)
{
	work = CoupledMeritInput(0.0f);
	if (range_idx < 0 || range_idx >= g_trial.range_count)
		return false;

	GpuArticulationMobilityRange range = g_trial_ranges[range_idx];
	GpuArticulation articulation = g_trial_articulations[range.articulation_index];
	if (range.link_count != articulation.link_count || articulation.link_count < 1 ||
		articulation.link_offset < 0 || articulation.link_offset + articulation.link_count > g_trial.link_count ||
		range.mobility_offset < 0 || range.mobility_offset + range.link_count > g_trial.mobility_count ||
		range.velocity_delta_offset < 0 || range.velocity_delta_offset + articulation.velocity_count > g_trial.velocity_count)
		return false;

	// A real primary identity must not create 0*infinity while constructing a kinetic metric.
	bool identical = true;
	for (int component = 0; component != articulation.velocity_count; ++component)
		identical = identical && CoupledWorkSame(CoupledTrialSpeed(range, articulation, component, a), CoupledTrialSpeed(range, articulation, component, b));

	if (identical)
		return true;

	for (int local_link = 0; local_link != articulation.link_count; ++local_link)
	{
		int link_idx = articulation.link_offset + local_link;
		GpuArticulationLink link = g_trial_links[link_idx];
		GpuCoupledConstraintWorkMotion motion;
		motion.delta = CoupledWorkZeroMotion();
		motion.midpoint = CoupledWorkZeroMotion();
		if (local_link == 0)
		{
			switch (articulation.root_type)
			{
				case GpuArticulationRootType_Fixed: { break; }
				case GpuArticulationRootType_Floating:
				{
					CoupledWorkVector delta_ang = CoupledWorkInput3(float3(0, 0, 0));
					CoupledWorkVector delta_lin = delta_ang;
					CoupledWorkVector midpoint_ang = delta_ang;
					CoupledWorkVector midpoint_lin = delta_ang;
					for (int component = 0; component != 3; ++component)
					{
						float aa = CoupledTrialSpeed(range, articulation, component, a);
						float ab = CoupledTrialSpeed(range, articulation, component, b);
						float la = CoupledTrialSpeed(range, articulation, component + 3, a);
						float lb = CoupledTrialSpeed(range, articulation, component + 3, b);
						CoupledWorkSetComponent(delta_ang, component, CoupledWorkDelta(aa, ab));
						CoupledWorkSetComponent(delta_lin, component, CoupledWorkDelta(la, lb));
						CoupledWorkSetComponent(midpoint_ang, component, CoupledWorkMidpoint(aa, ab));
						CoupledWorkSetComponent(midpoint_lin, component, CoupledWorkMidpoint(la, lb));
					}
					motion.delta = CoupledWorkPack(delta_ang, delta_lin);
					motion.midpoint = CoupledWorkPack(midpoint_ang, midpoint_lin);
					break;
				}
				default: { return false; }
			}
		}
		else
		{
			int parent = link.parent_link_index - articulation.link_offset;
			if (parent < 0 || parent >= local_link)
				return false;

			// One parent load and one current-link store per comparison; no ancestor reconstruction.
			GpuCoupledConstraintWorkMotion parent_motion = g_trial_work_motion[range.mobility_offset + parent];
			GpuConstraintFrame transform = g_trial_scratch[link_idx].child_to_parent;
			motion.delta = CoupledWorkChildMotion(transform, parent_motion.delta);
			motion.midpoint = CoupledWorkChildMotion(transform, parent_motion.midpoint);
			for (int row = 0; row != link.dof_count; ++row)
			{
				int component = link.velocity_offset - articulation.velocity_offset + row;
				float va = CoupledTrialSpeed(range, articulation, component, a);
				float vb = CoupledTrialSpeed(range, articulation, component, b);
				GpuArticulationSpatialVector screw = g_trial_dof_scratch[link.dof_offset + row].motion_subspace;
				motion.delta = CoupledWorkAddScrew(motion.delta, screw, CoupledWorkDelta(va, vb));
				motion.midpoint = CoupledWorkAddScrew(motion.midpoint, screw, CoupledWorkMidpoint(va, vb));
			}
		}
		g_trial_work_motion[range.mobility_offset + local_link] = motion;
		work = CoupledMeritAdd(work, CoupledWorkLink(link, motion.delta, motion.midpoint));
	}
	return true;
}

// Account for prescribed rigid endpoint work without counting it as dynamic kinetic energy.
CoupledMeritValue CoupledTrialPrescribed(int body, int link, int target, float3 angular, float3 translation)
{
	if (link >= 0 || target >= 0 || body < 0 || body >= g_trial.body_count)
		return CoupledMeritInput(0.0f);

	CoupledWorkMotion motion = CoupledWorkRigidTwist(g_trial_live_bodies[body]);
	return CoupledMeritAdd(CoupledWorkDot3(CoupledWorkInput3(angular), CoupledWorkPart(motion, true)),
		CoupledWorkDot3(CoupledWorkInput3(translation), CoupledWorkPart(motion, false)));
}

// Reduce one canonical actual-work difference; scratch belongs exclusively to this island's complete trees.
bool CoupledTrialWork(int island, int a, int b, out_(CoupledMeritValue) work)
{
	work = CoupledMeritInput(0.0f);
	GpuCoupledConstraintWorkSpan span = g_trial_spans[island];
	for (uint local = 0u; local != span.rigid_count; ++local)
	{
		uint ordinal = span.rigid_offset + local;
		if (ordinal >= (uint)g_trial.rigid_count)
			return false;

		GpuCoupledConstraintTarget target = g_trial_targets[g_trial_indices[ordinal]];
		if (target.island_idx != island || target.target_type != GpuCoupledConstraintTargetType_Rigid || target.target_idx < 0 || target.target_idx >= g_trial.body_count)
			return false;

		work = CoupledMeritAdd(work, CoupledWorkRigid(g_trial_live_bodies[target.target_idx],
			CoupledTrialMomentum(ordinal, target.target_idx, a), CoupledTrialMomentum(ordinal, target.target_idx, b)));
	}
	for (uint local = 0u; local != span.tree_count; ++local)
	{
		CoupledMeritValue tree_work;
		if (!CoupledTrialTreeWork(g_trial_indices[span.tree_offset + local], a, b, tree_work))
			return false;

		work = CoupledMeritAdd(work, tree_work);
	}
	GpuCoupledConstraintIsland block_range = g_trial_islands[island];
	if (block_range.block_offset < 0 || block_range.block_count < 1 || block_range.block_offset + block_range.block_count > g_trial.island_block_count)
		return false;

	for (int local = 0; local != block_range.block_count; ++local)
	{
		uint slot = g_trial_island_blocks[block_range.block_offset + local];
		if (slot >= (uint)g_trial.slot_count || g_trial_topology[slot].island_idx != island)
			return false;

		GpuConstraintBlock block = g_trial_blocks[slot];
		GpuCoupledConstraintEndpoint endpoints = g_trial_link_endpoints[slot];
		GpuCoupledConstraintBlockTopology topology = g_trial_topology[slot];
		for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
		{
			if ((block.velocity_mask & (1u << axis)) == 0u)
				continue;

			int row_idx = GpuConstraintRowsPerBlock * slot + axis;
			float aa = CoupledTrialCoefficient(row_idx, a);
			float bb = CoupledTrialCoefficient(row_idx, b);
			if (CoupledWorkSame(aa, bb))
				continue;

			GpuConstraintRow row = g_trial_rows[row_idx];
			CoupledMeritValue prescribed = CoupledMeritAdd(
				CoupledTrialPrescribed(block.body_idx_a, endpoints.link_idx_a, topology.target_idx_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz),
				CoupledTrialPrescribed(block.body_idx_b, endpoints.link_idx_b, topology.target_idx_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz));
			work = CoupledMeritAdd(work, CoupledWorkRow(row, aa, bb, prescribed));
		}
	}
	return true;
}

// Capture actual primary/reference data without interpreting its accepted coefficients as a physical response.
numthreads(CSCaptureCoupledReference, ConstraintThreadCount, 1, 1)
void CSCaptureCoupledReference(int3 DTID(dtid))
{
	int island = dtid.x;
	if (!CoupledTrialPending(island))
		return;

	GpuCoupledConstraintWorkSpan span = g_trial_spans[island];
	bool current = g_trial.reference_phase == CoupledCapture_Current;
	bool free = g_trial.reference_phase == CoupledCapture_Free;
	for (uint local = 0u; local != span.rigid_count; ++local)
	{
		uint ordinal = span.rigid_offset + local;
		int body_idx = g_trial_targets[g_trial_indices[ordinal]].target_idx;
		GpuRigidBody body = g_trial_bodies[body_idx];
		if (current)
			body = g_trial_live_bodies[body_idx];
		GpuArticulationSpatialVector momentum;
		momentum.ang = body.momentum_ang;
		momentum.lin = body.momentum_lin;
		if (free)
			g_trial_momentum_references[ordinal].free = momentum;
		else
			g_trial_momentum_references[ordinal].initial = momentum;
	}
	for (uint local = 0u; local != span.tree_count; ++local)
	{
		GpuArticulationMobilityRange range = g_trial_ranges[g_trial_indices[span.tree_offset + local]];
		GpuArticulation articulation = g_trial_articulations[range.articulation_index];
		for (int component = 0; component != articulation.velocity_count; ++component)
		{
			int offset = articulation.velocity_offset + component;
			float value = current ? g_trial_live_velocities[offset] : g_trial_velocities[offset];
			if (free)
				g_trial_speed_references[range.velocity_delta_offset + component].free = value;
			else
				g_trial_speed_references[range.velocity_delta_offset + component].initial = value;
		}
	}
	if (free)
		return;

	GpuCoupledConstraintIsland blocks = g_trial_islands[island];
	for (int local = 0; local != blocks.block_count; ++local)
	{
		uint slot = g_trial_island_blocks[blocks.block_offset + local];
		for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
		{
			int offset = GpuConstraintRowsPerBlock * slot + axis;
			g_trial_initial_coefficients[offset] = current
				? g_trial_accepted_rows[slot].impulses[axis]
				: g_trial_rows[offset].bounds.z;
		}
	}
}

// Choose the free/current-epoch reference under the current frozen row law.
numthreads(CSChooseCoupledReference, ConstraintThreadCount, 1, 1)
void CSChooseCoupledReference(int3 DTID(dtid))
{
	int island = dtid.x;
	if (!CoupledTrialPending(island))
		return;

	bool eligible = ConstraintAcceptedInEpoch(g_trial_acceptance[island], g_trial.epoch);
	bool active = false;
	GpuCoupledConstraintIsland blocks = g_trial_islands[island];
	for (int local = 0; local != blocks.block_count; ++local)
	{
		uint slot = g_trial_island_blocks[blocks.block_offset + local];
		GpuConstraintBlock block = g_trial_blocks[slot];
		GpuConstraintAcceptedRows accepted = g_trial_accepted_rows[slot];
		eligible = eligible && block.velocity_mask == accepted.velocity_mask;
		active = active || block.velocity_mask != 0u;
		for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
		{
			if ((block.velocity_mask & (1u << axis)) == 0u)
				continue;

			float impulse = accepted.impulses[axis];
			float2 bounds = g_trial_rows[GpuConstraintRowsPerBlock * slot + axis].bounds.xy;
			eligible = eligible && isfinite(impulse) && impulse >= bounds.x && impulse <= bounds.y &&
				((block.row_states ^ accepted.row_states) & (3u << (2 * axis))) == 0u &&
				((block.row_identities ^ accepted.row_identities) & (15u << (4 * axis))) == 0u;
		}
	}
	g_trial_choices[island] = active ? CoupledReference_Free : CoupledReference_Retirement;
	if (!active || !eligible)
		return;

	CoupledMeritValue work;
	if (!CoupledTrialWork(island, CoupledReference_Free, CoupledReference_Initial, work))
	{
		CoupledTrialRejectTopology(island);
		return;
	}
	if (!isfinite(work.value) || !isfinite(work.error))
	{
		CoupledTrialRejectAdmission(island);
		return;
	}
	if (work.value <= work.error)
		g_trial_choices[island] = CoupledReference_Initial;
}

// Safeguard the optional warm initialization before the first ordinary projected direction.
numthreads(CSAdmitCoupledInitialization, ConstraintThreadCount, 1, 1)
void CSAdmitCoupledInitialization(int3 DTID(dtid))
{
	int island = dtid.x;
	if (!CoupledTrialPending(island))
		return;

	if (g_trial_choices[island] == CoupledReference_Retirement)
	{
		g_trial_states[island].status = GpuCoupledConstraintIslandStatus_Accepted;
		return;
	}
	CoupledMeritValue complete;
	if (!CoupledTrialWork(island, CoupledReference_Free, CoupledReference_Current, complete))
	{
		CoupledTrialRejectTopology(island);
		return;
	}

	// The same reference pair has the same signed value and error; a different reference still needs its own comparison.
	CoupledMeritValue descent = complete;
	if (g_trial_choices[island] != CoupledReference_Free)
	{
		if (!CoupledTrialWork(island, g_trial_choices[island], CoupledReference_Current, descent))
		{
			CoupledTrialRejectTopology(island);
			return;
		}
	}
	if (!isfinite(complete.value) || !isfinite(complete.error) || !isfinite(descent.value) || !isfinite(descent.error))
	{
		CoupledTrialRejectAdmission(island);
		return;
	}
	if (complete.value <= complete.error && descent.value <= descent.error)
		g_trial_choices[island] = CoupledReference_Current;
}

// Restore only private primary state and coefficient pairs; accepted islands retain their final staged output.
numthreads(CSRestoreCoupledReference, ConstraintThreadCount, 1, 1)
void CSRestoreCoupledReference(int3 DTID(dtid))
{
	int island = dtid.x;
	if (!CoupledTrialPending(island))
		return;

	int reference = g_trial.reference_phase == CoupledRestore_Initial ? CoupledReference_Initial : (int)g_trial_choices[island];
	if (reference == CoupledReference_Current)
		return;

	GpuCoupledConstraintWorkSpan span = g_trial_spans[island];
	for (uint local = 0u; local != span.rigid_count; ++local)
	{
		uint ordinal = span.rigid_offset + local;
		int body_idx = g_trial_targets[g_trial_indices[ordinal]].target_idx;
		GpuRigidBody body = g_trial_live_bodies[body_idx];
		GpuArticulationSpatialVector momentum = CoupledTrialMomentum(ordinal, body_idx, reference);
		body.momentum_ang = momentum.ang;
		body.momentum_lin = momentum.lin;
		g_trial_bodies[body_idx] = body;
	}
	for (uint local = 0u; local != span.tree_count; ++local)
	{
		GpuArticulationMobilityRange range = g_trial_ranges[g_trial_indices[span.tree_offset + local]];
		GpuArticulation articulation = g_trial_articulations[range.articulation_index];
		for (int component = 0; component != articulation.velocity_count; ++component)
			g_trial_velocities[articulation.velocity_offset + component] = CoupledTrialSpeed(range, articulation, component, reference);
	}
	GpuCoupledConstraintIsland blocks = g_trial_islands[island];
	for (int local = 0; local != blocks.block_count; ++local)
	{
		uint slot = g_trial_island_blocks[blocks.block_offset + local];
		for (int axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
		{
			int offset = GpuConstraintRowsPerBlock * slot + axis;
			g_trial_rows[offset].bounds.z = (g_trial_blocks[slot].velocity_mask & (1u << axis)) != 0u
				? CoupledTrialCoefficient(offset, reference) : 0.0f;
		}
	}
}

// Count actual selected numerical candidates independently of where a later admission check fails.
numthreads(CSBeginCoupledVelocityTrial, ConstraintThreadCount, 1, 1)
void CSBeginCoupledVelocityTrial(int3 DTID(dtid))
{
	if (CoupledTrialPending(dtid.x))
		++g_trial_states[dtid.x].iteration_count;
}

// Apply the one authoritative complete-primary work gate after final staging and cache reconstruction.
numthreads(CSEvaluateCoupledVelocityMerit, ConstraintThreadCount, 1, 1)
void CSEvaluateCoupledVelocityMerit(int3 DTID(dtid))
{
	int island = dtid.x;
	if (island >= g_trial.island_count || g_trial_states[island].status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	GpuCoupledConstraintIslandState state = g_trial_states[island];
	if (g_trial_results[island] != GpuConstraintStage_Accepted || g_trial_failures[island] != GpuCoupledConstraintFailure_None)
	{
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
		state.failure_flags = g_trial_failures[island];
		g_trial_states[island] = state;
		return;
	}
	CoupledMeritValue complete;
	CoupledMeritValue descent;
	if (!CoupledTrialWork(island, CoupledReference_Free, CoupledReference_Current, complete) ||
		!CoupledTrialWork(island, CoupledReference_Initial, CoupledReference_Current, descent))
	{
		CoupledTrialRejectTopology(island);
		return;
	}

	// An infinite allowance is a failed numerical certificate, never permission to accept arbitrary positive work.
	bool finite_work = isfinite(complete.value) && isfinite(descent.value);
	bool finite_error = isfinite(complete.error) && isfinite(descent.error);
	bool complete_pass = complete.value <= complete.error;
	bool descent_pass = descent.value <= descent.error;
	if (finite_work && finite_error && complete_pass && descent_pass)
		state.status = GpuCoupledConstraintIslandStatus_Accepted;
	else if (g_trial.attempt_index < g_trial.backtrack_limit)
		state.relaxation *= 0.5f;
	else
	{
		state.failure_flags |= finite_work && finite_error ? GpuCoupledConstraintFailure_Merit : GpuCoupledConstraintFailure_NonFinite;
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
	}
	// Diagnostic conversion cannot change certificate validity, acceptance, or the physical reason for rejection.
	CoupledMeritValue reported = descent;
	if (!complete_pass)
		reported = complete;

	state.merit_change = CoupledMeritDiagnostic(reported);
	g_trial_states[island] = state;
}

#ifdef __cplusplus
}
#endif
