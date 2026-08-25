//*********************************************
// Physics Engine — Coupled Constraint Position Correction
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Solves hard articulation-coupled drift in detached pseudo state, then integrates coordinates once
// without changing rigid momentum, generalized velocity, or cached physical link velocity.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Packed bounds, numerical controls, and transaction mode for one coupled position pass.
struct cbCoupledConstraintPosition
{
	int slot_count;
	int body_count;
	int target_count;
	int island_count;

	int articulation_range_count;
	int mobility_count;
	int velocity_delta_count;
	int island_block_count;

	int phase;
	int attempt_index;
	int backtrack_limit;
	int pad0;

	float timestep;
	float relaxation;
	float position_beta;
	float max_position_speed;
};

ConstantBuffer<cbCoupledConstraintPosition> resource(g_coupled_position, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_coupled_position_bodies, u0);
StructuredBuffer<GpuCoupledConstraintEndpoint> resource(g_coupled_position_link_endpoints, t0);
StructuredBuffer<GpuCoupledConstraintBlockTopology> resource(g_coupled_position_block_topology, t1);
StructuredBuffer<GpuCoupledConstraintTarget> resource(g_coupled_position_targets, t2);
StructuredBuffer<uint> resource(g_coupled_position_adjacency, t3);
StructuredBuffer<GpuCoupledConstraintPreconditioner> resource(g_coupled_position_preconditioners, t4);
StructuredBuffer<int> resource(g_coupled_position_articulation_islands, t5);
StructuredBuffer<GpuCoupledConstraintIsland> resource(g_coupled_position_islands, t6);
StructuredBuffer<uint> resource(g_coupled_position_island_blocks, t7);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_coupled_position_ranges, t8);
StructuredBuffer<GpuArticulationLink> resource(g_coupled_position_links, t9);
RWStructuredBuffer<GpuConstraintBlock> resource(g_coupled_position_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_coupled_position_rows, u2);
RWStructuredBuffer<GpuCoupledConstraintSolveScratch> resource(g_coupled_position_scratch, u3);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_position_contributions, u4);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_position_target_impulses, u5);
RWStructuredBuffer<GpuCoupledConstraintIslandState> resource(g_coupled_position_island_states, u6);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_position_link_impulses, u7);
RWStructuredBuffer<uint> resource(g_coupled_position_tree_selection, u8);
RWStructuredBuffer<uint> resource(g_coupled_position_tree_results, u9);
RWStructuredBuffer<uint> resource(g_coupled_position_island_failures, u10);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_position_articulation_work, u11);
RWStructuredBuffer<float> resource(g_coupled_position_velocity_deltas, u12);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_coupled_position_rigid_pseudo, u13);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_position_link_pseudo, u14);
RWStructuredBuffer<float> resource(g_coupled_position_generalized_pseudo, u15);
RWStructuredBuffer<GpuArticulation> resource(g_coupled_position_articulations, u16);
RWStructuredBuffer<float> resource(g_coupled_position_positions, u17);

#define PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE coupled_constraint_position_ops_detail
#include "physics/src/compute/constraint_solver_ops.hlsli"
#undef PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE

static const float CoupledPositionMaximumAngularDisplacement = 1.0e4f;

// Return an explicitly zeroed spatial vector for both shader and deterministic replay builds.
GpuArticulationSpatialVector EmptyCoupledPositionSpatialVector()
{
	GpuArticulationSpatialVector value;
	value.ang = float4(0.0f, 0.0f, 0.0f, 0.0f);
	value.lin = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return value;
}

// Return an explicitly zeroed six-row candidate scratch value.
GpuCoupledConstraintSolveScratch EmptyCoupledPositionSolveScratch()
{
	GpuCoupledConstraintSolveScratch scratch;
	scratch.impulse_delta_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.impulse_delta_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.residual_before_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.residual_before_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return scratch;
}

// Return true when every meaningful component of one spatial value is finite.
bool CoupledPositionSpatialFinite(GpuArticulationSpatialVector value)
{
	return
		isfinite(value.ang.x) && isfinite(value.ang.y) && isfinite(value.ang.z) &&
		isfinite(value.lin.x) && isfinite(value.lin.y) && isfinite(value.lin.z);
}

// Return true when every component of one physical vector is finite.
bool CoupledPositionVectorFinite(float3 value)
{
	return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

// Return whether a detached pseudo vector contains a coordinate change worth integrating.
bool CoupledPositionVectorNonZero(float3 value)
{
	return value.x != 0.0f || value.y != 0.0f || value.z != 0.0f;
}

// Return whether an accumulated pseudo angular velocity can be integrated safely this substep.
bool CoupledPositionAngularDisplacementValid(float3 angular_velocity)
{
	float3 displacement = g_coupled_position.timestep * angular_velocity;
	return
		CoupledPositionVectorFinite(displacement) &&
		max(abs(displacement.x), max(abs(displacement.y), abs(displacement.z))) <= CoupledPositionMaximumAngularDisplacement;
}

// Return one spatial sum without relying on shader-only operator overloads.
GpuArticulationSpatialVector CoupledPositionAddSpatial(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	lhs.ang += rhs.ang;
	lhs.lin += rhs.lin;
	return lhs;
}

// Add one scaled row wrench to a detached endpoint impulse.
GpuArticulationSpatialVector CoupledPositionAddScaledWrench(
	GpuArticulationSpatialVector impulse,
	float3 jacobian_ang,
	float3 jacobian_lin,
	float scale)
{
	impulse.ang.xyz += scale * jacobian_ang;
	impulse.lin.xyz += scale * jacobian_lin;
	return impulse;
}

// Return true when one rigid endpoint is dynamic and can receive pseudo velocity.
bool CoupledPositionRigidDynamic(int body_idx)
{
	return
		body_idx >= 0 &&
		body_idx < g_coupled_position.body_count &&
		g_coupled_position_bodies[body_idx].os_com_and_invmass.w > 0.0f &&
		!AllSet(g_coupled_position_bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
}

// Return one dynamic rigid body's world-space inverse inertia at its centre of mass.
float3x3 CoupledPositionInverseInertia(int body_idx)
{
	GpuRigidBody body = g_coupled_position_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	return rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
}

// Return one endpoint's current contribution to a scalar pseudo-velocity row.
float CoupledPositionEndpointRowVelocity(
	int body_idx,
	int mobility_idx,
	float3 jacobian_ang,
	float3 jacobian_lin)
{
	if (mobility_idx >= 0)
	{
		if (mobility_idx >= g_coupled_position.mobility_count)
			return 0.0f;

		GpuArticulationSpatialVector velocity = g_coupled_position_link_pseudo[mobility_idx];
		return dot(jacobian_ang, velocity.ang.xyz) + dot(jacobian_lin, velocity.lin.xyz);
	}
	if (!CoupledPositionRigidDynamic(body_idx))
		return 0.0f;

	GpuConstraintPseudoVelocity velocity = g_coupled_position_rigid_pseudo[body_idx];
	return dot(jacobian_ang, velocity.angular_velocity.xyz) + dot(jacobian_lin, velocity.linear_velocity.xyz);
}

// Return the hard-drift residual of one coupled scalar row against detached pseudo state.
float CoupledPositionRowResidual(GpuConstraintBlock block, GpuCoupledConstraintEndpoint endpoint, GpuConstraintRow row)
{
	float velocity =
		CoupledPositionEndpointRowVelocity(block.body_idx_a, endpoint.mobility_idx_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz) +
		CoupledPositionEndpointRowVelocity(block.body_idx_b, endpoint.mobility_idx_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz);
	float correction = g_coupled_position.position_beta * row.solve.x / g_coupled_position.timestep;
	float target_velocity = -clamp(correction, -g_coupled_position.max_position_speed, +g_coupled_position.max_position_speed);
	return velocity - target_velocity;
}

// Store one canonical axis value in the split low/high candidate layout.
void CoupledPositionSetSixVectorComponent(inout_(float4) low, inout_(float4) high, int axis_idx, float value)
{
	if (axis_idx < 4)
		low[axis_idx] = value;
	else
		high[axis_idx - 4] = value;
}

// Return one canonical axis value from the split low/high candidate layout.
float CoupledPositionSixVectorComponent(float4 low, float4 high, int axis_idx)
{
	return axis_idx < 4 ? low[axis_idx] : high[axis_idx - 4];
}

// Mark one island's detached transaction invalid using deterministic integer reduction.
void CoupledPositionFailIsland(int island_idx, uint failure)
{
	if (island_idx >= 0 && island_idx < g_coupled_position.island_count)
	{
#ifdef __cplusplus
		g_coupled_position_island_failures[island_idx] |= failure;
#else
		InterlockedOr(g_coupled_position_island_failures[island_idx], failure);
#endif
	}
}

// Return the compact generalized width of one canonical participating range.
int CoupledPositionRangeVelocityCount(int range_idx)
{
	int begin = g_coupled_position_ranges[range_idx].velocity_delta_offset;
	int end = range_idx + 1 != g_coupled_position.articulation_range_count
		? g_coupled_position_ranges[range_idx + 1].velocity_delta_offset
		: g_coupled_position.velocity_delta_count;
	return end - begin;
}

// Return one gathered target's exact detached pseudo-velocity response for merit evaluation.
bool CoupledPositionTargetDeltaVelocity(int target_idx, int island_idx, out_(GpuArticulationSpatialVector) delta)
{
	delta = EmptyCoupledPositionSpatialVector();
	if (target_idx < 0)
		return true;
	if (target_idx >= g_coupled_position.target_count)
		return false;

	GpuCoupledConstraintTarget target = g_coupled_position_targets[target_idx];
	if (target.island_idx != island_idx)
		return false;
	if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
	{
		if (!CoupledPositionRigidDynamic(target.target_idx))
			return false;

		GpuArticulationSpatialVector impulse = g_coupled_position_target_impulses[target_idx];
		delta.ang.xyz = mul(CoupledPositionInverseInertia(target.target_idx), impulse.ang.xyz);
		delta.lin.xyz = g_coupled_position_bodies[target.target_idx].os_com_and_invmass.w * impulse.lin.xyz;
		return CoupledPositionSpatialFinite(delta);
	}
	if (target.target_type == GpuCoupledConstraintTargetType_Link)
	{
		if (target.target_idx < 0 || target.target_idx >= g_coupled_position.mobility_count)
			return false;

		delta = g_coupled_position_articulation_work[target.target_idx];
		return CoupledPositionSpatialFinite(delta);
	}
	return false;
}

// Clear pseudo state exactly once before the position-iteration sequence begins.
numthreads(CSClearCoupledPositionState, ConstraintThreadCount, 1, 1)
void CSClearCoupledPositionState(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index < g_coupled_position.body_count)
	{
		GpuConstraintPseudoVelocity pseudo;
		pseudo.angular_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
		pseudo.linear_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
		g_coupled_position_rigid_pseudo[index] = pseudo;
	}
	if (index < g_coupled_position.mobility_count)
		g_coupled_position_link_pseudo[index] = EmptyCoupledPositionSpatialVector();
	if (index < g_coupled_position.velocity_delta_count)
		g_coupled_position_generalized_pseudo[index] = 0.0f;
	if (index < g_coupled_position.slot_count)
	{
		for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		{
			int row_idx = index * GpuConstraintRowsPerBlock + axis_idx;
			GpuConstraintRow row = g_coupled_position_rows[row_idx];
			row.bounds.w = 0.0f;
			g_coupled_position_rows[row_idx] = row;
		}
	}
}

// Clear one iteration's detached candidate streams while preserving accumulated pseudo state.
numthreads(CSBeginCoupledPosition, ConstraintThreadCount, 1, 1)
void CSBeginCoupledPosition(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index < g_coupled_position.slot_count)
	{
		g_coupled_position_scratch[index] = EmptyCoupledPositionSolveScratch();
		g_coupled_position_contributions[2 * index + 0] = EmptyCoupledPositionSpatialVector();
		g_coupled_position_contributions[2 * index + 1] = EmptyCoupledPositionSpatialVector();
	}
	if (index < g_coupled_position.target_count)
		g_coupled_position_target_impulses[index] = EmptyCoupledPositionSpatialVector();
	if (index < g_coupled_position.island_count)
	{
		GpuCoupledConstraintIslandState state;
		state.status = GpuCoupledConstraintIslandStatus_Pending;
		state.failure_flags = GpuCoupledConstraintFailure_None;
		state.relaxation = g_coupled_position.relaxation;
		state.merit_change = 0.0f;
		g_coupled_position_island_states[index] = state;
		g_coupled_position_island_failures[index] = GpuCoupledConstraintFailure_None;
	}
	if (index < g_coupled_position.mobility_count)
		g_coupled_position_link_impulses[index] = EmptyCoupledPositionSpatialVector();
	if (index < g_coupled_position.articulation_range_count)
	{
		g_coupled_position_tree_selection[index] = 0u;
		g_coupled_position_tree_results[index] = 1u;
	}
}

// Build one projected exact-self pseudo candidate and two detached endpoint contributions.
numthreads(CSBuildCoupledPositionCandidates, ConstraintThreadCount, 1, 1)
void CSBuildCoupledPositionCandidates(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuCoupledConstraintBlockTopology topology = g_coupled_position_block_topology[slot_idx];
	if (topology.island_idx < 0 || topology.island_idx >= g_coupled_position.island_count)
		return;
	if (g_coupled_position_island_states[topology.island_idx].status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	GpuConstraintBlock block = g_coupled_position_blocks[slot_idx];
	uint active_axes[6];
	int row_count = 0;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		if ((block.position_mask & (1u << axis_idx)) != 0u)
			active_axes[row_count++] = axis_idx;
	if (row_count == 0)
		return;
	if (
		!AllSet(block.flags, ConstraintBlockFlags_Active) ||
		!AllSet(block.flags, ConstraintBlockFlags_CoupledPreconditionerValid))
	{
		CoupledPositionFailIsland(topology.island_idx, GpuCoupledConstraintFailure_Preconditioner);
		return;
	}

	GpuCoupledConstraintEndpoint endpoint = g_coupled_position_link_endpoints[slot_idx];
	GpuCoupledConstraintPreconditioner preconditioner = g_coupled_position_preconditioners[slot_idx];
	float residual[6];
	float delta[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_coupled_position_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		residual[row_idx] = CoupledPositionRowResidual(block, endpoint, row);
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += CoupledPreconditionerComponent(preconditioner, row_idx, column) * residual[column];

		int axis_idx = active_axes[row_idx];
		GpuConstraintRow row = g_coupled_position_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
		uint state = (block.row_states >> (2u * axis_idx)) & 3u;
		float2 bounds = ConstraintImpulseBounds(state, 3.402823466e+38f);
		float candidate = clamp(
			row.bounds.w - g_coupled_position_island_states[topology.island_idx].relaxation * correction,
			bounds.x,
			bounds.y);
		delta[row_idx] = candidate - row.bounds.w;
		if (!isfinite(residual[row_idx]) || !isfinite(correction) || !isfinite(delta[row_idx]) || !isfinite(candidate))
		{
			CoupledPositionFailIsland(topology.island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
	}

	// Publish only a complete finite block so deterministic gather never observes partial candidate state.
	GpuCoupledConstraintSolveScratch scratch = EmptyCoupledPositionSolveScratch();
	GpuArticulationSpatialVector impulse_a = EmptyCoupledPositionSpatialVector();
	GpuArticulationSpatialVector impulse_b = EmptyCoupledPositionSpatialVector();
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		int axis_idx = active_axes[row_idx];
		GpuConstraintRow row = g_coupled_position_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
		CoupledPositionSetSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx, delta[row_idx]);
		CoupledPositionSetSixVectorComponent(scratch.residual_before_low, scratch.residual_before_high, axis_idx, residual[row_idx]);
		impulse_a = CoupledPositionAddScaledWrench(impulse_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, delta[row_idx]);
		impulse_b = CoupledPositionAddScaledWrench(impulse_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, delta[row_idx]);
	}
	if (!CoupledPositionSpatialFinite(impulse_a) || !CoupledPositionSpatialFinite(impulse_b))
	{
		CoupledPositionFailIsland(topology.island_idx, GpuCoupledConstraintFailure_NonFinite);
		return;
	}

	g_coupled_position_scratch[slot_idx] = scratch;
	g_coupled_position_contributions[2 * slot_idx + 0] = impulse_a;
	g_coupled_position_contributions[2 * slot_idx + 1] = impulse_b;
}

// Gather every endpoint target in stable adjacency order and validate prospective rigid pseudo state.
numthreads(CSGatherCoupledPositionTargets, ConstraintThreadCount, 1, 1)
void CSGatherCoupledPositionTargets(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.target_count)
		return;

	int target_idx = dtid.x;
	GpuCoupledConstraintTarget target = g_coupled_position_targets[target_idx];
	if (
		target.island_idx < 0 || target.island_idx >= g_coupled_position.island_count ||
		target.adjacency_offset < 0 || target.adjacency_count < 1)
	{
		CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
		return;
	}
	if (g_coupled_position_island_states[target.island_idx].status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	GpuArticulationSpatialVector impulse = EmptyCoupledPositionSpatialVector();
	for (int adjacency_idx = 0; adjacency_idx != target.adjacency_count; ++adjacency_idx)
	{
		uint contribution_idx = g_coupled_position_adjacency[target.adjacency_offset + adjacency_idx];
		if (contribution_idx >= (uint)(2 * g_coupled_position.slot_count))
		{
			CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}

		impulse = CoupledPositionAddSpatial(impulse, g_coupled_position_contributions[contribution_idx]);
	}
	if (!CoupledPositionSpatialFinite(impulse))
	{
		CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_NonFinite);
		return;
	}

	g_coupled_position_target_impulses[target_idx] = impulse;
	if (target.target_type == GpuCoupledConstraintTargetType_Link)
	{
		if (target.target_idx < 0 || target.target_idx >= g_coupled_position.mobility_count)
		{
			CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}
		g_coupled_position_link_impulses[target.target_idx] = impulse;
	}
	else if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
	{
		if (!CoupledPositionRigidDynamic(target.target_idx))
		{
			CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}

		GpuConstraintPseudoVelocity current = g_coupled_position_rigid_pseudo[target.target_idx];
		float3 angular_velocity = current.angular_velocity.xyz + mul(CoupledPositionInverseInertia(target.target_idx), impulse.ang.xyz);
		float3 linear_velocity =
			current.linear_velocity.xyz +
			g_coupled_position_bodies[target.target_idx].os_com_and_invmass.w * impulse.lin.xyz;
		if (
			!CoupledPositionAngularDisplacementValid(angular_velocity) ||
			!CoupledPositionVectorFinite(linear_velocity))
		{
			CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
	}
	else
	{
		CoupledPositionFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
	}
}

// Select complete articulation ranges whose island is pending evaluation or accepted commit.
numthreads(CSSelectCoupledPositionTrees, ConstraintThreadCount, 1, 1)
void CSSelectCoupledPositionTrees(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.articulation_range_count)
		return;

	int island_idx = g_coupled_position_articulation_islands[dtid.x];
	if (island_idx < 0 || island_idx >= g_coupled_position.island_count)
	{
		g_coupled_position_tree_selection[dtid.x] = 0u;
		return;
	}

	uint required_status = g_coupled_position.phase == GpuCoupledConstraintPhase_Evaluate
		? GpuCoupledConstraintIslandStatus_Pending
		: GpuCoupledConstraintIslandStatus_Accepted;
	g_coupled_position_tree_selection[dtid.x] =
		g_coupled_position_island_states[island_idx].status == required_status ? 1u : 0u;
}

// Validate each selected complete-tree response against accumulated pseudo link and generalized state.
numthreads(CSValidateCoupledPositionTrees, ConstraintThreadCount, 1, 1)
void CSValidateCoupledPositionTrees(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.articulation_range_count)
		return;

	int range_idx = dtid.x;
	if (g_coupled_position_tree_selection[range_idx] == 0u)
		return;

	int island_idx = g_coupled_position_articulation_islands[range_idx];
	if (g_coupled_position_tree_results[range_idx] == 0u)
	{
		CoupledPositionFailIsland(island_idx, GpuCoupledConstraintFailure_Articulation);
		return;
	}

	GpuArticulationMobilityRange range = g_coupled_position_ranges[range_idx];
	int velocity_count = CoupledPositionRangeVelocityCount(range_idx);
	if (
		range.mobility_offset < 0 || range.link_count < 1 ||
		range.mobility_offset + range.link_count > g_coupled_position.mobility_count ||
		range.velocity_delta_offset < 0 || velocity_count < 0 ||
		range.velocity_delta_offset + velocity_count > g_coupled_position.velocity_delta_count)
	{
		CoupledPositionFailIsland(island_idx, GpuCoupledConstraintFailure_Topology);
		return;
	}

	for (int local_link_idx = 0; local_link_idx != range.link_count; ++local_link_idx)
	{
		int mobility_idx = range.mobility_offset + local_link_idx;
		GpuArticulationSpatialVector prospective = CoupledPositionAddSpatial(
			g_coupled_position_link_pseudo[mobility_idx],
			g_coupled_position_articulation_work[mobility_idx]);
		if (!CoupledPositionSpatialFinite(prospective))
		{
			CoupledPositionFailIsland(island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
	}
	for (int local_velocity_idx = 0; local_velocity_idx != velocity_count; ++local_velocity_idx)
	{
		int velocity_idx = range.velocity_delta_offset + local_velocity_idx;
		float prospective =
			g_coupled_position_generalized_pseudo[velocity_idx] +
			g_coupled_position_velocity_deltas[velocity_idx];
		if (!isfinite(prospective))
		{
			CoupledPositionFailIsland(island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
	}

	GpuArticulation articulation = g_coupled_position_articulations[range.articulation_index];
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		float3 root_angular_velocity = float3(
			g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 0] + g_coupled_position_velocity_deltas[range.velocity_delta_offset + 0],
			g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 1] + g_coupled_position_velocity_deltas[range.velocity_delta_offset + 1],
			g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 2] + g_coupled_position_velocity_deltas[range.velocity_delta_offset + 2]);
		if (velocity_count < 6 || !CoupledPositionAngularDisplacementValid(root_angular_velocity))
			CoupledPositionFailIsland(island_idx, GpuCoupledConstraintFailure_NonFinite);
	}
}

// Evaluate exact detached quadratic-merit change and accept, halve, or reject each island.
numthreads(CSEvaluateCoupledPositionMerit, ConstraintThreadCount, 1, 1)
void CSEvaluateCoupledPositionMerit(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.island_count)
		return;

	int island_idx = dtid.x;
	GpuCoupledConstraintIslandState state = g_coupled_position_island_states[island_idx];
	if (state.status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	state.failure_flags = g_coupled_position_island_failures[island_idx];
	if (state.failure_flags != GpuCoupledConstraintFailure_None)
	{
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
		g_coupled_position_island_states[island_idx] = state;
		return;
	}

	GpuCoupledConstraintIsland island = g_coupled_position_islands[island_idx];
	if (
		island.block_offset < 0 || island.block_count < 1 ||
		island.block_offset + island.block_count > g_coupled_position.island_block_count)
	{
		state.failure_flags |= GpuCoupledConstraintFailure_Topology;
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
		g_coupled_position_island_states[island_idx] = state;
		return;
	}

	float merit_change = 0.0f;
	for (int local_block_idx = 0; local_block_idx != island.block_count; ++local_block_idx)
	{
		uint slot_idx = g_coupled_position_island_blocks[island.block_offset + local_block_idx];
		if (slot_idx >= (uint)g_coupled_position.slot_count)
		{
			state.failure_flags |= GpuCoupledConstraintFailure_Topology;
			state.status = GpuCoupledConstraintIslandStatus_Rejected;
			g_coupled_position_island_states[island_idx] = state;
			return;
		}

		GpuCoupledConstraintBlockTopology topology = g_coupled_position_block_topology[slot_idx];
		GpuArticulationSpatialVector delta_a;
		GpuArticulationSpatialVector delta_b;
		if (
			topology.island_idx != island_idx ||
			!CoupledPositionTargetDeltaVelocity(topology.target_idx_a, island_idx, delta_a) ||
			!CoupledPositionTargetDeltaVelocity(topology.target_idx_b, island_idx, delta_b))
		{
			state.failure_flags |= GpuCoupledConstraintFailure_Topology;
			state.status = GpuCoupledConstraintIslandStatus_Rejected;
			g_coupled_position_island_states[island_idx] = state;
			return;
		}

		GpuConstraintBlock block = g_coupled_position_blocks[slot_idx];
		GpuCoupledConstraintSolveScratch scratch = g_coupled_position_scratch[slot_idx];
		for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		{
			if ((block.position_mask & (1u << axis_idx)) == 0u)
				continue;

			GpuConstraintRow row = g_coupled_position_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
			float impulse_delta = CoupledPositionSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx);
			float residual_before = CoupledPositionSixVectorComponent(scratch.residual_before_low, scratch.residual_before_high, axis_idx);
			float response =
				dot(row.jacobian_a_ang.xyz, delta_a.ang.xyz) +
				dot(row.jacobian_a_lin.xyz, delta_a.lin.xyz) +
				dot(row.jacobian_b_ang.xyz, delta_b.ang.xyz) +
				dot(row.jacobian_b_lin.xyz, delta_b.lin.xyz);
			float residual_after = residual_before + response;
			float row_merit_change =
				0.5f * impulse_delta * residual_before +
				0.5f * impulse_delta * residual_after;
			if (!isfinite(residual_after) || !isfinite(row_merit_change))
			{
				state.failure_flags |= GpuCoupledConstraintFailure_NonFinite;
				state.status = GpuCoupledConstraintIslandStatus_Rejected;
				g_coupled_position_island_states[island_idx] = state;
				return;
			}
			merit_change += row_merit_change;
		}
	}

	state.merit_change = merit_change;
	float tolerance = 64.0f * 1.192092896e-7f * (1.0f + abs(merit_change));
	if (isfinite(merit_change) && merit_change <= tolerance)
	{
		state.status = GpuCoupledConstraintIslandStatus_Accepted;
	}
	else if (g_coupled_position.attempt_index < g_coupled_position.backtrack_limit)
	{
		state.relaxation *= 0.5f;
	}
	else
	{
		state.failure_flags |= isfinite(merit_change)
			? GpuCoupledConstraintFailure_Merit
			: GpuCoupledConstraintFailure_NonFinite;
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
	}
	g_coupled_position_island_states[island_idx] = state;
}

// Accumulate accepted rigid and row deltas into detached pseudo state.
numthreads(CSCommitCoupledPositionState, ConstraintThreadCount, 1, 1)
void CSCommitCoupledPositionState(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index < g_coupled_position.target_count)
	{
		GpuCoupledConstraintTarget target = g_coupled_position_targets[index];
		if (
			target.target_type == GpuCoupledConstraintTargetType_Rigid &&
			CoupledPositionRigidDynamic(target.target_idx) &&
			target.island_idx >= 0 && target.island_idx < g_coupled_position.island_count &&
			g_coupled_position_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Accepted)
		{
			GpuArticulationSpatialVector impulse = g_coupled_position_target_impulses[index];
			GpuConstraintPseudoVelocity pseudo = g_coupled_position_rigid_pseudo[target.target_idx];
			pseudo.angular_velocity.xyz += mul(CoupledPositionInverseInertia(target.target_idx), impulse.ang.xyz);
			pseudo.linear_velocity.xyz += g_coupled_position_bodies[target.target_idx].os_com_and_invmass.w * impulse.lin.xyz;
			g_coupled_position_rigid_pseudo[target.target_idx] = pseudo;
		}
	}

	if (index < g_coupled_position.slot_count)
	{
		GpuCoupledConstraintBlockTopology topology = g_coupled_position_block_topology[index];
		if (
			topology.island_idx >= 0 && topology.island_idx < g_coupled_position.island_count &&
			g_coupled_position_island_states[topology.island_idx].status == GpuCoupledConstraintIslandStatus_Accepted)
		{
			GpuConstraintBlock block = g_coupled_position_blocks[index];
			GpuCoupledConstraintSolveScratch scratch = g_coupled_position_scratch[index];
			for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			{
				if ((block.position_mask & (1u << axis_idx)) == 0u)
					continue;

				int row_idx = index * GpuConstraintRowsPerBlock + axis_idx;
				GpuConstraintRow row = g_coupled_position_rows[row_idx];
				row.bounds.w += CoupledPositionSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx);
				g_coupled_position_rows[row_idx] = row;
			}
		}
	}

}

// Accumulate accepted complete-tree link and generalized deltas into detached pseudo state.
numthreads(CSCommitCoupledPositionArticulations, ConstraintThreadCount, 1, 1)
void CSCommitCoupledPositionArticulations(int3 DTID(dtid))
{
	if (dtid.x < g_coupled_position.articulation_range_count)
	{
		int range_idx = dtid.x;
		int island_idx = g_coupled_position_articulation_islands[range_idx];
		if (
			island_idx >= 0 && island_idx < g_coupled_position.island_count &&
			g_coupled_position_island_states[island_idx].status == GpuCoupledConstraintIslandStatus_Accepted &&
			g_coupled_position_tree_selection[range_idx] != 0u &&
			g_coupled_position_tree_results[range_idx] != 0u)
		{
			GpuArticulationMobilityRange range = g_coupled_position_ranges[range_idx];
			int velocity_count = CoupledPositionRangeVelocityCount(range_idx);
			for (int local_link_idx = 0; local_link_idx != range.link_count; ++local_link_idx)
			{
				int mobility_idx = range.mobility_offset + local_link_idx;
				g_coupled_position_link_pseudo[mobility_idx] = CoupledPositionAddSpatial(
					g_coupled_position_link_pseudo[mobility_idx],
					g_coupled_position_articulation_work[mobility_idx]);
			}
			for (int local_velocity_idx = 0; local_velocity_idx != velocity_count; ++local_velocity_idx)
			{
				int velocity_idx = range.velocity_delta_offset + local_velocity_idx;
				g_coupled_position_generalized_pseudo[velocity_idx] += g_coupled_position_velocity_deltas[velocity_idx];
			}
		}
	}
}

// Seal accepted islands so fixed later retry dispatches and commits cannot reapply them.
numthreads(CSFinalizeCoupledPositionIslands, ConstraintThreadCount, 1, 1)
void CSFinalizeCoupledPositionIslands(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_position.island_count)
		return;

	GpuCoupledConstraintIslandState state = g_coupled_position_island_states[dtid.x];
	if (state.status == GpuCoupledConstraintIslandStatus_Accepted)
		state.status = GpuCoupledConstraintIslandStatus_Committed;
	g_coupled_position_island_states[dtid.x] = state;
}

// Integrate a validated body-frame floating-root pseudo twist with midpoint translation.
GpuConstraintFrame CoupledPositionIntegrateRoot(GpuConstraintFrame root_to_world, int velocity_offset)
{
	float3 angular_velocity = float3(
		g_coupled_position_generalized_pseudo[velocity_offset + 0],
		g_coupled_position_generalized_pseudo[velocity_offset + 1],
		g_coupled_position_generalized_pseudo[velocity_offset + 2]);
	float3 linear_velocity = float3(
		g_coupled_position_generalized_pseudo[velocity_offset + 3],
		g_coupled_position_generalized_pseudo[velocity_offset + 4],
		g_coupled_position_generalized_pseudo[velocity_offset + 5]);
	float3 angular_displacement = g_coupled_position.timestep * angular_velocity;
	float4 half_rotation = quat_exp(0.25f * angular_displacement);
	float4 full_rotation = quat_exp(0.5f * angular_displacement);
	float4 midpoint_rotation = quat_mul(root_to_world.rotation, half_rotation);

	GpuConstraintFrame integrated = root_to_world;
	integrated.position = float4(
		root_to_world.position.xyz + quat_rotate(midpoint_rotation, g_coupled_position.timestep * linear_velocity),
		1.0f);
	integrated.rotation = normalize(quat_mul(root_to_world.rotation, full_rotation));
	return integrated;
}

// Integrate converged pseudo state exactly once into rigid transforms and articulation coordinates.
numthreads(CSApplyCoupledPosition, ConstraintThreadCount, 1, 1)
void CSApplyCoupledPosition(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index < g_coupled_position.body_count && CoupledPositionRigidDynamic(index))
	{
		GpuConstraintPseudoVelocity pseudo = g_coupled_position_rigid_pseudo[index];
		if (
			(CoupledPositionVectorNonZero(pseudo.angular_velocity.xyz) || CoupledPositionVectorNonZero(pseudo.linear_velocity.xyz)) &&
			CoupledPositionAngularDisplacementValid(pseudo.angular_velocity.xyz) &&
			CoupledPositionVectorFinite(pseudo.linear_velocity.xyz))
		{
			GpuRigidBody body = g_coupled_position_bodies[index];
			float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
			float3x3 rotation = (float3x3)body.o2w;
			float3x3 new_rotation = orthonorm3x3(mul(rotation, rodrigues_rotation(g_coupled_position.timestep * pseudo.angular_velocity.xyz)));
			float3 new_com_ws = com_ws + g_coupled_position.timestep * pseudo.linear_velocity.xyz;
			float3 new_position = new_com_ws - mul(body.os_com_and_invmass.xyz, new_rotation);
			body.o2w = float4x4(
				float4(new_rotation[0], 0),
				float4(new_rotation[1], 0),
				float4(new_rotation[2], 0),
				float4(new_position, 1));
			g_coupled_position_bodies[index] = body;
		}
	}

	if (index >= g_coupled_position.articulation_range_count)
		return;

	GpuArticulationMobilityRange range = g_coupled_position_ranges[index];
	GpuArticulation articulation = g_coupled_position_articulations[range.articulation_index];
	int velocity_count = CoupledPositionRangeVelocityCount(index);
	bool has_correction = false;
	for (int local_velocity_idx = 0; local_velocity_idx != velocity_count; ++local_velocity_idx)
		has_correction = has_correction || g_coupled_position_generalized_pseudo[range.velocity_delta_offset + local_velocity_idx] != 0.0f;
	if (!has_correction)
		return;

	GpuConstraintFrame root_to_world = articulation.root_to_world;
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		if (
			velocity_count < 6 ||
			!CoupledPositionAngularDisplacementValid(float3(
				g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 0],
				g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 1],
				g_coupled_position_generalized_pseudo[range.velocity_delta_offset + 2])))
			return;

		root_to_world = CoupledPositionIntegrateRoot(root_to_world, range.velocity_delta_offset);
		if (
			!isfinite(root_to_world.rotation.x) || !isfinite(root_to_world.rotation.y) ||
			!isfinite(root_to_world.rotation.z) || !isfinite(root_to_world.rotation.w) ||
			!CoupledPositionVectorFinite(root_to_world.position.xyz))
			return;
	}

	// Validate every reduced coordinate before writing any part of this articulation.
	for (int local_link_idx = 1; local_link_idx != articulation.link_count; ++local_link_idx)
	{
		GpuArticulationLink link = g_coupled_position_links[articulation.link_offset + local_link_idx];
		for (int row = 0; row != link.dof_count; ++row)
		{
			int local_velocity_idx = link.velocity_offset - articulation.velocity_offset + row;
			float position =
				g_coupled_position_positions[link.position_offset + row] +
				g_coupled_position.timestep * g_coupled_position_generalized_pseudo[range.velocity_delta_offset + local_velocity_idx];
			if (!isfinite(position))
				return;
		}
	}

	for (int local_link_idx = 1; local_link_idx != articulation.link_count; ++local_link_idx)
	{
		GpuArticulationLink link = g_coupled_position_links[articulation.link_offset + local_link_idx];
		for (int row = 0; row != link.dof_count; ++row)
		{
			int local_velocity_idx = link.velocity_offset - articulation.velocity_offset + row;
			g_coupled_position_positions[link.position_offset + row] +=
				g_coupled_position.timestep * g_coupled_position_generalized_pseudo[range.velocity_delta_offset + local_velocity_idx];
		}
	}
	articulation.root_to_world = root_to_world;
	g_coupled_position_articulations[range.articulation_index] = articulation;
}

#ifdef __cplusplus
}
#endif
