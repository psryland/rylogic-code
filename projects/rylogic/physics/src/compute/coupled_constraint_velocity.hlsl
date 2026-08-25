//*********************************************
// Physics Engine — Coupled Constraint Velocity Sweep
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Builds one simultaneous block-Jacobi candidate, gathers endpoint impulses deterministically,
// validates articulation responses without mutation, and commits only complete valid islands.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Packed bounds and mode shared by every phase of one transactional coupled velocity sweep.
struct cbCoupledConstraintVelocity
{
	int slot_count;
	int body_count;
	int target_count;
	int adjacency_count;

	int island_count;
	int articulation_range_count;
	int mobility_count;
	int work_count;

	int island_block_count;
	int selection_mode;
	int attempt_index;
	int backtrack_limit;

	float relaxation;
	float pad0;
	float pad1;
	float pad2;
};

ConstantBuffer<cbCoupledConstraintVelocity> resource(g_coupled_velocity, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_coupled_velocity_bodies, u0);
StructuredBuffer<GpuConstraintEndpoint> resource(g_coupled_velocity_endpoints, t0);
StructuredBuffer<GpuCoupledConstraintEndpoint> resource(g_coupled_velocity_link_endpoints, t1);
StructuredBuffer<GpuCoupledConstraintBlockTopology> resource(g_coupled_velocity_block_topology, t2);
StructuredBuffer<GpuCoupledConstraintTarget> resource(g_coupled_velocity_targets, t3);
StructuredBuffer<uint> resource(g_coupled_velocity_adjacency, t4);
StructuredBuffer<GpuCoupledConstraintPreconditioner> resource(g_coupled_velocity_preconditioners, t5);
StructuredBuffer<GpuArticulationAbaScratch> resource(g_coupled_velocity_aba_scratch, t6);
StructuredBuffer<int> resource(g_coupled_velocity_articulation_islands, t7);
StructuredBuffer<GpuCoupledConstraintIsland> resource(g_coupled_velocity_islands, t8);
StructuredBuffer<uint> resource(g_coupled_velocity_island_blocks, t9);
RWStructuredBuffer<GpuConstraintBlock> resource(g_coupled_velocity_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_coupled_velocity_rows, u2);
RWStructuredBuffer<GpuCoupledConstraintSolveScratch> resource(g_coupled_velocity_scratch, u3);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_velocity_contributions, u4);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_velocity_target_impulses, u5);
RWStructuredBuffer<GpuCoupledConstraintIslandState> resource(g_coupled_velocity_island_states, u6);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_velocity_link_impulses, u7);
RWStructuredBuffer<uint> resource(g_coupled_velocity_tree_selection, u8);
RWStructuredBuffer<uint> resource(g_coupled_velocity_tree_results, u9);
RWStructuredBuffer<uint> resource(g_coupled_velocity_island_failures, u10);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_velocity_articulation_work, u11);

#define PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE coupled_constraint_velocity_ops_detail
#include "physics/src/compute/constraint_solver_ops.hlsli"
#undef PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE

// Return an explicitly zeroed spatial vector for both shader and deterministic replay builds.
GpuArticulationSpatialVector EmptyCoupledSpatialVector()
{
	GpuArticulationSpatialVector value;
	value.ang = float4(0.0f, 0.0f, 0.0f, 0.0f);
	value.lin = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return value;
}

// Return an explicitly zeroed six-row candidate scratch value.
GpuCoupledConstraintSolveScratch EmptyCoupledSolveScratch()
{
	GpuCoupledConstraintSolveScratch scratch;
	scratch.impulse_delta_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.impulse_delta_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.residual_before_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.residual_before_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return scratch;
}

// Return true when both halves of one spatial value are finite.
bool CoupledSpatialFinite(GpuArticulationSpatialVector value)
{
	return
		isfinite(value.ang.x) && isfinite(value.ang.y) && isfinite(value.ang.z) &&
		isfinite(value.lin.x) && isfinite(value.lin.y) && isfinite(value.lin.z);
}

// Return true when all three physical components of one vector are finite.
bool CoupledVectorFinite(float3 value)
{
	return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

// Add one scaled row wrench to a detached endpoint impulse.
GpuArticulationSpatialVector CoupledAddScaledWrench(
	GpuArticulationSpatialVector impulse,
	float3 jacobian_ang,
	float3 jacobian_lin,
	float scale)
{
	impulse.ang.xyz += scale * jacobian_ang;
	impulse.lin.xyz += scale * jacobian_lin;
	return impulse;
}

// Return true when one rigid endpoint is dynamic and can exchange momentum with its island.
bool CoupledVelocityRigidDynamic(int body_idx)
{
	return
		body_idx >= 0 &&
		body_idx < g_coupled_velocity.body_count &&
		g_coupled_velocity_bodies[body_idx].os_com_and_invmass.w > 0.0f &&
		!AllSet(g_coupled_velocity_bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
}

// Return one dynamic rigid body's world-space inverse inertia at its centre of mass.
float3x3 CoupledVelocityInverseInertia(int body_idx)
{
	GpuRigidBody body = g_coupled_velocity_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	return rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
}

// Return one endpoint's current contribution to a scalar row velocity.
float CoupledEndpointRowVelocity(
	int body_idx,
	int link_idx,
	float3 jacobian_ang,
	float3 jacobian_lin)
{
	if (link_idx >= 0)
	{
		GpuArticulationSpatialVector velocity = g_coupled_velocity_aba_scratch[link_idx].link_velocity;
		return dot(jacobian_ang, velocity.ang.xyz) + dot(jacobian_lin, velocity.lin.xyz);
	}
	if (!CoupledVelocityRigidDynamic(body_idx))
		return 0.0f;

	GpuRigidBody body = g_coupled_velocity_bodies[body_idx];
	float3 angular_velocity = mul(CoupledVelocityInverseInertia(body_idx), body.momentum_ang.xyz);
	float3 linear_velocity = body.os_com_and_invmass.w * body.momentum_lin.xyz;
	return dot(jacobian_ang, angular_velocity) + dot(jacobian_lin, linear_velocity);
}

// Return one gathered target's exact detached velocity response for island-merit evaluation.
bool CoupledTargetDeltaVelocity(int target_idx, int island_idx, out_(GpuArticulationSpatialVector) delta)
{
	delta = EmptyCoupledSpatialVector();
	if (target_idx < 0)
		return true;
	if (target_idx >= g_coupled_velocity.target_count)
		return false;

	GpuCoupledConstraintTarget target = g_coupled_velocity_targets[target_idx];
	if (target.island_idx != island_idx)
		return false;
	if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
	{
		if (!CoupledVelocityRigidDynamic(target.target_idx))
			return false;

		GpuArticulationSpatialVector impulse = g_coupled_velocity_target_impulses[target_idx];
		delta.ang.xyz = mul(CoupledVelocityInverseInertia(target.target_idx), impulse.ang.xyz);
		delta.lin.xyz = g_coupled_velocity_bodies[target.target_idx].os_com_and_invmass.w * impulse.lin.xyz;
		return CoupledSpatialFinite(delta);
	}
	if (target.target_type == GpuCoupledConstraintTargetType_Link)
	{
		if (target.target_idx < 0 || target.target_idx >= g_coupled_velocity.mobility_count)
			return false;

		delta = g_coupled_velocity_articulation_work[target.target_idx];
		return CoupledSpatialFinite(delta);
	}
	return false;
}

// Return the current residual of one coupled scalar velocity row.
float CoupledRowResidual(GpuConstraintBlock block, GpuCoupledConstraintEndpoint endpoint, GpuConstraintRow row)
{
	float velocity =
		CoupledEndpointRowVelocity(block.body_idx_a, endpoint.link_idx_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz) +
		CoupledEndpointRowVelocity(block.body_idx_b, endpoint.link_idx_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz);
	return velocity - row.solve.y + row.solve.z + row.solve.w * row.bounds.z;
}

// Store one canonical axis value in the split low/high candidate layout.
void CoupledSetSixVectorComponent(inout_(float4) low, inout_(float4) high, int axis_idx, float value)
{
	if (axis_idx < 4)
		low[axis_idx] = value;
	else
		high[axis_idx - 4] = value;
}

// Return one canonical axis value from the split low/high candidate layout.
float CoupledSixVectorComponent(float4 low, float4 high, int axis_idx)
{
	return axis_idx < 4 ? low[axis_idx] : high[axis_idx - 4];
}

// Mark one island's detached transaction invalid using deterministic integer reduction.
void CoupledFailIsland(int island_idx, uint failure)
{
	if (island_idx >= 0 && island_idx < g_coupled_velocity.island_count)
	{
#ifdef __cplusplus
		g_coupled_velocity_island_failures[island_idx] |= failure;
#else
		InterlockedOr(g_coupled_velocity_island_failures[island_idx], failure);
#endif
	}
}

// Clear all per-sweep detached state while preserving persistent constraint rows and articulation factors.
numthreads(CSBeginCoupledVelocity, ConstraintThreadCount, 1, 1)
void CSBeginCoupledVelocity(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index >= g_coupled_velocity.work_count)
		return;

	if (index < g_coupled_velocity.slot_count)
	{
		g_coupled_velocity_scratch[index] = EmptyCoupledSolveScratch();
		g_coupled_velocity_contributions[2 * index + 0] = EmptyCoupledSpatialVector();
		g_coupled_velocity_contributions[2 * index + 1] = EmptyCoupledSpatialVector();
	}
	if (index < g_coupled_velocity.target_count)
		g_coupled_velocity_target_impulses[index] = EmptyCoupledSpatialVector();
	if (index < g_coupled_velocity.island_count)
	{
		GpuCoupledConstraintIslandState state;
		state.status = GpuCoupledConstraintIslandStatus_Pending;
		state.failure_flags = GpuCoupledConstraintFailure_None;
		state.relaxation = g_coupled_velocity.relaxation;
		state.merit_change = 0.0f;
		g_coupled_velocity_island_states[index] = state;
		g_coupled_velocity_island_failures[index] = GpuCoupledConstraintFailure_None;
	}
	if (index < g_coupled_velocity.mobility_count)
		g_coupled_velocity_link_impulses[index] = EmptyCoupledSpatialVector();
	if (index < g_coupled_velocity.articulation_range_count)
	{
		g_coupled_velocity_tree_selection[index] = 0u;
		g_coupled_velocity_tree_results[index] = 1u;
	}
}

// Build one projected exact-self candidate and two detached endpoint contributions per active coupled block.
numthreads(CSBuildCoupledVelocityCandidates, ConstraintThreadCount, 1, 1)
void CSBuildCoupledVelocityCandidates(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuCoupledConstraintBlockTopology topology = g_coupled_velocity_block_topology[slot_idx];
	if (topology.island_idx < 0 || topology.island_idx >= g_coupled_velocity.island_count)
		return;
	if (g_coupled_velocity_island_states[topology.island_idx].status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	GpuConstraintEndpoint packed_endpoint = g_coupled_velocity_endpoints[slot_idx];
	GpuConstraintBlock block = g_coupled_velocity_blocks[slot_idx];
	if (
		!AllSet(packed_endpoint.flags, GpuConstraintEndpointFlags_Enabled) ||
		!AllSet(packed_endpoint.flags, GpuConstraintEndpointFlags_Coupled))
	{
		CoupledFailIsland(topology.island_idx, GpuCoupledConstraintFailure_Topology);
		return;
	}

	uint active_axes[6];
	int row_count = 0;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		if ((block.velocity_mask & (1u << axis_idx)) != 0u)
			active_axes[row_count++] = axis_idx;
	if (row_count == 0)
		return;
	if (
		!AllSet(block.flags, ConstraintBlockFlags_Active) ||
		!AllSet(block.flags, ConstraintBlockFlags_CoupledPreconditionerValid))
	{
		CoupledFailIsland(topology.island_idx, GpuCoupledConstraintFailure_Preconditioner);
		return;
	}

	GpuCoupledConstraintEndpoint endpoint = g_coupled_velocity_link_endpoints[slot_idx];
	GpuCoupledConstraintPreconditioner preconditioner = g_coupled_velocity_preconditioners[slot_idx];
	float residual[6];
	float delta[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_coupled_velocity_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		residual[row_idx] = CoupledRowResidual(block, endpoint, row);
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += CoupledPreconditionerComponent(preconditioner, row_idx, column) * residual[column];
		if (!isfinite(correction))
		{
			CoupledFailIsland(topology.island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}

		GpuConstraintRow row = g_coupled_velocity_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		float candidate = clamp(
			row.bounds.z - g_coupled_velocity_island_states[topology.island_idx].relaxation * correction,
			row.bounds.x,
			row.bounds.y);
		delta[row_idx] = candidate - row.bounds.z;
		if (!isfinite(residual[row_idx]) || !isfinite(delta[row_idx]) || !isfinite(candidate))
		{
			CoupledFailIsland(topology.island_idx, GpuCoupledConstraintFailure_NonFinite);
			return;
		}
	}

	// Publish only a complete finite block candidate so gather can never observe partially initialized values.
	GpuCoupledConstraintSolveScratch scratch = EmptyCoupledSolveScratch();
	GpuArticulationSpatialVector impulse_a = EmptyCoupledSpatialVector();
	GpuArticulationSpatialVector impulse_b = EmptyCoupledSpatialVector();
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		int axis_idx = active_axes[row_idx];
		GpuConstraintRow row = g_coupled_velocity_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
		CoupledSetSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx, delta[row_idx]);
		CoupledSetSixVectorComponent(scratch.residual_before_low, scratch.residual_before_high, axis_idx, residual[row_idx]);
		impulse_a = CoupledAddScaledWrench(impulse_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, delta[row_idx]);
		impulse_b = CoupledAddScaledWrench(impulse_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, delta[row_idx]);
	}
	if (!CoupledSpatialFinite(impulse_a) || !CoupledSpatialFinite(impulse_b))
	{
		CoupledFailIsland(topology.island_idx, GpuCoupledConstraintFailure_NonFinite);
		return;
	}

	g_coupled_velocity_scratch[slot_idx] = scratch;
	g_coupled_velocity_contributions[2 * slot_idx + 0] = impulse_a;
	g_coupled_velocity_contributions[2 * slot_idx + 1] = impulse_b;
}

// Gather every target's detached impulse in stable adjacency order without floating-point atomics.
numthreads(CSGatherCoupledVelocityTargets, ConstraintThreadCount, 1, 1)
void CSGatherCoupledVelocityTargets(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.target_count)
		return;

	int target_idx = dtid.x;
	GpuCoupledConstraintTarget target = g_coupled_velocity_targets[target_idx];
	if (
		target.island_idx < 0 || target.island_idx >= g_coupled_velocity.island_count ||
		target.adjacency_offset < 0 || target.adjacency_count < 1 ||
		target.adjacency_offset + target.adjacency_count > g_coupled_velocity.adjacency_count)
	{
		CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
		return;
	}
	if (g_coupled_velocity_island_states[target.island_idx].status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	GpuArticulationSpatialVector impulse = EmptyCoupledSpatialVector();
	for (int adjacency_idx = 0; adjacency_idx != target.adjacency_count; ++adjacency_idx)
	{
		uint contribution_idx = g_coupled_velocity_adjacency[target.adjacency_offset + adjacency_idx];
		if (contribution_idx >= (uint)(2 * g_coupled_velocity.slot_count))
		{
			CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}

		GpuArticulationSpatialVector contribution = g_coupled_velocity_contributions[contribution_idx];
		impulse.ang += contribution.ang;
		impulse.lin += contribution.lin;
	}
	if (!CoupledSpatialFinite(impulse))
	{
		CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_NonFinite);
		return;
	}

	g_coupled_velocity_target_impulses[target_idx] = impulse;
	if (target.target_type == GpuCoupledConstraintTargetType_Link)
	{
		if (target.target_idx < 0 || target.target_idx >= g_coupled_velocity.mobility_count)
		{
			CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}
		g_coupled_velocity_link_impulses[target.target_idx] = impulse;
	}
	else if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
	{
		if (target.target_idx < 0 || target.target_idx >= g_coupled_velocity.body_count)
		{
			CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
			return;
		}

		// Reject before acceptance if finite impulses would overflow authoritative momentum or its derived velocity.
		if (CoupledVelocityRigidDynamic(target.target_idx))
		{
			GpuRigidBody body = g_coupled_velocity_bodies[target.target_idx];
			float3 momentum_ang = body.momentum_ang.xyz + impulse.ang.xyz;
			float3 momentum_lin = body.momentum_lin.xyz + impulse.lin.xyz;
			float3 velocity_ang = mul(CoupledVelocityInverseInertia(target.target_idx), momentum_ang);
			float3 velocity_lin = body.os_com_and_invmass.w * momentum_lin;
			if (
				!CoupledVectorFinite(momentum_ang) ||
				!CoupledVectorFinite(momentum_lin) ||
				!CoupledVectorFinite(velocity_ang) ||
				!CoupledVectorFinite(velocity_lin))
			{
				CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_NonFinite);
				return;
			}
		}
	}
	else
	{
		CoupledFailIsland(target.island_idx, GpuCoupledConstraintFailure_Topology);
	}
}

// Select articulation ranges whose owning island is pending evaluation or accepted commit.
numthreads(CSSelectCoupledVelocityTrees, ConstraintThreadCount, 1, 1)
void CSSelectCoupledVelocityTrees(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.articulation_range_count)
		return;

	int island_idx = g_coupled_velocity_articulation_islands[dtid.x];
	if (island_idx < 0 || island_idx >= g_coupled_velocity.island_count)
	{
		g_coupled_velocity_tree_selection[dtid.x] = 0u;
		return;
	}

	uint required_status = g_coupled_velocity.selection_mode == 0
		? GpuCoupledConstraintIslandStatus_Pending
		: GpuCoupledConstraintIslandStatus_Accepted;
	g_coupled_velocity_tree_selection[dtid.x] =
		g_coupled_velocity_island_states[island_idx].status == required_status ? 1u : 0u;
}

// Fold detached articulation evaluation failures into their owning island's integer status.
numthreads(CSValidateCoupledVelocityTrees, ConstraintThreadCount, 1, 1)
void CSValidateCoupledVelocityTrees(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.articulation_range_count)
		return;
	if (g_coupled_velocity_tree_selection[dtid.x] == 0u || g_coupled_velocity_tree_results[dtid.x] != 0u)
		return;

	int island_idx = g_coupled_velocity_articulation_islands[dtid.x];
	CoupledFailIsland(island_idx, GpuCoupledConstraintFailure_Articulation);
}

// Evaluate exact detached quadratic-merit change and either accept, halve relaxation, or reject each island.
numthreads(CSEvaluateCoupledVelocityMerit, ConstraintThreadCount, 1, 1)
void CSEvaluateCoupledVelocityMerit(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.island_count)
		return;

	int island_idx = dtid.x;
	GpuCoupledConstraintIslandState state = g_coupled_velocity_island_states[island_idx];
	if (state.status != GpuCoupledConstraintIslandStatus_Pending)
		return;

	state.failure_flags = g_coupled_velocity_island_failures[island_idx];
	if (state.failure_flags != GpuCoupledConstraintFailure_None)
	{
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
		g_coupled_velocity_island_states[island_idx] = state;
		return;
	}

	GpuCoupledConstraintIsland island = g_coupled_velocity_islands[island_idx];
	if (
		island.block_offset < 0 ||
		island.block_count < 1 ||
		island.block_offset + island.block_count > g_coupled_velocity.island_block_count)
	{
		state.failure_flags |= GpuCoupledConstraintFailure_Topology;
		state.status = GpuCoupledConstraintIslandStatus_Rejected;
		g_coupled_velocity_island_states[island_idx] = state;
		return;
	}

	// Stable island-block and canonical row order makes the floating-point reduction deterministic.
	float merit_change = 0.0f;
	for (int local_block_idx = 0; local_block_idx != island.block_count; ++local_block_idx)
	{
		uint slot_idx = g_coupled_velocity_island_blocks[island.block_offset + local_block_idx];
		if (slot_idx >= (uint)g_coupled_velocity.slot_count)
		{
			state.failure_flags |= GpuCoupledConstraintFailure_Topology;
			state.status = GpuCoupledConstraintIslandStatus_Rejected;
			g_coupled_velocity_island_states[island_idx] = state;
			return;
		}

		GpuCoupledConstraintBlockTopology topology = g_coupled_velocity_block_topology[slot_idx];
		if (topology.island_idx != island_idx)
		{
			state.failure_flags |= GpuCoupledConstraintFailure_Topology;
			state.status = GpuCoupledConstraintIslandStatus_Rejected;
			g_coupled_velocity_island_states[island_idx] = state;
			return;
		}

		GpuArticulationSpatialVector delta_a;
		GpuArticulationSpatialVector delta_b;
		if (
			!CoupledTargetDeltaVelocity(topology.target_idx_a, island_idx, delta_a) ||
			!CoupledTargetDeltaVelocity(topology.target_idx_b, island_idx, delta_b))
		{
			state.failure_flags |= GpuCoupledConstraintFailure_Topology;
			state.status = GpuCoupledConstraintIslandStatus_Rejected;
			g_coupled_velocity_island_states[island_idx] = state;
			return;
		}

		GpuConstraintBlock block = g_coupled_velocity_blocks[slot_idx];
		GpuCoupledConstraintSolveScratch scratch = g_coupled_velocity_scratch[slot_idx];
		for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		{
			if ((block.velocity_mask & (1u << axis_idx)) == 0u)
				continue;

			GpuConstraintRow row = g_coupled_velocity_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
			float impulse_delta = CoupledSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx);
			float residual_before = CoupledSixVectorComponent(scratch.residual_before_low, scratch.residual_before_high, axis_idx);
			float response =
				dot(row.jacobian_a_ang.xyz, delta_a.ang.xyz) +
				dot(row.jacobian_a_lin.xyz, delta_a.lin.xyz) +
				dot(row.jacobian_b_ang.xyz, delta_b.ang.xyz) +
				dot(row.jacobian_b_lin.xyz, delta_b.lin.xyz);
			float residual_after = residual_before + response + row.solve.w * impulse_delta;
			float row_merit_change = 0.5f * impulse_delta * (residual_before + residual_after);
			if (!isfinite(residual_after) || !isfinite(row_merit_change))
			{
				state.failure_flags |= GpuCoupledConstraintFailure_NonFinite;
				state.status = GpuCoupledConstraintIslandStatus_Rejected;
				g_coupled_velocity_island_states[island_idx] = state;
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
	else if (g_coupled_velocity.attempt_index < g_coupled_velocity.backtrack_limit)
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
	g_coupled_velocity_island_states[island_idx] = state;
}

// Commit accepted rigid impulses and accumulated row impulses while articulation state commits through the impulse-ABA pass.
numthreads(CSCommitCoupledVelocity, ConstraintThreadCount, 1, 1)
void CSCommitCoupledVelocity(int3 DTID(dtid))
{
	int index = dtid.x;
	if (index >= g_coupled_velocity.work_count)
		return;

	if (index < g_coupled_velocity.target_count)
	{
		GpuCoupledConstraintTarget target = g_coupled_velocity_targets[index];
		if (
			target.target_type == GpuCoupledConstraintTargetType_Rigid &&
			target.target_idx >= 0 && target.target_idx < g_coupled_velocity.body_count &&
			target.island_idx >= 0 && target.island_idx < g_coupled_velocity.island_count &&
			g_coupled_velocity_island_states[target.island_idx].status == GpuCoupledConstraintIslandStatus_Accepted)
		{
			GpuArticulationSpatialVector impulse = g_coupled_velocity_target_impulses[index];
			if (
				CoupledSpatialFinite(impulse) &&
				CoupledVelocityRigidDynamic(target.target_idx) &&
				(any(impulse.ang.xyz != float3(0.0f, 0.0f, 0.0f)) || any(impulse.lin.xyz != float3(0.0f, 0.0f, 0.0f))))
			{
				GpuRigidBody body = g_coupled_velocity_bodies[target.target_idx];
				body.momentum_ang += impulse.ang;
				body.momentum_lin += impulse.lin;
				if (AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) || body.sleep.island_id >= 0)
				{
					body.sleep.generation++;
					body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
					body.sleep.timer_s = 0.0f;
					body.sleep.island_id = -1;
					body.sleep.flags = 0u;
				}
				g_coupled_velocity_bodies[target.target_idx] = body;
			}
		}
	}

	if (index < g_coupled_velocity.slot_count)
	{
		GpuCoupledConstraintBlockTopology topology = g_coupled_velocity_block_topology[index];
		if (
			topology.island_idx >= 0 && topology.island_idx < g_coupled_velocity.island_count &&
			g_coupled_velocity_island_states[topology.island_idx].status == GpuCoupledConstraintIslandStatus_Accepted)
		{
			GpuConstraintBlock block = g_coupled_velocity_blocks[index];
			GpuCoupledConstraintSolveScratch scratch = g_coupled_velocity_scratch[index];
			for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			{
				if ((block.velocity_mask & (1u << axis_idx)) == 0u)
					continue;

				int row_idx = index * GpuConstraintRowsPerBlock + axis_idx;
				GpuConstraintRow row = g_coupled_velocity_rows[row_idx];
				row.bounds.z += CoupledSixVectorComponent(scratch.impulse_delta_low, scratch.impulse_delta_high, axis_idx);
				g_coupled_velocity_rows[row_idx] = row;
			}
		}
	}
}

// Seal accepted islands after all rigid, row, and articulation commits so later fixed retry passes cannot reapply them.
numthreads(CSFinalizeCoupledVelocityIslands, ConstraintThreadCount, 1, 1)
void CSFinalizeCoupledVelocityIslands(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_velocity.island_count)
		return;

	GpuCoupledConstraintIslandState state = g_coupled_velocity_island_states[dtid.x];
	if (state.status == GpuCoupledConstraintIslandStatus_Accepted)
		state.status = GpuCoupledConstraintIslandStatus_Committed;
	g_coupled_velocity_island_states[dtid.x] = state;
}

#ifdef __cplusplus
}
#endif
