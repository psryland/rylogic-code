//*********************************************
// Physics Engine — Persistent D6 Constraint Solver
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU pass order:
//   1. CSCompileConstraints
//   2. CSResetConstraintColours, CSAssignConstraintColourLevel once per schedule level, then CSFinishConstraintColours
//   3. CSApplyConstraintWarmStart, once per colour
//   4. CSClearConstraintPseudoVelocity
//   5. CSPrepareConstraintPositionSolve, once before the position iterations
//   6. CSSolveConstraintPosition, fixed iterations and once per colour
//   7. CSApplyConstraintPosition
//   8. CSPrepareConstraintVelocitySolve, once after every transform change and before the velocity iterations
//   9. CSSolveConstraintVelocity, fixed iterations and once per colour
//
// Resource layout:
//   b0: cbConstraintSolver                         - per-dispatch constants
//   u0: RWStructuredBuffer<GpuRigidBody>           - integrated body state
//   t0: StructuredBuffer<GpuConstraintEndpoint>    - stable-slot endpoint metadata
//   t1: StructuredBuffer<GpuD6ConstraintDesc>      - stable-slot D6 parameters
//   u1: RWStructuredBuffer<GpuConstraintBlock>     - stable-slot runtime blocks
//   u2: RWStructuredBuffer<GpuConstraintRow>       - six canonical rows per stable slot
//   u3: RWStructuredBuffer<GpuConstraintSolverState> - frame-local colour fallback and numerical-failure latch
//   u4: RWStructuredBuffer<GpuConstraintPseudoVelocity> - per-body split-correction state
//   u5: RWStructuredBuffer<GpuConstraintBreakState> - optional stable-slot overload latches
//   u6: RWStructuredBuffer<DispatchArguments>      - per-colour indirect arguments written by CSFinishConstraintColours
//   u7: RWStructuredBuffer<GpuConstraintSolveCache> - per-slot block response inverse for the current solve phase
//   t2: StructuredBuffer<GpuConstraintColourTask>  - colouring schedule, ordered by dependency level
//   u8: RWStructuredBuffer<uint2>                  - per-slot body colour masks after each slot is coloured (endpoint A, endpoint B)
//
// Matrices use the row-vector convention from resolve.hlsl. HLSL rows map to C++ columns,
// mul(v, M) transforms a vector, and mul(local, parent) composes local-to-parent-to-world.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/body_twist.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Per-dispatch controls mirrored by the CPU replay and D3D12 host runners.
struct cbConstraintSolver
{
	int slot_count;
	int body_count;
	int colour;
	int position_iterations;

	float timestep;
	float relaxation;
	float position_relaxation;
	float position_beta;
	float max_position_speed;
	float regularization;
	float warm_start_factor;
	float warm_start_scale;
	int retain_current_impulses;
	int breakable_count;
	int break_substep_index;
	int schedule_begin;
	int schedule_count;
	int pad0;
	int pad1;
	int pad2;
};

ConstantBuffer<cbConstraintSolver> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_constraint_bodies, u0);
StructuredBuffer<GpuConstraintEndpoint> resource(g_endpoints, t0);
StructuredBuffer<GpuD6ConstraintDesc> resource(g_descriptors, t1);
RWStructuredBuffer<GpuConstraintBlock> resource(g_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_rows, u2);
RWStructuredBuffer<GpuConstraintSolverState> resource(g_constraint_state, u3);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_pseudo_velocities, u4);
RWStructuredBuffer<GpuConstraintBreakState> resource(g_break_states, u5);
RWStructuredBuffer<DispatchArguments> resource(g_constraint_colour_dispatch, u6);
RWStructuredBuffer<GpuConstraintSolveCache> resource(g_solve_cache, u7);
StructuredBuffer<GpuConstraintColourTask> resource(g_colour_schedule, t2);
RWStructuredBuffer<uint2> resource(g_colour_masks, u8);

#define PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE constraint_solver_ops_detail
#include "physics/src/compute/constraint_solver_ops.hlsli"
#undef PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE

// Return true when a packed body is dynamic and may be mutated by a solve pass.
bool ConstraintBodyDynamic(int body_idx)
{
	return
		body_idx >= 0 &&
		body_idx < g.body_count &&
		g_constraint_bodies[body_idx].os_com_and_invmass.w > 0.0f &&
		!AllSet(g_constraint_bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
}

// Return one body rotation in the row-vector convention.
float3x3 ConstraintBodyRotation(int body_idx)
{
	return (float3x3)g_constraint_bodies[body_idx].o2w;
}

// Return the world-space inverse inertia at a body's centre of mass.
float3x3 ConstraintInverseInertia(int body_idx)
{
	GpuRigidBody body = g_constraint_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	return rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
}

// Return the world-space centre of mass while preserving off-origin body models.
float3 ConstraintCentreOfMass(int body_idx)
{
	GpuRigidBody body = g_constraint_bodies[body_idx];
	return body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
}

// Return true when an endpoint body changed motion type since the last step, so impulses cached against its old mass are not valid.
bool ConstraintBodyResetsWarmStart(int body_idx)
{
	// World endpoints have no body and never reset.
	return body_idx >= 0 && AllSet(g_constraint_bodies[body_idx].state_flags, ERigidBodyStateFlags_ResetWarmStart);
}

// Compose an endpoint-local compact frame with its body transform, or return it directly for world endpoints.
float4x4 ConstraintFrameToWorld(GpuConstraintFrame frame, int body_idx)
{
	float4x4 local = quat_to_float4x4(frame.rotation, frame.position.xyz);
	return body_idx >= 0 ? mul(local, g_constraint_bodies[body_idx].o2w) : local;
}

// Return a finite scale for a retained physical impulse across ordinary timestep changes.
float ConstraintWarmStartScale()
{
	if (g.retain_current_impulses != 0)
		return 1.0f;

	return isfinite(g.warm_start_scale) && g.warm_start_scale > 0.0f ? g.warm_start_scale : 0.0f;
}

// Compile one canonical scalar row and its activity, returning false when soft coefficients are not representable.
bool CompileConstraintRow(
	uint slot_idx,
	uint axis_idx,
	GpuConstraintAxisDesc axis,
	float3 axis_ws,
	float3 anchor_offset_a,
	float3 anchor_offset_b,
	float position,
	uint old_state,
	bool reset_warm_start,
	out_(uint) state,
	out_(bool) velocity_active,
	out_(bool) position_active)
{
	uint row_idx = slot_idx * GpuConstraintRowsPerBlock + axis_idx;
	GpuConstraintRow old_row = g_rows[row_idx];
	GpuConstraintRow row = EmptyConstraintRow();

	float position_error;
	state = ConstraintRowState(axis, position, position_error);
	velocity_active = state != ConstraintLimitState_Inactive;
	position_active =
		(axis.mode == GpuConstraintAxisMode_Locked || axis.mode == GpuConstraintAxisMode_Limited) &&
		state != ConstraintLimitState_Inactive &&
		axis.stiffness == 0.0f &&
		axis.damping == 0.0f;

	// Canonical linear rows precede angular rows and use the CPU compiler's endpoint signs.
	bool is_linear = axis_idx < 3u;
	if (is_linear)
	{
		row.jacobian_a_ang = float4(-cross(anchor_offset_a, axis_ws), 0.0f);
		row.jacobian_a_lin = float4(-axis_ws, 0.0f);
		row.jacobian_b_ang = float4(+cross(anchor_offset_b, axis_ws), 0.0f);
		row.jacobian_b_lin = float4(+axis_ws, 0.0f);
	}
	else
	{
		row.jacobian_a_ang = float4(-axis_ws, 0.0f);
		row.jacobian_a_lin = float4(0, 0, 0, 0);
		row.jacobian_b_ang = float4(+axis_ws, 0.0f);
		row.jacobian_b_lin = float4(0, 0, 0, 0);
	}

	// World endpoints have no Jacobian contribution even though their frame still defines geometry.
	GpuConstraintEndpoint endpoint = g_endpoints[slot_idx];
	if (endpoint.body_idx_a < 0)
	{
		row.jacobian_a_ang = float4(0, 0, 0, 0);
		row.jacobian_a_lin = float4(0, 0, 0, 0);
	}
	if (endpoint.body_idx_b < 0)
	{
		row.jacobian_b_ang = float4(0, 0, 0, 0);
		row.jacobian_b_lin = float4(0, 0, 0, 0);
	}

	// Positive spring denominators retain their physical scale rather than becoming hard rows below a geometric floor.
	float gamma = 0.0f;
	float bias = 0.0f;
	bool valid = !velocity_active || ConstraintSoftParameters(g.timestep, axis.stiffness, axis.damping, position_error, gamma, bias);

	// A compatible cached impulse is only a feasible starting point; the solve applies the response-metric projection.
	float max_impulse = max(axis.max_force * g.timestep, 0.0f);
	float2 bounds = ConstraintImpulseBounds(state, max_impulse);
	float retained = 0.0f;
	if (!reset_warm_start && state == old_state && velocity_active)
		retained = clamp(old_row.bounds.z * ConstraintWarmStartScale(), bounds.x, bounds.y);

	row.solve = float4(position_error, axis.mode == GpuConstraintAxisMode_Driven ? axis.target_velocity : 0.0f, bias, gamma);
	row.bounds = float4(bounds, retained, 0.0f);
	g_rows[row_idx] = row;
	return valid;
}

// Compile one stable D6 slot into fixed canonical runtime storage.
numthreads(CSCompileConstraints, ConstraintThreadCount, 1, 1)
void CSCompileConstraints(int3 DTID(dtid))
{
	if (dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuConstraintEndpoint endpoint = g_endpoints[slot_idx];
	GpuD6ConstraintDesc desc = g_descriptors[slot_idx];
	GpuConstraintBlock old_block = g_blocks[slot_idx];
	GpuConstraintBlock block = EmptyConstraintBlock();
	block.body_idx_a = endpoint.body_idx_a;
	block.body_idx_b = endpoint.body_idx_b;

	// Disabled and tombstone slots discard stale runtime state immediately.
	if (!AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled))
	{
		for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx] = EmptyConstraintRow();
		g_blocks[slot_idx] = block;
		return;
	}

	// The separate coupled compiler exclusively owns link blocks and rows, making preparation order independent of the rigid compiler.
	if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled))
		return;

	// A break detected earlier in this submitted frame remains disabled across every later substep and selective continuation.
	if (AllSet(old_block.flags, ConstraintBlockFlags_Broken))
	{
		for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx] = EmptyConstraintRow();
		g_blocks[slot_idx] = old_block;
		return;
	}

	bool any_non_free = false;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		any_non_free = any_non_free || desc.axes[axis_idx].mode != GpuConstraintAxisMode_Free;
	if (!any_non_free)
	{
		for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx] = EmptyConstraintRow();
		g_blocks[slot_idx] = block;
		return;
	}

	float4x4 frame_a = ConstraintFrameToWorld(desc.frame_a, endpoint.body_idx_a);
	float4x4 frame_b = ConstraintFrameToWorld(desc.frame_b, endpoint.body_idx_b);
	float3 linear_error = frame_b[3].xyz - frame_a[3].xyz;
	float4 quat_a = quat_from_float3x3((float3x3)frame_a);
	float4 quat_b = quat_from_float3x3((float3x3)frame_b);
	float3 angular_error = quat_rotation_vector(quat_a, quat_b);
	float3 anchor_offset_a = endpoint.body_idx_a >= 0 ? frame_a[3].xyz - ConstraintCentreOfMass(endpoint.body_idx_a) : float3(0, 0, 0);
	float3 anchor_offset_b = endpoint.body_idx_b >= 0 ? frame_b[3].xyz - ConstraintCentreOfMass(endpoint.body_idx_b) : float3(0, 0, 0);
	bool reset_warm_start =
		g.retain_current_impulses == 0 &&
		(
			AllSet(endpoint.flags, GpuConstraintEndpointFlags_ResetWarmStart) ||
			ConstraintBodyResetsWarmStart(endpoint.body_idx_a) ||
			ConstraintBodyResetsWarmStart(endpoint.body_idx_b)
		);

	// Frame A defines all six canonical world axes, matching the CPU compiler.
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		uint local_axis = axis_idx % 3u;
		float3 axis_ws = frame_a[local_axis].xyz;
		float position = dot(axis_idx < 3u ? linear_error : angular_error, axis_ws);
		uint old_state = (old_block.row_states >> (2u * axis_idx)) & 3u;
		uint state;
		bool velocity_active;
		bool position_active;
		// Linear coordinates also differentiate the rotation of frame A's axis through the full anchor separation.
		float3 lever_a = axis_idx < 3u ? anchor_offset_a + linear_error : anchor_offset_a;
		if (!CompileConstraintRow(slot_idx, axis_idx, desc.axes[axis_idx], axis_ws, lever_a, anchor_offset_b, position, old_state, reset_warm_start, state, velocity_active, position_active))
			block.flags |= ConstraintBlockFlags_NumericalFailure;

		if (velocity_active)
			block.velocity_mask |= 1u << axis_idx;
		if (position_active)
			block.position_mask |= 1u << axis_idx;
		block.row_states |= state << (2u * axis_idx);
	}

	// Retained same-solve recompilation preserves failure; a fresh solve may recover after corrected inputs or timestep.
	block.colour = MaxColours;
	block.flags |= (g.retain_current_impulses != 0 ? old_block.flags & ConstraintBlockFlags_NumericalFailure : 0u) | ConstraintBlockFlags_Active | (reset_warm_start ? ConstraintBlockFlags_ResetWarmStart : 0u);
	if (AllSet(block.flags, ConstraintBlockFlags_NumericalFailure))
	{
		// Preserve a deterministic frame-wide failure even when a later substep recompiles this slot successfully.
		InterlockedMax(g_constraint_state[0].failure_slot_plus_one, slot_idx + 1u);
		block.velocity_mask = 0u;
		block.position_mask = 0u;
	}
	g_blocks[slot_idx] = block;
}

// Latch an arithmetic failure and prevent later passes from treating this block as a successful partial solve.
void FailConstraintBlock(uint slot_idx)
{
	GpuConstraintBlock block = g_blocks[slot_idx];
	block.flags |= ConstraintBlockFlags_NumericalFailure;
	InterlockedMax(g_constraint_state[0].failure_slot_plus_one, slot_idx + 1u);
	block.velocity_mask = 0u;
	block.position_mask = 0u;
	g_blocks[slot_idx] = block;
}

// Latch the first threshold crossing and its substep while retaining per-frame peak load diagnostics.
numthreads(CSDetectBrokenConstraints, ConstraintThreadCount, 1, 1)
void CSDetectBrokenConstraints(int3 DTID(dtid))
{
	if (g.breakable_count == 0 || dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuConstraintEndpoint endpoint = g_endpoints[slot_idx];
	GpuConstraintBlock block = g_blocks[slot_idx];
	GpuConstraintBreakState state = g_break_states[slot_idx];
	if (!AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) ||
		!AllSet(block.flags, ConstraintBlockFlags_Active) ||
		AllSet(state.flags, GpuConstraintBreakFlags_Broken))
		return;

	// Frame-A axes are orthonormal, so the Euclidean norm of their canonical scalar impulses is the world-space load magnitude.
	float linear_impulse_squared = 0.0f;
	float angular_impulse_squared = 0.0f;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		if ((block.velocity_mask & (1u << axis_idx)) == 0u)
			continue;

		float impulse = g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx].bounds.z;
		if (!isfinite(impulse))
			return;
		if (axis_idx < 3u)
			linear_impulse_squared += impulse * impulse;
		else
			angular_impulse_squared += impulse * impulse;
	}

	float inverse_timestep = g.timestep > 0.0f ? 1.0f / g.timestep : 0.0f;
	float force = sqrt(max(linear_impulse_squared, 0.0f)) * inverse_timestep;
	float torque = sqrt(max(angular_impulse_squared, 0.0f)) * inverse_timestep;
	if (!isfinite(force) || !isfinite(torque))
		return;

	state.peak_force = max(state.peak_force, force);
	state.peak_torque = max(state.peak_torque, torque);
	if ((isfinite(endpoint.break_force) && force > endpoint.break_force) ||
		(isfinite(endpoint.break_torque) && torque > endpoint.break_torque))
	{
		state.flags |= GpuConstraintBreakFlags_Broken;
		state.substep_index = g.break_substep_index;

		// Later work in the same command list sees an inert block while the retained rows remain available for diagnostics.
		block.velocity_mask = 0u;
		block.position_mask = 0u;
		block.colour = MaxColours;
		block.flags = ConstraintBlockFlags_Broken;
		g_blocks[slot_idx] = block;
	}
	g_break_states[slot_idx] = state;
}

// Constraint colouring gives each active slot the lowest colour that neither of its dynamic bodies has used yet, visiting the slots in index
// order. The result must be identical to that serial visit, so the work is split into passes:
//   1. CSResetConstraintColours clears the body masks, the slot colours, and the overflow scratch words.
//   2. CSAssignConstraintColourLevel runs once per schedule level. A slot only depends on the earlier slots that share one of its bodies, and
//      the schedule (built on the CPU from the endpoints) places every slot in a later level than all of those slots. So each slot reads the
//      exact body masks that the serial visit would see, from the per-slot masks its predecessors wrote.
//   3. CSFinishConstraintColours publishes the final body masks, applies the overflow fallback, and writes the indirect arguments.
// When a slot finds no free colour, the serial visit stops there. Slots after it may compute different values here, but those values are
// discarded because only slots below the lowest overflowing slot contribute to the published result.

// Return the body mask recorded after an encoded predecessor 'slot * 2 + side', or zero when there is no predecessor.
uint ConstraintColourMaskAfter(int pred)
{
	// The predecessor stored both endpoint masks; the side selects the one for the shared body.
	if (pred < 0)
		return 0u;

	uint2 masks = g_colour_masks[pred >> 1];
	return (pred & 1) == 0 ? masks.x : masks.y;
}

// Return true when a compiled block takes part in colouring.
bool ConstraintBlockColoured(GpuConstraintBlock block)
{
	return AllSet(block.flags, ConstraintBlockFlags_Active) && block.velocity_mask != 0u;
}

// Reset per-body masks, every slot colour, and the overflow scratch words before the level passes. Dispatch max(body_count, slot_count) threads.
numthreads(CSResetConstraintColours, ConstraintThreadCount, 1, 1)
void CSResetConstraintColours(int3 DTID(dtid))
{
	// Slots that are coupled, inactive, or never coloured keep this value, which matches the serial visit.
	if (dtid.x < g.body_count)
		g_constraint_bodies[dtid.x].colour_used = 0u;
	if (dtid.x < g.slot_count)
		g_blocks[dtid.x].colour = MaxColours;

	// The overflow slot starts above every real slot so the level passes can lower it atomically.
	if (dtid.x == 0)
	{
		g_constraint_state[0].colour_overflow_slot = 0xFFFFFFFFu;
		g_constraint_state[0].used_colours = 0u;
	}
}

// Colour every slot in one schedule level. Slots in a level never share a body, so they do not affect each other.
numthreads(CSAssignConstraintColourLevel, ConstraintThreadCount, 1, 1)
void CSAssignConstraintColourLevel(int3 DTID(dtid))
{
	// Read the body masks left by the predecessors, which completed in earlier levels.
	if (dtid.x >= g.schedule_count)
		return;

	GpuConstraintColourTask task = g_colour_schedule[g.schedule_begin + dtid.x];
	GpuConstraintBlock block = g_blocks[task.slot_idx];
	uint before_a = ConstraintColourMaskAfter(task.pred_a);
	uint before_b = ConstraintColourMaskAfter(task.pred_b);

	// Choose the lowest colour unused by either dynamic body. Non-participating slots pass the masks through unchanged.
	uint bit = 0u;
	if (ConstraintBlockColoured(block))
	{
		uint used = 0u;
		if (ConstraintBodyDynamic(block.body_idx_a))
			used |= before_a;
		if (ConstraintBodyDynamic(block.body_idx_b))
			used |= before_b;

		uint available = ~used;
		if (available == 0u)
		{
			InterlockedMin(g_constraint_state[0].colour_overflow_slot, (uint)task.slot_idx);
		}
		else
		{
			uint colour = firstbitlow(available);
			bit = 1u << colour;
			g_blocks[task.slot_idx].colour = colour;
			InterlockedOr(g_constraint_state[0].used_colours, bit);
		}
	}
	g_colour_masks[task.slot_idx] = uint2(before_a | bit, before_b | bit);
}

// Publish the body masks and the fallback, then write the per-colour indirect arguments. Dispatch one thread per scheduled slot, and at least one.
numthreads(CSFinishConstraintColours, ConstraintThreadCount, 1, 1)
void CSFinishConstraintColours(int3 DTID(dtid))
{
	// Write the frame result once. A single exhausted mask selects one coherent serial order for every active block, and only the serial
	// thread in colour zero runs. Otherwise every occupied colour keeps the full stable-slot grid and empty colours do no work. Blocks that break
	// later in the frame only leave their colour, so these arguments remain a valid upper bound for the rest of the frame.
	uint overflow_slot = g_constraint_state[0].colour_overflow_slot;
	bool overflow = overflow_slot != 0xFFFFFFFFu;
	if (dtid.x == 0)
	{
		uint used_colours = g_constraint_state[0].used_colours;
		uint groups = (uint)((g.slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			DispatchArguments args;
			args.ThreadGroupCountX = overflow ? (colour == 0 ? 1u : 0u) : ((used_colours & (1u << colour)) != 0u ? groups : 0u);
			args.ThreadGroupCountY = 1;
			args.ThreadGroupCountZ = 1;
			g_constraint_colour_dispatch[colour] = args;
		}
		g_constraint_state[0].colour_overflow = overflow ? 1u : 0u;
	}
	if (dtid.x >= g.schedule_count)
		return;

	// The serial visit records colours in the body masks only for slots before the overflowing slot, and then clears every colour.
	int slot_idx = g_colour_schedule[dtid.x].slot_idx;
	GpuConstraintBlock block = g_blocks[slot_idx];
	if (block.colour >= MaxColours)
		return;
	if (overflow)
		g_blocks[slot_idx].colour = MaxColours;
	if (overflow && (uint)slot_idx >= overflow_slot)
		return;

	uint bit = 1u << block.colour;
	if (ConstraintBodyDynamic(block.body_idx_a))
		InterlockedOr(g_constraint_bodies[block.body_idx_a].colour_used, bit);
	if (ConstraintBodyDynamic(block.body_idx_b))
		InterlockedOr(g_constraint_bodies[block.body_idx_b].colour_used, bit);
}

// The mutable momentum and sleep state of one dynamic endpoint. A block holds these in registers while it applies its row impulses, then stores
// each body once instead of reading and writing the whole body for every row.
struct ConstraintBodyMomentum
{
	float4 momentum_ang;
	float4 momentum_lin;
	int state_flags;
	GpuSleepData sleep;
	bool changed;
};

// Read the mutable state of a dynamic body, or an unchanged empty state when 'body_idx' is negative.
ConstraintBodyMomentum LoadConstraintBodyMomentum(int body_idx)
{
	// Empty state for endpoints that never receive impulses.
	ConstraintBodyMomentum body;
	body.momentum_ang = float4(0, 0, 0, 0);
	body.momentum_lin = float4(0, 0, 0, 0);
	body.state_flags = 0;
	body.sleep.timer_s = 0.0f;
	body.sleep.island_id = -1;
	body.sleep.generation = 0u;
	body.sleep.flags = 0u;
	body.changed = false;
	if (body_idx < 0)
		return body;

	body.momentum_ang = g_constraint_bodies[body_idx].momentum_ang;
	body.momentum_lin = g_constraint_bodies[body_idx].momentum_lin;
	body.state_flags = g_constraint_bodies[body_idx].state_flags;
	body.sleep = g_constraint_bodies[body_idx].sleep;
	return body;
}

// Add a world-space spatial impulse to one endpoint's momentum. Zero and non-finite impulses leave the endpoint unchanged.
void AccumulateConstraintImpulse(inout_(ConstraintBodyMomentum) body, float3 jacobian_ang, float3 jacobian_lin, float impulse)
{
	// Skipped impulses must not wake the body.
	if (impulse == 0.0f || !isfinite(impulse))
		return;

	body.momentum_ang.xyz += impulse * jacobian_ang;
	body.momentum_lin.xyz += impulse * jacobian_lin;
	body.changed = true;

	// Routine support impulses leave an awake body's quiet-time assessment to the sleep pass, while persisted sleeping state must be invalidated immediately.
	if (AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) || body.sleep.island_id >= 0)
	{
		body.sleep.generation++;
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.sleep.timer_s = 0.0f;
		body.sleep.island_id = -1;
		body.sleep.flags = 0u;
	}
}

// Write an endpoint's mutable state back to its body if any impulse changed it.
void StoreConstraintBodyMomentum(int body_idx, ConstraintBodyMomentum body)
{
	// Other body fields are never changed by impulses, so only the mutable fields are written.
	if (!body.changed)
		return;

	g_constraint_bodies[body_idx].momentum_ang = body.momentum_ang;
	g_constraint_bodies[body_idx].momentum_lin = body.momentum_lin;
	g_constraint_bodies[body_idx].state_flags = body.state_flags;
	g_constraint_bodies[body_idx].sleep = body.sleep;
}

// The authoritative momenta of one block's dynamic endpoints. When both endpoints refer to the same body, endpoint B updates endpoint A's state,
// so every impulse is added to that body in the same order as separate per-endpoint updates. A colour never gives two concurrent blocks the same
// dynamic body, so the state cannot be changed by another thread while a block holds it.
struct ConstraintBlockMomenta
{
	ConstraintBodyMomentum a;
	ConstraintBodyMomentum b;
	bool dynamic_a;
	bool dynamic_b;
	bool same_body;
};

// Read the mutable state of a block's dynamic endpoints.
ConstraintBlockMomenta LoadConstraintBlockMomenta(GpuConstraintBlock block)
{
	// Static, kinematic, and missing endpoints are never mutated, so they are not loaded.
	ConstraintBlockMomenta momenta;
	momenta.dynamic_a = ConstraintBodyDynamic(block.body_idx_a);
	momenta.dynamic_b = ConstraintBodyDynamic(block.body_idx_b);
	momenta.same_body = block.body_idx_a == block.body_idx_b;
	momenta.a = LoadConstraintBodyMomentum(momenta.dynamic_a ? block.body_idx_a : -1);
	momenta.b = LoadConstraintBodyMomentum(momenta.dynamic_b && !momenta.same_body ? block.body_idx_b : -1);
	return momenta;
}

// Apply one scalar row impulse to both endpoints, endpoint A first.
void AccumulateConstraintRowImpulse(inout_(ConstraintBlockMomenta) momenta, GpuConstraintRow row, float impulse)
{
	// A shared body receives both endpoint impulses in sequence.
	if (momenta.dynamic_a)
		AccumulateConstraintImpulse(momenta.a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, impulse);
	if (momenta.dynamic_b && momenta.same_body)
		AccumulateConstraintImpulse(momenta.a, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
	else if (momenta.dynamic_b)
		AccumulateConstraintImpulse(momenta.b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
}

// Write a block's changed endpoint momenta back to their bodies.
void StoreConstraintBlockMomenta(GpuConstraintBlock block, ConstraintBlockMomenta momenta)
{
	// A shared body is held only in endpoint A.
	if (momenta.dynamic_a)
		StoreConstraintBodyMomentum(block.body_idx_a, momenta.a);
	if (momenta.dynamic_b && !momenta.same_body)
		StoreConstraintBodyMomentum(block.body_idx_b, momenta.b);
}

// Apply all retained physical impulses for one block.
void ApplyConstraintBlockWarmStart(uint slot_idx)
{
	if (AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
		return;

	GpuConstraintBlock block = g_blocks[slot_idx];
	ConstraintBlockMomenta momenta = LoadConstraintBlockMomenta(block);
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		if ((block.velocity_mask & (1u << axis_idx)) == 0u)
			continue;

		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
		AccumulateConstraintRowImpulse(momenta, row, row.bounds.z);
	}
	StoreConstraintBlockMomenta(block, momenta);
}

// Apply cached impulses by colour, or serially in stable-slot order after colour exhaustion.
numthreads(CSApplyConstraintWarmStart, ConstraintThreadCount, 1, 1)
void CSApplyConstraintWarmStart(int3 DTID(dtid))
{
	if (g.colour == 0 && g_constraint_state[0].colour_overflow != 0u)
	{
		if (dtid.x != 0)
			return;

		for (uint slot_idx = 0; slot_idx != (uint)g.slot_count; ++slot_idx)
			if (AllSet(g_blocks[slot_idx].flags, ConstraintBlockFlags_Active))
				ApplyConstraintBlockWarmStart(slot_idx);
		return;
	}
	if (dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	if (g_blocks[slot_idx].colour != (uint)g.colour)
		return;

	ApplyConstraintBlockWarmStart(slot_idx);
}

// The world-space twists of a block's endpoints. Momentum does not change while a block forms its residuals, so each twist is computed once per block
// rather than once per row.
struct ConstraintBlockTwists
{
	BodyTwist a;
	BodyTwist b;
	bool has_a;
	bool has_b;
};

// Read the twists of every endpoint that refers to a body. Kinematic bodies contribute their twist even though they are never mutated.
ConstraintBlockTwists LoadConstraintBlockTwists(GpuConstraintBlock block)
{
	// Missing endpoints keep zero twists and are skipped by ConstraintRowVelocity.
	ConstraintBlockTwists twists;
	twists.has_a = block.body_idx_a >= 0 && block.body_idx_a < g.body_count;
	twists.has_b = block.body_idx_b >= 0 && block.body_idx_b < g.body_count;
	twists.a.ang = float3(0, 0, 0);
	twists.a.lin = float3(0, 0, 0);
	twists.b = twists.a;
	if (twists.has_a)
		twists.a = BodyTwistWS(g_constraint_bodies[block.body_idx_a]);
	if (twists.has_b)
		twists.b = BodyTwistWS(g_constraint_bodies[block.body_idx_b]);

	return twists;
}

// Return the current physical row velocity J*v from the block's endpoint twists.
float ConstraintRowVelocity(ConstraintBlockTwists twists, GpuConstraintRow row)
{
	// Sum the contribution of each endpoint that refers to a body
	float velocity = 0.0f;
	if (twists.has_a)
		velocity += dot(row.jacobian_a_ang.xyz, twists.a.ang) + dot(row.jacobian_a_lin.xyz, twists.a.lin);
	if (twists.has_b)
		velocity += dot(row.jacobian_b_ang.xyz, twists.b.ang) + dot(row.jacobian_b_lin.xyz, twists.b.lin);

	return velocity;
}

// Return one entry of J M^-1 J^T for two rows of the same block.
float ConstraintResponse(GpuConstraintBlock block, GpuConstraintRow lhs, GpuConstraintRow rhs)
{
	float response = 0.0f;
	if (ConstraintBodyDynamic(block.body_idx_a))
	{
		GpuRigidBody body = g_constraint_bodies[block.body_idx_a];
		response += dot(lhs.jacobian_a_ang.xyz, mul(ConstraintInverseInertia(block.body_idx_a), rhs.jacobian_a_ang.xyz));
		response += body.os_com_and_invmass.w * dot(lhs.jacobian_a_lin.xyz, rhs.jacobian_a_lin.xyz);
	}
	if (ConstraintBodyDynamic(block.body_idx_b))
	{
		GpuRigidBody body = g_constraint_bodies[block.body_idx_b];
		response += dot(lhs.jacobian_b_ang.xyz, mul(ConstraintInverseInertia(block.body_idx_b), rhs.jacobian_b_ang.xyz));
		response += body.os_com_and_invmass.w * dot(lhs.jacobian_b_lin.xyz, rhs.jacobian_b_lin.xyz);
	}
	return response;
}

// Build and invert the active block response, regularizing only a singular nonzero matrix.
bool PrepareConstraintInverse(uint slot_idx, uint active_mask, bool physical_solve, arrayout_(uint, active_axes, 6), out_(int) row_count, arrayout_(float, inverse, 36))
{
	GpuConstraintBlock block = g_blocks[slot_idx];
	row_count = 0;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		if ((active_mask & (1u << axis_idx)) != 0u)
			active_axes[row_count++] = axis_idx;
	if (row_count == 0)
		return false;

	float matrix[36];
	for (int idx = 0; idx != 36; ++idx)
		matrix[idx] = 0.0f;
	float scale = 0.0f;
	for (int row = 0; row != row_count; ++row)
	{
		GpuConstraintRow lhs = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row]];
		for (int column = 0; column != row_count; ++column)
		{
			GpuConstraintRow rhs = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[column]];
			matrix[row * 6 + column] = ConstraintResponse(block, lhs, rhs);
			if (!isfinite(matrix[row * 6 + column]))
			{
				FailConstraintBlock(slot_idx);
				return false;
			}
		}
		if (physical_solve)
			matrix[row * 6 + row] += lhs.solve.w;

		if (!isfinite(matrix[row * 6 + row]))
		{
			FailConstraintBlock(slot_idx);
			return false;
		}
		scale = max(scale, abs(matrix[row * 6 + row]));
	}
	if (!(scale > 0.0f))
		return false;

	// Response thresholds depend on operator scale, including valid very small inverse masses.
	float tolerance = 1.0e-7f * scale;
	if (InvertConstraintMatrix(matrix, row_count, tolerance, inverse))
		return true;

	// Regularize nonzero rows in their own units, using the block scale only for exactly immovable row directions.
	for (int row = 0; row != row_count; ++row)
	{
		float diagonal = matrix[row * 6 + row];
		matrix[row * 6 + row] += g.regularization * (diagonal > 0.0f ? diagonal : scale);
	}
	if (InvertConstraintMatrix(matrix, row_count, tolerance, inverse))
		return true;

	FailConstraintBlock(slot_idx);
	return false;
}

// Store one block's active-row inverse for a solve phase. Body transforms, inertia, rows, and compliance are fixed during the phase's iterations,
// so the response is built once per phase rather than once per iteration. Inactive, coupled, and zero-response blocks store no rows.
void PrepareConstraintSolveCache(uint slot_idx, bool physical_solve)
{
	// Start from an empty entry so skipped blocks never reuse an earlier phase's inverse.
	GpuConstraintSolveCache cache;
	for (int idx = 0; idx != 36; ++idx)
		cache.inverse[idx] = 0.0f;
	cache.active_axes = 0u;
	cache.row_count = 0;
	cache.pad0 = 0u;
	cache.pad1 = 0u;

	// Only blocks that a sweep visits are prepared, so failures are latched for the same blocks as a per-iteration build.
	// The serial overflow fallback visits every active block; coloured sweeps visit only coloured blocks.
	GpuConstraintBlock block = g_blocks[slot_idx];
	bool visited = g_constraint_state[0].colour_overflow != 0u
		? AllSet(block.flags, ConstraintBlockFlags_Active)
		: block.colour < MaxColours;
	if (visited && !AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
	{
		uint active_axes[6];
		int row_count;
		uint active_mask = physical_solve ? block.velocity_mask : block.position_mask;
		if (PrepareConstraintInverse(slot_idx, active_mask, physical_solve, active_axes, row_count, cache.inverse))
		{
			// Four bits per row hold the canonical axis index.
			for (int row_idx = 0; row_idx != row_count; ++row_idx)
				cache.active_axes |= active_axes[row_idx] << (4u * (uint)row_idx);

			cache.row_count = row_count;
		}
	}
	g_solve_cache[slot_idx] = cache;
}

// Build the split-position response inverse of every stable slot. Dispatch after warm start and before the position iterations.
numthreads(CSPrepareConstraintPositionSolve, ConstraintThreadCount, 1, 1)
void CSPrepareConstraintPositionSolve(int3 DTID(dtid))
{
	if (dtid.x >= g.slot_count)
		return;

	PrepareConstraintSolveCache((uint)dtid.x, false);
}

// Build the physical response inverse of every stable slot. Dispatch after the last transform change and before the velocity iterations.
numthreads(CSPrepareConstraintVelocitySolve, ConstraintThreadCount, 1, 1)
void CSPrepareConstraintVelocitySolve(int3 DTID(dtid))
{
	if (dtid.x >= g.slot_count)
		return;

	PrepareConstraintSolveCache((uint)dtid.x, true);
}

// Solve one warm-started physical D6 block and commit finite projected impulses immediately.
void SolveConstraintVelocityBlock(uint slot_idx)
{
	// A failure earlier in this phase clears the masks, after which the block no longer contributes.
	GpuConstraintBlock block = g_blocks[slot_idx];
	GpuConstraintSolveCache cache = g_solve_cache[slot_idx];
	int row_count = cache.row_count;
	if (row_count == 0 || block.velocity_mask == 0u)
		return;

	uint active_axes[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
		active_axes[row_idx] = (cache.active_axes >> (4u * (uint)row_idx)) & 0xFu;

	float residual[6];
	float candidate[6];
	float lower[6];
	float upper[6];
	float projected[6];
	ConstraintBlockTwists twists = LoadConstraintBlockTwists(block);
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		residual[row_idx] = ConstraintRowVelocity(twists, row) - row.solve.y + row.solve.z + row.solve.w * row.bounds.z;
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += cache.inverse[row_idx * 6 + column] * residual[column];
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		candidate[row_idx] = row.bounds.z - g.relaxation * correction;
		lower[row_idx] = row.bounds.x;
		upper[row_idx] = row.bounds.y;
	}

	// Saturated rows alter the optimal free impulses through the full response; reject the whole block on numerical failure.
	if (!ProjectConstraintBox(row_count, cache.inverse, candidate, lower, upper, projected))
	{
		FailConstraintBlock(slot_idx);
		return;
	}
	// Commit the impulse changes in row order, then store each changed body once.
	ConstraintBlockMomenta momenta = LoadConstraintBlockMomenta(block);
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		uint row_storage_idx = slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx];
		GpuConstraintRow row = g_rows[row_storage_idx];
		float delta = projected[row_idx] - row.bounds.z;
		AccumulateConstraintRowImpulse(momenta, row, delta);
		row.bounds.z = projected[row_idx];
		g_rows[row_storage_idx] = row;
	}
	StoreConstraintBlockMomenta(block, momenta);
}

// Run one coloured physical PGS sweep, with a stable serial fallback on overflow.
numthreads(CSSolveConstraintVelocity, ConstraintThreadCount, 1, 1)
void CSSolveConstraintVelocity(int3 DTID(dtid))
{
	if (g.colour == 0 && g_constraint_state[0].colour_overflow != 0u)
	{
		if (dtid.x != 0)
			return;

		for (uint slot_idx = 0; slot_idx != (uint)g.slot_count; ++slot_idx)
			if (AllSet(g_blocks[slot_idx].flags, ConstraintBlockFlags_Active))
				SolveConstraintVelocityBlock(slot_idx);
		return;
	}
	if (dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	if (g_blocks[slot_idx].colour != (uint)g.colour)
		return;

	SolveConstraintVelocityBlock(slot_idx);
}

// The split-correction state of one dynamic endpoint, held in registers while a block forms its residuals and applies its row impulses. The
// world-space inverse inertia does not change during a position sweep, so it is computed once per block rather than once per row.
struct ConstraintBodyPseudo
{
	GpuConstraintPseudoVelocity pseudo;
	float3x3 inverse_inertia;
	float inv_mass;
	bool changed;
};

// Read the split-correction state of a dynamic body, or an unchanged empty state when 'body_idx' is negative.
ConstraintBodyPseudo LoadConstraintBodyPseudo(int body_idx)
{
	// Empty state for endpoints that never receive impulses.
	ConstraintBodyPseudo body;
	body.pseudo.angular_velocity = float4(0, 0, 0, 0);
	body.pseudo.linear_velocity = float4(0, 0, 0, 0);
	body.inverse_inertia = float3x3(float3(0, 0, 0), float3(0, 0, 0), float3(0, 0, 0));
	body.inv_mass = 0.0f;
	body.changed = false;
	if (body_idx < 0)
		return body;

	body.pseudo = g_pseudo_velocities[body_idx];
	body.inverse_inertia = ConstraintInverseInertia(body_idx);
	body.inv_mass = g_constraint_bodies[body_idx].os_com_and_invmass.w;
	return body;
}

// Add one scalar split impulse to an endpoint's accumulated pseudo twist. Zero and non-finite impulses leave the endpoint unchanged.
void AccumulateConstraintPseudoImpulse(inout_(ConstraintBodyPseudo) body, float3 jacobian_ang, float3 jacobian_lin, float impulse)
{
	// Skipped impulses leave the stored pseudo twist untouched.
	if (impulse == 0.0f || !isfinite(impulse))
		return;

	body.pseudo.angular_velocity.xyz += mul(body.inverse_inertia, impulse * jacobian_ang);
	body.pseudo.linear_velocity.xyz += body.inv_mass * impulse * jacobian_lin;
	body.changed = true;
}

// The split-correction state of one block's dynamic endpoints. A shared body is held only in endpoint A; see ConstraintBlockMomenta.
struct ConstraintBlockPseudo
{
	ConstraintBodyPseudo a;
	ConstraintBodyPseudo b;
	bool dynamic_a;
	bool dynamic_b;
	bool same_body;
};

// Read the split-correction state of a block's dynamic endpoints.
ConstraintBlockPseudo LoadConstraintBlockPseudo(GpuConstraintBlock block)
{
	// Only dynamic endpoints contribute to split correction.
	ConstraintBlockPseudo pseudo;
	pseudo.dynamic_a = ConstraintBodyDynamic(block.body_idx_a);
	pseudo.dynamic_b = ConstraintBodyDynamic(block.body_idx_b);
	pseudo.same_body = block.body_idx_a == block.body_idx_b;
	pseudo.a = LoadConstraintBodyPseudo(pseudo.dynamic_a ? block.body_idx_a : -1);
	pseudo.b = LoadConstraintBodyPseudo(pseudo.dynamic_b && !pseudo.same_body ? block.body_idx_b : -1);
	return pseudo;
}

// Return the current split-correction row velocity without consulting physical momentum.
float ConstraintPseudoRowVelocity(ConstraintBlockPseudo pseudo, GpuConstraintRow row)
{
	// A shared body contributes its one pseudo twist through both endpoint Jacobians.
	float velocity = 0.0f;
	if (pseudo.dynamic_a)
		velocity += dot(row.jacobian_a_ang.xyz, pseudo.a.pseudo.angular_velocity.xyz) + dot(row.jacobian_a_lin.xyz, pseudo.a.pseudo.linear_velocity.xyz);
	if (pseudo.dynamic_b)
	{
		float3 b_ang = pseudo.same_body ? pseudo.a.pseudo.angular_velocity.xyz : pseudo.b.pseudo.angular_velocity.xyz;
		float3 b_lin = pseudo.same_body ? pseudo.a.pseudo.linear_velocity.xyz : pseudo.b.pseudo.linear_velocity.xyz;
		velocity += dot(row.jacobian_b_ang.xyz, b_ang) + dot(row.jacobian_b_lin.xyz, b_lin);
	}
	return velocity;
}

// Apply one scalar split impulse to both endpoints, endpoint A first.
void AccumulateConstraintRowPseudoImpulse(inout_(ConstraintBlockPseudo) pseudo, GpuConstraintRow row, float impulse)
{
	// A shared body receives both endpoint impulses in sequence.
	if (pseudo.dynamic_a)
		AccumulateConstraintPseudoImpulse(pseudo.a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, impulse);
	if (pseudo.dynamic_b && pseudo.same_body)
		AccumulateConstraintPseudoImpulse(pseudo.a, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
	else if (pseudo.dynamic_b)
		AccumulateConstraintPseudoImpulse(pseudo.b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
}

// Write a block's changed pseudo twists back to their bodies.
void StoreConstraintBlockPseudo(GpuConstraintBlock block, ConstraintBlockPseudo pseudo)
{
	// A shared body is held only in endpoint A.
	if (pseudo.dynamic_a && pseudo.a.changed)
		g_pseudo_velocities[block.body_idx_a] = pseudo.a.pseudo;
	if (pseudo.dynamic_b && !pseudo.same_body && pseudo.b.changed)
		g_pseudo_velocities[block.body_idx_b] = pseudo.b.pseudo;
}

// Solve one hard passive position block against the accumulated split pseudo-twist.
void SolveConstraintPositionBlock(uint slot_idx)
{
	// A failure earlier in this phase clears the masks, after which the block no longer contributes.
	GpuConstraintBlock block = g_blocks[slot_idx];
	GpuConstraintSolveCache cache = g_solve_cache[slot_idx];
	int row_count = cache.row_count;
	if (row_count == 0 || block.position_mask == 0u)
		return;

	uint active_axes[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
		active_axes[row_idx] = (cache.active_axes >> (4u * (uint)row_idx)) & 0xFu;

	float residual[6];
	float candidate[6];
	float impulse[6];
	float lower[6];
	float upper[6];
	ConstraintBlockPseudo pseudo = LoadConstraintBlockPseudo(block);
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		float correction = g.position_beta * row.solve.x / g.timestep;
		float target_velocity = -clamp(correction, -g.max_position_speed, +g.max_position_speed);
		residual[row_idx] = ConstraintPseudoRowVelocity(pseudo, row) - target_velocity;
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += cache.inverse[row_idx * 6 + column] * residual[column];
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		uint state = (block.row_states >> (2u * active_axes[row_idx])) & 3u;
		float2 bounds = ConstraintImpulseBounds(state, 3.402823466e+38f);
		candidate[row_idx] = row.bounds.w - g.position_relaxation * correction;
		lower[row_idx] = bounds.x;
		upper[row_idx] = bounds.y;
	}

	// Unilateral hard rows use the same response-metric box solve as physical constraints.
	if (!ProjectConstraintBox(row_count, cache.inverse, candidate, lower, upper, impulse))
	{
		FailConstraintBlock(slot_idx);
		return;
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		uint row_storage_idx = slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx];
		GpuConstraintRow row = g_rows[row_storage_idx];
		float delta = impulse[row_idx] - row.bounds.w;
		AccumulateConstraintRowPseudoImpulse(pseudo, row, delta);
		row.bounds.w = impulse[row_idx];
		g_rows[row_storage_idx] = row;
	}
	StoreConstraintBlockPseudo(block, pseudo);
}

// Clear per-frame split-correction state before position iterations begin.
numthreads(CSClearConstraintPseudoVelocity, ConstraintThreadCount, 1, 1)
void CSClearConstraintPseudoVelocity(int3 DTID(dtid))
{
	if (dtid.x >= g.body_count)
		return;

	GpuConstraintPseudoVelocity pseudo;
	pseudo.angular_velocity = float4(0, 0, 0, 0);
	pseudo.linear_velocity = float4(0, 0, 0, 0);
	g_pseudo_velocities[dtid.x] = pseudo;
}

// Run one coloured split-position sweep without changing physical momentum or transforms.
numthreads(CSSolveConstraintPosition, ConstraintThreadCount, 1, 1)
void CSSolveConstraintPosition(int3 DTID(dtid))
{
	if (g.colour == 0 && g_constraint_state[0].colour_overflow != 0u)
	{
		if (dtid.x != 0)
			return;

		for (uint slot_idx = 0; slot_idx != (uint)g.slot_count; ++slot_idx)
			if (AllSet(g_blocks[slot_idx].flags, ConstraintBlockFlags_Active))
				SolveConstraintPositionBlock(slot_idx);
		return;
	}
	if (dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	if (g_blocks[slot_idx].colour != (uint)g.colour)
		return;

	SolveConstraintPositionBlock(slot_idx);
}

// Integrate each body's converged pseudo twist once about its centre of mass.
numthreads(CSApplyConstraintPosition, ConstraintThreadCount, 1, 1)
void CSApplyConstraintPosition(int3 DTID(dtid))
{
	if (dtid.x >= g.body_count || !ConstraintBodyDynamic(dtid.x))
		return;

	GpuConstraintPseudoVelocity pseudo = g_pseudo_velocities[dtid.x];
	if (!isfinite(pseudo.angular_velocity.x) || !isfinite(pseudo.angular_velocity.y) || !isfinite(pseudo.angular_velocity.z) ||
		!isfinite(pseudo.linear_velocity.x) || !isfinite(pseudo.linear_velocity.y) || !isfinite(pseudo.linear_velocity.z))
		return;

	GpuRigidBody body = g_constraint_bodies[dtid.x];
	float3 com_ws = ConstraintCentreOfMass(dtid.x);
	float3x3 rotation = (float3x3)body.o2w;
	float3x3 new_rotation = orthonorm3x3(mul(rotation, rodrigues_rotation(g.timestep * pseudo.angular_velocity.xyz)));
	float3 new_com_ws = com_ws + g.timestep * pseudo.linear_velocity.xyz;
	float3 new_position = new_com_ws - mul(body.os_com_and_invmass.xyz, new_rotation);
	body.o2w = float4x4(
		float4(new_rotation[0], 0),
		float4(new_rotation[1], 0),
		float4(new_rotation[2], 0),
		float4(new_position, 1));
	g_constraint_bodies[dtid.x] = body;
}

// ----- Fused colour sweeps -----
// A single thread group walks every occupied colour in the same order as the per-colour dispatches, with a group-wide device-memory barrier between colours.
// This removes one dispatch and one UAV barrier per colour, which dominate small constraint sets. The CPU chooses this path only for small slot counts
// because one group processes every slot. UAV writes are coherent within one thread group, so the barrier makes each colour's writes visible to the next.
// These entry points exist only in HLSL because the C++ replay emulator cannot synchronise the threads of a group.
#ifndef __cplusplus

// Solver operations shared by the fused entry points.
static const uint EFusedConstraintOp_ApplyWarmStart = 0;
static const uint EFusedConstraintOp_SolvePosition = 1;
static const uint EFusedConstraintOp_SolveVelocity = 2;

groupshared uint gs_fused_used_colours;

// Apply one solver operation to one constraint block.
void ApplyFusedConstraintOp(uint op, uint slot_idx)
{
	// 'op' is a compile-time constant in each entry point, so this switch is removed after inlining.
	switch (op)
	{
		case EFusedConstraintOp_ApplyWarmStart: { ApplyConstraintBlockWarmStart(slot_idx); break; }
		case EFusedConstraintOp_SolvePosition: { SolveConstraintPositionBlock(slot_idx); break; }
		case EFusedConstraintOp_SolveVelocity: { SolveConstraintVelocityBlock(slot_idx); break; }
	}
}

// Run one complete colour sweep inside one thread group. Must be called by every thread of a group of ConstraintThreadCount threads.
void FusedConstraintSweep(uint op, uint thread)
{
	// The overflow flag is the same for every thread, so the serial fallback keeps the whole group on one path. It matches the per-colour colour-zero fallback.
	if (g_constraint_state[0].colour_overflow != 0u)
	{
		if (thread != 0)
			return;

		for (uint slot_idx = 0; slot_idx != (uint)g.slot_count; ++slot_idx)
			if (AllSet(g_blocks[slot_idx].flags, ConstraintBlockFlags_Active))
				ApplyFusedConstraintOp(op, slot_idx);
		return;
	}

	// Collect the occupied colours so empty colours cost no group barrier. Colour assignments do not change during a sweep.
	if (thread == 0)
		gs_fused_used_colours = 0;

	GroupMemoryBarrierWithGroupSync();
	for (int i = (int)thread; i < g.slot_count; i += ConstraintThreadCount)
	{
		uint colour = g_blocks[i].colour;
		if (colour < MaxColours)
			InterlockedOr(gs_fused_used_colours, 1u << colour);
	}
	GroupMemoryBarrierWithGroupSync();

	// Process the colours in ascending order, as the per-colour dispatches do. 'used' is the same in every thread, so every thread reaches each barrier.
	uint used = gs_fused_used_colours;
	while (used != 0)
	{
		// Blocks within one colour never share a dynamic body, so threads can process them in any order.
		uint colour = firstbitlow(used);
		used &= used - 1;
		for (int j = (int)thread; j < g.slot_count; j += ConstraintThreadCount)
		{
			if (g_blocks[j].colour == colour)
				ApplyFusedConstraintOp(op, (uint)j);
		}
		DeviceMemoryBarrierWithGroupSync();
	}
}

// Single-group form of CSApplyConstraintWarmStart. Dispatch exactly one group.
numthreads(CSApplyConstraintWarmStartFused, ConstraintThreadCount, 1, 1)
void CSApplyConstraintWarmStartFused(int3 GTID(gtid))
{
	FusedConstraintSweep(EFusedConstraintOp_ApplyWarmStart, gtid.x);
}

// Single-group form of CSSolveConstraintPosition. Dispatch exactly one group.
numthreads(CSSolveConstraintPositionFused, ConstraintThreadCount, 1, 1)
void CSSolveConstraintPositionFused(int3 GTID(gtid))
{
	FusedConstraintSweep(EFusedConstraintOp_SolvePosition, gtid.x);
}

// Single-group form of CSSolveConstraintVelocity. Dispatch exactly one group.
numthreads(CSSolveConstraintVelocityFused, ConstraintThreadCount, 1, 1)
void CSSolveConstraintVelocityFused(int3 GTID(gtid))
{
	FusedConstraintSweep(EFusedConstraintOp_SolveVelocity, gtid.x);
}

#endif

#ifdef __cplusplus
}
#endif
