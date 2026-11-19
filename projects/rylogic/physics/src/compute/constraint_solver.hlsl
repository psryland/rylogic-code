//*********************************************
// Physics Engine — Persistent Typed Constraint Solver
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU pass order:
//   1. CSCompileConstraints
//   2. CSResetConstraintColours, CSAssignConstraintColourLevel once per schedule level, then CSFinishConstraintColours
//   Position: explicit pseudo clear, position inverse, configured coloured sweeps, apply once.
//   Velocity: current-pose row compilation and inverse, then staged commits in dependency-level order.
//   The resolver selects the established phase order; each physical compile/inverse pair describes the same pose.
//
// Resource layout:
//   b0: cbConstraintSolver                         - per-dispatch constants
//   u0: RWStructuredBuffer<GpuRigidBody>           - integrated body state
//   t0: StructuredBuffer<GpuConstraintEndpoint>    - stable-slot endpoint metadata
//   t1: StructuredBuffer<GpuConstraintDesc>        - stable-slot typed parameters
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
#include "physics/src/compute/constraint_coordinates.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/body_twist.hlsli"
#include "physics/src/compute/constraint_transaction.hlsli"

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
	int coupled_island_count;
	int pad0;
	int breakable_count;
	int break_substep_index;
	int schedule_begin;
	int schedule_count;
	uint epoch_low;
	uint epoch_high;
	int pad2;
};

ConstantBuffer<cbConstraintSolver> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_constraint_bodies, u0);
StructuredBuffer<GpuConstraintEndpoint> resource(g_endpoints, t0);
StructuredBuffer<GpuConstraintDesc> resource(g_descriptors, t1);
RWStructuredBuffer<GpuConstraintBlock> resource(g_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_rows, u2);
RWStructuredBuffer<GpuConstraintSolverState> resource(g_constraint_state, u3);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_pseudo_velocities, u4);
RWStructuredBuffer<GpuConstraintBreakState> resource(g_break_states, u5);
RWStructuredBuffer<DispatchArguments> resource(g_constraint_colour_dispatch, u6);
RWStructuredBuffer<GpuConstraintSolveCache> resource(g_solve_cache, u7);
StructuredBuffer<GpuConstraintColourTask> resource(g_colour_schedule, t2);
RWStructuredBuffer<uint2> resource(g_colour_masks, u8);
RWStructuredBuffer<GpuConstraintRigidRecord> resource(g_rigid_records, u9);
RWStructuredBuffer<uint> resource(g_transaction_results, u10);
RWStructuredBuffer<GpuConstraintRejectionState> resource(g_rejections, u11);
RWStructuredBuffer<GpuConstraintAcceptance> resource(g_coupled_acceptance, u12);
RWStructuredBuffer<GpuConstraintAcceptedRows> resource(g_coupled_accepted_rows, u13);
StructuredBuffer<GpuCoupledConstraintBlockTopology> resource(g_coupled_topology, t3);

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

// Compile one canonical scalar row and its activity, returning false when soft coefficients are not representable.
bool CompileConstraintRow(
	uint slot_idx,
	uint axis_idx,
	GpuConstraintAxisDesc axis,
	bool is_linear,
	float3 axis_ws,
	float3 anchor_offset_a,
	float3 anchor_offset_b,
	float position,
	out_(uint) state,
	out_(bool) velocity_active,
	out_(bool) position_active)
{
	uint row_idx = slot_idx * GpuConstraintRowsPerBlock + axis_idx;
	GpuConstraintRow row = EmptyConstraintRow();

	float position_error;
	state = ConstraintRowState(axis, position, position_error);
	velocity_active = state != ConstraintLimitState_Inactive;
	position_active =
		(axis.mode == GpuConstraintAxisMode_Locked || axis.mode == GpuConstraintAxisMode_Limited) &&
		state != ConstraintLimitState_Inactive &&
		axis.stiffness == 0.0f &&
		axis.damping == 0.0f;

	// Physical row kind is independent of the persistent slot number.
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

	// Accepted ownership supplies provisional guesses later; geometry compilation never reads a previous prepared coefficient.
	float max_impulse = max(axis.max_force * g.timestep, 0.0f);
	float2 bounds = ConstraintImpulseBounds(state, max_impulse);
	row.solve = float4(position_error, axis.mode == GpuConstraintAxisMode_Driven ? axis.target_velocity : 0.0f, bias, gamma);
	row.bounds = float4(bounds, 0.0f, 0.0f);
	g_rows[row_idx] = row;
	return valid;
}

// Compile one stable typed constraint slot into fixed canonical runtime storage.
numthreads(CSCompileConstraints, ConstraintThreadCount, 1, 1)
void CSCompileConstraints(int3 DTID(dtid))
{
	if (dtid.x >= g.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuConstraintEndpoint endpoint = g_endpoints[slot_idx];
	GpuConstraintDesc desc = g_descriptors[slot_idx];
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

	bool any_non_free = desc.type != GpuConstraintType_D6;
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
	SphericalAngularRows spherical_rows;
	switch (desc.type)
	{
		case GpuConstraintType_D6: { break; }
		case GpuConstraintType_SphericalAngular:
		{
			// Shared geometry preserves independent limit faces and physical damping rows.
			if (!PrepareSphericalAngularRows(quat_a, quat_b, desc.swing_limits, desc.twist_limits, desc.angular_damping, spherical_rows))
				block.flags |= ConstraintBlockFlags_NumericalFailure;

			block.row_identities = spherical_rows.m_identities;
			break;
		}
		default:
		{
			// Unsupported descriptor kinds fail the frame rather than being interpreted as an all-free D6.
			block.flags |= ConstraintBlockFlags_NumericalFailure;
			break;
		}
	}
	float3 anchor_offset_a = endpoint.body_idx_a >= 0 ? frame_a[3].xyz - ConstraintCentreOfMass(endpoint.body_idx_a) : float3(0, 0, 0);
	float3 anchor_offset_b = endpoint.body_idx_b >= 0 ? frame_b[3].xyz - ConstraintCentreOfMass(endpoint.body_idx_b) : float3(0, 0, 0);
	bool reset_warm_start =
		AllSet(endpoint.flags, GpuConstraintEndpointFlags_ResetWarmStart) ||
		ConstraintBodyResetsWarmStart(endpoint.body_idx_a) ||
		ConstraintBodyResetsWarmStart(endpoint.body_idx_b);

	// Each primitive defines stable slots; physical angular rows need not occupy slots three through five.
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		uint local_axis = axis_idx % 3u;
		float3 axis_ws = frame_a[local_axis].xyz;
		float position = dot(axis_idx < 3u ? linear_error : angular_error, axis_ws);
		GpuConstraintAxisDesc axis = desc.axes[axis_idx];
		bool is_linear = false;
		switch (desc.type)
		{
			case GpuConstraintType_D6:
			{
				// D6 axes retain their principal-log covectors and linear-first layout.
				is_linear = axis_idx < 3u;
				if (!is_linear)
					axis_ws = RotationVectorGradient(angular_error, axis_ws);

				break;
			}
			case GpuConstraintType_SphericalAngular:
			{
				// Ellipse and twist gradients are not used by the three orthonormal dampers.
				axis_ws = spherical_rows.m_axes[axis_idx];
				position = spherical_rows.m_positions[axis_idx];
				axis = spherical_rows.m_parameters[axis_idx];
				break;
			}
			default: { axis.mode = GpuConstraintAxisMode_Free; break; }
		}

		uint state;
		bool velocity_active;
		bool position_active;
		// Linear coordinates also differentiate the rotation of frame A's axis through the full anchor separation.
		float3 lever_a = is_linear ? anchor_offset_a + linear_error : anchor_offset_a;
		if (!CompileConstraintRow(slot_idx, axis_idx, axis, is_linear, axis_ws, lever_a, anchor_offset_b, position, state, velocity_active, position_active))
			block.flags |= ConstraintBlockFlags_NumericalFailure;

		if (velocity_active)
			block.velocity_mask |= 1u << axis_idx;
		if (position_active)
			block.position_mask |= 1u << axis_idx;
		block.row_states |= state << (2u * axis_idx);
	}

	// The frame latch preserves fatal failure across every geometry refresh; Upload owns its reset.
	block.colour = MaxColours;
	block.flags |= ConstraintBlockFlags_Active | (reset_warm_start ? ConstraintBlockFlags_ResetWarmStart : 0u);
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
		AllSet(state.flags, GpuConstraintBreakFlags_Broken))
		return;

	// Only accepted current-substep rows own physical load. A later rejected recompile cannot revoke that acceptance.
	GpuConstraintAcceptance accepted;
	GpuConstraintAcceptedRows rows;
	if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled))
	{
		// Coupled acceptance belongs to the island, while its measured row loads remain stable-slot indexed.
		if (g.coupled_island_count == 0)
			return;

		int island = g_coupled_topology[slot_idx].island_idx;
		if (island < 0 || island >= g.coupled_island_count)
			return;

		accepted = g_coupled_acceptance[island];
		rows = g_coupled_accepted_rows[slot_idx];
	}
	else
	{
		// Ordinary blocks retain their own acceptance and measured loads.
		accepted = g_rigid_records[slot_idx].accepted;
		rows = g_rigid_records[slot_idx].rows;
	}
	if (accepted.valid == 0u || any(accepted.epoch != uint2(g.epoch_low, g.epoch_high)))
		return;

	// Division preserves representable zero loads even when the reciprocal duration alone would overflow.
	float force = rows.linear_impulse_norm / accepted.timestep;
	float torque = rows.angular_impulse_norm / accepted.timestep;
	if (!isfinite(force) || !isfinite(torque))
	{
		// Accepted storage must have a representable load; never silently hide a corrupt break sample.
		FailConstraintBlock(slot_idx);
		return;
	}

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

	// Physical transactions visit active blocks through dependency levels, independently of position colouring.
	// Position's serial overflow fallback visits every active block; its coloured sweeps visit only coloured blocks.
	GpuConstraintBlock block = g_blocks[slot_idx];
	bool visited = physical_solve || g_constraint_state[0].colour_overflow != 0u
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

// Replace one ordinary block's accepted response, publishing neither endpoint nor cache on a rejected attempt.
numthreads(CSCommitConstraintVelocityTransaction, ConstraintThreadCount, 1, 1)
void CSCommitConstraintVelocityTransaction(int3 DTID(dtid))
{
	// The existing dependency levels include zero-current-row owners and give each dynamic body exactly one writer per dispatch.
	if (dtid.x >= g.schedule_count)
		return;

	uint slot_idx = (uint)g_colour_schedule[g.schedule_begin + dtid.x].slot_idx;
	GpuConstraintBlock block = g_blocks[slot_idx];
	if (AllSet(block.flags, ConstraintBlockFlags_NumericalFailure))
	{
		// Preparation/factorization retains its existing fatal latch; it is not a non-terminal physical rejection.
		g_transaction_results[slot_idx] = GpuConstraintStage_NumericalFailure;
		return;
	}
	GpuConstraintSolveCache inverse = g_solve_cache[slot_idx];
	if (block.velocity_mask != 0u && inverse.row_count == 0)
	{
		// Preserve the existing no-response no-op without publishing an unperformed solve or clearing accepted support.
		g_transaction_results[slot_idx] = 0xFFFFFFFFu;
		return;
	}

	// Missing world endpoints have explicitly zero physical state; kinematic endpoints keep their prescribed twist.
	#ifdef __cplusplus
	GpuRigidBody a = {};
	GpuRigidBody b = {};
	#else
	GpuRigidBody a = (GpuRigidBody)0;
	GpuRigidBody b = (GpuRigidBody)0;
	#endif
	if (block.body_idx_a >= 0)
		a = g_constraint_bodies[block.body_idx_a];

	if (block.body_idx_b >= 0)
		b = g_constraint_bodies[block.body_idx_b];

	GpuConstraintRow rows[GpuConstraintRowsPerBlock];
	for (uint axis = 0; axis != GpuConstraintRowsPerBlock; ++axis)
	{
		// Prepared coefficients are never read as an accepted cache by this pipeline.
		rows[axis] = g_rows[slot_idx * GpuConstraintRowsPerBlock + axis];
	}
	GpuConstraintEpoch epoch;
	epoch.identity = uint2(g.epoch_low, g.epoch_high);
	epoch.timestep = g.timestep;
	epoch.warm_start_factor = g.warm_start_factor;
	GpuConstraintRigidRecord previous = g_rigid_records[slot_idx];

	// A compiler flag is a repeated observation, not a new invalidation event. Only acceptance acknowledges the input revision.
	// Body reset levels are observed per endpoint: a rising level invalidates once within an epoch, and a historical cache resets again.
	uint revision = g_endpoints[slot_idx].invalidation_revision;
	uint body_resets = (ConstraintBodyResetsWarmStart(block.body_idx_a) ? 1u : 0u) | (ConstraintBodyResetsWarmStart(block.body_idx_b) ? 2u : 0u);
	uint previous_resets = previous.rows.invalidation_receipt & 3u;
	if (revision != (previous.rows.invalidation_receipt >> 2u) ||
		(body_resets & ~previous_resets) != 0u ||
		(body_resets != 0u && !ConstraintAcceptedInEpoch(previous.accepted, epoch)))
		previous.rows.velocity_mask = 0u;

	// Keep every prospective physical/cache/sleep member private until the complete-block admission result is known.
	GpuRigidBody staged_a;
	GpuRigidBody staged_b;
	GpuConstraintRigidRecord record;
	uint status = ConstraintStageRigidBlock(block, inverse, rows, a, b, previous, epoch, g.relaxation, staged_a, staged_b, record);
	g_transaction_results[slot_idx] = status;
	if (status == GpuConstraintStage_AdmissionRejected && g_rejections[slot_idx].substep_index < 0)
	{
		// Admission has no numerical reason or invented solve metadata.
		GpuConstraintRejectionState rejection;
		rejection.failure_flags = 0u;
		rejection.substep_index = g.break_substep_index;
		rejection.phase = GpuCoupledConstraintFailurePhase_Velocity;
		rejection.iteration_count = 0;
		rejection.relaxation = 0.0f;
		rejection.merit_change = 0.0f;
		rejection.outcome = status;
		rejection.owner = 0u;
		rejection.index = slot_idx;
		rejection.generation = g_endpoints[slot_idx].generation;
		g_rejections[slot_idx] = rejection;
	}
	if (status == GpuConstraintStage_NumericalFailure)
	{
		// The same fatal transport remains responsible for input and projection arithmetic failures.
		FailConstraintBlock(slot_idx);
		return;
	}
	if (status != GpuConstraintStage_Accepted)
		return;

	// All writes belong to this block, with no interleaved contact or other owner dispatch inside this invocation.
	if (ConstraintBodyDynamic(block.body_idx_a))
		g_constraint_bodies[block.body_idx_a] = staged_a;

	if (ConstraintBodyDynamic(block.body_idx_b))
		g_constraint_bodies[block.body_idx_b] = staged_b;

	// Rejection never acknowledges invalidation, and a later upload cannot erase an unaccepted revision.
	record.rows.invalidation_receipt = (revision << 2u) | body_resets;
	g_rigid_records[slot_idx] = record;
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

// Split-correction state for distinct validated endpoints.
struct ConstraintBlockPseudo
{
	ConstraintBodyPseudo a;
	ConstraintBodyPseudo b;
	bool dynamic_a;
	bool dynamic_b;
};

// Read the split-correction state of a block's dynamic endpoints.
ConstraintBlockPseudo LoadConstraintBlockPseudo(GpuConstraintBlock block)
{
	// Only dynamic endpoints contribute to split correction.
	ConstraintBlockPseudo pseudo;
	pseudo.dynamic_a = ConstraintBodyDynamic(block.body_idx_a);
	pseudo.dynamic_b = ConstraintBodyDynamic(block.body_idx_b);
	pseudo.a = LoadConstraintBodyPseudo(pseudo.dynamic_a ? block.body_idx_a : -1);
	pseudo.b = LoadConstraintBodyPseudo(pseudo.dynamic_b ? block.body_idx_b : -1);
	return pseudo;
}

// Return the current split-correction row velocity without consulting physical momentum.
float ConstraintPseudoRowVelocity(ConstraintBlockPseudo pseudo, GpuConstraintRow row)
{
	// Each endpoint contributes through its own Jacobian and pseudo twist.
	float velocity = 0.0f;
	if (pseudo.dynamic_a)
		velocity += dot(row.jacobian_a_ang.xyz, pseudo.a.pseudo.angular_velocity.xyz) + dot(row.jacobian_a_lin.xyz, pseudo.a.pseudo.linear_velocity.xyz);
	if (pseudo.dynamic_b)
	{
		velocity += dot(row.jacobian_b_ang.xyz, pseudo.b.pseudo.angular_velocity.xyz) + dot(row.jacobian_b_lin.xyz, pseudo.b.pseudo.linear_velocity.xyz);
	}
	return velocity;
}

// Apply one scalar split impulse to both endpoints, endpoint A first.
void AccumulateConstraintRowPseudoImpulse(inout_(ConstraintBlockPseudo) pseudo, GpuConstraintRow row, float impulse)
{
	// The caller contract excludes identical body endpoints.
	if (pseudo.dynamic_a)
		AccumulateConstraintPseudoImpulse(pseudo.a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, impulse);
	if (pseudo.dynamic_b)
		AccumulateConstraintPseudoImpulse(pseudo.b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
}

// Write a block's changed pseudo twists back to their bodies.
void StoreConstraintBlockPseudo(GpuConstraintBlock block, ConstraintBlockPseudo pseudo)
{
	// A shared body is held only in endpoint A.
	if (pseudo.dynamic_a && pseudo.a.changed)
		g_pseudo_velocities[block.body_idx_a] = pseudo.a.pseudo;
	if (pseudo.dynamic_b && pseudo.b.changed)
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

groupshared uint gs_fused_used_colours;

// Run one complete colour sweep inside one thread group. Must be called by every thread of a group of ConstraintThreadCount threads.
void FusedConstraintPositionSweep(uint thread)
{
	// The overflow flag is the same for every thread, so the serial fallback keeps the whole group on one path. It matches the per-colour colour-zero fallback.
	if (g_constraint_state[0].colour_overflow != 0u)
	{
		if (thread != 0)
			return;

		for (uint slot_idx = 0; slot_idx != (uint)g.slot_count; ++slot_idx)
			if (AllSet(g_blocks[slot_idx].flags, ConstraintBlockFlags_Active))
				SolveConstraintPositionBlock(slot_idx);
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
				SolveConstraintPositionBlock((uint)j);
		}
		DeviceMemoryBarrierWithGroupSync();
	}
}

// Single-group form of CSSolveConstraintPosition. Dispatch exactly one group.
numthreads(CSSolveConstraintPositionFused, ConstraintThreadCount, 1, 1)
void CSSolveConstraintPositionFused(int3 GTID(gtid))
{
	FusedConstraintPositionSweep(gtid.x);
}

#endif

#ifdef __cplusplus
}
#endif
