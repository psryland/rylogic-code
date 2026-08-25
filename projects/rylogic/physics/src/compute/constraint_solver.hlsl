//*********************************************
// Physics Engine — Persistent D6 Constraint Solver
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU pass order:
//   1. CSCompileConstraints
//   2. CSAssignConstraintColours
//   3. CSApplyConstraintWarmStart, once per colour
//   4. CSClearConstraintPseudoVelocity
//   5. CSSolveConstraintPosition, fixed iterations and once per colour
//   6. CSApplyConstraintPosition
//   7. CSSolveConstraintVelocity, fixed iterations and once per colour
//
// Resource layout:
//   b0: cbConstraintSolver                         - per-dispatch constants
//   u0: RWStructuredBuffer<GpuRigidBody>           - integrated body state
//   t0: StructuredBuffer<GpuConstraintEndpoint>    - stable-slot endpoint metadata
//   t1: StructuredBuffer<GpuD6ConstraintDesc>      - stable-slot D6 parameters
//   u1: RWStructuredBuffer<GpuConstraintBlock>     - stable-slot runtime blocks
//   u2: RWStructuredBuffer<GpuConstraintRow>       - six canonical rows per stable slot
//   u3: RWStructuredBuffer<uint>                   - one-element colour-overflow flag
//   u4: RWStructuredBuffer<GpuConstraintPseudoVelocity> - per-body split-correction state
//
// Matrices use the row-vector convention from resolve.hlsl. HLSL rows map to C++ columns,
// mul(v, M) transforms a vector, and mul(local, parent) composes local-to-parent-to-world.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

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
};

ConstantBuffer<cbConstraintSolver> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_constraint_bodies, u0);
StructuredBuffer<GpuConstraintEndpoint> resource(g_endpoints, t0);
StructuredBuffer<GpuD6ConstraintDesc> resource(g_descriptors, t1);
RWStructuredBuffer<GpuConstraintBlock> resource(g_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_rows, u2);
RWStructuredBuffer<uint> resource(g_colour_overflow, u3);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_pseudo_velocities, u4);

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

// Compose an endpoint-local compact frame with its body transform, or return it directly for world endpoints.
float4x4 ConstraintFrameToWorld(GpuConstraintFrame frame, int body_idx)
{
	float4x4 local = quat_to_float4x4(frame.rotation, frame.position.xyz);
	return body_idx >= 0 ? mul(local, g_constraint_bodies[body_idx].o2w) : local;
}

// Return a finite scale for a retained physical impulse across ordinary timestep changes.
float ConstraintWarmStartScale()
{
	return isfinite(g.warm_start_scale) && g.warm_start_scale > 0.0f ? g.warm_start_scale : 0.0f;
}

// Compile one canonical scalar row and return its physical and split-position activity.
void CompileConstraintRow(
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
		axis.damping == 0.0f &&
		position_error != 0.0f;

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

	float gamma = 0.0f;
	float bias = 0.0f;
	float denominator = g.timestep * (axis.damping + g.timestep * axis.stiffness);
	if (denominator > 1.0e-20f)
	{
		gamma = 1.0f / denominator;
		bias = position_error * g.timestep * axis.stiffness * gamma;
	}

	float max_impulse = max(axis.max_force * g.timestep, 0.0f);
	float2 bounds = ConstraintImpulseBounds(state, max_impulse);
	float retained = 0.0f;
	if (!reset_warm_start && state == old_state && velocity_active)
		retained = clamp(old_row.bounds.z * ConstraintWarmStartScale(), bounds.x, bounds.y);

	row.solve = float4(position_error, axis.mode == GpuConstraintAxisMode_Driven ? axis.target_velocity : 0.0f, bias, gamma);
	row.bounds = float4(bounds, retained, 0.0f);
	g_rows[row_idx] = row;
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
	bool reset_warm_start = AllSet(endpoint.flags, GpuConstraintEndpointFlags_ResetWarmStart);

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
		CompileConstraintRow(slot_idx, axis_idx, desc.axes[axis_idx], axis_ws, anchor_offset_a, anchor_offset_b, position, old_state, reset_warm_start, state, velocity_active, position_active);
		if (velocity_active)
			block.velocity_mask |= 1u << axis_idx;
		if (position_active)
			block.position_mask |= 1u << axis_idx;
		block.row_states |= state << (2u * axis_idx);
	}

	block.colour = MaxColours;
	block.flags = ConstraintBlockFlags_Active | (reset_warm_start ? ConstraintBlockFlags_ResetWarmStart : 0u);
	g_blocks[slot_idx] = block;
}

// Assign deterministic greedy colours after resetting per-body masks.
numthreads(CSAssignConstraintColours, 1, 1, 1)
void CSAssignConstraintColours(int3 DTID(dtid))
{
	if (dtid.x != 0)
		return;

	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
		g_constraint_bodies[body_idx].colour_used = 0u;

	bool overflow = false;
	for (int slot_idx = 0; slot_idx != g.slot_count; ++slot_idx)
	{
		if (AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
		{
			GpuConstraintBlock coupled_block = g_blocks[slot_idx];
			coupled_block.colour = MaxColours;
			g_blocks[slot_idx] = coupled_block;
			continue;
		}

		GpuConstraintBlock block = g_blocks[slot_idx];
		if (!AllSet(block.flags, ConstraintBlockFlags_Active) || block.velocity_mask == 0u)
		{
			block.colour = MaxColours;
			g_blocks[slot_idx] = block;
			continue;
		}

		uint used = 0u;
		if (ConstraintBodyDynamic(block.body_idx_a))
			used |= g_constraint_bodies[block.body_idx_a].colour_used;
		if (ConstraintBodyDynamic(block.body_idx_b))
			used |= g_constraint_bodies[block.body_idx_b].colour_used;
		uint available = ~used;
		if (available == 0u)
		{
			overflow = true;
			break;
		}

		uint colour = firstbitlow(available);
		block.colour = colour;
		g_blocks[slot_idx] = block;
		if (ConstraintBodyDynamic(block.body_idx_a))
			g_constraint_bodies[block.body_idx_a].colour_used |= 1u << colour;
		if (ConstraintBodyDynamic(block.body_idx_b))
			g_constraint_bodies[block.body_idx_b].colour_used |= 1u << colour;
	}

	// A single exhausted mask selects one coherent serial order for every active block.
	if (overflow)
	{
		for (int slot_idx = 0; slot_idx != g.slot_count; ++slot_idx)
		{
			GpuConstraintBlock block = g_blocks[slot_idx];
			block.colour = MaxColours;
			g_blocks[slot_idx] = block;
		}
	}
	g_colour_overflow[0] = overflow ? 1u : 0u;
}

// Apply a world-space spatial impulse to one endpoint's authoritative momentum.
void ApplyConstraintImpulseToBody(int body_idx, float3 jacobian_ang, float3 jacobian_lin, float impulse)
{
	if (!ConstraintBodyDynamic(body_idx) || impulse == 0.0f || !isfinite(impulse))
		return;

	GpuRigidBody body = g_constraint_bodies[body_idx];
	body.momentum_ang.xyz += impulse * jacobian_ang;
	body.momentum_lin.xyz += impulse * jacobian_lin;

	// Routine support impulses leave an awake body's quiet-time assessment to the sleep pass, while persisted sleeping state must be invalidated immediately.
	if (AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) || body.sleep.island_id >= 0)
	{
		body.sleep.generation++;
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.sleep.timer_s = 0.0f;
		body.sleep.island_id = -1;
		body.sleep.flags = 0u;
	}
	g_constraint_bodies[body_idx] = body;
}

// Apply one scalar row impulse to both endpoints.
void ApplyConstraintRowImpulse(GpuConstraintBlock block, GpuConstraintRow row, float impulse)
{
	ApplyConstraintImpulseToBody(block.body_idx_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, impulse);
	ApplyConstraintImpulseToBody(block.body_idx_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
}

// Apply all retained physical impulses for one block.
void ApplyConstraintBlockWarmStart(uint slot_idx)
{
	if (AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
		return;

	GpuConstraintBlock block = g_blocks[slot_idx];
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		if ((block.velocity_mask & (1u << axis_idx)) == 0u)
			continue;

		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx];
		ApplyConstraintRowImpulse(block, row, row.bounds.z);
	}
}

// Apply cached impulses by colour, or serially in stable-slot order after colour exhaustion.
numthreads(CSApplyConstraintWarmStart, ConstraintThreadCount, 1, 1)
void CSApplyConstraintWarmStart(int3 DTID(dtid))
{
	if (g.colour == 0 && g_colour_overflow[0] != 0u)
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

// Return the current physical row velocity J*v from world-space momenta.
float ConstraintRowVelocity(GpuConstraintBlock block, GpuConstraintRow row)
{
	float velocity = 0.0f;
	if (ConstraintBodyDynamic(block.body_idx_a))
	{
		GpuRigidBody body = g_constraint_bodies[block.body_idx_a];
		float3 angular = mul(ConstraintInverseInertia(block.body_idx_a), body.momentum_ang.xyz);
		float3 linear_velocity = body.os_com_and_invmass.w * body.momentum_lin.xyz;
		velocity += dot(row.jacobian_a_ang.xyz, angular) + dot(row.jacobian_a_lin.xyz, linear_velocity);
	}
	if (ConstraintBodyDynamic(block.body_idx_b))
	{
		GpuRigidBody body = g_constraint_bodies[block.body_idx_b];
		float3 angular = mul(ConstraintInverseInertia(block.body_idx_b), body.momentum_ang.xyz);
		float3 linear_velocity = body.os_com_and_invmass.w * body.momentum_lin.xyz;
		velocity += dot(row.jacobian_b_ang.xyz, angular) + dot(row.jacobian_b_lin.xyz, linear_velocity);
	}
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
		}
		if (physical_solve)
			matrix[row * 6 + row] += lhs.solve.w;
		scale = max(scale, abs(matrix[row * 6 + row]));
	}
	if (!(scale > 1.0e-20f))
		return false;

	float tolerance = max(1.0e-7f * scale, 1.0e-20f);
	if (InvertConstraintMatrix(matrix, row_count, tolerance, inverse))
		return true;

	float regularization = g.regularization * max(scale, 1.0f);
	for (int row = 0; row != row_count; ++row)
		matrix[row * 6 + row] += regularization;
	return InvertConstraintMatrix(matrix, row_count, tolerance, inverse);
}

// Solve one warm-started physical D6 block and commit finite projected impulses immediately.
void SolveConstraintVelocityBlock(uint slot_idx)
{
	if (AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
		return;

	GpuConstraintBlock block = g_blocks[slot_idx];
	uint active_axes[6];
	int row_count;
	float inverse[36];
	if (!PrepareConstraintInverse(slot_idx, block.velocity_mask, true, active_axes, row_count, inverse))
		return;

	float residual[6];
	float candidate[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		residual[row_idx] = ConstraintRowVelocity(block, row) - row.solve.y + row.solve.z + row.solve.w * row.bounds.z;
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += inverse[row_idx * 6 + column] * residual[column];
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		candidate[row_idx] = clamp(row.bounds.z - g.relaxation * correction, row.bounds.x, row.bounds.y);
	}

	// Independent box projection is intentional until persistent descriptors expose friction-cone blocks.
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		uint row_storage_idx = slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx];
		GpuConstraintRow row = g_rows[row_storage_idx];
		if (!isfinite(candidate[row_idx]))
			continue;

		float delta = candidate[row_idx] - row.bounds.z;
		ApplyConstraintRowImpulse(block, row, delta);
		row.bounds.z = candidate[row_idx];
		g_rows[row_storage_idx] = row;
	}
}

// Run one coloured physical PGS sweep, with a stable serial fallback on overflow.
numthreads(CSSolveConstraintVelocity, ConstraintThreadCount, 1, 1)
void CSSolveConstraintVelocity(int3 DTID(dtid))
{
	if (g.colour == 0 && g_colour_overflow[0] != 0u)
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

// Return the current split-correction row velocity without consulting physical momentum.
float ConstraintPseudoRowVelocity(GpuConstraintBlock block, GpuConstraintRow row)
{
	float velocity = 0.0f;
	if (ConstraintBodyDynamic(block.body_idx_a))
	{
		GpuConstraintPseudoVelocity pseudo = g_pseudo_velocities[block.body_idx_a];
		velocity += dot(row.jacobian_a_ang.xyz, pseudo.angular_velocity.xyz) + dot(row.jacobian_a_lin.xyz, pseudo.linear_velocity.xyz);
	}
	if (ConstraintBodyDynamic(block.body_idx_b))
	{
		GpuConstraintPseudoVelocity pseudo = g_pseudo_velocities[block.body_idx_b];
		velocity += dot(row.jacobian_b_ang.xyz, pseudo.angular_velocity.xyz) + dot(row.jacobian_b_lin.xyz, pseudo.linear_velocity.xyz);
	}
	return velocity;
}

// Apply one scalar split impulse to a body's accumulated pseudo twist.
void ApplyConstraintPseudoImpulseToBody(int body_idx, float3 jacobian_ang, float3 jacobian_lin, float impulse)
{
	if (!ConstraintBodyDynamic(body_idx) || impulse == 0.0f || !isfinite(impulse))
		return;

	GpuConstraintPseudoVelocity pseudo = g_pseudo_velocities[body_idx];
	pseudo.angular_velocity.xyz += mul(ConstraintInverseInertia(body_idx), impulse * jacobian_ang);
	pseudo.linear_velocity.xyz += g_constraint_bodies[body_idx].os_com_and_invmass.w * impulse * jacobian_lin;
	g_pseudo_velocities[body_idx] = pseudo;
}

// Apply one scalar split impulse to both endpoints.
void ApplyConstraintRowPseudoImpulse(GpuConstraintBlock block, GpuConstraintRow row, float impulse)
{
	ApplyConstraintPseudoImpulseToBody(block.body_idx_a, row.jacobian_a_ang.xyz, row.jacobian_a_lin.xyz, impulse);
	ApplyConstraintPseudoImpulseToBody(block.body_idx_b, row.jacobian_b_ang.xyz, row.jacobian_b_lin.xyz, impulse);
}

// Solve one hard passive position block against the accumulated split pseudo-twist.
void SolveConstraintPositionBlock(uint slot_idx)
{
	if (AllSet(g_endpoints[slot_idx].flags, GpuConstraintEndpointFlags_Coupled))
		return;

	GpuConstraintBlock block = g_blocks[slot_idx];
	uint active_axes[6];
	int row_count;
	float inverse[36];
	if (!PrepareConstraintInverse(slot_idx, block.position_mask, false, active_axes, row_count, inverse))
		return;

	float residual[6];
	float impulse[6];
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		float correction = g.position_beta * row.solve.x / g.timestep;
		float target_velocity = -clamp(correction, -g.max_position_speed, +g.max_position_speed);
		residual[row_idx] = ConstraintPseudoRowVelocity(block, row) - target_velocity;
	}
	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		float correction = 0.0f;
		for (int column = 0; column != row_count; ++column)
			correction += inverse[row_idx * 6 + column] * residual[column];
		GpuConstraintRow row = g_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx]];
		uint state = (block.row_states >> (2u * active_axes[row_idx])) & 3u;
		float2 bounds = ConstraintImpulseBounds(state, 3.402823466e+38f);
		impulse[row_idx] = clamp(row.bounds.w - g.position_relaxation * correction, bounds.x, bounds.y);
	}

	for (int row_idx = 0; row_idx != row_count; ++row_idx)
	{
		if (!isfinite(impulse[row_idx]))
			continue;

		uint row_storage_idx = slot_idx * GpuConstraintRowsPerBlock + active_axes[row_idx];
		GpuConstraintRow row = g_rows[row_storage_idx];
		float delta = impulse[row_idx] - row.bounds.w;
		ApplyConstraintRowPseudoImpulse(block, row, delta);
		row.bounds.w = impulse[row_idx];
		g_rows[row_storage_idx] = row;
	}
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
	if (g.colour == 0 && g_colour_overflow[0] != 0u)
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

#ifdef __cplusplus
}
#endif
