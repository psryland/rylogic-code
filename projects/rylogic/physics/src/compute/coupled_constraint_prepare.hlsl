//*********************************************
// Physics Engine — Coupled Constraint Preparation
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One lane per stable slot compiles articulation-coupled D6 rows and their exact-self block preconditioner.
//
// Resource layout:
//   b0: cbCoupledConstraintPrepare                         - packed range and numerical controls
//   u0: RWStructuredBuffer<GpuRigidBody>                   - rigid endpoint state
//   t0: StructuredBuffer<GpuConstraintEndpoint>            - stable-slot endpoint metadata
//   t1: StructuredBuffer<GpuD6ConstraintDesc>              - stable-slot D6 parameters
//   t2: StructuredBuffer<GpuCoupledConstraintEndpoint>     - link and compact-mobility ownership
//   t3: StructuredBuffer<GpuConstraintFrame>               - persistent forest-wide link frames
//   t4: StructuredBuffer<GpuArticulationSpatialMobility>   - compact exact self-link mobilities
//   t5: StructuredBuffer<GpuArticulationAbaScratch>        - tree factor validity
//   u1: RWStructuredBuffer<GpuConstraintBlock>             - shared stable-slot runtime blocks
//   u2: RWStructuredBuffer<GpuConstraintRow>               - shared canonical runtime rows
//   u3: RWStructuredBuffer<GpuCoupledConstraintPreconditioner> - optional coupled block inverses

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Bounds and solver controls for one preparation dispatch.
struct cbCoupledConstraintPrepare
{
	int slot_count;
	int body_count;
	int link_count;
	int mobility_count;

	float timestep;
	float regularization;
	float warm_start_scale;
	float pad0;
};

ConstantBuffer<cbCoupledConstraintPrepare> resource(g_coupled_prepare, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_coupled_bodies, u0);
StructuredBuffer<GpuConstraintEndpoint> resource(g_coupled_constraint_endpoints, t0);
StructuredBuffer<GpuD6ConstraintDesc> resource(g_coupled_descriptors, t1);
StructuredBuffer<GpuCoupledConstraintEndpoint> resource(g_coupled_link_endpoints, t2);
StructuredBuffer<GpuConstraintFrame> resource(g_coupled_link_to_world, t3);
StructuredBuffer<GpuArticulationSpatialMobility> resource(g_coupled_link_mobilities, t4);
StructuredBuffer<GpuArticulationAbaScratch> resource(g_coupled_aba_scratch, t5);
RWStructuredBuffer<GpuConstraintBlock> resource(g_coupled_blocks, u1);
RWStructuredBuffer<GpuConstraintRow> resource(g_coupled_rows, u2);
RWStructuredBuffer<GpuCoupledConstraintPreconditioner> resource(g_coupled_preconditioners, u3);

#define PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE coupled_constraint_ops_detail
#include "physics/src/compute/constraint_solver_ops.hlsli"
#undef PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE

// Return one symmetric component from a compact self-link mobility.
float CoupledMobilityComponent(GpuArticulationSpatialMobility mobility, int row, int column)
{
	int packed_index = ConstraintPackedSymmetricIndex(row, column);
	return mobility.packed[packed_index / 4][packed_index % 4];
}

// Return one angular-then-linear component from a row endpoint wrench.
float CoupledWrenchComponent(float3 angular, float3 linear_component, int index)
{
	return index < 3 ? angular[index] : linear_component[index - 3];
}

// Return true when a rigid endpoint has finite positive inverse mass and may contribute to the local response.
bool CoupledRigidBodyDynamic(int body_idx)
{
	return
		body_idx >= 0 &&
		body_idx < g_coupled_prepare.body_count &&
		g_coupled_bodies[body_idx].os_com_and_invmass.w > 0.0f &&
		!AllSet(g_coupled_bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
}

// Return the world-space inverse inertia at one rigid endpoint's centre of mass.
float3x3 CoupledRigidInverseInertia(int body_idx)
{
	GpuRigidBody body = g_coupled_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	return rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
}

// Return the world-space centre of mass while preserving off-origin rigid models.
float3 CoupledRigidCentreOfMass(int body_idx)
{
	GpuRigidBody body = g_coupled_bodies[body_idx];
	return body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
}

// Return one link's persistent world transform.
float4x4 CoupledLinkToWorld(int link_idx)
{
	GpuConstraintFrame frame = g_coupled_link_to_world[link_idx];
	return quat_to_float4x4(frame.rotation, frame.position.xyz);
}

// Compose one endpoint-local constraint frame with its current rigid, link, or world transform.
float4x4 CoupledFrameToWorld(GpuConstraintFrame frame, int body_idx, int link_idx)
{
	float4x4 local = quat_to_float4x4(frame.rotation, frame.position.xyz);
	if (link_idx >= 0)
		return mul(local, CoupledLinkToWorld(link_idx));
	return body_idx >= 0 ? mul(local, g_coupled_bodies[body_idx].o2w) : local;
}

// Return one endpoint's velocity origin in world space.
float3 CoupledEndpointOrigin(int body_idx, int link_idx)
{
	if (link_idx >= 0)
		return g_coupled_link_to_world[link_idx].position.xyz;
	return body_idx >= 0 ? CoupledRigidCentreOfMass(body_idx) : float3(0.0f, 0.0f, 0.0f);
}

// Express a world-space row wrench in a rigid body's world coordinates or an articulation link's local coordinates.
void CoupledEndpointJacobian(
	bool is_linear,
	float3 axis_ws,
	float3 anchor_offset_ws,
	float sign,
	int body_idx,
	int link_idx,
	out_(float4) angular,
	out_(float4) linear_component)
{
	// World endpoints define constraint geometry but contribute no velocity or impulse response.
	if (body_idx < 0 && link_idx < 0)
	{
		angular = float4(0.0f, 0.0f, 0.0f, 0.0f);
		linear_component = float4(0.0f, 0.0f, 0.0f, 0.0f);
		return;
	}

	float3 angular_ws = is_linear ? sign * cross(anchor_offset_ws, axis_ws) : sign * axis_ws;
	float3 linear_ws = is_linear ? sign * axis_ws : float3(0.0f, 0.0f, 0.0f);
	if (link_idx >= 0)
	{
		float4 world_to_link = quat_conjugate(g_coupled_link_to_world[link_idx].rotation);
		angular_ws = quat_rotate(world_to_link, angular_ws);
		linear_ws = quat_rotate(world_to_link, linear_ws);
	}
	angular = float4(angular_ws, 0.0f);
	linear_component = float4(linear_ws, 0.0f);
}

// Compile one canonical coupled row while retaining only a semantically compatible warm-start impulse.
void CompileCoupledConstraintRow(
	uint slot_idx,
	uint axis_idx,
	GpuConstraintAxisDesc axis,
	float3 axis_ws,
	float3 anchor_offset_a,
	float3 anchor_offset_b,
	int body_idx_a,
	int body_idx_b,
	int link_idx_a,
	int link_idx_b,
	float position,
	uint old_state,
	bool reset_warm_start,
	out_(uint) state,
	out_(bool) velocity_active,
	out_(bool) position_active)
{
	uint row_idx = slot_idx * GpuConstraintRowsPerBlock + axis_idx;
	GpuConstraintRow old_row = g_coupled_rows[row_idx];
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

	bool is_linear = axis_idx < 3u;
	CoupledEndpointJacobian(is_linear, axis_ws, anchor_offset_a, -1.0f, body_idx_a, link_idx_a, row.jacobian_a_ang, row.jacobian_a_lin);
	CoupledEndpointJacobian(is_linear, axis_ws, anchor_offset_b, +1.0f, body_idx_b, link_idx_b, row.jacobian_b_ang, row.jacobian_b_lin);

	float gamma = 0.0f;
	float bias = 0.0f;
	float denominator = g_coupled_prepare.timestep * (axis.damping + g_coupled_prepare.timestep * axis.stiffness);
	if (denominator > 1.0e-20f)
	{
		gamma = 1.0f / denominator;
		bias = position_error * g_coupled_prepare.timestep * axis.stiffness * gamma;
	}

	float max_impulse = max(axis.max_force * g_coupled_prepare.timestep, 0.0f);
	float2 bounds = ConstraintImpulseBounds(state, max_impulse);
	// Coupled warm start remains zero until retained impulses can be applied transactionally to rigid bodies and complete articulation trees.
	float retained = 0.0f;
	row.solve = float4(position_error, axis.mode == GpuConstraintAxisMode_Driven ? axis.target_velocity : 0.0f, bias, gamma);
	row.bounds = float4(bounds, retained, 0.0f);
	g_coupled_rows[row_idx] = row;
}

// Return one rigid endpoint's contribution to an exact-self block response.
float CoupledRigidResponse(int body_idx, float3 lhs_angular, float3 lhs_linear, float3 rhs_angular, float3 rhs_linear)
{
	if (!CoupledRigidBodyDynamic(body_idx))
		return 0.0f;

	GpuRigidBody body = g_coupled_bodies[body_idx];
	return
		dot(lhs_angular, mul(CoupledRigidInverseInertia(body_idx), rhs_angular)) +
		body.os_com_and_invmass.w * dot(lhs_linear, rhs_linear);
}

// Return one articulation endpoint's exact self-link response in link coordinates.
float CoupledLinkResponse(int mobility_idx, float3 lhs_angular, float3 lhs_linear, float3 rhs_angular, float3 rhs_linear)
{
	GpuArticulationSpatialMobility mobility = g_coupled_link_mobilities[mobility_idx];
	float response = 0.0f;
	for (int row = 0; row != 6; ++row)
	for (int column = 0; column != 6; ++column)
	{
		response +=
			CoupledWrenchComponent(lhs_angular, lhs_linear, row) *
			CoupledMobilityComponent(mobility, row, column) *
			CoupledWrenchComponent(rhs_angular, rhs_linear, column);
	}
	return response;
}

// Return the block-local response while deliberately omitting same-tree cross-link terms.
float CoupledConstraintResponse(GpuConstraintBlock block, GpuCoupledConstraintEndpoint endpoint, GpuConstraintRow lhs, GpuConstraintRow rhs)
{
	float response = 0.0f;
	response += endpoint.mobility_idx_a >= 0
		? CoupledLinkResponse(endpoint.mobility_idx_a, lhs.jacobian_a_ang.xyz, lhs.jacobian_a_lin.xyz, rhs.jacobian_a_ang.xyz, rhs.jacobian_a_lin.xyz)
		: CoupledRigidResponse(block.body_idx_a, lhs.jacobian_a_ang.xyz, lhs.jacobian_a_lin.xyz, rhs.jacobian_a_ang.xyz, rhs.jacobian_a_lin.xyz);
	response += endpoint.mobility_idx_b >= 0
		? CoupledLinkResponse(endpoint.mobility_idx_b, lhs.jacobian_b_ang.xyz, lhs.jacobian_b_lin.xyz, rhs.jacobian_b_ang.xyz, rhs.jacobian_b_lin.xyz)
		: CoupledRigidResponse(block.body_idx_b, lhs.jacobian_b_ang.xyz, lhs.jacobian_b_lin.xyz, rhs.jacobian_b_ang.xyz, rhs.jacobian_b_lin.xyz);
	return response;
}

// Build and invert one exact-self block response with the same bounded singular regularization as the rigid lane.
bool PrepareCoupledPreconditioner(
	uint slot_idx,
	GpuConstraintBlock block,
	GpuCoupledConstraintEndpoint endpoint,
	out_(GpuCoupledConstraintPreconditioner) preconditioner)
{
	preconditioner = EmptyCoupledConstraintPreconditioner();
	uint active_axes[6];
	int row_count = 0;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		if ((block.velocity_mask & (1u << axis_idx)) != 0u)
			active_axes[row_count++] = axis_idx;
	if (row_count == 0)
		return false;

	float matrix[36];
	for (int index = 0; index != 36; ++index)
		matrix[index] = 0.0f;
	float scale = 0.0f;
	for (int row = 0; row != row_count; ++row)
	{
		GpuConstraintRow lhs = g_coupled_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[row]];
		for (int column = 0; column != row_count; ++column)
		{
			GpuConstraintRow rhs = g_coupled_rows[slot_idx * GpuConstraintRowsPerBlock + active_axes[column]];
			matrix[row * 6 + column] = CoupledConstraintResponse(block, endpoint, lhs, rhs);
		}
		matrix[row * 6 + row] += lhs.solve.w;
		scale = max(scale, abs(matrix[row * 6 + row]));
	}
	if (!(scale > 1.0e-20f) || !isfinite(scale))
		return false;

	float inverse[36];
	float tolerance = max(1.0e-7f * scale, 1.0e-20f);
	bool valid = InvertConstraintMatrix(matrix, row_count, tolerance, inverse);
	if (!valid)
	{
		float regularization = g_coupled_prepare.regularization * max(scale, 1.0f);
		for (int row = 0; row != row_count; ++row)
			matrix[row * 6 + row] += regularization;
		valid = InvertConstraintMatrix(matrix, row_count, tolerance, inverse);
	}
	if (!valid)
		return false;

	for (int row = 0; row != row_count; ++row)
	for (int column = row; column != row_count; ++column)
	{
		float value = 0.5f * (inverse[row * 6 + column] + inverse[column * 6 + row]);
		if (!isfinite(value))
			return false;
		SetCoupledPreconditionerComponent(preconditioner, row, column, value);
	}
	return true;
}

// Return true when both link endpoints address valid final-configuration mobility factors.
bool CoupledEndpointMetadataValid(GpuCoupledConstraintEndpoint endpoint)
{
	bool link_a =
		endpoint.articulation_idx_a >= 0 &&
		endpoint.link_idx_a >= 0 &&
		endpoint.link_idx_a < g_coupled_prepare.link_count &&
		endpoint.mobility_idx_a >= 0 &&
		endpoint.mobility_idx_a < g_coupled_prepare.mobility_count &&
		endpoint.root_link_idx_a >= 0 &&
		endpoint.root_link_idx_a < g_coupled_prepare.link_count &&
		g_coupled_aba_scratch[endpoint.root_link_idx_a].solve_valid != 0;
	bool link_b =
		endpoint.articulation_idx_b >= 0 &&
		endpoint.link_idx_b >= 0 &&
		endpoint.link_idx_b < g_coupled_prepare.link_count &&
		endpoint.mobility_idx_b >= 0 &&
		endpoint.mobility_idx_b < g_coupled_prepare.mobility_count &&
		endpoint.root_link_idx_b >= 0 &&
		endpoint.root_link_idx_b < g_coupled_prepare.link_count &&
		g_coupled_aba_scratch[endpoint.root_link_idx_b].solve_valid != 0;

	bool rigid_a =
		endpoint.articulation_idx_a < 0 &&
		endpoint.link_idx_a < 0 &&
		endpoint.mobility_idx_a < 0 &&
		endpoint.root_link_idx_a < 0;
	bool rigid_b =
		endpoint.articulation_idx_b < 0 &&
		endpoint.link_idx_b < 0 &&
		endpoint.mobility_idx_b < 0 &&
		endpoint.root_link_idx_b < 0;
	return (link_a || rigid_a) && (link_b || rigid_b) && (link_a || link_b);
}

// Compile one stable coupled slot and publish a preconditioner only after all inputs and factorization succeed.
numthreads(CSPrepareCoupledConstraints, ConstraintThreadCount, 1, 1)
void CSPrepareCoupledConstraints(int3 DTID(dtid))
{
	if (dtid.x >= g_coupled_prepare.slot_count)
		return;

	uint slot_idx = (uint)dtid.x;
	GpuConstraintEndpoint packed_endpoint = g_coupled_constraint_endpoints[slot_idx];
	if (!AllSet(packed_endpoint.flags, GpuConstraintEndpointFlags_Enabled) ||
		!AllSet(packed_endpoint.flags, GpuConstraintEndpointFlags_Coupled))
		return;

	GpuCoupledConstraintEndpoint endpoint = g_coupled_link_endpoints[slot_idx];
	GpuConstraintBlock old_block = g_coupled_blocks[slot_idx];
	GpuConstraintBlock block = EmptyConstraintBlock();
	block.body_idx_a = endpoint.link_idx_a >= 0 ? -1 : packed_endpoint.body_idx_a;
	block.body_idx_b = endpoint.link_idx_b >= 0 ? -1 : packed_endpoint.body_idx_b;
	g_coupled_preconditioners[slot_idx] = EmptyCoupledConstraintPreconditioner();

	// Invalid retained factors disable the complete block rather than allowing partial or non-finite tree response.
	if (!CoupledEndpointMetadataValid(endpoint))
	{
		for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			g_coupled_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx] = EmptyConstraintRow();
		g_coupled_blocks[slot_idx] = block;
		return;
	}

	GpuD6ConstraintDesc desc = g_coupled_descriptors[slot_idx];
	bool any_non_free = false;
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
		any_non_free = any_non_free || desc.axes[axis_idx].mode != GpuConstraintAxisMode_Free;
	if (!any_non_free)
	{
		for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
			g_coupled_rows[slot_idx * GpuConstraintRowsPerBlock + axis_idx] = EmptyConstraintRow();
		g_coupled_blocks[slot_idx] = block;
		return;
	}

	float4x4 frame_a = CoupledFrameToWorld(desc.frame_a, packed_endpoint.body_idx_a, endpoint.link_idx_a);
	float4x4 frame_b = CoupledFrameToWorld(desc.frame_b, packed_endpoint.body_idx_b, endpoint.link_idx_b);
	float3 linear_error = frame_b[3].xyz - frame_a[3].xyz;
	float4 quat_a = quat_from_float3x3((float3x3)frame_a);
	float4 quat_b = quat_from_float3x3((float3x3)frame_b);
	float3 angular_error = quat_rotation_vector(quat_a, quat_b);
	float3 anchor_offset_a = frame_a[3].xyz - CoupledEndpointOrigin(packed_endpoint.body_idx_a, endpoint.link_idx_a);
	float3 anchor_offset_b = frame_b[3].xyz - CoupledEndpointOrigin(packed_endpoint.body_idx_b, endpoint.link_idx_b);
	bool reset_warm_start = AllSet(packed_endpoint.flags, GpuConstraintEndpointFlags_ResetWarmStart);

	// Frame A defines all six canonical world axes before link endpoint wrenches rotate into local coordinates.
	for (uint axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
	{
		uint local_axis = axis_idx % 3u;
		float3 axis_ws = frame_a[local_axis].xyz;
		float position = dot(axis_idx < 3u ? linear_error : angular_error, axis_ws);
		uint old_state = (old_block.row_states >> (2u * axis_idx)) & 3u;
		uint state;
		bool velocity_active;
		bool position_active;
		CompileCoupledConstraintRow(
			slot_idx,
			axis_idx,
			desc.axes[axis_idx],
			axis_ws,
			anchor_offset_a,
			anchor_offset_b,
			packed_endpoint.body_idx_a,
			packed_endpoint.body_idx_b,
			endpoint.link_idx_a,
			endpoint.link_idx_b,
			position,
			old_state,
			reset_warm_start,
			state,
			velocity_active,
			position_active);
		if (velocity_active)
			block.velocity_mask |= 1u << axis_idx;
		if (position_active)
			block.position_mask |= 1u << axis_idx;
		block.row_states |= state << (2u * axis_idx);
	}

	block.colour = MaxColours;
	block.flags = ConstraintBlockFlags_Active | (reset_warm_start ? ConstraintBlockFlags_ResetWarmStart : 0u);
	GpuCoupledConstraintPreconditioner preconditioner;
	if (PrepareCoupledPreconditioner(slot_idx, block, endpoint, preconditioner))
		block.flags |= ConstraintBlockFlags_CoupledPreconditionerValid;
	g_coupled_preconditioners[slot_idx] = preconditioner;
	g_coupled_blocks[slot_idx] = block;
}

#ifdef __cplusplus
}
#endif
