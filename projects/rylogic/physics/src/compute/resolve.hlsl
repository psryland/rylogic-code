//*********************************************
// Physics Engine — GPU Collision Resolution Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Graph-coloured batch collision resolution running on the GPU.
//
// Pipeline:
//   1. CSComputeCollisionTimes — estimates contact time and prepares sorting keys
//   2. CSAssignColours         — assigns graph colours to sorted contacts
//   3. CSPositionSolve         — split position correction, dispatched per colour
//   4. CSResolve               — velocity impulse solve, dispatched per colour
//
// Within a colour batch, no two contacts share a body, so writes to body
// momenta are data-race free.
//
// Buffer layout:
//   b0: cbuffer with colour index or max contacts
//   u0: RWStructuredBuffer<GpuRigidBody>        — per-body dynamic state (read/write)
//   u1: RWStructuredBuffer<uint>                — per-contact colour assignment
//   u2: RWStructuredBuffer<GpuResolveContact>   — prepared contacts (output of CSPrepareContacts)
//   t0: StructuredBuffer<GpuCollisionCounters>  — counters (body_count, pair_count, contact_count)
//   t1: StructuredBuffer<GpuContact>            — raw contacts from collision detection
//   t2: StructuredBuffer<GpuCollisionPair>      — collision pairs (for body index lookup)
//   t3: StructuredBuffer<GpuMaterial>           — material properties
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).
//   HLSL 'row_major float4x4' rows = C++ columns = basis vectors.
//   mul(v, M) for vector transforms, mul(A, B) = A then B in row-vector convention.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "src/compute/physics_types.hlsli"

// HLSL flow control hints; expand to nothing in C++ replay builds.
#ifdef __cplusplus
#define PR_HLSL_BRANCH
#define PR_HLSL_UNROLL
#else
#define PR_HLSL_BRANCH [branch]
#define PR_HLSL_UNROLL [unroll]
#endif

#ifdef __cplusplus
namespace pr::physics {
#endif

static const int TgsAngularDriftSubstepMax = 32;
static const float TgsAngularDriftMaxRadians = 0.25f;

// Per-dispatch constants
struct cbResolve
{
	int max_contacts; // The max capacity of the contacts buffer
	int body_count;   // The number of bodies in the scene
	int colour;       // Current colour batch being processed (for CSResolve)
	int tgs_steps;    // Number of Temporal Gauss-Seidel substeps

	float dt;         // timestep in seconds
	float tgs_velocity_bias_max;
	float pad2;
	float pad3;

	float penetration_slop;
	float velocity_baumgarte;
	float deep_penetration_threshold;
	float deep_penetration_range;

	float deep_penetration_baumgarte_min;
	float deep_penetration_baumgarte_max;
	float pad4;
	float pad5;

	float position_slop;
	float position_baumgarte;
	float position_correction_scale;
	float pad6;
};

// Shader resources
ConstantBuffer<cbResolve> resource(g, b0);
StructuredBuffer<GpuCollisionCounters> resource(g_counters, t0);
StructuredBuffer<GpuMaterial> resource(g_materials, t1);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u0);
RWStructuredBuffer<uint> resource(g_colours, u1);
RWStructuredBuffer<GpuResolveContact> resource(g_contacts, u2);
RWStructuredBuffer<float> resource(g_contact_times, u3); // scratch: collision_time keys for radix sort
RWStructuredBuffer<uint> resource(g_contact_order, u4);   // scratch: contact indices for radix sort

// ----- Helper functions -----
int ContactCount()
{
	return min(g_counters[0].contact_count, g.max_contacts);
}

// Compute kinetic energy from momentum and inverse inertia (world space).
// Momentum is at CoM, inertia is block-diagonal.
float KineticEnergy(GpuRigidBody body)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 rot = (float3x3)body.o2w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, rot);

	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;
	return 0.5f * spatial_dot(vel_ang, vel_lin, body.momentum_ang.xyz, body.momentum_lin.xyz);
}

// Keep transform slot access centralised because the C++ replay path indexes m4x4 by storage vector, while the shader path uses explicit matrix elements.
void SetRigidTransform(inout_(GpuRigidBody) body, float3x3 rot, float3 pos)
{
	#ifdef __cplusplus
	body.o2w[0] = float4(rot[0], 0);
	body.o2w[1] = float4(rot[1], 0);
	body.o2w[2] = float4(rot[2], 0);
	body.o2w[3] = float4(pos, 1);
	#else
	body.o2w._m00_m01_m02_m03 = float4(rot[0].x, rot[0].y, rot[0].z, 0.0f);
	body.o2w._m10_m11_m12_m13 = float4(rot[1].x, rot[1].y, rot[1].z, 0.0f);
	body.o2w._m20_m21_m22_m23 = float4(rot[2].x, rot[2].y, rot[2].z, 0.0f);
	body.o2w._m30_m31_m32_m33 = float4(pos.x, pos.y, pos.z, 1.0f);
	#endif
}
float3 BodyOriginWS(GpuRigidBody body)
{
	return mul(float4(0, 0, 0, 1), body.o2w).xyz;
}
void SetBodyOriginWS(inout_(GpuRigidBody) body, float3 pos)
{
	#ifdef __cplusplus
	body.o2w[3] = float4(pos, 1);
	#else
	body.o2w._m30_m31_m32_m33 = float4(pos.x, pos.y, pos.z, 1.0f);
	#endif
}

// Compute a body's object-space inverse inertia matrix (scaled by inv_mass).
float3x3 OsInverseInertia(GpuRigidBody body)
{
	float inv_mass = body.os_com_and_invmass.w;
	return inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
}

// Extrapolate a body's o2w transform by dt using current momentum and forces.
// Mirrors the CPU ExtrapolateO2W: S = S0 + 0.5*dt * I^(2*momentum + force*dt).
// Used to rewind bodies to the estimated collision time.
float4x4 ExtrapolateO2W(GpuRigidBody body, float dt)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3 os_com = body.os_com_and_invmass.xyz;
	float3x3 os_iinv = OsInverseInertia(body);
	float3x3 rot = (float3x3)body.o2w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, rot);

	// Spatial displacement: dx = 0.5 * dt * I^(2*h + f*dt)
	float3 h_ang = 2.0f * body.momentum_ang.xyz + dt * body.force_ang.xyz;
	float3 h_lin = 2.0f * body.momentum_lin.xyz + dt * body.force_lin.xyz;
	float3 dx_ang = 0.5f * dt * mul(ws_iinv, h_ang);
	float3 dx_lin = 0.5f * dt * inv_mass * h_lin;

	// CoM-based position update
	float3 com_ws = mul(float4(os_com, 1), body.o2w).xyz;

	// Small-angle rotation: R_new ≈ (I + [dx_ang×]) * R_old
	float3x3 skew = CrossProductMatrix(dx_ang);
	float3x3 new_rot = rot + mul(skew, rot);

	float3 new_com = com_ws + dx_lin;
	float3 new_pos = new_com - mul(os_com, new_rot);

	return float4x4(
		float4(new_rot[0], 0),
		float4(new_rot[1], 0),
		float4(new_rot[2], 0),
		float4(new_pos, 1));
}

void DriftBody(inout_(GpuRigidBody) body, float dt)
{
	float inv_mass = body.os_com_and_invmass.w;
	if (inv_mass <= 0.0f)
		return;

	float3 os_com = body.os_com_and_invmass.xyz;
	float3x3 os_iinv_unit = build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 rot = (float3x3)body.o2w;
	float3x3 ws_iinv_unit = rotate_inertia_inv(os_iinv_unit, rot);
	float3x3 ws_iinv = inv_mass * ws_iinv_unit;
	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;

	int angular_steps = clamp((int)ceil(length(vel_ang) * abs(dt) / TgsAngularDriftMaxRadians), 1, TgsAngularDriftSubstepMax);
	float angular_dt = dt / (float)angular_steps;
	float3x3 new_rot = rot;
	for (int angular_step = 0; angular_step != angular_steps; ++angular_step)
	{
		float3x3 step_iinv_unit = rotate_inertia_inv(os_iinv_unit, new_rot);
		float3x3 step_iinv = inv_mass * step_iinv_unit;
		float3 step_vel_ang = mul(step_iinv, body.momentum_ang.xyz);

		float3x3 half_dR = rodrigues_rotation(step_vel_ang * (angular_dt * 0.5f));
		float3x3 mid_rot = mul(new_rot, half_dR);
		mid_rot = orthonorm3x3(mid_rot);

		float3x3 mid_iinv_unit = rotate_inertia_inv(os_iinv_unit, mid_rot);
		float3x3 mid_iinv = inv_mass * mid_iinv_unit;
		float3 mid_vel_ang = mul(mid_iinv, body.momentum_ang.xyz);

		float3x3 dR = rodrigues_rotation(mid_vel_ang * angular_dt);
		new_rot = mul(new_rot, dR);
		new_rot = orthonorm3x3(new_rot);
	}

	float3 com_pos = mul(float4(os_com, 1), body.o2w).xyz;
	float3 new_com_pos = com_pos + vel_lin * dt;
	float3 new_pos = new_com_pos - mul(os_com, new_rot);

	SetRigidTransform(body, new_rot, new_pos);
}
void KickBodyGravity(inout_(GpuRigidBody) body, float dt)
{
	float inv_mass = body.os_com_and_invmass.w;
	if (inv_mass <= 0.0f)
		return;

	body.momentum_lin.xyz += body.ws_gravity.xyz * (dt / inv_mass);
}
void TemporalStepBody(inout_(GpuRigidBody) body, float dt)
{
	KickBodyGravity(body, 0.5f * dt);
	DriftBody(body, dt);
	KickBodyGravity(body, 0.5f * dt);
}

// Compute a body's velocity at a point, expressed in A's object space.
// 'rot_a' is body A's object-to-world rotation.
float3 BodyVelocityAtPoint(GpuRigidBody body, float3x3 os_iinv, float3 pt_in_a, float3 com_in_a, float3x3 rot_a)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);

	// World→body A: with HLSL row_major matrices (rows = body axes in world), mul(rot_a, v_world)
	// applies the world→body transformation directly. Using transpose(rot_a) here would give
	// the (geometrically meaningless) sum of body-axes-in-world weighted by world components,
	// which yields wrong body-frame velocities for any non-axis-aligned rotation.
	float3 omega_in_a = mul(rot_a, mul(ws_iinv, body.momentum_ang.xyz));
	float3 v_com_in_a = mul(rot_a, inv_mass * body.momentum_lin.xyz);
	return v_com_in_a + cross(omega_in_a, pt_in_a - com_in_a);
}

// Compute the relative velocity of B w.r.t. A at point 'pt' (in A's object space).
float3 RelativeVelocityAtPoint(GpuRigidBody bodyA, GpuRigidBody bodyB, float3 pt, float3x3 os_iinv_a, float3x3 os_iinv_b, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
{
	float3 v_a = BodyVelocityAtPoint(bodyA, os_iinv_a, pt, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(bodyB, os_iinv_b, pt, com_b_in_a, rot_a);
	return v_b - v_a;
}

// Convenience wrapper: relative velocity at the contact centroid.
float3 RelativeVelocityAtContact(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB,	float3x3 os_iinv_a, float3x3 os_iinv_b, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
{
	return RelativeVelocityAtPoint(bodyA, bodyB, c.contact_point.xyz, os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);
}

// Build the 3x3 collision mass matrix from lever arms and inverse inertias.
// Returns the inverse of the collision mass matrix (used to compute impulse from velocity).
float3x3 CollisionMassMatrix(float3 rA, float3 rB, float inv_mass_a, float inv_mass_b, float3x3 Ia_inv, float3x3 Ib_inv)
{
	float3x3 cpm_rA = CrossProductMatrix(rA);
	float3x3 cpm_rB = CrossProductMatrix(rB);

	float3x3 I = float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1));
	float3x3 col_I_inv = (inv_mass_a * I - mul(mul(cpm_rA, Ia_inv), cpm_rA))
	                    + (inv_mass_b * I - mul(mul(cpm_rB, Ib_inv), cpm_rB));

	return Invert(col_I_inv);
}

// Compute the restitution impulse with Coulomb friction cone clamping.
// Returns the final impulse vector in A's object space.
float3 ComputeImpulse(float3x3 col_I, float3 V_rel, float3 axis, float elasticity, float friction)
{
	float3x3 col_I_inv = Invert(col_I);

	// Decompose into normal and tangential components
	float3 impulse0 = -mul(col_I, V_rel);
	float denom = dot(axis, mul(col_I_inv, axis));
	float3 impulseN = (float3)0;
	if (abs(denom) > 1e-12f)
		impulseN = -(dot(axis, V_rel) / denom) * axis;

	float3 impulseT = impulse0 - impulseN;
	float3 impulse = (1.0f + elasticity) * impulseN + impulseT;

	// Coulomb friction cone clamping
	float clamped_friction = min(friction, 0.9999f);
	float static_friction = clamped_friction / (1.000001f - clamped_friction);
	float Jn = dot(impulse, axis);
	float Jt_sq = max(0.0f, dot(impulse, impulse) - Jn * Jn);
	float Jt = sqrt(Jt_sq);
	if (Jt > static_friction * abs(Jn))
	{
		Jt = static_friction * abs(Jn);
		float impulseT_lenSq = dot(impulseT, impulseT);
		if (impulseT_lenSq > 1e-12f)
			impulseT = Jt * (impulseT / sqrt(impulseT_lenSq));
		impulse = (1.0f + elasticity) * impulseN + impulseT;
	}
	return impulse;
}

// Compute the friction impulse vector for a sequential impulse solver.
// Solves for the tangent impulse that would stop tangential motion at this point,
// then clamps its magnitude to the Coulomb cone (mu * Jn). Returns an impulse vector
// in the tangent plane (i.e. perpendicular to 'axis'). 'V_rel' should be the relative
// velocity *after* the normal impulse has been applied at this point.
float3 ComputeFrictionImpulse(float3x3 col_I_inv, float3 V_rel, float3 axis, float friction, float Jn)
{
	float3 V_tan = V_rel - dot(V_rel, axis) * axis;
	float v_tan_sq = dot(V_tan, V_tan);
	if (v_tan_sq < 1e-12f)
		return float3(0, 0, 0);

	float v_tan_len = sqrt(v_tan_sq);
	float3 tangent = V_tan / v_tan_len;
	float k_t = dot(tangent, mul(col_I_inv, tangent));
	if (k_t < 1e-12f)
		return float3(0, 0, 0);

	float Jt = v_tan_len / k_t;             // impulse magnitude that fully stops tangential motion
	Jt = min(Jt, friction * abs(Jn));       // Coulomb cone clamp using this point's normal impulse
	return -Jt * tangent;                   // opposes tangential motion
}

// Convert a point impulse at 'pt' (in A's space) into world-space momentum changes,
// apply to both bodies, and clamp using an energy conservation guard.
void ApplyImpulseWithEnergyGuard(
	inout_(GpuRigidBody) bodyA, inout_(GpuRigidBody) bodyB,
	float3 impulse, float3 pt, float3 com_a_in_a, float3 com_b_in_a,
	float3x3 rot_a, float3x3 ws_iinv_a, float3x3 ws_iinv_b,
	float inv_mass_a, float inv_mass_b, float max_ke_gain)
{
	// Convert point impulse to spatial wrenches at each body's CoM
	float3 forceA_in_a = -impulse;
	float3 torqueA_in_a = -cross(impulse, com_a_in_a - pt);
	float3 forceB_in_a = impulse;
	float3 torqueB_in_a = cross(impulse, com_b_in_a - pt);

	// Transform wrenches to world space
	float3 torqueA_ws = mul(torqueA_in_a, rot_a);
	float3 forceA_ws = mul(forceA_in_a, rot_a);
	float3 torqueB_ws = mul(torqueB_in_a, rot_a);
	float3 forceB_ws = mul(forceB_in_a, rot_a);

	// Pre-compute impulse KE coefficient for energy guard
	float3 va_j_ang = mul(ws_iinv_a, torqueA_ws);
	float3 va_j_lin = inv_mass_a * forceA_ws;
	float3 vb_j_ang = mul(ws_iinv_b, torqueB_ws);
	float3 vb_j_lin = inv_mass_b * forceB_ws;
	float A = 0.5f * (spatial_dot(va_j_ang, va_j_lin, torqueA_ws, forceA_ws)
	                 + spatial_dot(vb_j_ang, vb_j_lin, torqueB_ws, forceB_ws));

	float ke_before = KineticEnergy(bodyA) + KineticEnergy(bodyB);

	// Apply impulses to body momenta
	bodyA.momentum_ang.xyz += torqueA_ws;
	bodyA.momentum_lin.xyz += forceA_ws;
	bodyB.momentum_ang.xyz += torqueB_ws;
	bodyB.momentum_lin.xyz += forceB_ws;

	// Energy guard: if KE increased beyond the allowed budget, scale the impulse down
	float ke_after = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	float delta = ke_after - ke_before;
	max_ke_gain = max(max_ke_gain, 0.0f);
	if (delta > max_ke_gain && A > 1e-12f)
	{
		float B = delta - A;
		float scale = clamp((-B + sqrt(max(B * B + 4.0f * A * max_ke_gain, 0.0f))) / (2.0f * A), 0.0f, 1.0f);
		float correction = 1.0f - scale;
		bodyA.momentum_ang.xyz -= correction * torqueA_ws;
		bodyA.momentum_lin.xyz -= correction * forceA_ws;
		bodyB.momentum_ang.xyz -= correction * torqueB_ws;
		bodyB.momentum_lin.xyz -= correction * forceB_ws;
	}
}
float BiasEnergyGainLimit(float bias, float vn_now, float k_n)
{
	if (k_n <= 1e-12f)
		return 0.0f;

	// The Baumgarte pseudo-impulse is allowed to add only the kinetic energy implied by changing the normal relative velocity to 'bias'.
	return max(0.0f, 0.5f * (bias * bias - vn_now * vn_now) / k_n);
}

// Estimate the sub-step collision time for a contact.
// Projects the contact point backward along the relative velocity to find when penetration began.
// Returns a negative value (earlier in the timestep = more negative).
float EstimateCollisionTime(GpuResolveContact c)
{
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

	float3 os_com_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);

	float3 rel_vel = RelativeVelocityAtContact(c, bodyA, bodyB,
		OsInverseInertia(bodyA), OsInverseInertia(bodyB),
		rot_a, os_com_a, com_b_in_a);

	// Project backward to estimate collision time
	float3 point_at_t0 = c.contact_point.xyz - g.dt * rel_vel;
	float distance = abs(dot(c.contact_point.xyz - point_at_t0, c.axis.xyz));
	float sub_step = distance > c.depth ? -c.depth / distance : 0.0f;
	return sub_step * g.dt;
}

float PositionCorrectionDistance(float depth)
{
	float position_pen = max(depth - g.position_slop, 0.0f);
	float position_correction = g.position_baumgarte * position_pen;

	float deep_pen = max(depth - g.deep_penetration_threshold, 0.0f);
	float deep_range = max(g.deep_penetration_range, 1e-6f);
	float deep_baumgarte = lerp(g.deep_penetration_baumgarte_min, g.deep_penetration_baumgarte_max, saturate(deep_pen / deep_range));
	float deep_correction = deep_baumgarte * deep_pen;

	return g.position_correction_scale * max(position_correction, deep_correction);
}

float CurrentContactDepth(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB)
{
	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	float3 pt_a_ws = mul(float4(c.contact_point.xyz, 1), bodyA.o2w).xyz;
	float4x4 a2b_initial = InvertOrthonormal(c.b2a);
	float3 pt_b = mul(float4(c.contact_point.xyz, 1), a2b_initial).xyz;
	float3 pt_b_ws = mul(float4(pt_b, 1), bodyB.o2w).xyz;
	return max(c.depth - dot(pt_b_ws - pt_a_ws, axis_ws), 0.0f);
}
float EffectiveContactDepth(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB)
{
	return g.tgs_steps > 1 ? CurrentContactDepth(c, bodyA, bodyB) : c.depth;
}

void ApplyPositionCorrection(GpuResolveContact c)
{
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float total_inv = inv_mass_a + inv_mass_b;
	float correction = PositionCorrectionDistance(EffectiveContactDepth(c, bodyA, bodyB));
	if (correction <= 0.0f || total_inv <= 0.0f)
		return;

	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	float lift_a = correction * (inv_mass_a / total_inv);
	float lift_b = correction * (inv_mass_b / total_inv);
	float3 posA = BodyOriginWS(bodyA);
	float3 posB = BodyOriginWS(bodyB);
	SetBodyOriginWS(bodyA, posA - lift_a * axis_ws);
	SetBodyOriginWS(bodyB, posB + lift_b * axis_ws);

	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

// ----- CSComputeCollisionTimes -----
// Parallel: one thread per contact. Computes collision time
numthreads(CSComputeCollisionTimes, ResolveThreadCount, 1, 1)
void CSComputeCollisionTimes(int3 DTID(dtid))
{
	int idx = dtid.x;
	int contact_count = ContactCount();
	if (idx < contact_count)
	{
		// Calculate the estimated collision time for this contact
		GpuResolveContact c = g_contacts[idx];
		float collision_time = EstimateCollisionTime(c);
		g_contacts[idx].collision_time = collision_time;

		// Transform the contact point to world space to compute its height along gravity for tie-breaking in sorting.
		float3x3 rot_a = (float3x3)g_bodies[c.body_idx_a].o2w;
		float3 ws_point = mul(float4(c.contact_point.xyz, 1), g_bodies[c.body_idx_a].o2w).xyz;

		// Get the direction of gravity for this contact so we can determine "up" for height sorting.
		// Use body A, if the objects are in contact then gravity should be similar for both bodies.
		float3 gravity = NormaliseOrZero(g_bodies[c.body_idx_a].ws_gravity).xyz;
		float height = -dot(ws_point, gravity);
		
		// Compute a composite sort key: collision_time primary, contact height secondary.
		// For contacts with similar collision times (e.g., a stacked column landing),
		// lower contacts (closer to the support surface) sort first so the impulse propagates upward through the stack.
		// Height is measured against the gravity direction, so lower contacts have smaller height and sort earlier.
		float height_bias = height * 1e-6f;
		g_contact_times[idx] = collision_time + height_bias;
		g_contact_order[idx] = idx;

		// Position correction is applied after graph colouring by CSPositionSolve so contacts sharing a dynamic body are never written in parallel.
	}

	// Get thread 0 to do serial operations
	if (idx == 0)
	{
		int i;
	
		// Zero the body colour_used bitmask (one thread per body, reusing the same dispatch)
		for (i = 0; i != g.body_count; ++i)
			g_bodies[i].colour_used = 0;
		
		// Set the out of bounds contact times to a large positive value so they sort to the end
		for (i = contact_count; i != g.max_contacts; ++i)
			g_contact_times[i] = 1e30f;
	}
}

// ----- CSAssignColours -----
// Serial: single thread. Walks contacts in order sorted by collision time.
// Uses per-body colour bitmasks stored in g_bodies[].colour_used.
numthreads(CSAssignColours, 1, 1, 1)
void CSAssignColours(int3 DTID(dtid))
{
	if (dtid.x != 0)
		return;

	int contact_count = ContactCount();
	for (int i = 0; i != contact_count; ++i)
	{
		int idx = g_contact_order[i]; // get contact index from sorted order
		int a = g_contacts[idx].body_idx_a;
		int b = g_contacts[idx].body_idx_b;

		// Only consider colour conflicts for dynamic bodies (inv_mass > 0).
		// Static bodies (inv_mass == 0) never have their momentum changed,
		// so contacts involving them don't conflict with each other.
		bool a_dynamic = g_bodies[a].os_com_and_invmass.w > 0;
		bool b_dynamic = g_bodies[b].os_com_and_invmass.w > 0;
		uint used_a = a_dynamic ? g_bodies[a].colour_used : 0;
		uint used_b = b_dynamic ? g_bodies[b].colour_used : 0;

		uint used = used_a | used_b;
		uint colour = min(firstbitlow(~used), (uint)MaxColours);

		g_colours[idx] = colour;
		if (a_dynamic) g_bodies[a].colour_used |= (1u << colour);
		if (b_dynamic) g_bodies[b].colour_used |= (1u << colour);
	}
}

// ----- CSTemporalDrift -----
// Rewinds or advances TGS state with the same gravity kick-drift-kick structure used by the main integrator.
numthreads(CSTemporalDrift, ResolveThreadCount, 1, 1)
void CSTemporalDrift(int3 DTID(dtid))
{
	int idx = dtid.x;
	if (idx >= g.body_count)
		return;

	GpuRigidBody body = g_bodies[idx];
	TemporalStepBody(body, g.dt);
	g_bodies[idx] = body;
}

// ----- CSPositionSolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact and only moves body transforms.
numthreads(CSPositionSolve, ResolveThreadCount, 1, 1)
void CSPositionSolve(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	ApplyPositionCorrection(g_contacts[idx]);
}

// ----- CSSerialPositionSolve -----
// Single-threaded sorted position correction. Used by TGS experiments where support propagation through tall stacks
// matters more than within-colour parallelism.
numthreads(CSSerialPositionSolve, 1, 1, 1)
void CSSerialPositionSolve(int3 DTID(dtid))
{
	if (dtid.x != 0)
		return;

	int contact_count = ContactCount();
	for (int i = 0; i != contact_count; ++i)
		ApplyPositionCorrection(g_contacts[g_contact_order[i]]);
}

// ----- CSResolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact.
// Only contacts matching the current colour are processed — all others return immediately.
//
// The resolver runs in two phases per contact:
//
//   Phase 1 (centroid): Apply a coupled restitution+friction impulse at the contact centroid
//   using the classic 3D Coulomb-cone-clamped formula. This handles dynamic collisions
//   (elastic bounces, head-on impacts) and matches the analytic 1D elastic solution for
//   symmetric pair-wise contacts (preserving the conservation tests).
//
//   Phase 2 (per manifold point, only when penetrating): Apply a centroid Baumgarte bias
//   impulse with a bounded KE budget, plus per-point friction limited by that point's share
//   of the bias impulse magnitude. This distributes the contact normal force across the
//   manifold for stable stacking (a 4-point face contact can resist torque, a single centroid
//   contact cannot).
void ResolveContact(uint idx)
{
	// Load the contact and both bodies
	GpuResolveContact c = g_contacts[idx];
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];
	float current_depth = EffectiveContactDepth(c, bodyA, bodyB);

	// Rewind the b2a transform to the estimated collision time.
	// This gives contact geometry at the moment of first contact rather than
	// at the post-integration position where the body has already penetrated.
	float ct = c.collision_time;
	if (abs(ct) > 1e-6f)
	{
		float4x4 o2w_a_at_t = ExtrapolateO2W(bodyA, ct);
		float4x4 o2w_b_at_t = ExtrapolateO2W(bodyB, ct);
		c.b2a = mul(o2w_b_at_t, InvertOrthonormal(o2w_a_at_t));
	}

	float3 axis = c.axis.xyz;
	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);

	// Compute inverse inertia tensors
	float3x3 os_iinv_a = OsInverseInertia(bodyA);
	float3x3 os_iinv_b = OsInverseInertia(bodyB);
	float3x3 ws_iinv_a = rotate_inertia_inv(os_iinv_a, rot_a);
	float3x3 ws_iinv_b = rotate_inertia_inv(os_iinv_b, (float3x3)bodyB.o2w);
	float3x3 b2a_iinv_b = rotate_inertia_inv(os_iinv_b, b2a_rot);

	// Load material properties
	GpuMaterial mat_a = g_materials[c.mat_id_a];
	GpuMaterial mat_b = g_materials[c.mat_id_b];
	float friction = sqrt(mat_a.friction_static * mat_b.friction_static);

	// Baumgarte velocity bias: the per-frame separation velocity we'd like to inject
	// to drive penetration to zero. The same depth value is used for every manifold
	// point because the contact carries a single penetration depth covering the manifold.
	float bias = (g.velocity_baumgarte / g.dt) * max(current_depth - g.penetration_slop, 0.0f);
	if (g.tgs_steps > 1)
		bias = min(bias, g.tgs_velocity_bias_max);

	// Number of manifold points to process (1 = no manifold, fall back to centroid).
	// Clamped to GpuContactMaxPoints to keep the [unroll] bounds well-defined.
	int point_count = clamp(max(1, c.feature), 1, GpuContactMaxPoints);

	// Track whether any impulse was applied so we only update support contacts and
	// the Collided flag when the contact was actually doing work.
	bool any_impulse = false;

	// ===== Phase 1: Centroid restitution + friction (energy-guarded) =====
	// This is the dynamic-collision part. Using the centroid (rather than per-point) for
	// the restitution impulse is critical for symmetric pair-wise elastic collisions to
	// match the analytic v' = ((m1-m2)v1 + 2 m2 v2) / (m1+m2) result, because per-point
	// sequential impulses systematically underestimate the total impulse needed.
	{
		float3 pt = c.contact_point.xyz;
		float3 V_rel = RelativeVelocityAtPoint(bodyA, bodyB, pt,
			os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);

		// Adjust the contact point to the estimated collision time along the normal only.
		// Tangential components in V_rel would shift the point sideways, creating a
		// spurious lever arm for the normal impulse.
		if (abs(ct) > 1e-6f)
			pt += (0.5f * ct * dot(V_rel, axis)) * axis;

		float closing_speed = dot(V_rel, axis);

		// Only apply the restitution impulse when actually approaching (vn < 0 with our
		// convention: V_rel = v_b - v_a, axis A→B, so closing => negative vn).
		PR_HLSL_BRANCH
		if (closing_speed < 0.0f)
		{
			// Build collision mass matrix at the centroid for this phase.
			float3x3 col_I = CollisionMassMatrix(
				pt - com_a_in_a, pt - com_b_in_a,
				inv_mass_a, inv_mass_b,
				os_iinv_a, b2a_iinv_b);

			// Reduce elasticity for resting contacts so they don't bounce on micro-impacts.
			float rest_factor = saturate(abs(closing_speed) * 10.0f);
			bool impact_contact = g.tgs_steps == 1 || c.collision_time < -1e-6f;
			float elasticity = impact_contact ? rest_factor * (mat_a.elasticity_norm + mat_b.elasticity_norm) * 0.5f : 0.0f;

			float3 impulse = ComputeImpulse(col_I, V_rel, axis, elasticity, friction);
			ApplyImpulseWithEnergyGuard(bodyA, bodyB, impulse, pt,
				com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
				inv_mass_a, inv_mass_b, 0.0f);
			any_impulse = true;
		}
	}

	// ===== Phase 2: Centroid Baumgarte bias + per-manifold-point friction =====
	// Applying the bias at the centroid (rather than at every manifold point) is essential.
	// A per-point bias creates an angular feedback runaway on asymmetric/tilted contacts:
	// the bias impulse at one corner of the manifold rotates the body, which changes vn at
	// the other corners, allowing fresh bias applications, and so on. The result is
	// unbounded angular and linear momentum injection — the box "explodes" off the ground.
	// A single centroid bias produces no torque, so the depenetration push is purely linear.
	//
	// Friction is still applied per manifold point. Distributing friction across the contact
	// face is what lets a stack of boxes resist sliding/torsion at every contact point. The
	// per-point friction cone limit is mu * (Jn_bias_centroid / point_count) so the total
	// friction across the manifold matches mu * Jn_bias_centroid (Coulomb at the contact).
	PR_HLSL_BRANCH
	if (bias > 0.0f)
	{
		float3 pt_c = c.contact_point.xyz;
		float3 V_rel_c = RelativeVelocityAtPoint(bodyA, bodyB, pt_c,
			os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);

		float3x3 col_I_c = CollisionMassMatrix(
			pt_c - com_a_in_a, pt_c - com_b_in_a,
			inv_mass_a, inv_mass_b,
			os_iinv_a, b2a_iinv_b);
		float3x3 col_I_inv_c = Invert(col_I_c);
		float k_n_c = dot(axis, mul(col_I_inv_c, axis));

		float vn_now = dot(V_rel_c, axis);
		float Jn_bias_centroid = (k_n_c > 1e-12f) ? max(0.0f, (bias - vn_now) / k_n_c) : 0.0f;
		float bias_ke_limit = BiasEnergyGainLimit(bias, vn_now, k_n_c);

		PR_HLSL_BRANCH
		if (Jn_bias_centroid > 0.0f)
		{
			// ----- Centroid bias impulse -----
			// Pseudo-impulse that drives penetration to zero. It uses a finite KE budget
			// based on the requested separation speed, and is applied at the centroid so no
			// torque is generated between manifold points.
			ApplyImpulseWithEnergyGuard(bodyA, bodyB, Jn_bias_centroid * axis, pt_c,
				com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
				inv_mass_a, inv_mass_b, bias_ke_limit);
			any_impulse = true;

			// Per-point friction cone limit: split the centroid Jn equally so the
			// summed friction across the manifold respects the Coulomb limit.
			float Jn_per_point = Jn_bias_centroid / float(point_count);

			// ----- Per-manifold-point friction + support tracking -----
			PR_HLSL_UNROLL
			for (int pi = 0; pi != GpuContactMaxPoints; ++pi)
			{
				if (pi >= point_count)
					continue;

				float3 pt = (point_count > 1) ? c.manifold[pi].xyz : c.contact_point.xyz;

				// V_rel at this point uses the *current* body momenta (post centroid bias
				// and post any earlier points' friction) for Gauss-Seidel convergence.
				float3 V_rel = RelativeVelocityAtPoint(bodyA, bodyB, pt,
					os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);

				float3x3 col_I_pt = CollisionMassMatrix(
					pt - com_a_in_a, pt - com_b_in_a,
					inv_mass_a, inv_mass_b,
					os_iinv_a, b2a_iinv_b);
				float3x3 col_I_inv_pt = Invert(col_I_pt);

				// Friction limited by this point's share of the centroid bias.
				// Energy-guarded because friction is dissipative — should never inject KE.
				float3 friction_impulse = ComputeFrictionImpulse(col_I_inv_pt, V_rel, axis, friction, Jn_per_point);
				if (any(friction_impulse != float3(0, 0, 0)))
				{
					ApplyImpulseWithEnergyGuard(bodyA, bodyB, friction_impulse, pt,
						com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
						inv_mass_a, inv_mass_b, 0.0f);
				}

			}
		}
	}

	// If no point applied any impulse, the contact was fully separating and there's nothing
	// to write back. This avoids touching body memory for trivially separating contacts.
	if (!any_impulse)
		return;

	// Mark both bodies as having taken part in a collision this frame.
	bodyA.state_flags = SetFlag(bodyA.state_flags, ERigidBodyStateFlags_Collided, true);
	bodyB.state_flags = SetFlag(bodyB.state_flags, ERigidBodyStateFlags_Collided, true);
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

numthreads(CSResolve, ResolveThreadCount, 1, 1)
void CSResolve(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];

	// Only process contacts assigned to the current colour batch
	if (g_colours[idx] != (uint)g.colour)
		return;

	ResolveContact(idx);
}

// ----- CSSerialResolve -----
// Single-threaded sorted velocity solve. This preserves the contact sort order exactly, so a support impulse can
// propagate through a stack in one sweep rather than one graph-colour layer per iteration.
numthreads(CSSerialResolve, 1, 1, 1)
void CSSerialResolve(int3 DTID(dtid))
{
	if (dtid.x != 0)
		return;

	int contact_count = ContactCount();
	for (int i = 0; i != contact_count; ++i)
		ResolveContact(g_contact_order[i]);
}

#ifdef __cplusplus
}
#endif
