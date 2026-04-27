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
#include "pr/hlsl/geometry.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Per-dispatch constants
struct cbResolve
{
	int max_contacts; // The max capacity of the contacts buffer
	int body_count;   // The number of bodies in the scene
	int colour;       // Current colour batch being processed (for CSResolve)
	int pad0;

	float dt;         // timestep in seconds
	float sleep_velocity_threshold_lin;
	float sleep_velocity_threshold_ang;
	float pad1;

	float penetration_slop;
	float velocity_baumgarte;
	float deep_penetration_threshold;
	float deep_penetration_range;

	float deep_penetration_baumgarte_min;
	float deep_penetration_baumgarte_max;
	float pad2;
	float pad3;

	float position_slop;
	float position_baumgarte;
	float position_correction_scale;
	float pad4;
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
	float3 com_ws = body.o2w[3].xyz + mul(os_com, rot);

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

// Compute a body's velocity at a point, expressed in A's object space.
// 'rot_a' transforms from world space to A's object space.
float3 BodyVelocityAtPoint(GpuRigidBody body, float3x3 os_iinv, float3 pt_in_a, float3 com_in_a, float3x3 rot_a)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 omega_in_a = mul(rot_a, mul(ws_iinv, body.momentum_ang.xyz));
	float3 v_com_in_a = mul(rot_a, inv_mass * body.momentum_lin.xyz);
	return v_com_in_a + cross(omega_in_a, pt_in_a - com_in_a);
}

// Compute the relative velocity of B w.r.t. A at the contact point, in A's object space.
float3 RelativeVelocityAtContact(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB,	float3x3 os_iinv_a, float3x3 os_iinv_b, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
{
	float3 pt = c.contact_point.xyz;
	float3 v_a = BodyVelocityAtPoint(bodyA, os_iinv_a, pt, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(bodyB, os_iinv_b, pt, com_b_in_a, rot_a);
	return v_b - v_a;
}

// Build the 3x3 collision mass matrix from lever arms and inverse inertias.
// Returns the inverse of the collision mass matrix (used to compute impulse from velocity).
float3x3 CollisionMassMatrix(float3 rA, float3 rB, float inv_mass_a, float inv_mass_b, float3x3 Ia_inv, float3x3 Ib_inv)
{
	float3x3 cpm_rA = CrossProductMatrix(rA);
	float3x3 cpm_rB = CrossProductMatrix(rB);

	float3x3 I = float3x3(1,0,0, 0,1,0, 0,0,1);
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

// Convert a point impulse at 'pt' (in A's space) into world-space momentum changes,
// apply to both bodies, and clamp using an energy conservation guard.
void ApplyImpulseWithEnergyGuard(
	inout GpuRigidBody bodyA, inout GpuRigidBody bodyB,
	float3 impulse, float3 pt, float3 com_a_in_a, float3 com_b_in_a,
	float3x3 rot_a, float3x3 ws_iinv_a, float3x3 ws_iinv_b,
	float inv_mass_a, float inv_mass_b)
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

	// Energy conservation guard: if KE increased, scale the impulse down
	float ke_after = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	float delta = ke_after - ke_before;
	if (delta > 0 && A > 1e-12f)
	{
		float correction = 1.0f - clamp((A - delta) / A, 0.0f, 1.0f);
		bodyA.momentum_ang.xyz -= correction * torqueA_ws;
		bodyA.momentum_lin.xyz -= correction * forceA_ws;
		bodyB.momentum_ang.xyz -= correction * torqueB_ws;
		bodyB.momentum_lin.xyz -= correction * forceB_ws;
	}
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

void ApplyPositionCorrection(GpuResolveContact c)
{
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float total_inv = inv_mass_a + inv_mass_b;
	float correction = PositionCorrectionDistance(c.depth);
	if (correction <= 0.0f || total_inv <= 0.0f)
		return;

	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	float lift_a = correction * (inv_mass_a / total_inv);
	float lift_b = correction * (inv_mass_b / total_inv);
	float4 posA = bodyA.o2w[3];
	float4 posB = bodyB.o2w[3];
	posA.xyz -= lift_a * axis_ws;
	posB.xyz += lift_b * axis_ws;
	bodyA.o2w[3] = posA;
	bodyB.o2w[3] = posB;

	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

// Add a contact point to the body's contact simplex, maintaining a maximum of 4 points.
// The .w component stores the body index of the support body (as asfloat(int)).
void AddSupportContact(inout GpuRigidBody body, float3 ws_pt, int support_body_idx)
{
	if (body.contact_simplex_count == 4)
	{
		body.contact_simplex[0] = body.contact_simplex[1];
		body.contact_simplex[1] = body.contact_simplex[2];
		body.contact_simplex[2] = body.contact_simplex[3];
		body.contact_simplex_count = 3;
	}
	body.contact_simplex[body.contact_simplex_count] = float4(ws_pt, asfloat(support_body_idx));
	body.contact_simplex_count++;
}

// Test if the body's CoM projects inside the contact simplex along the 'down' direction.
bool IsSupportedBySimplex(GpuRigidBody body, float3 down)
{
	int simplex_count = min(body.contact_simplex_count, 4);
	if (simplex_count < 3)
		return false;

	// Get the CoM position in world space relative to the body's origin
	float3 com_ws_rel = mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
	float3 com = Project(com_ws_rel, down);

	// Project the simplex and the CoM into the plane perpendicular to 'down' (the support plane)
	float3 pt0 = Project(body.contact_simplex[0].xyz, down);
	float3 pt1 = Project(body.contact_simplex[1].xyz, down);
	float3 pt2 = Project(body.contact_simplex[2].xyz, down);
	float3 pt3 = Project(body.contact_simplex[3].xyz, down); // Will be invalid if count is 3, but that's ok because we won't use it in that case

	// If the contact points are degenerate and near the CoM, we can still consider it supported.
	//const float threshold = 1e-3f;
	//bool4 near_com = bool4(
	//	length_sq(com - pt0) < sqr(threshold),
	//	length_sq(com - pt1) < sqr(threshold),
	//	length_sq(com - pt2) < sqr(threshold),
	//	length_sq(com - pt3) < sqr(threshold));

	bool r0 = PointInTriangle(com, pt0, pt1, pt2) != 0;
	if (simplex_count == 3)
		return r0;// || all(near_com.xyz);

	// For 4 points, check all 4 triangle faces of the tetrahedron. If any are valid, the CoM is supported.
	bool r1 = PointInTriangle(com, pt0, pt1, pt3) != 0;
	bool r2 = PointInTriangle(com, pt0, pt2, pt3) != 0;
	bool r3 = PointInTriangle(com, pt1, pt2, pt3) != 0;
	return (r0 || r1 || r2 || r3);// || all(near_com);
}

// Check if any body referenced by the contact simplex has woken up.
// If a support body is no longer sleeping (and is not static), this body should also wake.
bool SupportStillSleeping(GpuRigidBody body)
{
	int count = min(body.contact_simplex_count, 4);
	for (int i = 0; i != count; ++i)
	{
		int support_idx = asint(body.contact_simplex[i].w);
		GpuRigidBody support = g_bodies[support_idx];
	
		// Static bodies (inv_mass == 0) are always valid support
		float support_inv_mass = support.os_com_and_invmass.w;
		if (support_inv_mass <= 0 || HasFlag(support.state_flags, ERigidBodyStateFlags_Static))
			continue;

		// If the support body is no longer sleeping, our support is lost
		if (!HasFlag(support.state_flags, ERigidBodyStateFlags_Sleeping))
			return false;
	}
	return true;
}

// ----- CSComputeCollisionTimes -----
// Parallel: one thread per contact. Computes collision time
numthreads(CSComputeCollisionTimes, ResolveThreadCount, 1, 1)
void CSComputeCollisionTimes(int3 dtid : SV_DispatchThreadID)
{
	int idx = dtid.x;
	if (idx < g_counters[0].contact_count)
	{
		// Calculate the estimated collision time for this contact
		GpuResolveContact c = g_contacts[idx];
		float collision_time = EstimateCollisionTime(c);
		g_contacts[idx].collision_time = collision_time;

		// Transform the contact point to world space to compute its height along gravity for tie-breaking in sorting.
		float3x3 rot_a = (float3x3)g_bodies[c.body_idx_a].o2w;
		float3 ws_point = g_bodies[c.body_idx_a].o2w[3].xyz + mul(c.contact_point.xyz, rot_a);

		// Get the direction of gravity for this contact so we can determine "down" for height sorting.
		// Use body A, if the objects are in contact then gravity should be similar for both bodies.
		float3 gravity = NormaliseOrZero(g_bodies[c.body_idx_a].ws_gravity).xyz;
		float height = dot(ws_point, gravity);
		
		// Compute a composite sort key: collision_time primary, contact height secondary.
		// For contacts with similar collision times (e.g., a stacked column landing),
		// lower contacts (closer to the support surface) sort first so the impulse propagates upward through the stack.
		// Height is measured along the gravity direction — contacts further along -gravity sort earlier.
		float height_bias = height * 1e-6f; // contacts lower along gravity sort first (more negative = earlier)
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
		for (i = g_counters[0].contact_count; i != g.max_contacts; ++i)
			g_contact_times[i] = 1e30f;
	}
}

// ----- CSAssignColours -----
// Serial: single thread. Walks contacts in order sorted by collision time.
// Uses per-body colour bitmasks stored in g_bodies[].colour_used.
numthreads(CSAssignColours, 1, 1, 1)
void CSAssignColours(int3 dtid : SV_DispatchThreadID)
{
	for (int i = 0; i != g_counters[0].contact_count; ++i)
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
		uint colour = min(firstbitlow(~used), MaxColours);

		g_colours[idx] = colour;
		if (a_dynamic) g_bodies[a].colour_used |= (1u << colour);
		if (b_dynamic) g_bodies[b].colour_used |= (1u << colour);
	}
}

// ----- CSPositionSolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact and only moves body transforms.
numthreads(CSPositionSolve, ResolveThreadCount, 1, 1)
void CSPositionSolve(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= g_counters[0].contact_count)
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	ApplyPositionCorrection(g_contacts[idx]);
}

// ----- CSResolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact.
// Only contacts matching the current colour are processed — all others return immediately.
numthreads(CSResolve, ResolveThreadCount, 1, 1)
void CSResolve(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= g_counters[0].contact_count)
		return;

	uint idx = g_contact_order[dtid.x];

	// Only process contacts assigned to the current colour batch
	if (g_colours[idx] != (uint)g.colour)
		return;

	// Load the contact and both bodies
	GpuResolveContact c = g_contacts[idx];
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

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
	float3 pt = c.contact_point.xyz;

	// Adjust the contact point to the estimated collision time.
	// Only adjust along the contact normal to avoid introducing tangential offsets
	// that produce spurious torque for face-face contacts. The full V_rel includes
	// tangential sliding velocity which would shift the contact point sideways,
	// creating a systematic lever arm that generates angular momentum from normal impulses.
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

	// Compute relative velocity at the contact point (in A's object space)
	float3 V_rel = RelativeVelocityAtContact(c, bodyA, bodyB, os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);

	// Adjust contact point along the normal only
	if (abs(ct) > 1e-6f)
		pt += (0.5f * ct * dot(V_rel, axis)) * axis;

	// Position correction is handled separately by CSPositionSolve. This pass only changes momenta.

	// Baumgarte velocity bias: add a corrective velocity proportional to penetration depth.
	// This prevents resting contacts from sinking under sustained load (e.g., stacked boxes)
	// and ensures the impulse solver produces a non-zero separation impulse for shallow
	// penetrations that the position correction skips.
	float bias = (g.velocity_baumgarte / g.dt) * max(c.depth - g.penetration_slop, 0.0f);

	// Skip if already separating faster than the bias correction requires
	float closing_speed = dot(V_rel, axis);
	if (closing_speed > bias)
		return;

	// Inject the bias as a virtual closing velocity so the impulse computation
	// generates a corrective force to push overlapping bodies apart.
	V_rel -= bias * axis;

	// Build collision mass matrix
	float3x3 col_I = CollisionMassMatrix(
		pt - com_a_in_a, pt - com_b_in_a,
		inv_mass_a, inv_mass_b,
		os_iinv_a, rotate_inertia_inv(os_iinv_b, b2a_rot));

	// Load material properties
	GpuMaterial mat_a = g_materials[c.mat_id_a];
	GpuMaterial mat_b = g_materials[c.mat_id_b];

	// For resting contacts (low closing speed), reduce elasticity to prevent bouncing.
	float rest_factor = saturate(abs(closing_speed) * 10.0f); // fade from 0 at rest to 1 at speed 0.1
	float elasticity = rest_factor * (mat_a.elasticity_norm + mat_b.elasticity_norm) * 0.5f;
	float friction = sqrt(mat_a.friction_static * mat_b.friction_static);

	// Compute impulse with friction cone clamping
	float3 impulse = ComputeImpulse(col_I, V_rel, axis, elasticity, friction);

	// Apply impulse with energy conservation guard
	ApplyImpulseWithEnergyGuard(bodyA, bodyB, impulse, pt,
		com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
		inv_mass_a, inv_mass_b);

	// Add support contacts for sleep testing.
	// Body A is pushed in -axis direction, body B in +axis direction.
	// A support contact is one where the push direction opposes gravity (holding the body up).
	// The .w component stores the index of the OTHER body (the support provider).
	float3 ws_axis = mul(axis, rot_a); // contact normal in world space
	float4 ws_contact = mul(float4(pt, 1), bodyA.o2w);
	if (dot(bodyA.ws_gravity.xyz, ws_axis) > +0.1f) // body A's push is -axis, opposing gravity when dot > 0
	{
		float3 ws_pt = ws_contact.xyz - bodyA.o2w[3].xyz;
		AddSupportContact(bodyA, ws_pt, c.body_idx_b);
	}
	if (dot(bodyB.ws_gravity.xyz, ws_axis) < -0.1f) // body B's push is +axis, opposing gravity when dot < 0
	{
		float3 ws_pt = ws_contact.xyz - bodyB.o2w[3].xyz;
		AddSupportContact(bodyB, ws_pt, c.body_idx_a);
	}
	
	// Write updated bodies
	bodyA.state_flags = SetFlag(bodyA.state_flags, ERigidBodyStateFlags_Collided, true);
	bodyB.state_flags = SetFlag(bodyB.state_flags, ERigidBodyStateFlags_Collided, true);
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

// ----- CSUpdateSleepState -----
// One thread per body. Determines whether the body should sleep or wake based on:
//   1. Momentum below threshold
//   2. Gravity is non-zero (no sleeping in zero-g)
//   3. Contact simplex indicates support (CoM above contact points, or degenerate fallback)
//   4. Support bodies are still sleeping/static (cascade wake-up if support wakes)
numthreads(CSUpdateSleepState, ResolveThreadCount, 1, 1)
void CSUpdateSleepState(int3 dtid : SV_DispatchThreadID)
{
	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	GpuRigidBody body = g_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;

	// Static bodies (inv_mass == 0) are always "sleeping" — skip
	if (inv_mass <= 0 || HasFlag(body.state_flags, ERigidBodyStateFlags_Static))
		return;

	// Only bodies with near zero velocities can sleep
	float vel_lin_sq = dot(body.momentum_lin.xyz, body.momentum_lin.xyz) * sqr(inv_mass);
	float vel_ang_sq = dot(body.momentum_ang.xyz, body.momentum_ang.xyz) * sqr(inv_mass);

	// Sleeping requires resting contact which requires gravity to define "resting"
	float grav_len = length(body.ws_gravity.xyz);
	bool has_gravity = grav_len > 1e-6f;

	// Check basic sleep conditions
	bool can_sleep =
		vel_lin_sq < sqr(g.sleep_velocity_threshold_lin) &&
		vel_ang_sq < sqr(g.sleep_velocity_threshold_ang) &&
		has_gravity &&
		SupportStillSleeping(body);

	// Clear sleep flag and simplex if velocity/gravity conditions not met
	if (!can_sleep)
	{
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.contact_simplex_count = 0;
		g_bodies[body_idx] = body;
		return;
	}

	// If the body could sleep, but doesn't have enough contact points yet, keep waiting
	if (body.contact_simplex_count < 3)
	{
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		g_bodies[body_idx] = body;
		return;
	}

	// Test if the body is supported by its contact simplex.
	bool is_supported = IsSupportedBySimplex(body, body.ws_gravity.xyz / grav_len);
	body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, is_supported);
	g_bodies[body_idx] = body;
}

#ifdef __cplusplus
}
#endif
