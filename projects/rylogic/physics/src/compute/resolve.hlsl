//*********************************************
// Physics Engine — GPU Collision Resolution Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Graph-coloured batch collision resolution running on the GPU.
//
// Pipeline:
//   1. CSPrepareContacts — converts GpuContact → GpuResolveContact (one thread per contact)
//   2. CSGraphColouring  — assigns colours to contacts, single thread, O(n²)
//   3. CSResolve          — dispatched once per colour from CPU loop (fixed MaxColours iterations)
//
// Within a colour batch, no two contacts share a body, so writes to body
// momenta are data-race free.
//
// Buffer layout:
//   b0: cbuffer with colour index or max contacts
//   u0: RWStructuredBuffer<RigidBodyDynamics>   — per-body dynamic state (read/write)
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
#include "src/compute/rigid_body_dynamics.hlsli"
#include "src/compute/collision_types.hlsli"

// Maximum contacts supported by the single-threaded graph colouring.
static const int MaxContacts = 8192;

// Per-dispatch constants
cbuffer cbResolve : register(b0)
{
	int g_colour; // current colour batch being processed (for CSResolve)
	float g_dt;   // timestep in seconds (for CSGraphColouring collision time estimation)
	int g_pad1;
	int g_pad2;
};

// Shader resources
StructuredBuffer<GpuCollisionCounters> g_counters : register(t0);
StructuredBuffer<GpuMaterial> g_materials : register(t1);
RWStructuredBuffer<RigidBodyDynamics> g_bodies : register(u0);
RWStructuredBuffer<uint> g_colours : register(u1);
RWStructuredBuffer<GpuResolveContact> g_contacts : register(u2); // UAV: CSComputeCollisionTimes writes collision_time; read by all others

// ----- Helper functions -----

// Compute kinetic energy from momentum and inverse inertia (world space).
// Momentum is at CoM, inertia is block-diagonal.
float KineticEnergy(RigidBodyDynamics body)
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
float3x3 OsInverseInertia(RigidBodyDynamics body)
{
	float inv_mass = body.os_com_and_invmass.w;
	return inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
}

// Compute a body's velocity at a point, expressed in A's object space.
// 'rot_a' transforms from world space to A's object space.
float3 BodyVelocityAtPoint(RigidBodyDynamics body, float3x3 os_iinv, float3 pt_in_a, float3 com_in_a, float3x3 rot_a)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 omega_in_a = mul(rot_a, mul(ws_iinv, body.momentum_ang.xyz));
	float3 v_com_in_a = mul(rot_a, inv_mass * body.momentum_lin.xyz);
	return v_com_in_a + cross(omega_in_a, pt_in_a - com_in_a);
}

// Compute the relative velocity of B w.r.t. A at the contact point, in A's object space.
float3 RelativeVelocityAtContact(GpuResolveContact c, RigidBodyDynamics bodyA, RigidBodyDynamics bodyB,
	float3x3 os_iinv_a, float3x3 os_iinv_b, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
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
	inout RigidBodyDynamics bodyA, inout RigidBodyDynamics bodyB,
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
	RigidBodyDynamics bodyA = g_bodies[c.body_idx_a];
	RigidBodyDynamics bodyB = g_bodies[c.body_idx_b];

	float3 os_com_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);

	float3 rel_vel = RelativeVelocityAtContact(c, bodyA, bodyB,
		OsInverseInertia(bodyA), OsInverseInertia(bodyB),
		rot_a, os_com_a, com_b_in_a);

	// Project backward to estimate collision time
	float3 point_at_t0 = c.contact_point.xyz - g_dt * rel_vel;
	float distance = abs(dot(c.contact_point.xyz - point_at_t0, c.axis.xyz));
	float sub_step = distance > c.depth ? -c.depth / distance : 0.0f;
	return sub_step * g_dt;
}

// ----- CSComputeCollisionTimes -----
// Parallel: one thread per contact. Computes collision time and writes to the contact.
// Also zeroes the per-body colour_used bitmask (one thread per body, reusing the same dispatch).
[numthreads(ResolveThreadCount, 1, 1)]
void CSComputeCollisionTimes(int3 dtid : SV_DispatchThreadID)
{
	uint idx = dtid.x;

	// Zero the body colour_used bitmask (one thread per body, reusing the same dispatch)
	if (idx < (uint)g_counters[0].body_count)
		g_bodies[idx].colour_used = 0;

	if (idx < (uint)g_counters[0].contact_count)
		g_contacts[idx].collision_time = EstimateCollisionTime(g_contacts[idx]);
}

// ----- CSAssignColours -----
// Serial: single thread. Reads pre-computed collision times from contacts,
// sorts by time, then does greedy graph colouring using per-body bitmasks
// stored in g_bodies[].colour_used.
[numthreads(1, 1, 1)]
void CSAssignColours(uint3 dtid : SV_DispatchThreadID)
{
	int n = min(g_counters[0].contact_count, MaxContacts);

	// Build sorted index array from pre-computed times
	int order[MaxContacts];
	for (int i = 0; i < n; ++i)
		order[i] = i;

	// Insertion sort by collision time (ascending = earliest first)
	for (int i = 1; i < n; ++i)
	{
		int key_idx = order[i];
		float key_time = g_contacts[key_idx].collision_time;
		int j = i - 1;
		while (j >= 0 && g_contacts[order[j]].collision_time > key_time)
		{
			order[j + 1] = order[j];
			--j;
		}
		order[j + 1] = key_idx;
	}

	// Greedy colouring in time-sorted order using per-body colour bitmasks.
	for (int si = 0; si < n; ++si)
	{
		int i = order[si];
		int a = g_contacts[i].body_idx_a;
		int b = g_contacts[i].body_idx_b;

		uint used = g_bodies[a].colour_used | g_bodies[b].colour_used;
		uint colour = firstbitlow(~used);
		g_colours[i] = colour;

		if (colour < 32)
		{
			g_bodies[a].colour_used |= (1u << colour);
			g_bodies[b].colour_used |= (1u << colour);
		}
	}
}

// ----- CSResolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact.
// Only contacts matching the current colour are processed — all others return immediately.
[numthreads(ResolveThreadCount, 1, 1)]
void CSResolve(int3 dtid : SV_DispatchThreadID)
{
	uint idx = dtid.x;
	if (idx >= g_counters[0].contact_count)
		return;

	// Only process contacts assigned to the current colour batch
	if (g_colours[idx] != (uint)g_colour)
		return;

	// Load the contact and both bodies
	GpuResolveContact c = g_contacts[idx];
	float3 axis = c.axis.xyz;
	float3 pt = c.contact_point.xyz;

	RigidBodyDynamics bodyA = g_bodies[c.body_idx_a];
	RigidBodyDynamics bodyB = g_bodies[c.body_idx_b];

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
	float3 V_rel = RelativeVelocityAtContact(c, bodyA, bodyB,
		os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);

	// Skip if already separating
	if (dot(V_rel, axis) > 0)
		return;

	// Build collision mass matrix
	float3x3 col_I = CollisionMassMatrix(
		pt - com_a_in_a, pt - com_b_in_a,
		inv_mass_a, inv_mass_b,
		os_iinv_a, rotate_inertia_inv(os_iinv_b, b2a_rot));

	// Load material properties
	GpuMaterial mat_a = g_materials[c.mat_id_a];
	GpuMaterial mat_b = g_materials[c.mat_id_b];
	float elasticity = (mat_a.elasticity_norm + mat_b.elasticity_norm) * 0.5f;
	float friction = sqrt(mat_a.friction_static * mat_b.friction_static);

	// Compute impulse with friction cone clamping
	float3 impulse = ComputeImpulse(col_I, V_rel, axis, elasticity, friction);

	// Apply impulse with energy conservation guard
	ApplyImpulseWithEnergyGuard(bodyA, bodyB, impulse, pt,
		com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
		inv_mass_a, inv_mass_b);

	// Write updated bodies
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}
