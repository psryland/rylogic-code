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

// Maximum number of colour batches. The CPU dispatches CSResolve this many times.
// Empty batches (no contacts with that colour) are free — threads return immediately.
static const int MaxColours = 32;

// Per-dispatch constants
cbuffer cbResolve : register(b0)
{
	int g_colour; // current colour batch being processed (for CSResolve)
	int g_pad0;
	int g_pad1;
	int g_pad2;
};

// Shader resources
StructuredBuffer<GpuCollisionCounters> g_counters : register(t0);
StructuredBuffer<GpuResolveContact> g_contacts : register(t1);
RWStructuredBuffer<RigidBodyDynamics> g_bodies : register(u0);
RWStructuredBuffer<uint> g_colours : register(u1);

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

// ----- CSGraphColouring -----
// Greedy graph colouring: assigns colours so no two contacts sharing a body have the same colour.
// Single-thread shader — O(n²) is fine for typical contact counts (<10k).
// Writes per-contact colour to g_colours[].
[numthreads(1, 1, 1)]
void CSGraphColouring(uint3 dtid : SV_DispatchThreadID)
{
	int n = g_counters[0].contact_count;

	// Greedy colouring: for each contact, find the lowest colour not used by
	// any earlier contact that shares a body.
	for (int i = 0; i < n; ++i)
	{
		int a = g_contacts[i].body_idx_a;
		int b = g_contacts[i].body_idx_b;

		// Bitmask of used colours (supports up to 32 colours)
		uint used = 0;
		for (int j = 0; j < i; ++j)
		{
			if (g_contacts[j].body_idx_a == a || g_contacts[j].body_idx_b == a ||
				g_contacts[j].body_idx_a == b || g_contacts[j].body_idx_b == b)
			{
				uint c = g_colours[j];
				if (c < 32)
					used |= (1u << c);
			}
		}

		// Find lowest clear bit
		g_colours[i] = firstbitlow(~used);
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

	// Load the contact
	GpuResolveContact c = g_contacts[idx];
	float3 axis = c.axis.xyz;
	float3 pt = c.contact_pt.xyz;

	// Load both bodies
	RigidBodyDynamics bodyA = g_bodies[c.body_idx_a];
	RigidBodyDynamics bodyB = g_bodies[c.body_idx_b];

	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float3 os_com_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;

	// ----- Compute relative velocity at the contact point (in A's object space) -----
	float3x3 os_iinv_a = inv_mass_a * build_symmetric_3x3(bodyA.inertia_inv_diagonal.xyz, bodyA.inertia_inv_products.xyz);
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 ws_iinv_a = rotate_inertia_inv(os_iinv_a, rot_a);

	float3 omega_a_ws = mul(ws_iinv_a, bodyA.momentum_ang.xyz);
	float3 v_com_a_ws = inv_mass_a * bodyA.momentum_lin.xyz;

	// Transform velocity to A's object space
	float3 omega_a = mul(rot_a, omega_a_ws);
	float3 v_com_a = mul(rot_a, v_com_a_ws);

	float3 com_a_in_a = os_com_a;
	float3 v_a_at_pt = v_com_a + cross(omega_a, pt - com_a_in_a);

	// Body B: need B's velocity in A-space
	float3x3 os_iinv_b = inv_mass_b * build_symmetric_3x3(bodyB.inertia_inv_diagonal.xyz, bodyB.inertia_inv_products.xyz);
	float3x3 rot_b = (float3x3)bodyB.o2w;
	float3x3 ws_iinv_b = rotate_inertia_inv(os_iinv_b, rot_b);

	float3 omega_b_ws = mul(ws_iinv_b, bodyB.momentum_ang.xyz);
	float3 v_com_b_ws = inv_mass_b * bodyB.momentum_lin.xyz;

	float3 omega_b_in_a = mul(rot_a, omega_b_ws);
	float3 v_com_b_in_a = mul(rot_a, v_com_b_ws);

	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 b2a_pos = c.b2a[3].xyz;
	float3 com_b_in_a = b2a_pos + mul(os_com_b, b2a_rot);
	float3 v_b_at_pt = v_com_b_in_a + cross(omega_b_in_a, pt - com_b_in_a);

	// Relative velocity of B w.r.t A at the contact point
	float3 V_inv = v_b_at_pt - v_a_at_pt;

	// Re-check the separating condition with current velocities
	float sep_dot = dot(V_inv, axis);
	if (sep_dot > 0)
		return;

	// ----- Measure pre-collision kinetic energy -----
	float ke_before = KineticEnergy(bodyA) + KineticEnergy(bodyB);

	// ----- Build collision mass matrix -----
	float3 rA = pt - com_a_in_a;
	float3 rB = pt - com_b_in_a;

	float3x3 Ia_inv = os_iinv_a;
	float3x3 Ib_inv = rotate_inertia_inv(os_iinv_b, b2a_rot);

	float3x3 cpm_rA = CrossProductMatrix(rA);
	float3x3 cpm_rB = CrossProductMatrix(rB);

	float3x3 col_Ia_inv = inv_mass_a * float3x3(1,0,0, 0,1,0, 0,0,1) - mul(mul(cpm_rA, Ia_inv), cpm_rA);
	float3x3 col_Ib_inv = inv_mass_b * float3x3(1,0,0, 0,1,0, 0,0,1) - mul(mul(cpm_rB, Ib_inv), cpm_rB);
	float3x3 col_I_inv = col_Ia_inv + col_Ib_inv;

	float3x3 col_I = Invert(col_I_inv);

	// ----- Decompose impulse into normal and tangential components -----
	float3 impulse0 = -mul(col_I, V_inv);
	float denom = dot(axis, mul(col_I_inv, axis));
	float3 impulseN = (float3)0;
	if (abs(denom) > 1e-12f)
		impulseN = -(dot(axis, V_inv) / denom) * axis;

	float3 impulseT = impulse0 - impulseN;
	float3 impulse4 = (1.0f + c.elasticity) * impulseN + impulseT;

	// ----- Coulomb friction cone clamping -----
	{
		float clamped_friction = min(c.friction, 0.9999f);
		float static_friction = clamped_friction / (1.000001f - clamped_friction);
		float Jn = dot(impulse4, axis);
		float Jt_sq = max(0.0f, dot(impulse4, impulse4) - Jn * Jn);
		float Jt = sqrt(Jt_sq);
		if (Jt > static_friction * abs(Jn))
		{
			Jt = static_friction * abs(Jn);
			float impulseT_lenSq = dot(impulseT, impulseT);
			if (impulseT_lenSq > 1e-12f)
				impulseT = Jt * (impulseT / sqrt(impulseT_lenSq));
			impulse4 = (1.0f + c.elasticity) * impulseN + impulseT;
		}
	}

	// ----- Convert point impulse to spatial wrenches at each body's CoM -----
	float3 forceA_in_a = -impulse4;
	float3 torqueA_in_a = -cross(impulse4, com_a_in_a - pt);
	float3 forceB_in_a = impulse4;
	float3 torqueB_in_a = cross(impulse4, com_b_in_a - pt);

	// Transform wrenches to world space
	float3 torqueA_ws = mul(torqueA_in_a, rot_a);
	float3 forceA_ws = mul(forceA_in_a, rot_a);
	float3 torqueB_ws = mul(torqueB_in_a, rot_a);
	float3 forceB_ws = mul(forceB_in_a, rot_a);

	// ----- Pre-compute impulse KE coefficient for energy guard -----
	float3 va_j_ang = mul(ws_iinv_a, torqueA_ws);
	float3 va_j_lin = inv_mass_a * forceA_ws;
	float3 vb_j_ang = mul(ws_iinv_b, torqueB_ws);
	float3 vb_j_lin = inv_mass_b * forceB_ws;
	float A = 0.5f * (spatial_dot(va_j_ang, va_j_lin, torqueA_ws, forceA_ws)
	                 + spatial_dot(vb_j_ang, vb_j_lin, torqueB_ws, forceB_ws));

	// ----- Apply impulses to body momenta -----
	bodyA.momentum_ang.xyz += torqueA_ws;
	bodyA.momentum_lin.xyz += forceA_ws;
	bodyB.momentum_ang.xyz += torqueB_ws;
	bodyB.momentum_lin.xyz += forceB_ws;

	// ----- Energy conservation guard -----
	float ke_after = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	float delta = ke_after - ke_before;
	if (delta > 0 && A > 1e-12f)
	{
		float alpha = clamp((A - delta) / A, 0.0f, 1.0f);
		float correction = 1.0f - alpha;
		bodyA.momentum_ang.xyz -= correction * torqueA_ws;
		bodyA.momentum_lin.xyz -= correction * forceA_ws;
		bodyB.momentum_ang.xyz -= correction * torqueB_ws;
		bodyB.momentum_lin.xyz -= correction * forceB_ws;
	}

	// ----- Write updated bodies -----
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}
