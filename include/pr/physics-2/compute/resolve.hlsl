//*********************************************
// Physics Engine — GPU Collision Resolution Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Graph-coloured batch collision resolution running on the GPU.
// Each thread processes one contact. Contacts are dispatched in colour
// batches — within a batch, no two contacts share a body, so writes
// to body momenta are data-race free.
//
// The algorithm mirrors the CPU RestitutionImpulse() + ResolveCollision()
// path in impulse.cpp and engine.cpp. See those files for detailed comments.
//
// Buffer layout:
//   u0: RWStructuredBuffer<RigidBodyDynamics> — per-body dynamic state (read/write)
//   t0: StructuredBuffer<GpuResolveContact>   — filtered contacts with materials
//   t1: StructuredBuffer<uint>                — per-contact colour assignment
//   b0: cbuffer with contact count and current colour batch
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).
//   HLSL 'row_major float4x4' rows = C++ columns = basis vectors.
//   mul(v, M) for vector transforms, mul(A, B) = A then B in row-vector convention.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"

// Must match the C++ RigidBodyDynamics struct exactly (208 bytes per element).
struct RigidBodyDynamics
{
	row_major float4x4 o2w;
	float4 momentum_ang;
	float4 momentum_lin;
	float4 force_ang;
	float4 force_lin;
	float4 inertia_inv_diagonal;  // {Ixx_inv, Iyy_inv, Izz_inv, 0}
	float4 inertia_inv_products;  // {Ixy_inv, Ixz_inv, Iyz_inv, 0}
	float4 os_com_and_invmass;    // {com_x, com_y, com_z, inv_mass}
	float4 os_bbox_centre;
	float4 os_bbox_radius;
};

// Must match the C++ GpuResolveContact struct (112 bytes).
struct GpuResolveContact
{
	float4 axis;                   // collision normal (in A's object space)
	float4 contact_pt;             // contact point at estimated collision time (in A's space)
	row_major float4x4 b2a;       // B-to-A transform
	int body_idx_a;
	int body_idx_b;
	float elasticity;              // combined material elasticity (normal)
	float friction;                // combined material static friction
};

// Shader resources
RWStructuredBuffer<RigidBodyDynamics> g_bodies : register(u0);
StructuredBuffer<GpuResolveContact> g_contacts : register(t0);
StructuredBuffer<uint> g_colours : register(t1);

// Per-dispatch constants
cbuffer cbResolve : register(b0)
{
	uint g_contact_count;
	uint g_colour;         // current colour batch being processed
	uint g_pad0;
	uint g_pad1;
};

// ----- Helper functions -----

// Cross-product matrix: CPM(r) * v = cross(r, v)
float3x3 cross_product_matrix(float3 r)
{
	return float3x3(
		 0,    -r.z,  r.y,
		 r.z,  0,    -r.x,
		-r.y,  r.x,  0
	);
}

// Invert a 3x3 matrix using cofactor expansion
float3x3 invert3x3(float3x3 m)
{
	float3 c0 = cross(m[1], m[2]);
	float3 c1 = cross(m[2], m[0]);
	float3 c2 = cross(m[0], m[1]);
	float det = dot(m[0], c0);
	float inv_det = 1.0f / det;
	return transpose(float3x3(c0, c1, c2)) * inv_det;
}

// Compute kinetic energy from momentum and inverse inertia (world space).
// Momentum is at CoM, inertia is block-diagonal.
float kinetic_energy(RigidBodyDynamics body)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 rot = (float3x3)body.o2w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, rot);

	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;
	return 0.5f * spatial_dot(vel_ang, vel_lin, body.momentum_ang.xyz, body.momentum_lin.xyz);
}

// ----- Main resolve kernel -----

[numthreads(64, 1, 1)]
void CSResolve(uint3 dtid : SV_DispatchThreadID)
{
	uint idx = dtid.x;
	if (idx >= g_contact_count)
		return;

	// Only process contacts assigned to the current colour batch
	if (g_colours[idx] != g_colour)
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
	// Body A: A-space = A's object space, so rotation from OS to A-space is identity.
	// Note: stored diagonal/products are unscaled — multiply by inv_mass to get actual Ic_inv.
	float3x3 os_iinv_a = inv_mass_a * build_symmetric_3x3(bodyA.inertia_inv_diagonal.xyz, bodyA.inertia_inv_products.xyz);
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 ws_iinv_a = rotate_inertia_inv(os_iinv_a, rot_a);

	// Compute A's velocity at model origin in world space, then transform to A-space.
	// Momentum is at CoM, velocity at CoM: omega = I_inv * h_ang, v_com = h_lin / m
	float3 omega_a_ws = mul(ws_iinv_a, bodyA.momentum_ang.xyz);
	float3 v_com_a_ws = inv_mass_a * bodyA.momentum_lin.xyz;

	// Transform velocity to A's object space (w2a rotation = transpose of rot_a in row-vector convention)
	float3 omega_a = mul(rot_a, omega_a_ws);
	float3 v_com_a = mul(rot_a, v_com_a_ws);

	// Velocity at contact point: v_pt = v_com + cross(omega, pt - com_pos)
	float3 com_a_in_a = os_com_a; // A's CoM in A-space is just the OS CoM offset
	float3 v_a_at_pt = v_com_a + cross(omega_a, pt - com_a_in_a);

	// Body B: need B's velocity in A-space
	float3x3 os_iinv_b = inv_mass_b * build_symmetric_3x3(bodyB.inertia_inv_diagonal.xyz, bodyB.inertia_inv_products.xyz);
	float3x3 rot_b = (float3x3)bodyB.o2w;
	float3x3 ws_iinv_b = rotate_inertia_inv(os_iinv_b, rot_b);

	float3 omega_b_ws = mul(ws_iinv_b, bodyB.momentum_ang.xyz);
	float3 v_com_b_ws = inv_mass_b * bodyB.momentum_lin.xyz;

	// Transform B's WS velocity to A-space: use w2a rotation
	float3 omega_b_in_a = mul(rot_a, omega_b_ws);
	float3 v_com_b_in_a = mul(rot_a, v_com_b_ws);

	// B's CoM position in A-space
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
	float ke_before = kinetic_energy(bodyA) + kinetic_energy(bodyB);

	// ----- Build collision mass matrix -----

	// Lever arms from each body's CoM to the contact point (in A-space)
	float3 rA = pt - com_a_in_a;
	float3 rB = pt - com_b_in_a;

	// Inverse inertia tensors at CoM, in A-space
	// For A: A-space IS A's object space, so no rotation needed
	float3x3 Ia_inv = os_iinv_a;

	// For B: rotate from B's OS to A-space
	float3x3 Ib_inv = rotate_inertia_inv(os_iinv_b, b2a_rot);

	// Collision inverse-mass matrix (3x3):
	// col_I_inv = (1/mA)*I - CPM(rA)*Ia_inv*CPM(rA) + (1/mB)*I - CPM(rB)*Ib_inv*CPM(rB)
	float3x3 cpm_rA = cross_product_matrix(rA);
	float3x3 cpm_rB = cross_product_matrix(rB);

	float3x3 col_Ia_inv = inv_mass_a * float3x3(1,0,0, 0,1,0, 0,0,1) - mul(mul(cpm_rA, Ia_inv), cpm_rA);
	float3x3 col_Ib_inv = inv_mass_b * float3x3(1,0,0, 0,1,0, 0,0,1) - mul(mul(cpm_rB, Ib_inv), cpm_rB);
	float3x3 col_I_inv = col_Ia_inv + col_Ib_inv;

	// The collision mass matrix
	float3x3 col_I = invert3x3(col_I_inv);

	// ----- Decompose impulse into normal and tangential components -----

	// impulse0: kills ALL relative velocity (zero restitution)
	float3 impulse0 = -mul(col_I, V_inv);

	// impulseN: normal component only
	float denom = dot(axis, mul(col_I_inv, axis));
	float3 impulseN = (float3)0;
	if (abs(denom) > 1e-12f)
		impulseN = -(dot(axis, V_inv) / denom) * axis;

	float3 impulseT = impulse0 - impulseN;

	// Apply restitution to normal component
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
	// Pure force at contact point → torque = cross(r, F) about CoM

	// For A: impulse wrench at CoM in A-space (negate — A receives the reaction)
	// Shift: torque = Cross(force, com - pt) per Featherstone RBDS 2.22
	float3 forceA_in_a = -impulse4;
	float3 torqueA_in_a = -cross(impulse4, com_a_in_a - pt);

	// For B: impulse wrench at B's CoM in A-space
	float3 forceB_in_a = impulse4;
	float3 torqueB_in_a = cross(impulse4, com_b_in_a - pt);

	// Rotate from A-space to B's object space: a2b_rot = transpose(b2a_rot) in row-vector convention
	float3x3 a2b_rot = transpose(b2a_rot);
	float3 torqueB_in_b = mul(a2b_rot, torqueB_in_a);
	float3 forceB_in_b = mul(a2b_rot, forceB_in_a);

	// Transform OS wrenches to world space for momentum update
	// Momentum is stored in world space, wrenches are in OS → rotate by o2w
	float3 torqueA_ws = mul(torqueA_in_a, rot_a); // mul(v, rot) in row-vector = rotate v by rot
	float3 forceA_ws = mul(forceA_in_a, rot_a);
	float3 torqueB_ws = mul(torqueB_in_b, rot_b);
	float3 forceB_ws = mul(forceB_in_b, rot_b);

	// ----- Pre-compute impulse KE coefficient for energy guard -----
	// A = 0.5 * (vAj·jA + vBj·jB) where vXj = IinvX * jX
	float3 ja_ang = torqueA_ws;
	float3 ja_lin = forceA_ws;
	float3 jb_ang = torqueB_ws;
	float3 jb_lin = forceB_ws;

	// va_j = InertiaInvWS * impulseA (at CoM, block-diagonal)
	float3 va_j_ang = mul(ws_iinv_a, ja_ang);
	float3 va_j_lin = inv_mass_a * ja_lin;
	float3 vb_j_ang = mul(ws_iinv_b, jb_ang);
	float3 vb_j_lin = inv_mass_b * jb_lin;
	float A = 0.5f * (spatial_dot(va_j_ang, va_j_lin, ja_ang, ja_lin)
	                 + spatial_dot(vb_j_ang, vb_j_lin, jb_ang, jb_lin));

	// ----- Apply impulses to body momenta -----
	bodyA.momentum_ang.xyz += ja_ang;
	bodyA.momentum_lin.xyz += ja_lin;
	bodyB.momentum_ang.xyz += jb_ang;
	bodyB.momentum_lin.xyz += jb_lin;

	// ----- Energy conservation guard -----
	// If the impulse injected energy, scale it back.
	// KE(α) = KE₀ + αB + α²A is quadratic in the impulse scale factor α.
	float ke_after = kinetic_energy(bodyA) + kinetic_energy(bodyB);
	float delta = ke_after - ke_before;
	if (delta > 0 && A > 1e-12f)
	{
		float alpha = clamp((A - delta) / A, 0.0f, 1.0f);
		float correction = 1.0f - alpha;
		bodyA.momentum_ang.xyz -= correction * ja_ang;
		bodyA.momentum_lin.xyz -= correction * ja_lin;
		bodyB.momentum_ang.xyz -= correction * jb_ang;
		bodyB.momentum_lin.xyz -= correction * jb_lin;
	}

	// ----- Write updated bodies -----
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}
