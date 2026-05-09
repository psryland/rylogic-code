//*********************************************
// Physics Engine — GPU Collision Resolution Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Graph-coloured batch collision resolution running on the GPU.
//
// Pipeline:
//   1. CSComputeCollisionTimes - estimates time-of-impact and writes the first sort key.
//   2. Shock-priority passes   - optionally propagate contact priority through body/contact adjacency and rewrite the sort key.
//   3. RadixSort               - sorts g_contact_order by g_contact_times. This pass is driven from C++ using RadixSort, not an entry point here.
//   4. CSAssignColours         - greedily graph-colours the sorted contacts so independent contacts can be solved together.
//   5. CSPositionSolve         - split position correction, dispatched once per colour.
//   6. CSResolve               - velocity impulse solve, dispatched once per colour.
//
// Within a colour batch, no two contacts share a dynamic body, so writes to body transforms/momenta are data-race free. Static bodies are allowed to appear
// in multiple contacts in one colour because they are never written by the solver.
//
// Scratch ownership:
//   - g_contact_times is first a sort-key buffer, then a temporary priority buffer during shock propagation, then the final sort-key buffer again.
//   - g_contact_order starts as [0..contact_count) and is permuted by the external radix sort.
//   - g_colours is used as uint/asfloat scratch for Jacobi shock-priority propagation, then reset and reused as the final per-contact colour assignment.
//
// Resource layout:
//   b0: cbResolve                               - per-dispatch constants
//   t0: StructuredBuffer<GpuCollisionCounters>  - counters (body_count, pair_count, contact_count)
//   t1: StructuredBuffer<GpuMaterial>           - material properties
//   u0: RWStructuredBuffer<GpuRigidBody>        - per-body dynamic state (read/write)
//   u1: RWStructuredBuffer<uint>                - per-contact colour assignment
//   u2: RWStructuredBuffer<GpuResolveContact>   - prepared contacts (output of the contact preparation pass)
//   u3: RWStructuredBuffer<float>               - sort keys / propagated priority scratch
//   u4: RWStructuredBuffer<uint>                - sorted contact indices
//   u5: RWStructuredBuffer<uint>                - per-body linked-list head for contacts touching that dynamic body
//   u6: RWStructuredBuffer<uint>                - per-contact linked-list next pointer for body A
//   u7: RWStructuredBuffer<uint>                - per-contact linked-list next pointer for body B
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).
//   HLSL 'row_major float4x4' rows = C++ columns = basis vectors.
//   mul(v, M) for vector transforms, mul(A, B) = A then B in row-vector convention.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

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

// Per-dispatch constants. Keep this layout mirrored with the C++ cbResolve structure in resolve_gpu.cpp and the interop runner.
struct cbResolve
{
	int max_contacts; // The max capacity of the contacts buffer
	int body_count;   // The number of bodies in the scene
	int colour;       // Current colour batch being processed (for CSResolve)
	int sort_capacity; // The number of sort keys the radix sorter will read, which can be larger than this pass's contact count

	int shock_iterations;  // Number of contact-priority propagation sweeps
	int shock_padding0;
	int shock_padding1;
	float shock_alignment; // Minimum directed impulse influence needed to create a propagation edge

	float shock_min_strength; // Minimum priority delta worth propagating/storing
	float dt;                 // Timestep in seconds
	float support_only;       // Non-zero means only support-aligned contacts are solved
	float support_alignment;  // Minimum abs(dot(contact_axis, gravity_dir)) for support-only contacts

	float restitution_scale; // Scene/pass scale for elastic response
	float penetration_slop;  // Depth tolerance before velocity-level bias starts
	float velocity_baumgarte;// Velocity bias fraction used to clear penetration
	float deep_penetration_threshold;

	float deep_penetration_range;
	float deep_penetration_baumgarte_min;
	float deep_penetration_baumgarte_max;
	float bias_scale; // Per-pass scale used by selective refresh and normal resolve paths

	float propagation_key_scale;      // Tiny scale that folds local/shock priority into the collision-time sort key
	float position_slop;              // Depth tolerance before split-position correction starts
	float position_baumgarte;         // Position correction fraction
	float position_correction_scale;  // 1 / position iteration count, so the same stale depth is not applied in full each iteration

	float shock_decay; // Priority attenuation per propagated edge
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
RWStructuredBuffer<uint> resource(g_body_contact_head, u5); // scratch: body -> first contact+1 for priority adjacency, 0 means empty
RWStructuredBuffer<uint> resource(g_contact_next_a, u6);    // scratch: next contact+1 in body A's adjacency list
RWStructuredBuffer<uint> resource(g_contact_next_b, u7);    // scratch: next contact+1 in body B's adjacency list

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

	// KE is evaluated from momentum rather than velocity because momenta are the state this solver mutates. This keeps the energy guard independent of how
	// velocity is represented elsewhere and matches the spatial-algebra identity: KE = 0.5 * h^T * I^-1 * h.
	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;
	return 0.5f * spatial_dot(vel_ang, vel_lin, body.momentum_ang.xyz, body.momentum_lin.xyz);
}

// Compute a body's object-space inverse inertia matrix (scaled by inv_mass).
float3x3 OsInverseInertia(GpuRigidBody body)
{
	// The stored inertia tensor is normalised by mass. Scaling by inverse mass gives the actual inverse inertia used by this frame's body state. Static bodies
	// have inv_mass == 0, which naturally turns both linear and angular response into zero.
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

	// Spatial displacement: dx = 0.5 * dt * I^(2*h + f*dt). This is only used over a small sub-step rewind/advance, so the linearised rotation below is
	// preferable to a heavier quaternion/exponential-map update here.
	float3 h_ang = 2.0f * body.momentum_ang.xyz + dt * body.force_ang.xyz;
	float3 h_lin = 2.0f * body.momentum_lin.xyz + dt * body.force_lin.xyz;
	float3 dx_ang = 0.5f * dt * mul(ws_iinv, h_ang);
	float3 dx_lin = 0.5f * dt * inv_mass * h_lin;

	// Update the CoM and then reconstruct the object origin from the new CoM and new rotation. This matters for shapes whose object origin is not their CoM.
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

	// Velocity is returned in body A space because contacts store their centroid/normal in A space. Keeping all contact math in one frame avoids repeated
	// world<->object conversions and keeps the normal impulse direction consistent with c.axis.
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
	// The contact-space effective mass tells us how much relative velocity changes for a point impulse. Linear response contributes inv_mass * I; angular
	// response contributes the lever-arm terms. The result is inverted because impulse solve uses J = -M_eff * V_rel.
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

	// Start with the impulse that would cancel the full relative velocity, then split it into normal and tangential components. The normal part gets
	// restitution, while the tangential part is clamped by the Coulomb cone below.
	float3 impulse0 = -mul(col_I, V_rel);
	float denom = dot(axis, mul(col_I_inv, axis));
	float3 impulseN = (float3)0;
	if (abs(denom) > 1e-12f)
		impulseN = -(dot(axis, V_rel) / denom) * axis;

	float3 impulseT = impulse0 - impulseN;
	float3 impulse = (1.0f + elasticity) * impulseN + impulseT;

	// Coulomb friction cone clamping. The scene material stores a friction ratio in [0,1), converted here to the slope of the cone so values near 1 can
	// approach very high static friction without dividing by zero.
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
	// This function assumes the normal impulse budget is already known. It solves only the tangent direction that opposes current tangential slip, then caps
	// that impulse to mu * Jn so friction cannot exceed the normal force that generated it.
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
	float inv_mass_a, float inv_mass_b, bool enable_energy_guard)
{
	// Convert a point impulse into spatial wrenches at each body's CoM. The contact normal points from A to B in A space, so A receives -impulse and B
	// receives +impulse.
	float3 forceA_in_a = -impulse;
	float3 torqueA_in_a = -cross(impulse, com_a_in_a - pt);
	float3 forceB_in_a = impulse;
	float3 torqueB_in_a = cross(impulse, com_b_in_a - pt);

	// Transform wrenches to world space before updating momenta because rigid body state stores world-space linear/angular momentum.
	float3 torqueA_ws = mul(torqueA_in_a, rot_a);
	float3 forceA_ws = mul(forceA_in_a, rot_a);
	float3 torqueB_ws = mul(torqueB_in_a, rot_a);
	float3 forceB_ws = mul(forceB_in_a, rot_a);

	// Pre-compute the quadratic KE coefficient for this impulse direction. If the trial impulse increases KE, this gives a cheap scalar way to back out the
	// energy-injecting fraction without recomputing the full impulse solve.
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

	// Energy conservation guard: if KE increased, scale the impulse down. Bias impulses disable this because they are pseudo-impulses whose purpose is to
	// remove positional error, not conserve physical kinetic energy.
	float ke_after = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	float delta = ke_after - ke_before;
	if (enable_energy_guard && delta > 0 && A > 1e-12f)
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

	// Most resolver calculations use body A space. Body B's CoM is transformed into A space using the current b2a transform so relative velocity can be
	// evaluated at the stored contact centroid without changing coordinate frames.
	float3 os_com_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);

	float3 rel_vel = RelativeVelocityAtContact(c, bodyA, bodyB,
		OsInverseInertia(bodyA), OsInverseInertia(bodyB),
		rot_a, os_com_a, com_b_in_a);

	// Project backward along relative motion and ask what fraction of the current normal displacement corresponds to the penetration depth. The value is
	// negative because it is a rewind within the current step, and more negative values sort before later impacts.
	float3 point_at_t0 = c.contact_point.xyz - g.dt * rel_vel;
	float distance = abs(dot(c.contact_point.xyz - point_at_t0, c.axis.xyz));
	float sub_step = distance > c.depth ? -c.depth / distance : 0.0f;
	return sub_step * g.dt;
}

// Measure how closely a contact normal aligns with gravity for support-contact filtering.
// Returns 0 when gravity is unavailable, otherwise abs(dot(axis_ws, gravity_dir)) in [0,1].
float ContactSupportAlignment(GpuResolveContact c, GpuRigidBody bodyA)
{
	// Support-only passes should only touch contacts whose normal is gravity-aligned.
	// This avoids wasting selective refresh work on side contacts when the pass is
	// intended to re-stabilise load-bearing support contacts.
	float3 gravity = NormaliseOrZero(bodyA.ws_gravity.xyz);
	if (dot(gravity, gravity) == 0.0f)
		return 0.0f;

	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	return abs(dot(axis_ws, gravity));
}

// Classify a contact as load-bearing support for support-only resolve passes.
// A contact is support if its normal is sufficiently parallel to gravity, regardless of sign.
bool SupportContact(GpuResolveContact c, GpuRigidBody bodyA)
{
	return ContactSupportAlignment(c, bodyA) >= g.support_alignment;
}

// Apply the current resolve-pass contact filter.
// Normal resolve accepts every contact; selective support-only refresh accepts only support contacts.
bool ContactEnabled(GpuResolveContact c, GpuRigidBody bodyA)
{
	return g.support_only == 0.0f || SupportContact(c, bodyA);
}

// Compute the local contact-priority tie-breaker that is folded into the collision-time radix key.
// Lower values sort earlier. The key favours low support contacts, high momentum, closing velocity, and deeper penetration.
float PropagationSortKey(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB)
{
	// This is the local part of contact priority. It intentionally remains small compared with collision time and
	// is multiplied by propagation_key_scale before sorting, so it only reorders contacts with nearly equal time-of-impact.
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3 ws_point = bodyA.o2w[3].xyz + mul(c.contact_point.xyz, rot_a);
	float3 gravity = NormaliseOrZero(bodyA.ws_gravity.xyz);
	float support_height = dot(gravity, gravity) != 0.0f ? -dot(ws_point, gravity) : 0.0f;

	// Lower support_height means "lower in the gravity stack", so bottom contacts sort earlier.
	// Source momentum and closing/depth terms pull high-demand contacts earlier within that height ordering.
	float3x3 os_iinv_a = OsInverseInertia(bodyA);
	float3x3 os_iinv_b = OsInverseInertia(bodyB);
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 com_b_in_a = c.b2a[3].xyz + mul(bodyB.os_com_and_invmass.xyz, b2a_rot);
	float3 v_a = BodyVelocityAtPoint(bodyA, os_iinv_a, c.contact_point.xyz, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(bodyB, os_iinv_b, c.contact_point.xyz, com_b_in_a, rot_a);
	float3 v_rel = v_b - v_a;
	float3 axis_ws = mul(c.axis.xyz, rot_a);
	float closing_speed = max(0.0f, -dot(v_rel, c.axis.xyz));
	float source_momentum = max(abs(dot(bodyA.momentum_lin.xyz, axis_ws)), abs(dot(bodyB.momentum_lin.xyz, axis_ws)));

	return support_height - source_momentum - 0.05f * closing_speed - 0.50f * c.depth;
}

// Estimate how urgently this contact needs a normal impulse before considering graph propagation.
// This seed priority combines normal closing speed with Baumgarte bias demand from penetration depth.
float ContactNormalDemand(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB)
{
	// This is the seed strength for shock-priority propagation. Closing contacts and deeply penetrating contacts are the places where an early impulse is
	// most likely to help neighbouring contacts in a dependency chain.
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 os_iinv_a = OsInverseInertia(bodyA);
	float3x3 os_iinv_b = OsInverseInertia(bodyB);
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 com_b_in_a = c.b2a[3].xyz + mul(bodyB.os_com_and_invmass.xyz, b2a_rot);
	float3 v_rel = RelativeVelocityAtContact(c, bodyA, bodyB, os_iinv_a, os_iinv_b, rot_a, com_a_in_a, com_b_in_a);
	float closing_speed = max(0.0f, -dot(v_rel, c.axis.xyz));
	float bias_speed = (g.velocity_baumgarte / max(g.dt, 1e-6f)) * max(c.depth - g.penetration_slop, 0.0f);
	return closing_speed + bias_speed;
}

// Find the dynamic body shared by two contacts, if any.
// Returns the shared body index, or -1 when the contacts are disjoint or only share a static body.
int SharedDynamicBody(GpuResolveContact lhs, GpuResolveContact rhs)
{
	// Priority only propagates through a body whose velocity can actually change. Static shared bodies (inv_mass == 0) are ignored because an impulse at one
	// static contact cannot transmit momentum to another contact through the static body.
	if (lhs.body_idx_a == rhs.body_idx_a && g_bodies[lhs.body_idx_a].os_com_and_invmass.w > 0.0f)
		return lhs.body_idx_a;
	if (lhs.body_idx_a == rhs.body_idx_b && g_bodies[lhs.body_idx_a].os_com_and_invmass.w > 0.0f)
		return lhs.body_idx_a;
	if (lhs.body_idx_b == rhs.body_idx_a && g_bodies[lhs.body_idx_b].os_com_and_invmass.w > 0.0f)
		return lhs.body_idx_b;
	if (lhs.body_idx_b == rhs.body_idx_b && g_bodies[lhs.body_idx_b].os_com_and_invmass.w > 0.0f)
		return lhs.body_idx_b;
	return -1;
}

// Approximate the world-space linear velocity change that a unit normal impulse at this contact would apply to body_idx.
// The priority metric uses this as a cheap directional influence test and deliberately ignores angular velocity changes.
float3 ContactUnitImpulseDeltaVelocityWS(GpuResolveContact c, int body_idx)
{
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody body = g_bodies[body_idx];
	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	if (body_idx == c.body_idx_a)
		return -body.os_com_and_invmass.w * axis_ws;
	if (body_idx == c.body_idx_b)
		return +body.os_com_and_invmass.w * axis_ws;
	return float3(0, 0, 0);
}

// Score a directed priority edge from src to dst through their shared dynamic body.
// Positive values mean a normal impulse at src would make dst more closing/active, so src should be resolved before dst.
float ContactPriorityInfluence(GpuResolveContact src, GpuResolveContact dst, int shared_body_idx)
{
	GpuRigidBody bodyA = g_bodies[dst.body_idx_a];
	float3 axis_ws = mul(dst.axis.xyz, (float3x3)bodyA.o2w);
	float3 delta_velocity = ContactUnitImpulseDeltaVelocityWS(src, shared_body_idx);
	float3 delta_relative_velocity = float3(0, 0, 0);
	if (shared_body_idx == dst.body_idx_a)
		delta_relative_velocity = -delta_velocity;
	else if (shared_body_idx == dst.body_idx_b)
		delta_relative_velocity = +delta_velocity;

	// Positive influence means the upstream contact impulse makes the downstream contact more closing/active.
	return max(0.0f, -dot(delta_relative_velocity, axis_ws));
}

// Convert penetration depth into the split-position correction distance for one position-solve iteration.
// Uses a gentle shallow correction and a stronger deep-penetration ramp, then applies the current pass scale.
float PositionCorrectionDistance(float depth)
{
	float position_pen = max(depth - g.position_slop, 0.0f);
	float position_correction = g.position_baumgarte * position_pen;

	float deep_pen = max(depth - g.deep_penetration_threshold, 0.0f);
	float deep_range = max(g.deep_penetration_range, 1e-6f);
	float deep_baumgarte = lerp(g.deep_penetration_baumgarte_min, g.deep_penetration_baumgarte_max, saturate(deep_pen / deep_range));
	float deep_correction = deep_baumgarte * deep_pen;

	return g.bias_scale * g.position_correction_scale * max(position_correction, deep_correction);
}

// Apply split positional correction for one contact without changing momenta.
// Dynamic bodies are moved along the contact normal in inverse-mass proportion; static bodies remain fixed.
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

// Adds a contact to one dynamic body's intrusive adjacency list used by shock-priority propagation.
void LinkContactToBody(int contact_idx, int body_idx, bool body_a_slot)
{
	if (g_bodies[body_idx].os_com_and_invmass.w <= 0.0f)
		return;

	uint old_head = 0;
	InterlockedExchange(g_body_contact_head[body_idx], (uint)(contact_idx + 1), old_head);
	if (body_a_slot)
		g_contact_next_a[contact_idx] = old_head;
	else
		g_contact_next_b[contact_idx] = old_head;
}

// Propagates one source contact's shock priority to neighbouring contacts that share the given dynamic body.
void PropagateShockThroughBody(int src_idx, GpuResolveContact src, int body_idx, float src_priority)
{
	if (g_bodies[body_idx].os_com_and_invmass.w <= 0.0f)
		return;

	uint node = g_body_contact_head[body_idx];
	for (int guard = 0; node != 0 && guard != g.max_contacts; ++guard)
	{
		int dst_idx = (int)node - 1;
		GpuResolveContact dst = g_contacts[dst_idx];
		node = body_idx == dst.body_idx_a ? g_contact_next_a[dst_idx] : g_contact_next_b[dst_idx];
		if (dst_idx == src_idx)
			continue;

		float influence = min(ContactPriorityInfluence(src, dst, body_idx), 1.0f);
		if (influence <= g.shock_alignment)
			continue;

		float propagated = g.shock_decay * influence * src_priority;
		if (propagated > g.shock_min_strength)
			InterlockedMax(g_colours[dst_idx], asuint(propagated));
	}
}

// ----- CSComputeCollisionTimes -----
// Parallel: one thread per contact. Computes the primary time-of-impact sort key and initialises the order buffer for the external radix sort.
numthreads(CSComputeCollisionTimes, ResolveThreadCount, 1, 1)
void CSComputeCollisionTimes(int3 DTID(dtid))
{
	int idx = dtid.x;
	int contact_count = ContactCount();
	if (idx < contact_count)
	{
		// The contact buffer has already been prepared by the narrowphase/interop path. This pass does not
		// change contact geometry; it only records the estimated collision time and a sortable index.
		GpuResolveContact c = g_contacts[idx];
		GpuRigidBody bodyA = g_bodies[c.body_idx_a];
		GpuRigidBody bodyB = g_bodies[c.body_idx_b];
		float collision_time = EstimateCollisionTime(c);
		g_contacts[idx].collision_time = collision_time;

		// Collision time remains primary. The propagation key is deliberately tiny and only reorders near-simultaneous
		// contacts so support reactions and high-residual impact contacts get an earlier colour in the sequential solve.
		float propagation_key = ContactEnabled(c, bodyA) ? PropagationSortKey(c, bodyA, bodyB) : 1e30f;
		g_contact_times[idx] = collision_time + g.propagation_key_scale * propagation_key;
		g_contact_order[idx] = idx;

		// Position correction is applied after graph colouring by CSPositionSolve so contacts sharing a dynamic body are never written in parallel.
	}

	// Thread 0 also initialises per-dispatch scratch that is not naturally covered by one-thread-per-contact work. These writes do not depend on the other
	// threads in this dispatch; the UAV barrier after the dispatch provides synchronisation before the next pass.
	if (idx == 0)
	{
		int i;
	
		// Reset the graph-colouring bitmask on every body. CSAssignColours will OR colour bits into dynamic bodies as it walks the sorted contacts.
		for (i = 0; i != g.body_count; ++i)
			g_bodies[i].colour_used = 0;
		
		// Set the out-of-bounds contact times to a large positive value so they sort to the end.
		// The resolver's sort scratch can be larger than a compact selective contact buffer, so initialise
		// every key that the radix sorter will read, not just every contact slot in the current pass.
		for (i = contact_count; i != g.sort_capacity; ++i)
			g_contact_times[i] = 1e30f;
	}
}

// ----- Shock-priority propagation -----
// These passes implement a parallel Jacobi max-propagation over contact adjacency. Contacts are linked into per-body lists, so propagation only visits
// contacts that share a dynamic body instead of testing every contact pair. g_contact_times holds the current priority as float; g_colours holds the next
// priority as uint/asfloat so InterlockedMax can merge parallel candidates.
numthreads(CSClearShockLists, ResolveThreadCount, 1, 1)
void CSClearShockLists(int3 DTID(dtid))
{
	if (dtid.x < g.body_count)
		g_body_contact_head[dtid.x] = 0;
}

// Seed each contact's priority from local normal demand and build the dynamic body -> contacts adjacency lists.
// The next-priority buffer is initialised to the same value so each Jacobi pass starts from "keep current priority unless a stronger path arrives".
numthreads(CSSeedShockPriority, ResolveThreadCount, 1, 1)
void CSSeedShockPriority(int3 DTID(dtid))
{
	int contact_idx = dtid.x;
	if (contact_idx >= ContactCount())
		return;

	GpuResolveContact c = g_contacts[contact_idx];
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];
	float priority = ContactEnabled(c, bodyA) ? ContactNormalDemand(c, bodyA, bodyB) : 0.0f;
	g_contact_times[contact_idx] = priority;
	g_colours[contact_idx] = asuint(priority);
	g_contact_next_a[contact_idx] = 0;
	g_contact_next_b[contact_idx] = 0;

	LinkContactToBody(contact_idx, c.body_idx_a, true);
	LinkContactToBody(contact_idx, c.body_idx_b, false);
}

// Propagate one Jacobi step from every source contact to neighbouring contacts that share a dynamic body.
// This is parallel over source contacts; atomics merge multiple incoming paths into each destination's next priority.
numthreads(CSPropagateShockPriority, ResolveThreadCount, 1, 1)
void CSPropagateShockPriority(int3 DTID(dtid))
{
	int src_idx = dtid.x;
	if (src_idx >= ContactCount())
		return;

	float src_priority = g_contact_times[src_idx];
	if (src_priority < g.shock_min_strength)
		return;

	GpuResolveContact src = g_contacts[src_idx];
	PropagateShockThroughBody(src_idx, src, src.body_idx_a, src_priority);
	PropagateShockThroughBody(src_idx, src, src.body_idx_b, src_priority);
}

// Commit the max-reduced next priority to the current priority buffer for the next Jacobi step.
// g_colours intentionally remains equal to g_contact_times so the following propagation step starts with each contact's current priority already present.
numthreads(CSCommitShockPriority, ResolveThreadCount, 1, 1)
void CSCommitShockPriority(int3 DTID(dtid))
{
	int contact_idx = dtid.x;
	if (contact_idx >= ContactCount())
		return;

	g_contact_times[contact_idx] = asfloat(g_colours[contact_idx]);
}

// Convert final propagated priority back into radix-sort keys and reset colour scratch ready for graph colouring.
// Larger propagated priority sorts earlier via the negative sign; the local key remains a bottom-up/support/depth tie-breaker.
numthreads(CSFinalizeShockPriority, ResolveThreadCount, 1, 1)
void CSFinalizeShockPriority(int3 DTID(dtid))
{
	int contact_idx = dtid.x;
	if (contact_idx >= ContactCount())
		return;

	GpuResolveContact c = g_contacts[contact_idx];
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];
	float propagation_key = 1e30f;
	if (ContactEnabled(c, bodyA))
	{
		float local_key = PropagationSortKey(c, bodyA, bodyB);
		propagation_key = -g_contact_times[contact_idx] + 0.25f * local_key;
	}

	g_contact_times[contact_idx] = c.collision_time + g.propagation_key_scale * propagation_key;
	g_contact_order[contact_idx] = (uint)contact_idx;
	g_colours[contact_idx] = 0;
}

// ----- CSAssignColours -----
// Serial: single thread. Walks contacts in order sorted by collision time.
// Uses per-body colour bitmasks stored in g_bodies[].colour_used.
//
// The greedy colour assignment is serial because each contact's colour depends on the colours already reserved by earlier contacts. Earlier contacts in
// g_contact_order get lower colours and therefore get solved earlier in each solver iteration.
numthreads(CSAssignColours, 1, 1, 1)
void CSAssignColours(int3 DTID(dtid))
{
	if (dtid.x != 0)
		return;

	int contact_count = ContactCount();
	for (int i = 0; i != contact_count; ++i)
	{
		// g_contact_order is the radix-sorted permutation. Colouring in this order is what turns the contact-priority sort into actual solve order.
		int idx = g_contact_order[i]; // get contact index from sorted order
		GpuResolveContact c = g_contacts[idx];

		if (!ContactEnabled(c, g_bodies[c.body_idx_a]))
		{
			g_colours[idx] = MaxColours;
			continue;
		}

		int a = c.body_idx_a;
		int b = c.body_idx_b;

		// Only consider colour conflicts for dynamic bodies (inv_mass > 0).
		// Static bodies (inv_mass == 0) never have their momentum changed,
		// so contacts involving them don't conflict with each other.
		bool a_dynamic = g_bodies[a].os_com_and_invmass.w > 0;
		bool b_dynamic = g_bodies[b].os_com_and_invmass.w > 0;
		uint used_a = a_dynamic ? g_bodies[a].colour_used : 0;
		uint used_b = b_dynamic ? g_bodies[b].colour_used : 0;

		// Pick the first colour that neither dynamic body has used.
		uint used = used_a | used_b;
		uint colour = min(firstbitlow(~used), (uint)(MaxColours - 1));

		g_colours[idx] = colour;
		if (a_dynamic) g_bodies[a].colour_used |= (1u << colour);
		if (b_dynamic) g_bodies[b].colour_used |= (1u << colour);
	}
}

// ----- CSPositionSolve -----
// Dispatched once per colour from the CPU loop. Each thread processes one contact and only moves body transforms.
// Contacts in other colours return immediately; the CPU changes g.colour and dispatches this entry point repeatedly.
numthreads(CSPositionSolve, ResolveThreadCount, 1, 1)
void CSPositionSolve(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	// This pass uses the contact depth captured by collision detection. The C++ caller scales correction by 1 / push-out iterations so repeated sweeps do
	// not apply the full stale penetration depth each time.
	ApplyPositionCorrection(g_contacts[idx]);
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
//   Phase 2 (per manifold point, only when penetrating): For each manifold point apply a
//   scalar Baumgarte bias impulse (NOT energy-guarded — bias is positional correction, not
//   real KE) plus a friction impulse limited by that point's bias impulse magnitude. This
//   distributes the contact normal force across the manifold for stable stacking (a 4-point
//   face contact can resist torque, a single centroid contact cannot).
numthreads(CSResolve, ResolveThreadCount, 1, 1)
void CSResolve(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];

	// Only process contacts assigned to the current colour batch
	if (g_colours[idx] != (uint)g.colour)
		return;

	// Load the contact and both bodies. Updates are written back only if an impulse is applied, which avoids unnecessary UAV traffic for separating contacts.
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

	// From this point down, contact-space quantities are expressed in body A space unless explicitly suffixed with _ws. Body B's CoM and inverse inertia are
	// transformed into A space so the impulse equations can use one coordinate frame.
	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);

	// Compute inverse inertia tensors in the frames needed by later phases. os_iinv_* are for helper calls in object/A space; ws_iinv_* are for the energy
	// guard after impulses have been transformed into world-space momenta.
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
	float bias = g.bias_scale * (g.velocity_baumgarte / g.dt) * max(c.depth - g.penetration_slop, 0.0f);

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
			float elasticity = g.restitution_scale * rest_factor * (mat_a.elasticity_norm + mat_b.elasticity_norm) * 0.5f;

			float3 impulse = ComputeImpulse(col_I, V_rel, axis, elasticity, friction);
			ApplyImpulseWithEnergyGuard(bodyA, bodyB, impulse, pt,
				com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
				inv_mass_a, inv_mass_b, true);
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

		PR_HLSL_BRANCH
		if (Jn_bias_centroid > 0.0f)
		{
			// ----- Centroid bias impulse (NOT energy-guarded) -----
			// Pseudo-impulse that drives penetration to zero. Applied at the centroid so
			// no torque is generated (no lever arm from the impulse to the CoM offset can
			// drive angular runaway between manifold points).
			ApplyImpulseWithEnergyGuard(bodyA, bodyB, Jn_bias_centroid * axis, pt_c,
				com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
				inv_mass_a, inv_mass_b, false);
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
						inv_mass_a, inv_mass_b, true);
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

#ifdef __cplusplus
}
#endif
