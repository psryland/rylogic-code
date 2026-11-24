//*********************************************
// Physics Engine — GPU Collision Resolution Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Graph-coloured batch collision resolution running on the GPU.
//
// Pipeline:
//   1. CSComputeCollisionTimes - estimates time-of-impact and writes the first sort key.
//   2. Shock-priority passes   - optionally propagate contact priority through body/contact adjacency and rewrite the sort key.
//   3. RadixSort               - stably sorts the live contact prefix by g_contact_times using the GPU count. Driven from C++, not an entry point here.
//   4. CSAssignColours         - greedily graph-colours the sorted contacts so independent contacts can be solved together.
//   5. CSPositionSolve         - split position correction, dispatched once per colour or serially for a colour-overflow frame.
//   6. CSResolve               - velocity impulse solve, dispatched once per colour or serially for a colour-overflow frame.
//
// Within a colour batch, no two contacts share a dynamic body, so writes to body transforms/momenta are data-race free. Static bodies are allowed to appear
// in multiple contacts in one colour because they are never written by the solver. If the bounded colour mask is exhausted, the complete contact set is solved
// serially by thread zero. This rare fallback preserves the same sequential-impulse semantics without allowing same-body writes to race.
//
// Scratch ownership:
//   - g_contact_times is first a sort-key buffer, then a temporary priority buffer during shock propagation, then the final sort-key buffer again.
//   - g_contact_order starts as [0..contact_count) and is permuted by the external radix sort.
//   - g_colours is used as uint/asfloat scratch for Jacobi shock-priority propagation, then reset and reused as the final per-contact colour assignment.
//     Two extra elements store the frame-wide overflow flag and occupied-colour mask.
//   The CPU records leading colours separately, then one complete group-wide tail. A stale head never omits newly occupied colours.
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
//   u8: RWStructuredBuffer<GpuWarmStartEntry>   - previous-frame warm-start cache
//   u9: RWStructuredBuffer<GpuWarmStartEntry>   - current-frame warm-start cache
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).
//   HLSL 'row_major float4x4' rows = C++ columns = basis vectors.
//   mul(v, M) for vector transforms, mul(A, B) = A then B in row-vector convention.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/body_twist.hlsli"

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
	int colour;       // One colour, or MaxColours + head for the complete single-group tail.
	int colour_capacity; // Retained capacity followed by the overflow flag and occupied-colour mask.

	int shock_iterations;  // Number of contact-priority propagation sweeps
	float max_position_speed; // Maximum target speed for shared detached position correction.
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

	float shock_decay;                // Priority attenuation per propagated edge
	float contact_slop_scale;         // Fraction of minimum contact-body thickness used to cap penetration/position slop
	float support_contact_slop_scale; // Fraction used for load-bearing support contacts
	float warm_start_scale;           // Scale applied to cached physical impulses before they are applied this frame

	int warm_start_capacity;  // Number of entries in the open-addressed warm-start cache
	int rigid_body_count;     // Ordinary rigid prefix; larger body indices are articulation-link proxies
	int warm_start_preloaded; // Non-zero when the coupled-contact lane required a separate cache-load pass
	int shared_position_state; // Non-zero when contacts and persistent constraints share detached rigid pseudo velocities.
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
RWStructuredBuffer<GpuWarmStartEntry> resource(g_warm_start_prev, u8); // previous-frame physical impulse cache
RWStructuredBuffer<GpuWarmStartEntry> resource(g_warm_start_curr, u9); // current-frame physical impulse cache
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_position_pseudo, u10);
RWStructuredBuffer<DispatchArguments> resource(g_colour_dispatch, u11);

// ----- Helper functions -----
int ContactCount()
{
	return min(g_counters[0].contact_count, g.max_contacts);
}

// Return true when both endpoints belong to the ordinary rigid-body prefix handled by this solver.
bool RigidContact(GpuResolveContact contact)
{
	return
		contact.body_idx_a >= 0 && contact.body_idx_a < g.rigid_body_count &&
		contact.body_idx_b >= 0 && contact.body_idx_b < g.rigid_body_count;
}

// Return the reserved colour-buffer slot that records whether any enabled contact exhausted the bounded colour mask.
int ColourOverflowIndex()
{
	return g.colour_capacity;
}

// Return true when the complete contact set must use the coherent serial fallback instead of parallel colour batches.
bool ColourOverflow()
{
	return g_colours[ColourOverflowIndex()] != 0;
}

// Returns the packed leaf identity of a contact. Bodies with a single convex shape use leaf 0, so
// this is zero for non-compound scenes and the key is unchanged from a body-pair-only identity.
uint WarmStartChildKey(GpuResolveContact c)
{
	return ((uint)c.child_idx_a << 16) | ((uint)c.child_idx_b & 0xFFFFu);
}

// Return the reserved occupied-colour mask slot. Overflow frames publish an empty mask.
int OccupiedColoursIndex()
{
	return g.colour_capacity + 1;
}

// Return the contact point quantised to 1/64 m in body A space.
int3 WarmStartPoint(GpuResolveContact c)
{
	return int3(
		(int)floor(c.contact_point.x * 64.0f + 0.5f),
		(int)floor(c.contact_point.y * 64.0f + 0.5f),
		(int)floor(c.contact_point.z * 64.0f + 0.5f));
}

// Returns a stable non-zero cache key for a canonical body pair, its colliding leaves, and a quantised local contact point.
uint WarmStartKey(GpuResolveContact c)
{
	int3 q = WarmStartPoint(c);
	uint key =
		(uint)c.body_idx_a * 73856093u ^
		(uint)c.body_idx_b * 19349663u ^
		(uint)q.x * 83492791u ^
		(uint)q.y * 2654435761u ^
		(uint)q.z * 1597334677u ^
		WarmStartChildKey(c) * 2971215073u ^
		0x9E3779B9u;
	return key != 0 ? key : 1u;
}

// Returns the half-extents stored in a bounding box in both HLSL and C++ interop builds.
float3 BBoxRadius(BBox bbox)
{
#ifdef __cplusplus
	return bbox.m_radius.xyz;
#else
	return bbox.radius.xyz;
#endif
}

// Returns the minimum positive object-space thickness represented by a body's bounding box.
float BodyMinThickness(GpuRigidBody body)
{
	float3 size = 2.0f * max(BBoxRadius(body.os_bbox), float3(0, 0, 0));
	float min_size = 1e30f;
	if (size.x > 1e-5f) min_size = min(min_size, size.x);
	if (size.y > 1e-5f) min_size = min(min_size, size.y);
	if (size.z > 1e-5f) min_size = min(min_size, size.z);
	return min_size < 1e29f ? min_size : max(max(size.x, size.y), size.z);
}

// Scales a configured slop by the thinnest contact body while keeping the configured value as an upper bound.
float ContactSlop(float configured_slop, GpuRigidBody bodyA, GpuRigidBody bodyB, bool support_contact)
{
	float slop_scale = support_contact ? g.support_contact_slop_scale : g.contact_slop_scale;
	if (slop_scale <= 0.0f)
		return configured_slop;

	float min_thickness = min(BodyMinThickness(bodyA), BodyMinThickness(bodyB));
	float scaled_slop = slop_scale * min_thickness;
	return min(configured_slop, max(scaled_slop, 0.0f));
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

// Extrapolate a body's o2w transform by dt using current velocity and forces.
// Mirrors the CPU ExtrapolateO2W: S = S0 + dt * v + 0.5*dt^2 * I^f.
// Used to rewind bodies to the estimated collision time.
float4x4 ExtrapolateO2W(GpuRigidBody body, float dt)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3 os_com = body.os_com_and_invmass.xyz;
	float3x3 os_iinv = OsInverseInertia(body);
	float3x3 rot = (float3x3)body.o2w;
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, rot);

	// Spatial displacement: dx = dt * v + 0.5 * dt^2 * I^f. This is only used over a small sub-step rewind/advance, so the linearised rotation below is
	// preferable to a heavier quaternion/exponential-map update here. Kinematic bodies have infinite mass, so only their twist moves them.
	BodyTwist twist = BodyTwistWS(body);
	float3 dx_ang = dt * twist.ang + 0.5f * dt * dt * mul(ws_iinv, body.force_ang.xyz);
	float3 dx_lin = dt * twist.lin + 0.5f * dt * dt * inv_mass * body.force_lin.xyz;

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
float3 BodyVelocityAtPoint(GpuRigidBody body, float3 pt_in_a, float3 com_in_a, float3x3 rot_a)
{
	// World→body A: with HLSL row_major matrices (rows = body axes in world), mul(rot_a, v_world)
	// applies the world→body transformation directly. Using transpose(rot_a) here would give
	// the (geometrically meaningless) sum of body-axes-in-world weighted by world components,
	// which yields wrong body-frame velocities for any non-axis-aligned rotation.
	BodyTwist twist = BodyTwistWS(body);
	float3 omega_in_a = mul(rot_a, twist.ang);
	float3 v_com_in_a = mul(rot_a, twist.lin);

	// Velocity is returned in body A space because contacts store their centroid/normal in A space. Keeping all contact math in one frame avoids repeated
	// world<->object conversions and keeps the normal impulse direction consistent with c.axis.
	return v_com_in_a + cross(omega_in_a, pt_in_a - com_in_a);
}

// Compute the relative velocity of B w.r.t. A at point 'pt' (in A's object space).
float3 RelativeVelocityAtPoint(GpuRigidBody bodyA, GpuRigidBody bodyB, float3 pt, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
{
	float3 v_a = BodyVelocityAtPoint(bodyA, pt, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(bodyB, pt, com_b_in_a, rot_a);
	return v_b - v_a;
}

// Convenience wrapper: relative velocity at the contact centroid.
float3 RelativeVelocityAtContact(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB, float3x3 rot_a, float3 com_a_in_a, float3 com_b_in_a)
{
	return RelativeVelocityAtPoint(bodyA, bodyB, c.contact_point.xyz, rot_a, com_a_in_a, com_b_in_a);
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

// Convert the material's bounded friction ratio to the Coulomb cone slope.
float FrictionCoefficient(float friction)
{
	float ratio = min(friction, 0.9999f);
	return ratio / (1.000001f - ratio);
}

// Return only the change needed to move this point's accumulated tangent impulse within its physical support budget.
float3 ComputeFrictionImpulse(float3x3 col_I_inv, float3 V_rel, float3 axis, float friction, float Jn, float3 old_impulse)
{
	// Oppose the current slip, but project the accumulated impulse so extra solver iterations cannot spend the same normal support again.
	float3 V_tan = V_rel - dot(V_rel, axis) * axis;
	float v_tan_sq = dot(V_tan, V_tan);
	if (v_tan_sq < 1e-12f)
		return float3(0, 0, 0);

	float v_tan_len = sqrt(v_tan_sq);
	float3 tangent = V_tan / v_tan_len;
	float k_t = dot(tangent, mul(col_I_inv, tangent));
	if (k_t < 1e-12f)
		return float3(0, 0, 0);

	float3 candidate = old_impulse - (v_tan_len / k_t) * tangent;
	float candidate_length = length(candidate);
	float limit = friction * max(Jn, 0.0f);
	if (candidate_length > limit && candidate_length > 0.0f)
		candidate *= limit / candidate_length;

	return candidate - old_impulse;
}

// Convert a point impulse at 'pt' (in A's space), apply it to both bodies, and return the impulse that survived the energy guard.
float3 ApplyImpulseWithEnergyGuard(
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
	AddBodyMomentumWS(bodyA, torqueA_ws, forceA_ws);
	AddBodyMomentumWS(bodyB, torqueB_ws, forceB_ws);

	// Energy conservation guard: if KE increased, scale the impulse down. Bias impulses disable this because they are pseudo-impulses whose purpose is to
	// remove positional error, not conserve physical kinetic energy. A moving kinematic body correctly adds energy when it pushes, so the guard is off for
	// contacts with a kinematic body.
	float ke_after = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	float delta = ke_after - ke_before;
	if (enable_energy_guard && !KinematicBody(bodyA) && !KinematicBody(bodyB) && delta > 0 && A > 1e-12f)
	{
		float correction = 1.0f - clamp((A - delta) / A, 0.0f, 1.0f);
		AddBodyMomentumWS(bodyA, -correction * torqueA_ws, -correction * forceA_ws);
		AddBodyMomentumWS(bodyB, -correction * torqueB_ws, -correction * forceB_ws);
		return (1.0f - correction) * impulse;
	}

	return impulse;
}

// Apply an equal and opposite pure angular impulse (in A's space) to the two bodies: A receives '-impulse' and B receives '+impulse'.
// Returns the part of the impulse that survived the energy guard. The guard scales the impulse down if it would increase kinetic energy.
float3 ApplyAngularImpulseWithEnergyGuard(
	inout_(GpuRigidBody) bodyA, inout_(GpuRigidBody) bodyB,
	float3 impulse, float3x3 rot_a, float3x3 ws_iinv_a, float3x3 ws_iinv_b)
{
	// Rigid body state stores world-space momentum, so convert the A-space torque impulse to world space first
	float3 impulse_ws = mul(impulse, rot_a);

	// Quadratic kinetic energy coefficient for this impulse direction. It gives the fraction to remove if the trial impulse adds energy.
	float A = 0.5f * (dot(mul(ws_iinv_a, impulse_ws), impulse_ws) + dot(mul(ws_iinv_b, impulse_ws), impulse_ws));
	float ke_before = KineticEnergy(bodyA) + KineticEnergy(bodyB);
	AddBodyMomentumWS(bodyA, -impulse_ws, float3(0, 0, 0));
	AddBodyMomentumWS(bodyB, +impulse_ws, float3(0, 0, 0));

	// A moving kinematic body may correctly add energy, so the guard only applies between non-kinematic bodies
	float delta = KineticEnergy(bodyA) + KineticEnergy(bodyB) - ke_before;
	if (!KinematicBody(bodyA) && !KinematicBody(bodyB) && delta > 0 && A > 1e-12f)
	{
		float correction = 1.0f - clamp((A - delta) / A, 0.0f, 1.0f);
		AddBodyMomentumWS(bodyA, +correction * impulse_ws, float3(0, 0, 0));
		AddBodyMomentumWS(bodyB, -correction * impulse_ws, float3(0, 0, 0));
		return (1.0f - correction) * impulse;
	}
	return impulse;
}

// Return the change to the accumulated angular impulse 'old_impulse' (A space) that opposes the relative angular velocity 'w_rel' (B relative to A)
// while keeping the accumulated rolling part (perpendicular to 'axis') within 'roll_limit' and the spin part (along 'axis') within 'spin_limit'.
// 'K' is the sum of the two bodies' inverse inertias in A space, mapping angular impulse to change in relative angular velocity.
float3 ComputeRollingResistanceImpulse(float3x3 K, float3 w_rel, float3 axis, float roll_limit, float spin_limit, float3 old_impulse)
{
	// Split the relative angular velocity and the accumulator into spin (about the normal) and rolling (perpendicular to the normal) parts
	float w_spin = dot(w_rel, axis);
	float3 w_roll = w_rel - w_spin * axis;
	float old_spin = dot(old_impulse, axis);
	float3 old_roll = old_impulse - old_spin * axis;

	// Spin: a scalar accumulated impulse about the normal, clamped to [-spin_limit, +spin_limit]
	float new_spin = old_spin;
	float k_s = dot(axis, mul(K, axis));
	if (spin_limit > 0.0f && k_s > 1e-12f)
		new_spin = clamp(old_spin - w_spin / k_s, -spin_limit, spin_limit);

	// Rolling: oppose the current rolling direction, then project the accumulated impulse back into a disc of radius 'roll_limit'.
	// Using the current rolling direction (like the tangent friction solve) avoids inverting a possibly singular 2x2 block.
	float3 new_roll = old_roll;
	float w_roll_len = length(w_roll);
	if (roll_limit > 0.0f && w_roll_len > 1e-6f)
	{
		float3 dir = w_roll / w_roll_len;
		float k_r = dot(dir, mul(K, dir));
		if (k_r > 1e-12f)
		{
			new_roll = old_roll - (w_roll_len / k_r) * dir;
			float len = length(new_roll);
			if (len > roll_limit)
				new_roll *= roll_limit / len;
		}
	}

	return (new_roll + new_spin * axis) - old_impulse;
}

// Apply a non-physical Baumgarte bias impulse to linear momentum only, so positional error correction does not inject spin through an unstable contact point.
float3 ApplyLinearBiasImpulse(inout_(GpuRigidBody) bodyA, inout_(GpuRigidBody) bodyB, float3 impulse, float3x3 rot_a)
{
	float3 impulse_ws = mul(impulse, rot_a);
	AddBodyMomentumWS(bodyA, float3(0, 0, 0), -impulse_ws);
	AddBodyMomentumWS(bodyB, float3(0, 0, 0), impulse_ws);
	return impulse;
}

// Extracts the positive normal part of a cached impulse in the current contact frame.
float3 WarmStartNormalImpulse(GpuResolveContact c, float3 impulse)
{
	// Warm starting is deliberately conservative here: the cache key does not identify a stable manifold feature/tangent basis, so replaying old tangential
	// components can inject sideways or angular energy when the contact patch shifts. Keep only the part that still pushes along the current contact normal.
	float normal_impulse = dot(impulse, c.axis.xyz);
	return normal_impulse > 1e-6f ? normal_impulse * c.axis.xyz : float3(0, 0, 0);
}

// Return true when 'body_idx' changed motion type since the last step, so impulses cached against its old mass are not valid.
bool ResetsWarmStart(int body_idx)
{
	// Only bodies in the body buffer carry state flags. World endpoints never reset.
	return body_idx >= 0 && body_idx < g.body_count && AllSet(g_bodies[body_idx].state_flags, ERigidBodyStateFlags_ResetWarmStart);
}

// Compare the complete contact identity, not only its hash.
bool WarmStartMatches(GpuWarmStartEntry entry, GpuResolveContact c, uint key)
{
	int3 q = WarmStartPoint(c);
	return
		entry.key == key &&
		entry.body_idx_a == c.body_idx_a &&
		entry.body_idx_b == c.body_idx_b &&
		entry.child_key == WarmStartChildKey(c) &&
		entry.point_key.x == q.x &&
		entry.point_key.y == q.y &&
		entry.point_key.z == q.z;
}

// Order duplicate candidates independently of insertion order, preferring the smaller cached impulse.
bool WarmStartPrecedes(float3 a, float3 b)
{
	float length_sq_a = dot(a, a);
	float length_sq_b = dot(b, b);
	if (length_sq_a != length_sq_b)
		return length_sq_a < length_sq_b;

	// Raw-bit tie breaking makes the choice total even when magnitudes match.
	uint3 bits_a = asuint(a);
	uint3 bits_b = asuint(b);
	if (bits_a.x != bits_b.x)
		return bits_a.x < bits_b.x;
	if (bits_a.y != bits_b.y)
		return bits_a.y < bits_b.y;

	return bits_a.z < bits_b.z;
}

// Looks up the previous-frame normal impulse for this contact identity and projects it onto the current contact normal.
bool LoadWarmStartImpulse(GpuResolveContact c, out_(float3) impulse)
{
	impulse = float3(0, 0, 0);
	if (g.warm_start_capacity <= 0 || g.warm_start_scale <= 0.0f)
		return false;

	// A body whose motion type changed starts its contacts cold, because the cached impulse was computed for its old mass.
	if (ResetsWarmStart(c.body_idx_a) || ResetsWarmStart(c.body_idx_b))
		return false;

	uint key = WarmStartKey(c);
	uint start = key % (uint)g.warm_start_capacity;
	bool found = false;
	float3 cached = float3(0, 0, 0);
	for (int probe = 0; probe != g.warm_start_capacity; ++probe)
	{
		uint slot = (start + (uint)probe) % (uint)g.warm_start_capacity;
		GpuWarmStartEntry entry = g_warm_start_prev[slot];
		if (entry.key == 0)
			break;
		if (!WarmStartMatches(entry, c, key))
			continue;
		if (found && !WarmStartPrecedes(entry.impulse.xyz, cached))
			continue;

		cached = entry.impulse.xyz;
		found = true;
	}
	if (!found)
		return false;

	impulse = WarmStartNormalImpulse(c, g.warm_start_scale * cached);
	return any(impulse != float3(0, 0, 0));
}

// Inserts this frame's accumulated normal contact impulse into the current warm-start cache.
void StoreWarmStartImpulse(GpuResolveContact c)
{
	if (g.warm_start_capacity <= 0 || g.warm_start_scale <= 0.0f)
		return;

	float3 impulse = WarmStartNormalImpulse(c, c.warmstart_impulse.xyz);
	if (!any(impulse != float3(0, 0, 0)))
		return;

	// Each writer owns an empty slot, including duplicates. The cache has at least twice the contact capacity, so a free slot always exists.
	// Later lookup selects a duplicate deterministically; no two threads write one multi-field entry.
	uint key = WarmStartKey(c);
	int3 q = WarmStartPoint(c);
	uint start = key % (uint)g.warm_start_capacity;
	for (int probe = 0; probe != g.warm_start_capacity; ++probe)
	{
		uint slot = (start + (uint)probe) % (uint)g.warm_start_capacity;
		uint old_key = 0;
		InterlockedCompareExchange(g_warm_start_curr[slot].key, 0u, key, old_key);
		if (old_key == 0)
		{
			g_warm_start_curr[slot].body_idx_a = c.body_idx_a;
			g_warm_start_curr[slot].body_idx_b = c.body_idx_b;
			g_warm_start_curr[slot].child_key = WarmStartChildKey(c);
			g_warm_start_curr[slot].point_key = int4(q, 0);
			g_warm_start_curr[slot].impulse = float4(impulse, 0);
			return;
		}
	}
}

// Return the relative velocity of B w.r.t. A at the stored contact centroid, in A's object space.
float3 ContactRelativeVelocity(GpuResolveContact c)
{
	// Most resolver calculations use body A space. Body B's CoM is transformed into A space using the current b2a transform so relative velocity can be
	// evaluated at the stored contact centroid without changing coordinate frames.
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];
	float3 os_com_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_b_in_a = c.b2a[3].xyz + mul(os_com_b, b2a_rot);
	return RelativeVelocityAtContact(c, bodyA, bodyB, rot_a, os_com_a, com_b_in_a);
}

// Estimate the sub-step collision time for a contact with relative contact velocity 'rel_vel' (see ContactRelativeVelocity).
// Projects the contact point backward along the relative velocity to find when penetration began.
// Returns a negative value (earlier in the timestep = more negative).
float EstimateCollisionTime(GpuResolveContact c, float3 rel_vel)
{
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

// Return whether a loaded contact belongs to the current full or selective resolve pass.
bool ContactEnabledFor(GpuResolveContact c)
{
	return RigidContact(c) && ContactEnabled(c, g_bodies[c.body_idx_a]);
}

// Return whether a contact index belongs to the current full or selective resolve pass.
bool ContactEnabledAt(uint idx)
{
	return ContactEnabledFor(g_contacts[idx]);
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
	// Kinematic bodies store a twist rather than momentum in the momentum slots, so they add no source momentum; their motion still raises the closing speed.
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 com_b_in_a = c.b2a[3].xyz + mul(bodyB.os_com_and_invmass.xyz, b2a_rot);
	float3 v_a = BodyVelocityAtPoint(bodyA, c.contact_point.xyz, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(bodyB, c.contact_point.xyz, com_b_in_a, rot_a);
	float3 v_rel = v_b - v_a;
	float3 axis_ws = mul(c.axis.xyz, rot_a);
	float closing_speed = max(0.0f, -dot(v_rel, c.axis.xyz));
	float source_momentum_a = KinematicBody(bodyA) ? 0.0f : abs(dot(bodyA.momentum_lin.xyz, axis_ws));
	float source_momentum_b = KinematicBody(bodyB) ? 0.0f : abs(dot(bodyB.momentum_lin.xyz, axis_ws));
	float source_momentum = max(source_momentum_a, source_momentum_b);

	return support_height - source_momentum - 0.05f * closing_speed - 0.50f * c.depth;
}

// Estimate how urgently this contact needs a normal impulse before considering graph propagation.
// This seed priority combines normal closing speed with Baumgarte bias demand from penetration depth.
float ContactNormalDemand(GpuResolveContact c, GpuRigidBody bodyA, GpuRigidBody bodyB)
{
	// This is the seed strength for shock-priority propagation. Closing contacts and deeply penetrating contacts are the places where an early impulse is
	// most likely to help neighbouring contacts in a dependency chain.
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 com_b_in_a = c.b2a[3].xyz + mul(bodyB.os_com_and_invmass.xyz, b2a_rot);
	float3 v_rel = RelativeVelocityAtContact(c, bodyA, bodyB, rot_a, com_a_in_a, com_b_in_a);
	float closing_speed = max(0.0f, -dot(v_rel, c.axis.xyz));
	bool support_contact = SupportContact(c, bodyA);
	float bias_speed = (g.velocity_baumgarte / max(g.dt, 1e-6f)) *
		max(c.depth - ContactSlop(g.penetration_slop, bodyA, bodyB, support_contact), 0.0f);
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
float PositionCorrectionDistance(float depth, float position_slop)
{
	float position_pen = max(depth - position_slop, 0.0f);
	float position_correction = g.position_baumgarte * position_pen;

	float deep_pen = max(depth - g.deep_penetration_threshold, 0.0f);
	float deep_range = max(g.deep_penetration_range, 1e-6f);
	float deep_baumgarte = lerp(g.deep_penetration_baumgarte_min, g.deep_penetration_baumgarte_max, saturate(deep_pen / deep_range));
	float deep_correction = deep_baumgarte * deep_pen;

	return g.bias_scale * g.position_correction_scale * max(position_correction, deep_correction);
}

// Add contact separation to shared detached state while respecting corrections already contributed by connected constraints.
void ApplySharedPositionCorrection(GpuResolveContact contact, GpuRigidBody body_a, GpuRigidBody body_b, float3 axis_ws, float correction, float inv_mass_a, float inv_mass_b)
{
	// Static endpoints are not written because independent colour groups may share the same static body.
	GpuConstraintPseudoVelocity pseudo_a;
	pseudo_a.angular_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
	pseudo_a.linear_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
	GpuConstraintPseudoVelocity pseudo_b = pseudo_a;
	if (inv_mass_a > 0.0f)
		pseudo_a = g_position_pseudo[contact.body_idx_a];

	if (inv_mass_b > 0.0f)
		pseudo_b = g_position_pseudo[contact.body_idx_b];

	// Angular joint corrections also move the contact point, even though contact separation itself remains linear-only.
	float3 point_ws = mul(float4(contact.contact_point.xyz, 1.0f), body_a.o2w).xyz;
	float3 com_a_ws = body_a.o2w[3].xyz + mul(body_a.os_com_and_invmass.xyz, (float3x3)body_a.o2w);
	float3 com_b_ws = body_b.o2w[3].xyz + mul(body_b.os_com_and_invmass.xyz, (float3x3)body_b.o2w);
	float3 velocity_a = pseudo_a.linear_velocity.xyz + cross(pseudo_a.angular_velocity.xyz, point_ws - com_a_ws);
	float3 velocity_b = pseudo_b.linear_velocity.xyz + cross(pseudo_b.angular_velocity.xyz, point_ws - com_b_ws);
	float target_speed = min(g.max_position_speed, correction / (g.dt * g.position_correction_scale));
	float correction_speed = max(target_speed - dot(velocity_b - velocity_a, axis_ws), 0.0f);
	float3 impulse = (correction_speed / (inv_mass_a + inv_mass_b)) * axis_ws;

	// Colouring makes these dynamic-body updates exclusive within each dispatch.
	if (inv_mass_a > 0.0f)
	{
		pseudo_a.linear_velocity += float4(-inv_mass_a * impulse, 0.0f);
		g_position_pseudo[contact.body_idx_a] = pseudo_a;
	}
	if (inv_mass_b > 0.0f)
	{
		pseudo_b.linear_velocity += float4(inv_mass_b * impulse, 0.0f);
		g_position_pseudo[contact.body_idx_b] = pseudo_b;
	}
}

// Apply split positional correction without changing momenta, using shared pseudo state when constraints are present.
// Dynamic endpoints separate in inverse-mass proportion; static bodies remain fixed.
void ApplyPositionCorrection(GpuResolveContact c)
{
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float total_inv = inv_mass_a + inv_mass_b;
	bool support_contact = SupportContact(c, bodyA);
	float correction = PositionCorrectionDistance(c.depth, ContactSlop(g.position_slop, bodyA, bodyB, support_contact));
	if ((correction <= 0.0f && g.shared_position_state == 0) || total_inv <= 0.0f)
		return;

	// Connected constraints and contacts must see one fixed-configuration correction state before any transform changes.
	float3 axis_ws = mul(c.axis.xyz, (float3x3)bodyA.o2w);
	if (g.shared_position_state != 0)
	{
		ApplySharedPositionCorrection(c, bodyA, bodyB, axis_ws, correction, inv_mass_a, inv_mass_b);
		return;
	}

	// Contact-only frames retain direct transform correction without allocating optional pseudo storage.
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
// Parallel: compute only live contact keys; the GPU-counted sort never reads or writes the inactive suffix.
numthreads(CSComputeCollisionTimes, ResolveThreadCount, 1, 1)
void CSComputeCollisionTimes(int3 DTID(dtid))
{
	// Live keys and indices retain their original arithmetic and contact order.
	int idx = dtid.x;
	int contact_count = ContactCount();
	if (idx < contact_count)
	{
		// The contact buffer has already been prepared by the narrowphase/interop path. This pass does not
		// change contact geometry; it records the estimated collision time, the pre-solve approach speed, and a sortable index.
		GpuResolveContact c = g_contacts[idx];
		GpuRigidBody bodyA = g_bodies[c.body_idx_a];
		GpuRigidBody bodyB = g_bodies[c.body_idx_b];
		float3 rel_vel = ContactRelativeVelocity(c);
		float collision_time = EstimateCollisionTime(c, rel_vel);
		g_contacts[idx].collision_time = collision_time;

		// Capture the normal speed before warm starting and velocity iterations change it. Restitution must respond to this impact speed only; closing speed
		// that other constraints re-create during the iterations (for example a motor pushing into a slope) is not an impact and must not bounce.
		g_contacts[idx].approach_speed = dot(rel_vel, c.axis.xyz);

		// Collision time remains primary. The propagation key is deliberately tiny and only reorders near-simultaneous
		// contacts so support reactions and high-residual impact contacts get an earlier colour in the sequential solve.
		float propagation_key = ContactEnabled(c, bodyA) ? PropagationSortKey(c, bodyA, bodyB) : 1e30f;
		g_contact_times[idx] = collision_time + g.propagation_key_scale * propagation_key;
		g_contact_order[idx] = idx;

		// Position correction is applied after graph colouring by CSPositionSolve so contacts sharing a dynamic body are never written in parallel.
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
	if (!RigidContact(c))
		return;

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
	if (!RigidContact(src))
		return;

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
// Walks contacts in the order sorted by collision time and greedily gives each one the lowest colour that neither of its dynamic bodies has used.
// Per-body colour bitmasks are kept in g_bodies[].colour_used, which this pass clears first. The assignment is inherently serial because each
// contact's colour depends on the colours already reserved by earlier contacts. Earlier contacts in g_contact_order get lower colours and therefore get
// solved earlier in each solver iteration.

// Return whether a body's colour mask constrains colouring. Static bodies (inv_mass == 0) never have their momentum changed, so contacts involving them
// do not conflict with each other.
bool ColourDynamic(int body)
{
	return g_bodies[body].os_com_and_invmass.w > 0;
}

// Write the frame-wide colouring results. Threads 'thread' of 'thread_count' share the work, so a serial caller passes 0 and 1.
void PublishColourResults(bool colour_overflow, uint used_colours, int contact_count, uint thread, uint thread_count)
{
	// A single overflow makes the complete solve use one coherent serial order, so partial assignments are replaced with the disabled sentinel. Colour
	// passes after zero can then return through their ordinary local filter without reading the overflow flag.
	if (colour_overflow)
	{
		for (int i = (int)thread; i < contact_count; i += (int)thread_count)
			g_colours[g_contact_order[i]] = MaxColours;
	}

	// Overflow has no parallel colours. The same mask drives fused sweeps, the complete tail and the next CPU head estimate.
	if (thread == 0)
	{
		g_colours[ColourOverflowIndex()] = colour_overflow ? 1u : 0u;
		g_colours[OccupiedColoursIndex()] = colour_overflow ? 0u : used_colours;
	}

	// Preserve the full live-contact grid for occupied colours. Empty colours do no work; overflow needs only the original serial thread in colour zero.
	uint groups = (contact_count + ResolveThreadCount - 1) / ResolveThreadCount;
	for (int colour = (int)thread; colour < MaxColours; colour += (int)thread_count)
	{
		DispatchArguments args;
		args.ThreadGroupCountX = colour_overflow ? (colour == 0 ? 1u : 0u) : ((used_colours & (1u << colour)) != 0 ? groups : 0u);
		args.ThreadGroupCountY = 1;
		args.ThreadGroupCountZ = 1;
		g_colour_dispatch[colour] = args;
	}
}

#ifdef __cplusplus

// Serial form for the C++ replay emulator, which cannot synchronise the threads of a group. It produces the same assignment as the HLSL form below.
numthreads(CSAssignColours, 1, 1, 1)
void CSAssignColours(int3 DTID(dtid))
{
	// Only one thread exists. Start every body with no reserved colours.
	if (dtid.x != 0)
		return;

	for (int i = 0; i != g.body_count; ++i)
		g_bodies[i].colour_used = 0;

	// Colour the sorted contacts. Exhaustion is represented by the MaxColours sentinel rather than aliasing the final colour, because aliasing would let
	// multiple contacts write the same dynamic body concurrently. The first exhausted mask stops the walk.
	bool colour_overflow = false;
	uint used_colours = 0;
	int contact_count = ContactCount();
	for (int i = 0; i != contact_count; ++i)
	{
		// g_contact_order is the radix-sorted permutation. Colouring in this order is what turns the contact-priority sort into actual solve order.
		int idx = g_contact_order[i];
		GpuResolveContact c = g_contacts[idx];
		// Contacts outside this pass receive the MaxColours sentinel, which every colour pass skips.
		if (!ContactEnabledFor(c))
		{
			g_colours[idx] = MaxColours;
			continue;
		}

		// Pick the first colour that neither dynamic body has used.
		int a = c.body_idx_a;
		int b = c.body_idx_b;
		bool a_dynamic = ColourDynamic(a);
		bool b_dynamic = ColourDynamic(b);
		uint used = (a_dynamic ? g_bodies[a].colour_used : 0) | (b_dynamic ? g_bodies[b].colour_used : 0);
		if (used == ~0u)
		{
			g_colours[idx] = MaxColours;
			colour_overflow = true;
			break;
		}

		uint colour = firstbitlow(~used);
		g_colours[idx] = colour;
		used_colours |= (1u << colour);
		if (a_dynamic) g_bodies[a].colour_used |= (1u << colour);
		if (b_dynamic) g_bodies[b].colour_used |= (1u << colour);
	}
	PublishColourResults(colour_overflow, used_colours, contact_count, 0, 1);
}

#else

// The GPU form keeps the same serial greedy walk on thread zero, but other threads first stage each chunk of sorted contacts and their bodies' masks
// in group-shared memory. The walk then reads only group-shared memory instead of waiting on several dependent device-memory loads per contact.
// Masks are written back to g_bodies after each chunk, so a body that appears in several chunks keeps the colours reserved by earlier chunks.
static const int ColourThreadCount = 256;
static const int ColourChunkSize = 512;
static const uint ColourBodySlots = 2048; // Power of two, more than the 2 * ColourChunkSize bodies a chunk can reference, so probing always ends.
static const uint ColourNoBody = 0xFFFFFFFFu;
static const int ColourIneligible = -2;

groupshared uint gs_colour_body[ColourBodySlots];
groupshared uint gs_colour_mask[ColourBodySlots];
groupshared uint gs_colour_contact[ColourChunkSize];
groupshared int gs_colour_slot_a[ColourChunkSize];
groupshared int gs_colour_slot_b[ColourChunkSize];
groupshared uint gs_colour_used;
groupshared uint gs_colour_overflow;

// Return the group-shared mask slot of a dynamic body, or -1 for a static body. Concurrent callers for the same body receive the same slot.
int ColourBodySlot(int body)
{
	// Static bodies never reserve colours.
	if (!ColourDynamic(body))
		return -1;

	// Open addressing with linear probing. The compare-exchange claims an empty slot or finds the slot that another thread already claimed for this body.
	uint slot = ((uint)body * 2654435761u) & (ColourBodySlots - 1);
	for (;;)
	{
		uint previous;
		InterlockedCompareExchange(gs_colour_body[slot], ColourNoBody, (uint)body, previous);
		if (previous == ColourNoBody || previous == (uint)body)
			return (int)slot;

		slot = (slot + 1) & (ColourBodySlots - 1);
	}
}

numthreads(CSAssignColours, ColourThreadCount, 1, 1)
void CSAssignColours(int3 GTID(gtid))
{
	// Start every body with no reserved colours. The device barrier makes the cleared masks visible to the mask loads of the first chunk.
	uint thread = gtid.x;
	for (int i = (int)thread; i < g.body_count; i += ColourThreadCount)
		g_bodies[i].colour_used = 0;
	if (thread == 0)
	{
		gs_colour_used = 0;
		gs_colour_overflow = 0;
	}
	DeviceMemoryBarrierWithGroupSync();

	// Colour the sorted contacts one chunk at a time. Every value that controls the loop is uniform, so every thread reaches every barrier.
	int contact_count = ContactCount();
	for (int base = 0; base < contact_count; base += ColourChunkSize)
	{
		// Empty the body table; each chunk references at most 2 * ColourChunkSize bodies.
		int chunk_count = min(ColourChunkSize, contact_count - base);
		for (uint s = thread; s < ColourBodySlots; s += ColourThreadCount)
			gs_colour_body[s] = ColourNoBody;
		GroupMemoryBarrierWithGroupSync();

		// Stage the chunk's contacts in sorted order and give each dynamic body a table slot. g_contact_order is the radix-sorted permutation;
		// colouring in this order is what turns the contact-priority sort into actual solve order.
		for (int j = (int)thread; j < chunk_count; j += ColourThreadCount)
		{
			uint idx = g_contact_order[base + j];
			GpuResolveContact c = g_contacts[idx];
			bool eligible = ContactEnabledFor(c);
			gs_colour_contact[j] = idx;
			gs_colour_slot_a[j] = eligible ? ColourBodySlot(c.body_idx_a) : ColourIneligible;
			gs_colour_slot_b[j] = eligible ? ColourBodySlot(c.body_idx_b) : ColourIneligible;
		}
		GroupMemoryBarrierWithGroupSync();

		// Load the colours each body has reserved in earlier chunks.
		for (uint s = thread; s < ColourBodySlots; s += ColourThreadCount)
		{
			if (gs_colour_body[s] != ColourNoBody)
				gs_colour_mask[s] = g_bodies[gs_colour_body[s]].colour_used;
		}
		GroupMemoryBarrierWithGroupSync();

		// Walk the chunk serially, picking the first colour that neither dynamic body has used. Exhaustion is represented by the MaxColours sentinel
		// rather than aliasing the final colour, because aliasing would let multiple contacts write the same dynamic body concurrently.
		if (thread == 0)
		{
			uint used_colours = gs_colour_used;
			for (int k = 0; k != chunk_count; ++k)
			{
				uint idx = gs_colour_contact[k];
				int slot_a = gs_colour_slot_a[k];
				int slot_b = gs_colour_slot_b[k];
				if (slot_a == ColourIneligible)
				{
					g_colours[idx] = MaxColours;
					continue;
				}

				// Both slots are the same when a contact joins a body to itself; the masks then combine exactly as in the serial form.
				uint used = (slot_a >= 0 ? gs_colour_mask[slot_a] : 0u) | (slot_b >= 0 ? gs_colour_mask[slot_b] : 0u);
				if (used == ~0u)
				{
					g_colours[idx] = MaxColours;
					gs_colour_overflow = 1;
					break;
				}

				uint colour = firstbitlow(~used);
				g_colours[idx] = colour;
				used_colours |= (1u << colour);
				if (slot_a >= 0) gs_colour_mask[slot_a] |= (1u << colour);
				if (slot_b >= 0) gs_colour_mask[slot_b] |= (1u << colour);
			}
			gs_colour_used = used_colours;
		}
		GroupMemoryBarrierWithGroupSync();

		// Persist the updated masks for later chunks. The device barrier orders these writes before the next chunk's mask loads.
		for (uint s = thread; s < ColourBodySlots; s += ColourThreadCount)
		{
			if (gs_colour_body[s] != ColourNoBody)
				g_bodies[gs_colour_body[s]].colour_used = gs_colour_mask[s];
		}
		DeviceMemoryBarrierWithGroupSync();

		// The first exhausted mask stops the walk, as in the serial form.
		if (gs_colour_overflow != 0)
			break;
	}

	// The barrier above makes the shared totals visible to every thread.
	PublishColourResults(gs_colour_overflow != 0, gs_colour_used, contact_count, thread, ColourThreadCount);
}

#endif

// ----- CSWarmStartClear -----
// Clears the open-addressed current-frame warm-start cache before contacts insert their final accumulated impulses.
numthreads(CSWarmStartClear, ResolveThreadCount, 1, 1)
void CSWarmStartClear(int3 DTID(dtid))
{
	if (dtid.x >= g.warm_start_capacity)
		return;

	g_warm_start_curr[dtid.x].key = 0;
	g_warm_start_curr[dtid.x].body_idx_a = -1;
	g_warm_start_curr[dtid.x].body_idx_b = -1;
	g_warm_start_curr[dtid.x].child_key = 0;
	g_warm_start_curr[dtid.x].point_key = int4(0, 0, 0, 0);
	g_warm_start_curr[dtid.x].impulse = float4(0, 0, 0, 0);
}

// Load one cached physical impulse into the shared per-contact accumulator without mutating either endpoint.
void LoadWarmStartContact(uint idx)
{
	GpuResolveContact c = g_contacts[idx];
	float3 impulse = float3(0, 0, 0);
	LoadWarmStartImpulse(c, impulse);
	c.warmstart_impulse = float4(impulse, 0);
	g_contacts[idx] = c;
}

// Apply one cached physical impulse to an ordinary rigid contact, loading it on demand when no coupled consumer required the separate preload pass.
// Callers guarantee that no other executing invocation can write either dynamic endpoint.
void ApplyWarmStartContact(uint idx)
{
	GpuResolveContact c = g_contacts[idx];
	float3 impulse = float3(0, 0, 0);
	if (g.warm_start_preloaded != 0)
	{
		impulse = c.warmstart_impulse.xyz;
	}
	else if (!LoadWarmStartImpulse(c, impulse))
	{
		return;
	}
	if (!any(impulse != float3(0, 0, 0)))
		return;

	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 os_iinv_a = OsInverseInertia(bodyA);
	float3x3 os_iinv_b = OsInverseInertia(bodyB);
	float3x3 ws_iinv_a = rotate_inertia_inv(os_iinv_a, rot_a);
	float3x3 ws_iinv_b = rotate_inertia_inv(os_iinv_b, (float3x3)bodyB.o2w);
	float3x3 b2a_rot = (float3x3)c.b2a;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 com_b_in_a = c.b2a[3].xyz + mul(bodyB.os_com_and_invmass.xyz, b2a_rot);
	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;

	float3 applied_impulse = ApplyImpulseWithEnergyGuard(bodyA, bodyB, impulse, c.contact_point.xyz,
		com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
		inv_mass_a, inv_mass_b, true);

	// A preload is only a proposal. Persist the accepted support, including zero, before friction or cache storage can consume it.
	c.warmstart_impulse = float4(applied_impulse, 0);
	g_contacts[idx] = c;
	if (!any(applied_impulse != float3(0, 0, 0)))
		return;

	bodyA.state_flags = SetFlag(bodyA.state_flags, ERigidBodyStateFlags_Collided, true);
	bodyB.state_flags = SetFlag(bodyB.state_flags, ERigidBodyStateFlags_Collided, true);

	// Leave sleep state to CSReduceSleepStats after the complete solve.
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

// ----- CSLoadWarmStart -----
// Loads every current contact before the rigid and articulation-coupled lanes consume their disjoint subsets.
numthreads(CSLoadWarmStart, ResolveThreadCount, 1, 1)
void CSLoadWarmStart(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	LoadWarmStartContact(dtid.x);
}

// Operations shared by per-colour, tail and fused sweeps.
static const uint EContactSweepOp_ApplyWarmStart = 0;
static const uint EContactSweepOp_PositionSolve = 1;
static const uint EContactSweepOp_Resolve = 2;

// Apply an operation to every occupied colour from first_colour in one synchronised group.
void ContactColourSweep(uint op, uint first_colour, uint thread, uint thread_count);

// Tail dispatches and overflow colour zero use one group rather than one parallel colour.
bool ColourWalkDispatch()
{
	return g.colour >= MaxColours || (g.colour == 0 && ColourOverflow());
}

// Decode the first colour covered by a group-wide walk.
uint ColourWalkFirst()
{
	return g.colour >= MaxColours ? (uint)(g.colour - MaxColours) : 0u;
}

// ----- CSApplyWarmStart -----
// Applies previous-frame physical impulses through race-free colour batches. A colour-overflow frame is handled serially by the first thread of colour zero so
// every enabled contact participates without requiring another dispatch or a CPU readback.
numthreads(CSApplyWarmStart, ResolveThreadCount, 1, 1)
void CSApplyWarmStart(int3 DTID(dtid))
{
	if (ColourWalkDispatch())
	{
		// The walk reuses priority order for overflow and keeps normal colour dependencies intact.
		ContactColourSweep(EContactSweepOp_ApplyWarmStart, ColourWalkFirst(), (uint)dtid.x, ResolveThreadCount);
		return;
	}

	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	ApplyWarmStartContact(idx);
}

// ----- CSStoreWarmStart -----
// Stores each contact's accumulated physical impulse for use as next frame's warm-start seed.
numthreads(CSStoreWarmStart, ResolveThreadCount, 1, 1)
void CSStoreWarmStart(int3 DTID(dtid))
{
	if (dtid.x >= ContactCount())
		return;

	StoreWarmStartImpulse(g_contacts[dtid.x]);
}

// ----- CSPositionSolve -----
// Dispatched once per colour from the CPU loop. Each thread updates one contact's detached state or body transforms.
// Contacts in other colours return immediately; the CPU changes g.colour and dispatches this entry point repeatedly.
numthreads(CSPositionSolve, ResolveThreadCount, 1, 1)
void CSPositionSolve(int3 DTID(dtid))
{
	if (ColourWalkDispatch())
	{
		// The complete sorted overflow sweep keeps position-state writes coherent.
		ContactColourSweep(EContactSweepOp_PositionSolve, ColourWalkFirst(), (uint)dtid.x, ResolveThreadCount);
		return;
	}

	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	// Direct correction is divided across sweeps; detached correction instead solves the remaining error against a fixed total target.
	ApplyPositionCorrection(g_contacts[idx]);
}

// Resolve one contact against the latest body momenta. Callers provide either a race-free colour batch or exclusive serial execution.
//
// The contact solve runs in four phases:
//
//   Phase 1 (centroid): Apply a coupled restitution+friction impulse at the contact centroid
//   using the classic 3D Coulomb-cone-clamped formula. This handles dynamic collisions
//   (elastic bounces, head-on impacts) and matches the analytic 1D elastic solution for
//   symmetric pair-wise contacts (preserving the conservation tests).
//
//   Phase 2 (linear bias, only when penetrating and the centre of mass is not closing): Apply a scalar Baumgarte bias impulse to linear momentum only.
//   Phase 3 (per-manifold-point friction): Project accumulated tangent impulses against physical normal support, including warm starting.
//   Broad face contacts can resist sliding and torsion without bias-driven angular runaway.
//   Phase 4 (rolling and spin resistance): Clamp an accumulated pure angular impulse against the material rolling and spin resistance limits.
void ResolveContact(uint idx)
{
	// Load the contact and both bodies. Updates are written back only if an impulse is applied, which avoids unnecessary UAV traffic for separating contacts.
	GpuResolveContact c = g_contacts[idx];
	GpuRigidBody bodyA = g_bodies[c.body_idx_a];
	GpuRigidBody bodyB = g_bodies[c.body_idx_b];

	// Rewind only the solver transform to the estimated collision time. Keep the stored transform in the same generating frame as the axis and manifold.
	float4x4 solver_b2a = c.b2a;
	float ct = c.collision_time;
	if (abs(ct) > 1e-6f)
	{
		float4x4 o2w_a_at_t = ExtrapolateO2W(bodyA, ct);
		float4x4 o2w_b_at_t = ExtrapolateO2W(bodyB, ct);
		solver_b2a = mul(o2w_b_at_t, InvertOrthonormal(o2w_a_at_t));
	}
	float3 axis = c.axis.xyz;

	// From this point down, contact-space quantities are expressed in body A space unless explicitly suffixed with _ws. Body B's CoM and inverse inertia are
	// transformed into A space so the impulse equations can use one coordinate frame.
	float inv_mass_a = bodyA.os_com_and_invmass.w;
	float inv_mass_b = bodyB.os_com_and_invmass.w;
	float3 com_a_in_a = bodyA.os_com_and_invmass.xyz;
	float3 os_com_b = bodyB.os_com_and_invmass.xyz;
	float3x3 rot_a = (float3x3)bodyA.o2w;
	float3x3 b2a_rot = (float3x3)solver_b2a;
	float3 com_b_in_a = solver_b2a[3].xyz + mul(os_com_b, b2a_rot);

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
	float friction = FrictionCoefficient(sqrt(mat_a.friction_static * mat_b.friction_static));

	// Baumgarte velocity bias: the per-frame separation velocity we'd like to inject
	// to drive penetration to zero. The same depth value is used for every manifold
	// point because the contact carries a single penetration depth covering the manifold.
	bool support_contact = SupportContact(c, bodyA);
	float bias = g.bias_scale * (g.velocity_baumgarte / g.dt) *
		max(c.depth - ContactSlop(g.penetration_slop, bodyA, bodyB, support_contact), 0.0f);

	// Number of manifold points to process (1 = no manifold, fall back to centroid).
	// Clamped to GpuContactMaxPoints to keep the [unroll] bounds well-defined.
	int point_count = clamp(max(1, c.feature), 1, GpuContactMaxPoints);

	// Track whether any impulse was applied so we only update support contacts and
	// the Collided flag when the contact was actually doing work.
	bool any_impulse = false;

	// ===== Phase 1: Centroid normal impulse (energy-guarded) =====
	// This is the dynamic-collision part. Using the centroid (rather than per-point) for
	// the restitution impulse is critical for symmetric pair-wise elastic collisions to
	// match the analytic v' = ((m1-m2)v1 + 2 m2 v2) / (m1+m2) result, because per-point
	// sequential impulses systematically underestimate the total impulse needed.
	// Friction is left to Phase 3 so that all tangent impulse is limited by the accumulated normal impulse of this solve.
	{
		float3 pt = c.contact_point.xyz;
		float3 V_rel = RelativeVelocityAtPoint(bodyA, bodyB, pt, rot_a, com_a_in_a, com_b_in_a);

		// Adjust the contact point to the estimated collision time along the normal only.
		// Tangential components in V_rel would shift the point sideways, creating a
		// spurious lever arm for the normal impulse.
		if (abs(ct) > 1e-6f)
			pt += (0.5f * ct * dot(V_rel, axis)) * axis;

		float closing_speed = dot(V_rel, axis);

		// Restitution targets a separation speed derived from the approach speed captured before the velocity iterations (see CSComputeCollisionTimes).
		// Every iteration then solves toward the same target, so the response does not grow with the iteration count when other constraints re-create
		// closing speed between iterations. Slow approaches get reduced elasticity so resting contacts do not bounce on micro-impacts.
		float approach_speed = min(c.approach_speed, 0.0f);
		float rest_factor = saturate(abs(approach_speed) * 10.0f);
		float elasticity = g.restitution_scale * rest_factor * (mat_a.elasticity_norm + mat_b.elasticity_norm) * 0.5f;
		float target_normal_speed = -elasticity * approach_speed;

		// Normal speed change per unit normal impulse at the contact point (V_rel = v_b - v_a, axis A→B, so closing means negative normal speed).
		float k_n = dot(axis, mul(Invert(CollisionMassMatrix(
			pt - com_a_in_a, pt - com_b_in_a,
			inv_mass_a, inv_mass_b,
			os_iinv_a, b2a_iinv_b)), axis));

		// The accumulated normal impulse is the contact's support for this solve, so it must be able to shrink as well as grow. Push when the normal speed is
		// below the restitution target. Other constraints (for example a motor pushing a body into a slope) can also create closing speed that they relax later
		// in the same solve; keeping that excess would push the bodies apart and inflate the friction budget that Phase 3 derives from it. So when the speed is
		// above the larger of the restitution and Baumgarte targets, withdraw impulse, but never so much that the accumulated impulse becomes a pull.
		float Jn_accumulated = dot(c.warmstart_impulse.xyz, axis);
		float release_speed = max(target_normal_speed, bias);
		float Jn_delta = 0.0f;
		if (k_n > 1e-12f && closing_speed < target_normal_speed)
		{
			Jn_delta = (target_normal_speed - closing_speed) / k_n;
		}
		else if (k_n > 1e-12f && closing_speed > release_speed && Jn_accumulated > 0.0f)
		{
			Jn_delta = max((release_speed - closing_speed) / k_n, -Jn_accumulated);

			// Phase 2 keeps the centre-of-mass normal speed at or above the Baumgarte bias using linear momentum only. Releasing below that floor would
			// let Phase 2 re-add the linear part every iteration while the released torque is lost, steadily converting spin into linear motion.
			float k_n_c = inv_mass_a + inv_mass_b;
			if (bias > 0.0f && k_n_c > 1e-12f)
			{
				float3 v_com_a = mul(rot_a, BodyTwistWS(bodyA).lin);
				float3 v_com_b = mul(rot_a, BodyTwistWS(bodyB).lin);
				float vn_com = dot(v_com_b - v_com_a, axis);
				Jn_delta = max(Jn_delta, -max(vn_com - bias, 0.0f) / k_n_c);
			}
		}

		// Apply the normal change and keep the accumulator equal to what the energy guard accepted.
		PR_HLSL_BRANCH
		if (Jn_delta != 0.0f)
		{
			float3 applied_impulse = ApplyImpulseWithEnergyGuard(bodyA, bodyB, Jn_delta * axis, pt,
				com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
				inv_mass_a, inv_mass_b, true);
			c.warmstart_impulse.xyz += applied_impulse;
			any_impulse = any(applied_impulse != float3(0, 0, 0));
		}
	}
	// ===== Phase 2: Linear Baumgarte bias =====
	// Baumgarte bias is a pseudo-impulse that clears position error. Applying it through the current contact point makes broad resting contacts sensitive to
	// transient manifold reduction: a one-point contact patch can inject spin before the next frame restores the wider manifold. Use the centre-of-mass
	// relative normal speed and change only linear momentum, leaving physical contact-point torque to the restitution/friction phases.
	PR_HLSL_BRANCH
	if (bias > 0.0f)
	{
		// Add only separation speed beyond a non-closing centre of mass. While the centre of mass still closes, Phase 1 of this and the other contacts must
		// supply that support. A one-point contact away from the centre of mass (for example one corner of a box) stops its point mostly by spin, so a bias
		// impulse that lifted the centre of mass from a closing speed would carry the body's weight in place of the physical impulse. Phase 1 never releases
		// below this floor, so that bias would then stay as the support, and friction, which is limited by the physical normal impulse alone, would starve.
		float k_n_c = inv_mass_a + inv_mass_b;
		float3 v_com_a = mul(rot_a, BodyTwistWS(bodyA).lin);
		float3 v_com_b = mul(rot_a, BodyTwistWS(bodyB).lin);
		float vn_now = dot(v_com_b - v_com_a, axis);
		float Jn_bias_centroid = (k_n_c > 1e-12f && vn_now >= 0.0f) ? max(0.0f, (bias - vn_now) / k_n_c) : 0.0f;

		PR_HLSL_BRANCH
		if (Jn_bias_centroid > 0.0f)
		{
			// ----- Linear Baumgarte bias impulse (NOT energy-guarded) -----
			// This is a pseudo-impulse for clearing positional error, not a physical contact force. Restricting it to linear momentum avoids injecting angular
			// jostle when a resting broad contact temporarily reduces to a single manifold point.
			float3 applied_bias_impulse = ApplyLinearBiasImpulse(bodyA, bodyB, Jn_bias_centroid * axis, rot_a);

			// Baumgarte bias is a one-frame positional correction, not a reusable support impulse. Persisting it into the warm-start cache replays stale
			// push-out on the next frame and can keep resting contact networks subtly energised. It still changes momentum, so record it for contact reports.
			c.bias_impulse += dot(applied_bias_impulse, axis);
			any_impulse = any_impulse || any(applied_bias_impulse != float3(0, 0, 0));
		}
	}

	// ===== Phase 3: Accumulated per-manifold-point friction =====
	// Warm starting can satisfy normal velocity before this sweep. Its accepted physical impulse still supplies friction, independently of fresh closing or bias.
	// Sharing the total normal budget across the face preserves torsional resistance without granting another full Coulomb allowance each iteration.
	float Jn_per_point = max(dot(c.warmstart_impulse.xyz, axis), 0.0f) / float(point_count);
	PR_HLSL_BRANCH
	if (Jn_per_point > 0.0f && friction > 0.0f)
	{
		PR_HLSL_UNROLL
		for (int pi = 0; pi != GpuContactMaxPoints; ++pi)
		{
			if (pi >= point_count)
				continue;

			// Re-read velocities after earlier points so the bounded deltas converge without replaying already applied friction.
			float3 pt = (point_count > 1) ? c.manifold[pi].xyz : c.contact_point.xyz;
			float3 V_rel = RelativeVelocityAtPoint(bodyA, bodyB, pt, rot_a, com_a_in_a, com_b_in_a);
			float3x3 col_I_pt = CollisionMassMatrix(
				pt - com_a_in_a, pt - com_b_in_a,
				inv_mass_a, inv_mass_b,
				os_iinv_a, b2a_iinv_b);
			float3 friction_impulse = ComputeFrictionImpulse(Invert(col_I_pt), V_rel, axis, friction, Jn_per_point, c.friction_impulses[pi].xyz);
			if (any(friction_impulse != float3(0, 0, 0)))
			{
				// Retain only the passive delta that was actually applied, keeping the solver accumulator consistent with body momentum.
				float3 applied_friction_impulse = ApplyImpulseWithEnergyGuard(bodyA, bodyB, friction_impulse, pt,
					com_a_in_a, com_b_in_a, rot_a, ws_iinv_a, ws_iinv_b,
					inv_mass_a, inv_mass_b, true);
				c.friction_impulses[pi].xyz += applied_friction_impulse;
				c.warmstart_impulse.xyz += applied_friction_impulse;
				any_impulse = any_impulse || any(applied_friction_impulse != float3(0, 0, 0));
			}
		}
	}

	// ===== Phase 4: Rolling and spin resistance =====
	// A round body touches at a single point, so the point friction above cannot stop it rolling or spinning. Apply an accumulated pure angular impulse
	// that opposes the relative angular velocity. The rolling and spin parts are limited to 'coefficient * Jn * r', where Jn is this solve's physical normal
	// impulse and r is the larger dynamic-body distance from the contact point to the centre of mass along the normal. For a ball, r is its radius, so the
	// coefficient behaves like a rolling friction coefficient: a round body rests on a slope whose tangent is less than the coefficient.
	float rolling_resistance = max(mat_a.rolling_resistance, mat_b.rolling_resistance);
	float spin_resistance = max(mat_a.spin_resistance, mat_b.spin_resistance);
	float Jn_total = max(dot(c.warmstart_impulse.xyz, axis), 0.0f);
	PR_HLSL_BRANCH
	if ((rolling_resistance > 0.0f || spin_resistance > 0.0f) && Jn_total > 0.0f)
	{
		// Static and kinematic bodies have no lever arm of their own; their motion is still included through the relative angular velocity
		float3 pt = c.contact_point.xyz;
		float r_a = inv_mass_a > 0.0f ? abs(dot(pt - com_a_in_a, axis)) : 0.0f;
		float r_b = inv_mass_b > 0.0f ? abs(dot(pt - com_b_in_a, axis)) : 0.0f;
		float r = max(r_a, r_b);

		// Relative angular velocity of B with respect to A, and the angular response of both bodies, in A space
		float3 w_rel = mul(rot_a, BodyTwistWS(bodyB).ang - BodyTwistWS(bodyA).ang);
		float3x3 K = os_iinv_a + b2a_iinv_b;
		float3 angular_impulse = ComputeRollingResistanceImpulse(K, w_rel, axis, rolling_resistance * Jn_total * r, spin_resistance * Jn_total * r, c.angular_impulse.xyz);
		if (any(angular_impulse != float3(0, 0, 0)))
		{
			// Keep the accumulator equal to what the energy guard accepted
			float3 applied_angular_impulse = ApplyAngularImpulseWithEnergyGuard(bodyA, bodyB, angular_impulse, rot_a, ws_iinv_a, ws_iinv_b);
			c.angular_impulse.xyz += applied_angular_impulse;
			any_impulse = any_impulse || any(applied_angular_impulse != float3(0, 0, 0));
		}
	}

	// If no point applied any impulse, the contact was fully separating and there's nothing
	// to write back. This avoids touching body memory for trivially separating contacts.
	if (!any_impulse)
		return;

	// Mark both bodies as having taken part in a collision this frame.
	bodyA.state_flags = SetFlag(bodyA.state_flags, ERigidBodyStateFlags_Collided, true);
	bodyB.state_flags = SetFlag(bodyB.state_flags, ERigidBodyStateFlags_Collided, true);

	// Leave sleep state to CSReduceSleepStats after the complete solve.
	g_contacts[idx] = c;
	g_bodies[c.body_idx_a] = bodyA;
	g_bodies[c.body_idx_b] = bodyB;
}

// ----- CSResolve -----
// Dispatched once per colour from the CPU loop. Normal frames process one independent contact per thread. A colour-overflow frame uses thread zero of colour
// zero to process the complete sorted set serially, preserving correctness without adding normal-frame dispatches.
numthreads(CSResolve, ResolveThreadCount, 1, 1)
void CSResolve(int3 DTID(dtid))
{
	if (ColourWalkDispatch())
	{
		// Immediate serial overflow updates preserve the sequential-impulse dependency order.
		ContactColourSweep(EContactSweepOp_Resolve, ColourWalkFirst(), (uint)dtid.x, ResolveThreadCount);
		return;
	}

	if (dtid.x >= ContactCount())
		return;

	uint idx = g_contact_order[dtid.x];
	if (g_colours[idx] != (uint)g.colour)
		return;

	ResolveContact(idx);
}

// Apply one solver operation to one contact.
void ApplyContactSweepOp(uint op, uint idx)
{
	// 'op' is a compile-time constant in each entry point, so this switch is removed after inlining.
	switch (op)
	{
		case EContactSweepOp_ApplyWarmStart: { ApplyWarmStartContact(idx); break; }
		case EContactSweepOp_PositionSolve: { ApplyPositionCorrection(g_contacts[idx]); break; }
		case EContactSweepOp_Resolve: { ResolveContact(idx); break; }
	}
}

// Walk occupied colours in ascending order with a device-memory group barrier between them.
// Every lane of the single group must call this function. Overflow belongs only to the walk that starts at colour zero.
void ContactColourSweep(uint op, uint first_colour, uint thread, uint thread_count)
{
	// The overflow flag is the same for every thread, so the serial fallback keeps the whole group on one path. It matches the per-colour colour-zero fallback.
	if (ColourOverflow())
	{
		if (first_colour != 0 || thread != 0)
			return;

		for (int contact_order_idx = 0; contact_order_idx != ContactCount(); ++contact_order_idx)
		{
			uint idx = g_contact_order[contact_order_idx];
			if (ContactEnabledAt(idx))
				ApplyContactSweepOp(op, idx);
		}
		return;
	}

	// Serial replay supports the overflow branch above, but cannot simulate a synchronised parallel group.
#ifdef __cplusplus
	(void)thread_count;
	throw std::runtime_error("Contact colour tails require native thread-group execution");
#else
	// A stale CPU head is only a scheduling estimate. The current mask includes all newly occupied colours beyond it.
	int contact_count = ContactCount();
	// Process the colours in ascending order, as the per-colour dispatches do. 'used' is the same in every thread, so every thread reaches each barrier.
	uint used = g_colours[OccupiedColoursIndex()] & (first_colour < (uint)MaxColours ? (~0u << first_colour) : 0u);
	while (used != 0)
	{
		// Contacts within one colour never share a dynamic body, so threads can process them in any order.
		uint colour = firstbitlow(used);
		used &= used - 1;
		for (int j = (int)thread; j < contact_count; j += (int)thread_count)
		{
			uint idx = g_contact_order[j];
			if (g_colours[idx] == colour)
				ApplyContactSweepOp(op, idx);
		}
		DeviceMemoryBarrierWithGroupSync();
	}
#endif
}

// ----- Fused colour sweeps -----
// Small contact sets use one group for every colour. UAV writes become visible to the next colour at the device-memory group barrier.
// C++ replay cannot synchronise lanes; native group execution validates these entry points and nonempty parallel tails.
#ifndef __cplusplus
static const int FusedThreadCount = 256;

// ----- CSApplyWarmStartFused -----
// Single-group form of CSApplyWarmStart. Dispatch exactly one group.
numthreads(CSApplyWarmStartFused, FusedThreadCount, 1, 1)
void CSApplyWarmStartFused(int3 GTID(gtid))
{
	ContactColourSweep(EContactSweepOp_ApplyWarmStart, 0, (uint)gtid.x, FusedThreadCount);
}

// ----- CSPositionSolveFused -----
// Single-group form of CSPositionSolve. Dispatch exactly one group.
numthreads(CSPositionSolveFused, FusedThreadCount, 1, 1)
void CSPositionSolveFused(int3 GTID(gtid))
{
	ContactColourSweep(EContactSweepOp_PositionSolve, 0, (uint)gtid.x, FusedThreadCount);
}

// ----- CSResolveFused -----
// Single-group form of CSResolve. Dispatch exactly one group.
numthreads(CSResolveFused, FusedThreadCount, 1, 1)
void CSResolveFused(int3 GTID(gtid))
{
	ContactColourSweep(EContactSweepOp_Resolve, 0, (uint)gtid.x, FusedThreadCount);
}

#endif

#ifdef __cplusplus
}
#endif
