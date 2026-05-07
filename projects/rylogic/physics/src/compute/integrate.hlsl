//*********************************************
// Physics Engine — GPU Integration Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// Störmer-Verlet (kick-drift-kick) symplectic integrator running on the GPU.
// Each thread processes one rigid body. The algorithm is identical to the CPU
// Evolve() function in integrator.cpp — see that file for detailed comments.
//
// Buffer layout:
//   u0: RWStructuredBuffer<GpuCollisionCounters> - shared step counters
//   u1: RWStructuredBuffer<GpuRigidBody>      — per-body dynamic state (read/write)
//   u2: RWStructuredBuffer<float>             — per-body world-space AABB X value (write)
//   u3: RWStructuredBuffer<float>             — per-body world-space AABB Y value (write)
//   u4: RWStructuredBuffer<float>             — per-body world-space AABB Z value (write)
//   u5: RWStructuredBuffer<int>               — per-body encoded BodyIndex (write)
//   b0: cbuffer with time step and body count
//
// Matrix convention (row-vector / DirectX-style):
//   C++ m4x4 stores columns contiguously as x, y, z, w members.
//   HLSL 'row_major float4x4' maps each row to one C++ column.
//   For vector transforms: mul(v, M) applies the rotation/transform.
//   For matrix compose:    mul(A, B) composes A then B (C++ equivalent: B_cpp * A_cpp).
//   See spatial_algebra.hlsli for full convention documentation.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Integration parameters
struct cbIntegrate
{
	float dt;
	int body_count;
	int sleeping_enabled;
	float pad0;
	float sleep_velocity_threshold_lin;
	float sleep_velocity_threshold_ang;
	float pad1;
	float pad2;
};

// Shader resources
ConstantBuffer<cbIntegrate> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u1);
RWStructuredBuffer<float> resource(g_aabb_x, u2);
RWStructuredBuffer<float> resource(g_aabb_y, u3);
RWStructuredBuffer<float> resource(g_aabb_z, u4);
RWStructuredBuffer<int> resource(g_aabb_idx, u5);

static const int AngularDriftSubstepMax = 32;
static const float AngularDriftMaxRadians = 0.25f;

// Compute the world-space AABB for a body and write it to the output buffers.
odr void UpdateAABB(in_(GpuRigidBody) body, int idx)
{
	float3x3 rot = (float3x3)body.o2w;
	float3 os_centre = body.os_bbox.centre.xyz;
	float3 os_radius = body.os_bbox.radius.xyz;
	float3 ws_centre = mul(float4(os_centre, 1), body.o2w).xyz;
	float3 ws_radius = float3(
		abs(rot[0].x) * os_radius.x + abs(rot[1].x) * os_radius.y + abs(rot[2].x) * os_radius.z,
		abs(rot[0].y) * os_radius.x + abs(rot[1].y) * os_radius.y + abs(rot[2].y) * os_radius.z,
		abs(rot[0].z) * os_radius.x + abs(rot[1].z) * os_radius.y + abs(rot[2].z) * os_radius.z);

	// Write the aabb min/max values
	g_aabb_x[2 * idx + 0] = ws_centre.x - ws_radius.x;
	g_aabb_x[2 * idx + 1] = ws_centre.x + ws_radius.x;
	g_aabb_y[2 * idx + 0] = ws_centre.y - ws_radius.y;
	g_aabb_y[2 * idx + 1] = ws_centre.y + ws_radius.y;
	g_aabb_z[2 * idx + 0] = ws_centre.z - ws_radius.z;
	g_aabb_z[2 * idx + 1] = ws_centre.z + ws_radius.z;
	g_aabb_idx[2 * idx + 0] = (idx << 1) | 0;
	g_aabb_idx[2 * idx + 1] = (idx << 1) | 1;
}

numthreads(CSIntegrate, IntegrateThreadCount, 1, 1)
void CSIntegrate(int3 DTID(dtid))
{
	int idx = dtid.x;
	if (idx >= g.body_count)
		return;

	// Load the body's dynamic state
	GpuRigidBody body = g_bodies[idx];
	float half_dt = g.dt * 0.5f;
	float inv_mass = body.os_com_and_invmass.w;
	float3 os_com = body.os_com_and_invmass.xyz;

	// ---- Step 1: Half-kick — advance momentum by half the force impulse ----
	body.momentum_ang += body.force_ang * half_dt;
	body.momentum_lin += body.force_lin * half_dt;

	// ---- Sleep check: if body is sleeping and momentum is below thresholds, skip dynamics ----
	bool low_velocity =
		dot(body.momentum_lin.xyz, body.momentum_lin.xyz) < sqr(g.sleep_velocity_threshold_lin) / (sqr(inv_mass) + 1e-30f) &&
		dot(body.momentum_ang.xyz, body.momentum_ang.xyz) < sqr(g.sleep_velocity_threshold_ang) / (sqr(inv_mass) + 1e-30f);

	bool force_awake = !g.sleeping_enabled || AllSet(body.state_flags, ERigidBodyStateFlags_NeverSleep);
	if (force_awake)
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);

	bool stay_asleep = !force_awake && AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) && low_velocity;
	if (stay_asleep)
	{
		// Sleeping body: zero momentum and forces, keep position unchanged.
		body.momentum_ang = float4(0, 0, 0, 0);
		body.momentum_lin = float4(0, 0, 0, 0);
		body.force_ang = float4(0, 0, 0, 0);
		body.force_lin = float4(0, 0, 0, 0);
		g_bodies[idx] = body;

		// Still need to output the AABBs so the broadphase can detect collisions that wake us
		UpdateAABB(body, idx);
		return;
	}
	if (AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping))
	{
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.sleep.timer_s = 0.0f;
		body.sleep.island_id = -1;
		body.sleep.generation++;
		body.sleep.flags = 0;
	}

	// ---- Step 2: Drift — update position and orientation ----
	// Build the object-space unit inverse inertia (not mass-scaled)
	float3x3 os_iinv_unit = build_symmetric_3x3(
		body.inertia_inv_diagonal.xyz,
		body.inertia_inv_products.xyz);

	// Extract the 3x3 rotation from the transform (rows = basis vectors in row-vector convention)
	float3x3 rot = (float3x3)body.o2w;

	// Rotate the inverse inertia from object space to world space:
	//   I⁻¹_ws = R * I⁻¹_os * Rᵀ
	float3x3 ws_iinv_unit = rotate_inertia_inv(os_iinv_unit, rot);

	// Mass-scaled world-space inverse inertia
	float3x3 ws_iinv = inv_mass * ws_iinv_unit;

	// Compute velocity from momentum (block-diagonal — no coupling terms).
	// omega = Ic_inv * h_ang, v_com = h_lin / m.
	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;

	// Midpoint predictor for the rotation step:
	// For anisotropic bodies, angular velocity changes during the drift step because the world-space inertia tensor changes with orientation. Large
	// angular displacements amplify this approximation error, so split the drift into small rotation increments while keeping the same angular momentum.
	int angular_steps = clamp((int)ceil(length(vel_ang) * g.dt / AngularDriftMaxRadians), 1, AngularDriftSubstepMax);
	float angular_dt = g.dt / (float)angular_steps;
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

	// CoM-based position update: translate CoM, derive model origin from new rotation.
	float3 com_ws = mul(os_com, rot);          // world-space CoM offset
	float3 com_pos = body.o2w[3].xyz + com_ws; // world-space CoM position
	float3 new_com_pos = com_pos + vel_lin * g.dt;
	float3 new_pos = new_com_pos - mul(os_com, new_rot);

	// Write back the updated transform
	body.o2w[0] = float4(new_rot[0], 0);
	body.o2w[1] = float4(new_rot[1], 0);
	body.o2w[2] = float4(new_rot[2], 0);
	body.o2w[3] = float4(new_pos, 1);

	// ---- Step 3: Half-kick — advance momentum by second half ----
	body.momentum_ang += body.force_ang * half_dt;
	body.momentum_lin += body.force_lin * half_dt;

	// Zero forces (they must be re-applied each frame)
	body.force_ang = float4(0, 0, 0, 0);
	body.force_lin = float4(0, 0, 0, 0);

	// Write results
	g_bodies[idx] = body;

	// Compute world-space AABB from object-space bbox and the updated transform.
	UpdateAABB(body, idx);
}

#ifdef __cplusplus
}
#endif
