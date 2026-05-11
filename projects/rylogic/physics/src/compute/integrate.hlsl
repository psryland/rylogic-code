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
//   u2: RWStructuredBuffer<int>               — per-body encoded BodyIndex (write)
//   u3: RWStructuredBuffer<float>             — expanded sort-axis AABB value (write)
//   u4: RWStructuredBuffer<BBox>              — per-body exact world-space bounding box (write)
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
#include "pr/hlsl/bounding_box.hlsli"
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
	float broadphase_aabb_margin;
	float sleep_velocity_threshold_lin;
	float sleep_velocity_threshold_ang;
	int broadphase_sort_axis;
	float pad2;
};

// Shader resources
ConstantBuffer<cbIntegrate> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u1);
RWStructuredBuffer<int> resource(g_aabb_idx, u2);
RWStructuredBuffer<float> resource(g_aabb_sort, u3);
RWStructuredBuffer<BBox> resource(g_aabb_box, u4);

static const int AngularDriftSubstepMax = 32;
static const float AngularDriftMaxRadians = 0.25f;
static const int AngularDriftIterationCount = 8;

// Build an axis-angle vector for a principal-axis rotation.
float3 PrincipalAxisAngle(int axis, float angle)
{
	switch (axis)
	{
		case 0:
		{
			return float3(angle, 0, 0);
		}
		case 1:
		{
			return float3(0, angle, 0);
		}
		case 2:
		{
			return float3(0, 0, angle);
		}
		default:
		{
			return float3(0, 0, 0);
		}
	}
}

// Return the component of 'vec' selected by a principal-axis index.
float PrincipalAxisComponent(float3 vec, int axis)
{
	switch (axis)
	{
		case 0:
		{
			return vec.x;
		}
		case 1:
		{
			return vec.y;
		}
		case 2:
		{
			return vec.z;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Apply the exact flow for one diagonal inertia principal-axis Hamiltonian term.
void SymplecticAxisDrift(inout float3x3 rot, inout float3 momentum_os, float3 inertia_inv_diagonal, float inv_mass, int axis, float elapsed_seconds)
{
	float omega = inv_mass * PrincipalAxisComponent(inertia_inv_diagonal, axis) * PrincipalAxisComponent(momentum_os, axis);
	float3 axis_angle = PrincipalAxisAngle(axis, omega * elapsed_seconds);
	float3x3 axis_rot = rodrigues_rotation(axis_angle);
	float3x3 axis_inv = rodrigues_rotation(-axis_angle);
	rot = mul(axis_rot, rot);
	momentum_os = mul(momentum_os, axis_inv);
}

// Integrate a torque-free diagonal-inertia angular drift using symmetric principal-axis splitting.
void SymplecticAngularDrift(inout float3x3 rot, float3 momentum_ang, float3 inertia_inv_diagonal, float inv_mass, float elapsed_seconds, int angular_steps)
{
	float3 momentum_os = mul(rot, momentum_ang);
	float angular_dt = elapsed_seconds / (float)angular_steps;
	for (int angular_step = 0; angular_step != angular_steps; ++angular_step)
	{
		// Strang-split the free-rigid-body Hamiltonian into exact principal-axis flows. Each axis flow rotates the orientation in the body frame
		// and counter-rotates body-space angular momentum, preserving the fixed world angular momentum without solving an implicit midpoint.
		SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 0, angular_dt * 0.5f);
		SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 1, angular_dt * 0.5f);
		SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 2, angular_dt);
		SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 1, angular_dt * 0.5f);
		SymplecticAxisDrift(rot, momentum_os, inertia_inv_diagonal, inv_mass, 0, angular_dt * 0.5f);

		rot = orthonorm3x3(rot);
		if (angular_step + 1 != angular_steps)
			momentum_os = mul(rot, momentum_ang);
	}
}

// True when the body's current momentum represents linear and angular velocities below the configured sleep thresholds.
bool LowVelocity(GpuRigidBody body, float inv_mass)
{
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;
	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);

	return dot(vel_lin, vel_lin) < sqr(g.sleep_velocity_threshold_lin) &&
		dot(vel_ang, vel_ang) < sqr(g.sleep_velocity_threshold_ang);
}

// Compute the world-space AABB for a body and write it to the output buffers.
odr void UpdateAABB(in_(GpuRigidBody) body, int idx)
{
	BBox ws_bbox = BBox_Transform(body.os_bbox, body.o2w);
	float3 ws_centre = ws_bbox.centre.xyz;
	float3 ws_radius = ws_bbox.radius.xyz;
	float margin = max(g.broadphase_aabb_margin, 0.0f);
	float sort_centre = ws_centre.x;
	float sort_radius = ws_radius.x;
	switch (g.broadphase_sort_axis)
	{
		case 1:
		{
			sort_centre = ws_centre.y;
			sort_radius = ws_radius.y;
			break;
		}
		case 2:
		{
			sort_centre = ws_centre.z;
			sort_radius = ws_radius.z;
			break;
		}
		default:
		{
			break;
		}
	}

	// Write exact bounds for final broadphase filtering and readback, plus a conservative
	// expanded sort-axis interval so just-touching chains are considered candidates.
	g_aabb_box[idx] = ws_bbox;
	g_aabb_sort[2 * idx + 0] = sort_centre - sort_radius - margin;
	g_aabb_sort[2 * idx + 1] = sort_centre + sort_radius + margin;
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
	bool low_velocity = inv_mass > 0.0f && LowVelocity(body, inv_mass);

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
	float3 inertia_diag = body.inertia_inv_diagonal.xyz;
	float3 inertia_prod = body.inertia_inv_products.xyz;
	bool isotropic_inertia =
		inertia_prod.x == 0.0f &&
		inertia_prod.y == 0.0f &&
		inertia_prod.z == 0.0f &&
		inertia_diag.x == inertia_diag.y &&
		inertia_diag.x == inertia_diag.z;
	bool diagonal_inertia =
		inertia_prod.x == 0.0f &&
		inertia_prod.y == 0.0f &&
		inertia_prod.z == 0.0f;

	// Build the object-space unit inverse inertia (not mass-scaled).
	float3x3 os_iinv_unit = build_symmetric_3x3(
		inertia_diag,
		inertia_prod);

	// Extract the 3x3 rotation from the transform (rows = basis vectors in row-vector convention)
	float3x3 rot = (float3x3)body.o2w;

	// Compute velocity from momentum (block-diagonal — no coupling terms).
	// omega = Ic_inv * h_ang, v_com = h_lin / m.
	float3 vel_ang;
	if (isotropic_inertia)
	{
		vel_ang = (inv_mass * inertia_diag.x) * body.momentum_ang.xyz;
	}
	else
	{
		float3x3 ws_iinv_unit = rotate_inertia_inv(os_iinv_unit, rot);
		float3x3 ws_iinv = inv_mass * ws_iinv_unit;
		vel_ang = mul(ws_iinv, body.momentum_ang.xyz);
	}
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;

	float3x3 new_rot = rot;
	float angular_speed_sq = dot(vel_ang, vel_ang);
	if (angular_speed_sq != 0.0f)
	{
		if (isotropic_inertia)
		{
			float3x3 dR = rodrigues_rotation(vel_ang * g.dt);
			new_rot = mul(new_rot, dR);
			new_rot = orthonorm3x3(new_rot);
		}
		else if (diagonal_inertia)
		{
			int angular_steps = clamp((int)ceil(sqrt(angular_speed_sq) * g.dt / AngularDriftMaxRadians), 1, AngularDriftSubstepMax);
			SymplecticAngularDrift(new_rot, body.momentum_ang.xyz, inertia_diag, inv_mass, g.dt, angular_steps);
		}
		else
		{
			// Solve the midpoint orientation implicitly rather than using a one-shot predictor. The drift is torque-free, so world angular momentum
			// remains fixed while the orientation-dependent inertia determines the midpoint angular velocity.
			int angular_steps = clamp((int)ceil(sqrt(angular_speed_sq) * g.dt / AngularDriftMaxRadians), 1, AngularDriftSubstepMax);
			float angular_dt = g.dt / (float)angular_steps;
			float3 step_vel_ang = vel_ang;
			for (int angular_step = 0; angular_step != angular_steps; ++angular_step)
			{
				float3 mid_vel_ang = step_vel_ang;
				float3x3 mid_rot = mul(new_rot, rodrigues_rotation(mid_vel_ang * (angular_dt * 0.5f)));
				for (int iteration = 0; iteration != AngularDriftIterationCount; ++iteration)
				{
					float3x3 mid_iinv_unit = rotate_inertia_inv(os_iinv_unit, mid_rot);
					float3x3 mid_iinv = inv_mass * mid_iinv_unit;
					mid_vel_ang = mul(mid_iinv, body.momentum_ang.xyz);
					mid_rot = mul(new_rot, rodrigues_rotation(mid_vel_ang * (angular_dt * 0.5f)));
				}

				float3x3 dR = rodrigues_rotation(mid_vel_ang * angular_dt);
				new_rot = mul(new_rot, dR);
				if (angular_step + 1 != angular_steps)
				{
					float3x3 step_iinv_unit = rotate_inertia_inv(os_iinv_unit, new_rot);
					float3x3 step_iinv = inv_mass * step_iinv_unit;
					step_vel_ang = mul(step_iinv, body.momentum_ang.xyz);
				}
			}
			new_rot = orthonorm3x3(new_rot);
		}
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
