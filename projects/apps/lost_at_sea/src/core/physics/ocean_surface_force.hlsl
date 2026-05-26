//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Placeholder flat-ocean contact force used to exercise the physics engine's external-force hook.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "physics/src/compute/physics_types.hlsli"

struct CBufOceanSurfaceForce
{
	float water_level;
	float spring_constant;
	float damping_constant;
	float max_force;
	int body_count;
	float3 pad;
};

ConstantBuffer<CBufOceanSurfaceForce> resource(g, b0);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u0);

// Apply an upward spring/damper force when a body's object-space bounds penetrate the flat ocean surface.
numthreads(CSOceanSurfaceForce, 64, 1, 1)
void CSOceanSurfaceForce(uint3 DTID(dispatch_thread_id))
{
	int body_index = int(dispatch_thread_id.x);
	if (body_index >= g.body_count)
	{
		return;
	}

	GpuRigidBody body = g_bodies[body_index];
	if ((body.state_flags & ERigidBodyStateFlags_Static) != 0 || body.os_com_and_invmass.w <= 0.0f)
	{
		return;
	}

	// Project the oriented object-space bounds onto world Z so tilted boxes still make contact at their lowest point.
	float3 bbox_radius = body.os_bbox.radius.xyz;
	float3 z_axis_projection = abs(float3(body.o2w[0].z, body.o2w[1].z, body.o2w[2].z));
	float bbox_vertical_radius = dot(z_axis_projection, bbox_radius);
	float bbox_centre_z = mul(float4(body.os_bbox.centre.xyz, 1.0f), body.o2w).z;
	float penetration = g.water_level - (bbox_centre_z - bbox_vertical_radius);
	if (penetration <= 0.0f)
	{
		return;
	}

	float vertical_velocity = body.momentum_lin.z * body.os_com_and_invmass.w;
	float force_z = clamp(penetration * g.spring_constant - vertical_velocity * g.damping_constant, 0.0f, g.max_force);
	g_bodies[body_index].force_lin.z += force_z;
}
