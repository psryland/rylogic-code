//*********************************************
// Physics Engine — GPU Sleep/Wake Compute Shader
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/bounding_box.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

struct cbSleep
{
	int body_count;
	int island_count;
	int sleeping_enabled;
	int pad0;
};

ConstantBuffer<cbSleep> resource(g, b0);
RWStructuredBuffer<GpuSleepIsland> resource(g_sleep_islands, u0);
RWStructuredBuffer<GpuRigidBody> resource(g_rw_bodies, u1);
StructuredBuffer<GpuRigidBody> resource(g_bodies, t0);

odr bool DynamicBody(in_(GpuRigidBody) body)
{
	return body.os_com_and_invmass.w > 0.0f &&
		!HasFlag(body.state_flags, ERigidBodyStateFlags_Static);
}

odr bool ActuallyAwake(in_(GpuRigidBody) body)
{
	return DynamicBody(body) &&
		!HasFlag(body.state_flags, ERigidBodyStateFlags_Sleeping);
}

numthreads(CSDisturbIslands, SleepThreadCount, 1, 1)
void CSDisturbIslands(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0 || g.island_count == 0)
		return;

	int pair_idx = dtid.x;
	int pair_count = g.body_count * g.island_count;
	if (pair_idx >= pair_count)
		return;

	int body_idx = pair_idx / g.island_count;
	int island_idx = pair_idx - body_idx * g.island_count;

	GpuRigidBody body = g_bodies[body_idx];
	if (!ActuallyAwake(body))
		return;

	GpuSleepIsland island = g_sleep_islands[island_idx];
	if ((island.flags & (GpuSleepIslandFlags_Valid | GpuSleepIslandFlags_Sleeping)) != (GpuSleepIslandFlags_Valid | GpuSleepIslandFlags_Sleeping))
		return;

	BBox body_bbox_ws = BBox_Transform(body.os_bbox, body.o2w);
	if (!BBox_IsIntersection(body_bbox_ws, island.bbox_ws))
		return;

	InterlockedOr(g_sleep_islands[island_idx].flags, GpuSleepIslandFlags_Disturbed);
}

numthreads(CSWakeCollidedBodies, SleepThreadCount, 1, 1)
void CSWakeCollidedBodies(int3 dtid : SV_DispatchThreadID)
{
	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	GpuRigidBody body = g_rw_bodies[body_idx];
	if (!HasFlag(body.state_flags, ERigidBodyStateFlags_Sleeping) ||
		!HasFlag(body.state_flags, ERigidBodyStateFlags_Collided))
		return;

	body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
	body.sleep.timer_s = 0.0f;
	body.sleep.island_id = -1;
	body.sleep.generation++;
	body.sleep.flags = 0;
	g_rw_bodies[body_idx] = body;
}

#ifdef __cplusplus
}
#endif
