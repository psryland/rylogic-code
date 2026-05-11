//*********************************************
// Physics Engine — GPU Sleep/Wake Compute Shader
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "pr/hlsl/bounding_box.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

struct cbSleep
{
	float dt;
	float sleep_velocity_threshold_lin;
	float sleep_velocity_threshold_ang;
	float sleep_delay_s;
	int body_count;
	int island_count;
	int max_contacts;
	int sleeping_enabled;
};

ConstantBuffer<cbSleep> resource(g, b0);
RWStructuredBuffer<GpuSleepIsland> resource(g_sleep_islands, u0);
RWStructuredBuffer<GpuRigidBody> resource(g_rw_bodies, u1);
RWStructuredBuffer<int> resource(g_sleep_parents, u2);
RWStructuredBuffer<GpuSleepIslandStats> resource(g_sleep_stats, u3);
StructuredBuffer<GpuRigidBody> resource(g_bodies, t0);
StructuredBuffer<GpuCollisionCounters> resource(g_counters, t1);
StructuredBuffer<GpuResolveContact> resource(g_contacts, t2);

// Sleeping is split into two GPU stages. 'CSDisturbIslands' runs before broadphase and conservatively marks existing sleeping islands whose bounding
// boxes overlap awake bodies. The update kernels run after narrowphase/resolve, using real contacts to build the next set of islands.
static const int SleepIslandNoId = 2147483647;

// True if 'body' is dynamic (ie can go to sleep) as opposed to static (never sleeps)
odr bool DynamicBody(in_(GpuRigidBody) body)
{
	return body.os_com_and_invmass.w > 0.0f &&
		!AllSet(body.state_flags, ERigidBodyStateFlags_Static);
}

// True if 'body' is awake (ie not sleeping)
odr bool ActuallyAwake(in_(GpuRigidBody) body)
{
	return DynamicBody(body) &&
		!AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
}

// True if 'body' is below the sleep velocity threshold and can potentially go to sleep
odr bool LowVelocity(in_(GpuRigidBody) body)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	float3 vel_lin = inv_mass * body.momentum_lin.xyz;
	float3 vel_ang = mul(ws_iinv, body.momentum_ang.xyz);

	return inv_mass > 0.0f &&
		dot(vel_lin, vel_lin) < sqr(g.sleep_velocity_threshold_lin) &&
		dot(vel_ang, vel_ang) < sqr(g.sleep_velocity_threshold_ang);
}

// Find the representative body for a dynamic connected component. Static bodies are stored as parent -1 and are never part of a sleep island.
odr int FindSleepRootNoCompress(int body_idx)
{
	int parent = g_sleep_parents[body_idx];
	if (parent < 0)
		return -1;

	// Walk up the union-find parent chain. The bounded loop avoids a malformed chain becoming an unbounded GPU loop.
	int root = parent;
	for (int i = 0; i != g.body_count; ++i)
	{
		int next = g_sleep_parents[root];
		if (next == root)
			break;

		root = next;
	}

	return root;
}

odr int FindSleepRoot(int body_idx)
{
	int root = FindSleepRootNoCompress(body_idx);
	if (root < 0)
		return -1;

	// Path-compress while walking back from 'body_idx'. This keeps later root lookups cheap in the following update kernels.
	int node = body_idx;
	for (int i = 0; i != g.body_count; ++i)
	{
		int next = g_sleep_parents[node];
		if (next == root)
			break;

		g_sleep_parents[node] = root;
		node = next;
	}

	return root;
}

// Merge the contact components containing 'body_idx_a' and 'body_idx_b'. Only dynamic-dynamic contacts reach this function.
odr void UnionSleepRoots(int body_idx_a, int body_idx_b)
{
	// Roots only move toward lower body indices, so racing unions converge deterministically. Retrying when another thread changed the higher root keeps
	// disconnected components from remaining split just because two contacts touched the same component in the same dispatch.
	for (int i = 0; i != g.body_count; ++i)
	{
		int root_a = FindSleepRootNoCompress(body_idx_a);
		int root_b = FindSleepRootNoCompress(body_idx_b);
		if (root_a < 0 || root_b < 0 || root_a == root_b)
			return;

		int root = min(root_a, root_b);
		int child = max(root_a, root_b);
		int original_parent;
		InterlockedMin(g_sleep_parents[child], root, original_parent);
		if (original_parent == child || original_parent == root)
			return;
	}
}

odr int SleepStatsIslandId(in_(GpuSleepIslandStats) stats, int root)
{
	return stats.island_id != SleepIslandNoId ? stats.island_id : g.island_count + root;
}

numthreads(CSDisturbIslands, SleepThreadCount, 1, 1)
void CSDisturbIslands(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0 || g.island_count == 0)
		return;

	// One thread tests one awake-body/existing-sleeping-island pair. This is deliberately conservative: a disturbed island is allowed to enter
	// broadphase, but the island only becomes awake later if narrowphase/resolve finds real contact.
	int pair_idx = dtid.x;
	int pair_count = g.body_count * g.island_count;
	if (pair_idx >= pair_count)
		return;

	int body_idx = pair_idx / g.island_count;
	int island_idx = pair_idx - body_idx * g.island_count;

	GpuRigidBody body = g_bodies[body_idx];
	if (!ActuallyAwake(body))
		return;

	// Ignore invalid, awake, or already-disturbed-free islands. Only valid sleeping islands can be disturbed by moving awake bodies.
	GpuSleepIsland island = g_sleep_islands[island_idx];
	if (!AllSet(island.flags, GpuSleepIslandFlags_Valid | GpuSleepIslandFlags_Sleeping))
		return;

	BBox body_bbox_ws = BBox_Transform(body.os_bbox, body.o2w);
	if (!BBox_IsIntersection(body_bbox_ws, island.bbox_ws))
		return;

	InterlockedOr(g_sleep_islands[island_idx].flags, GpuSleepIslandFlags_Disturbed);
}

numthreads(CSInitSleepState, SleepThreadCount, 1, 1)
void CSInitSleepState(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	// Start with every dynamic body as a singleton island candidate. Static bodies get parent -1, so they can collide without joining stacks together.
	GpuRigidBody body = g_rw_bodies[body_idx];
	g_sleep_parents[body_idx] = DynamicBody(body) ? body_idx : -1;
	GpuSleepIslandStats stats = (GpuSleepIslandStats)0;
	stats.flags = GpuSleepIslandStatsFlags_AllLow | GpuSleepIslandStatsFlags_AllReady;
	stats.island_id = SleepIslandNoId;
	g_sleep_stats[body_idx] = stats;
}

numthreads(CSUnionSleepContacts, SleepThreadCount, 1, 1)
void CSUnionSleepContacts(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	// Union bodies that actually contacted this frame. This builds connected components from real narrowphase contacts, not broadphase overlaps.
	int contact_idx = dtid.x;
	int contact_count = min(g_counters[0].contact_count, g.max_contacts);
	if (contact_idx >= contact_count)
		return;

	GpuResolveContact contact = g_contacts[contact_idx];
	GpuRigidBody body_a = g_rw_bodies[contact.body_idx_a];
	GpuRigidBody body_b = g_rw_bodies[contact.body_idx_b];
	if (DynamicBody(body_a) && DynamicBody(body_b))
		UnionSleepRoots(contact.body_idx_a, contact.body_idx_b);
}

numthreads(CSCanonicaliseSleepRoots, SleepThreadCount, 1, 1)
void CSCanonicaliseSleepRoots(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	// Canonicalise the parent array so later loops can use g_sleep_parents[body_idx] directly as the island root/stat index.
	if (g_sleep_parents[body_idx] >= 0)
		g_sleep_parents[body_idx] = FindSleepRoot(body_idx);
}

numthreads(CSReduceSleepStats, SleepThreadCount, 1, 1)
void CSReduceSleepStats(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	// Reduce per-body state into per-island stats. An island can only sleep when every member is below the thresholds for long enough. Any member that is
	// moving, never-sleep, or part of a hit sleeping island wakes the whole island.
	GpuRigidBody body = g_rw_bodies[body_idx];
	int root = g_sleep_parents[body_idx];
	if (root < 0)
		return;

	bool sleeping = AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
	bool never_sleep = AllSet(body.state_flags, ERigidBodyStateFlags_NeverSleep);
	bool low_velocity = LowVelocity(body);
	float timer_s = low_velocity ? body.sleep.timer_s + g.dt : 0.0f;

	InterlockedOr(g_sleep_stats[root].flags, GpuSleepIslandStatsFlags_Valid);
	InterlockedAdd(g_sleep_stats[root].body_count, 1);
	if (!low_velocity || never_sleep)
		InterlockedAnd(g_sleep_stats[root].flags, ~GpuSleepIslandStatsFlags_AllLow);
	if ((!sleeping && timer_s < g.sleep_delay_s) || never_sleep)
		InterlockedAnd(g_sleep_stats[root].flags, ~GpuSleepIslandStatsFlags_AllReady);
	if ((!sleeping && !low_velocity) || never_sleep)
		InterlockedOr(g_sleep_stats[root].flags, GpuSleepIslandStatsFlags_Wake);

	int island_id = body.sleep.island_id;
	if (sleeping && island_id >= 0)
		InterlockedMin(g_sleep_stats[root].island_id, island_id);
}

numthreads(CSApplySleepState, SleepThreadCount, 1, 1)
void CSApplySleepState(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	int body_idx = dtid.x;
	if (body_idx >= g.body_count)
		return;

	// Apply the island decision back to each body. This is where the transient contact components become persistent sleeping island ids for the next frame.
	GpuRigidBody body = g_rw_bodies[body_idx];
	int root = g_sleep_parents[body_idx];
	if (root < 0)
		return;

	GpuSleepIslandStats stats = g_sleep_stats[root];
	bool sleeping = AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
	bool low_velocity = LowVelocity(body);
	bool wake = AnySet(stats.flags, GpuSleepIslandStatsFlags_Wake);
	bool sleep = AllSet(stats.flags, GpuSleepIslandStatsFlags_Valid | GpuSleepIslandStatsFlags_AllLow | GpuSleepIslandStatsFlags_AllReady);

	if (wake)
	{
		// Waking invalidates persisted sleep-island membership, but low awake bodies in the same contact component should keep accumulating sleep time. A
		// single moving neighbour should not erase the settled history for every other body in the island candidate.
		if (sleeping || body.sleep.island_id >= 0)
			body.sleep.generation++;

		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.sleep.timer_s = (!sleeping && low_velocity) ? body.sleep.timer_s + g.dt : 0.0f;
		body.sleep.island_id = -1;
		body.sleep.flags = 0;
	}
	else if (sleeping)
	{
		// A still-sleeping body stays inert. If it was created asleep, use the contact component to create its first island id.
		body.momentum_ang = float4(0, 0, 0, 0);
		body.momentum_lin = float4(0, 0, 0, 0);
		body.force_ang = float4(0, 0, 0, 0);
		body.force_lin = float4(0, 0, 0, 0);
		body.sleep.timer_s = 0.0f;

		int island_id = SleepStatsIslandId(stats, root);
		if (body.sleep.island_id != island_id)
		{
			body.sleep.island_id = island_id;
			body.sleep.generation++;
		}
	}
	else if (sleep)
	{
		// New sleeping islands use ids above the uploaded island range. The CPU remaps these frame-local GPU ids to stable ids after readback.
		int island_id = SleepStatsIslandId(stats, root);
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, true);
		body.momentum_ang = float4(0, 0, 0, 0);
		body.momentum_lin = float4(0, 0, 0, 0);
		body.force_ang = float4(0, 0, 0, 0);
		body.force_lin = float4(0, 0, 0, 0);
		body.sleep.timer_s = 0.0f;
		body.sleep.island_id = island_id;
		body.sleep.generation++;
		body.sleep.flags = 0;
	}
	else
	{
		body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
		body.sleep.timer_s = low_velocity ? body.sleep.timer_s + g.dt : 0.0f;
		body.sleep.island_id = -1;
		body.sleep.flags = 0;
	}

	g_rw_bodies[body_idx] = body;
}

#ifdef __cplusplus
}
#endif
