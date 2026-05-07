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

// Sleeping is split into two GPU passes. 'CSDisturbIslands' runs before broadphase and conservatively marks existing sleeping islands whose
// bounding boxes overlap awake bodies. 'CSUpdateSleepState' runs after narrowphase/resolve, using real contacts to build the next set of islands.

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
	return inv_mass > 0.0f &&
		dot(body.momentum_lin.xyz, body.momentum_lin.xyz) < sqr(g.sleep_velocity_threshold_lin) / (sqr(inv_mass) + 1e-30f) &&
		dot(body.momentum_ang.xyz, body.momentum_ang.xyz) < sqr(g.sleep_velocity_threshold_ang) / (sqr(inv_mass) + 1e-30f);
}

// Find the representative body for a dynamic connected component. Static bodies are stored as parent -1 and are never part of a sleep island.
odr int FindSleepRoot(int body_idx)
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

	// Path-compress while walking back from 'body_idx'. This keeps later root lookups cheap in the serial update pass.
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
	int root_a = FindSleepRoot(body_idx_a);
	int root_b = FindSleepRoot(body_idx_b);
	if (root_a < 0 || root_b < 0 || root_a == root_b)
		return;

	// Use the lowest body index as the root so generated GPU island ids are deterministic for a given contact graph.
	int root = min(root_a, root_b);
	int child = max(root_a, root_b);
	g_sleep_parents[child] = root;
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

numthreads(CSUpdateSleepState, 1, 1, 1)
void CSUpdateSleepState(int3 dtid : SV_DispatchThreadID)
{
	if (g.sleeping_enabled == 0)
		return;

	// This pass is intentionally single-threaded for now. It keeps the contact-island logic simple while still avoiding a CPU readback/wake decision.
	int contact_count = min(g_counters[0].contact_count, g.max_contacts);

	// Start with every dynamic body as a singleton island candidate. Static bodies get parent -1, so they can collide without joining stacks together.
	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
	{
		GpuRigidBody body = g_rw_bodies[body_idx];
		g_sleep_parents[body_idx] = DynamicBody(body) ? body_idx : -1;
		g_sleep_stats[body_idx] = (GpuSleepIslandStats)0;
	}

	// Union bodies that actually contacted this frame. This builds connected components from real narrowphase contacts, not broadphase overlaps.
	for (int contact_idx = 0; contact_idx != contact_count; ++contact_idx)
	{
		GpuResolveContact contact = g_contacts[contact_idx];
		GpuRigidBody body_a = g_rw_bodies[contact.body_idx_a];
		GpuRigidBody body_b = g_rw_bodies[contact.body_idx_b];
		if (DynamicBody(body_a) && DynamicBody(body_b))
			UnionSleepRoots(contact.body_idx_a, contact.body_idx_b);
	}

	// Canonicalise the parent array so later loops can use g_sleep_parents[body_idx] directly as the island root/stat index.
	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
	{
		if (g_sleep_parents[body_idx] >= 0)
			g_sleep_parents[body_idx] = FindSleepRoot(body_idx);
	}

	// Existing sleeping islands can be hit by broadphase/narrowphase because CSDisturbIslands marked them. A resolver contact turns that conservative
	// disturbance into a real same-frame wake for the whole persisted island.
	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
	{
		GpuRigidBody body = g_rw_bodies[body_idx];
		int island_id = body.sleep.island_id;
		if (AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) &&
			AllSet(body.state_flags, ERigidBodyStateFlags_Collided) &&
			island_id >= 0 &&
			island_id < g.island_count)
		{
			g_sleep_islands[island_id].flags = SetFlag(g_sleep_islands[island_id].flags, GpuSleepIslandFlags_HitThisFrame, true);
		}
	}

	// Reduce per-body state into per-island stats. An island can only sleep when every member is below the thresholds for long enough. Any member that is
	// moving, never-sleep, or part of a hit sleeping island wakes the whole island.
	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
	{
		GpuRigidBody body = g_rw_bodies[body_idx];
		int root = g_sleep_parents[body_idx];
		if (root < 0)
			continue;

		GpuSleepIslandStats stats = g_sleep_stats[root];
		if (!AllSet(stats.flags, GpuSleepIslandStatsFlags_Valid))
		{
			stats.flags = GpuSleepIslandStatsFlags_Valid | GpuSleepIslandStatsFlags_AllLow | GpuSleepIslandStatsFlags_AllReady;
			stats.body_count = 0;
			stats.island_id = -1;
		}

		bool sleeping = AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
		bool never_sleep = AllSet(body.state_flags, ERigidBodyStateFlags_NeverSleep);
		bool low_velocity = LowVelocity(body);
		float timer_s = low_velocity ? body.sleep.timer_s + g.dt : 0.0f;

		stats.body_count += 1;
		if (!low_velocity || never_sleep)
			stats.flags = SetFlag(stats.flags, GpuSleepIslandStatsFlags_AllLow, false);
		if ((!sleeping && timer_s < g.sleep_delay_s) || never_sleep)
			stats.flags = SetFlag(stats.flags, GpuSleepIslandStatsFlags_AllReady, false);
		if ((!sleeping && !low_velocity) || never_sleep)
			stats.flags = SetFlag(stats.flags, GpuSleepIslandStatsFlags_Wake, true);

		int island_id = body.sleep.island_id;
		if (sleeping && island_id >= 0)
			stats.island_id = stats.island_id < 0 ? island_id : min(stats.island_id, island_id);

		if (sleeping &&
			island_id >= 0 &&
			island_id < g.island_count &&
			AnySet(g_sleep_islands[island_id].flags, GpuSleepIslandFlags_HitThisFrame))
		{
			stats.flags = SetFlag(stats.flags, GpuSleepIslandStatsFlags_Wake, true);
		}

		g_sleep_stats[root] = stats;
	}

	// Apply the island decision back to each body. This is where the transient contact components become persistent sleeping island ids for the next frame.
	for (int body_idx = 0; body_idx != g.body_count; ++body_idx)
	{
		GpuRigidBody body = g_rw_bodies[body_idx];
		int root = g_sleep_parents[body_idx];
		if (root < 0)
			continue;

		GpuSleepIslandStats stats = g_sleep_stats[root];
		bool sleeping = AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
		bool low_velocity = LowVelocity(body);
		bool wake = AnySet(stats.flags, GpuSleepIslandStatsFlags_Wake);
		bool sleep = AllSet(stats.flags, GpuSleepIslandStatsFlags_Valid | GpuSleepIslandStatsFlags_AllLow | GpuSleepIslandStatsFlags_AllReady);

		if (wake)
		{
			// Waking invalidates any persisted sleep island membership. The generation bump lets CPU-side staging notice the membership change.
			if (sleeping || body.sleep.island_id >= 0)
				body.sleep.generation++;

			body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
			body.sleep.timer_s = 0.0f;
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

			int island_id = stats.island_id >= 0 ? stats.island_id : g.island_count + root;
			if (body.sleep.island_id != island_id)
			{
				body.sleep.island_id = island_id;
				body.sleep.generation++;
			}
		}
		else if (sleep)
		{
			// New sleeping islands use ids above the uploaded island range. The CPU remaps these frame-local GPU ids to stable ids after readback.
			int island_id = stats.island_id >= 0 ? stats.island_id : g.island_count + root;
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
}

#ifdef __cplusplus
}
#endif
