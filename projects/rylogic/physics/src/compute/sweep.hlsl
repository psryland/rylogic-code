//*********************************************
// Physics Engine — GPU Sort and Sweep Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// This shader performs the 'sweep' step of a sort and sweep.
// It takes a sorted index list, performs AABB overlap tests,
// and outputs a list of potential collision pairs.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "pr/hlsl/bounding_box.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Shader parameters
struct cbSweep
{
	int max_pair_count; // The maximum length of the g_collision_pairs buffer
	int body_count;
	int sleeping_enabled;
	int sleep_island_count;
};

// Shader Resources
ConstantBuffer<cbSweep> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuCollisionPair> resource(g_collision_pairs, u1);
RWStructuredBuffer<DispatchArguments> resource(g_dispatch_args, u2);
StructuredBuffer<GpuRigidBody> resource(g_bodies, t0);
StructuredBuffer<int> resource(g_aabb_idx, t1);
StructuredBuffer<GpuSleepIsland> resource(g_sleep_islands, t2);

odr bool DynamicBody(in_(GpuRigidBody) body)
{
	return body.os_com_and_invmass.w > 0.0f &&
		!HasFlag(body.state_flags, ERigidBodyStateFlags_Static);
}

odr bool EffectiveAwake(in_(GpuRigidBody) body)
{
	if (!DynamicBody(body))
		return false;

	if (g.sleeping_enabled == 0 || !HasFlag(body.state_flags, ERigidBodyStateFlags_Sleeping))
		return true;

	int island_id = body.sleep.island_id;
	return island_id >= 0 && island_id < g.sleep_island_count &&
		(g_sleep_islands[island_id].flags & GpuSleepIslandFlags_Disturbed) != 0;
}

numthreads(CSSweep, SweepThreadCount, 1, 1)
void CSSweep(int3 dtid : SV_DispatchThreadID)
{
	// 'aabb_idx' is a list of encoded body indices sorted on some axes.
	// There is one thread per aabb_idx array element. Indexes that are 'end' bounds, return early.
	// For indexes that are start bounds, search forward in the index buffer, adding pairs for any overlap that
	// is a full 3D overlap.
	// Since if A overlaps B, then B overlaps A, only add a collision pair if A < B
	// Note: indices are encoded as:
	//   start = (body_index << 1) | 0
	//   end = (body_index << 1) | 1
	
	// The number of bounds in the 'g_aabb_idx' buffer.
	int bounds_count = 2 * g.body_count;

	// The index into 'g_aabb_idx'
	int idx = dtid.x;
	if (idx >= bounds_count)
		return;

	// Only threads on 'start' bounds are needed
	if ((g_aabb_idx[idx] & 1) == 1)
		return;

	// Get the object we're testing against the other objects
	int rbA_idx = g_aabb_idx[idx] >> 1;
	GpuRigidBody rb = g_bodies[rbA_idx];
	
	// Get the index value that ends our search
	int end_idx = g_aabb_idx[idx] | 1;

	// The world-space bbox of the object we're testing
	BBox ws_bbox = BBox_Transform(rb.os_bbox, rb.o2w);
	
	// Search for overlaps on this axis.
	// Walk forward from this body's start bound until our end bound is reached.
	// Check all encountered start bounds for 3D AABB overlap.
	for (++idx; idx != bounds_count && g_aabb_idx[idx] != end_idx; ++idx)
	{
		int payload = g_aabb_idx[idx];
		if ((payload & 1) == 1)
			continue; // Skip 'end' bounds

		int rbB_idx = payload >> 1;
		
		GpuRigidBody other_rb = g_bodies[rbB_idx];
		BBox other_ws_bbox = BBox_Transform(other_rb.os_bbox, other_rb.o2w);
		
		// If intersection on all three axes, add a pair to the output buffer.
		// Canonicalise pair so lower body index is always 'a'.
		if (BBox_IsIntersection(ws_bbox, other_ws_bbox))
		{
			if (!EffectiveAwake(rb) && !EffectiveAwake(other_rb))
				continue;

			int idxA = min(rbA_idx, rbB_idx);
			int idxB = max(rbA_idx, rbB_idx);
			
			// Allocate a slot in the collision pairs buffer atomically
			uint slot;
			InterlockedAdd(g_counters[0].pair_count, 1, slot);
			if (slot >= g.max_pair_count)
				return;

			// Write the contact
			GpuCollisionPair pair;
			pair.body_idx_a = idxA;
			pair.body_idx_b = idxB;
			pair.shape_idx_a = g_bodies[idxA].shape_id;
			pair.shape_idx_b = g_bodies[idxB].shape_id;
			pair.b2a = mul(g_bodies[idxB].o2w, InvertOrthonormal(g_bodies[idxA].o2w));
			g_collision_pairs[slot] = pair;
		}
	}
}

// This shader is dispatched with 1 thread. It calculates the number of thread groups needed for the collision detection
// shader based on the number of pairs found in the sweep step, and writes that to the dispatch arguments buffer.
numthreads(CSCalcCDDispatch, 1,1,1)
void CSCalcCDDispatch(int3 dtid : SV_DispatchThreadID)
{
	uint pair_count = min(g_counters[0].pair_count, g.max_pair_count);
	g_dispatch_args[0].ThreadGroupCountX = (pair_count + CollideThreadCount - 1) / CollideThreadCount;
	g_dispatch_args[0].ThreadGroupCountY = 1;
	g_dispatch_args[0].ThreadGroupCountZ = 1;
}

#ifdef __cplusplus
}
#endif
