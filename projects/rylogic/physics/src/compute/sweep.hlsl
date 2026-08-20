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
StructuredBuffer<BBox> resource(g_aabb_box, t2);
StructuredBuffer<GpuSleepIsland> resource(g_sleep_islands, t3);
StructuredBuffer<GpuShape> resource(g_shapes, t4);

odr bool DynamicBody(in_(GpuRigidBody) body)
{
	return body.os_com_and_invmass.w > 0.0f &&
		!AllSet(body.state_flags, ERigidBodyStateFlags_Static);
}

odr bool EffectiveAwake(in_(GpuRigidBody) body)
{
	if (!DynamicBody(body))
		return false;

	if (g.sleeping_enabled == 0 || !AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping))
		return true;

	int island_id = body.sleep.island_id;
	return island_id >= 0 && island_id < g.sleep_island_count &&
		AnySet(g_sleep_islands[island_id].flags, GpuSleepIslandFlags_Disturbed);
}

odr bool CachedBoundsOverlap(int body_a, int body_b)
{
	return g_aabb_box[body_a].IsIntersection(g_aabb_box[body_b]);
}

// Emit the narrowphase pairs for two bodies whose bounds overlap.
// The narrowphase only understands convex operands, so a compound body is expanded here into its
// flattened convex leaves. A body with a single convex shape has one leaf and therefore emits
// exactly one pair, which keeps the common case identical to a non-compound scene.
// Returns false when the pair buffer is full, telling the caller to stop emitting.
odr bool StorePair(int rbA_idx, int rbB_idx, in_(GpuRigidBody) rb, in_(GpuRigidBody) other_rb)
{
	if (!EffectiveAwake(rb) && !EffectiveAwake(other_rb))
		return true;

	int idxA = min(rbA_idx, rbB_idx);
	int idxB = max(rbA_idx, rbB_idx);

	int root_idx_a = g_bodies[idxA].shape_id;
	int root_idx_b = g_bodies[idxB].shape_id;
	GpuShape root_a = g_shapes[root_idx_a];
	GpuShape root_b = g_shapes[root_idx_b];

	// A compound contributes its leaves, anything else contributes itself. A compound with no leaves
	// has nothing to collide with, so the body pair is silently dropped.
	bool compound_a = root_a.type == SHAPE_ARRAY;
	bool compound_b = root_b.type == SHAPE_ARRAY;
	int count_a = compound_a ? root_a.child_count : 1;
	int count_b = compound_b ? root_b.child_count : 1;
	if (count_a == 0 || count_b == 0)
		return true;

	float4x4 b2a = mul(g_bodies[idxB].o2w, InvertOrthonormal(g_bodies[idxA].o2w));

	// Leaf bounds only need testing when at least one side actually expands; the caller has already
	// established that the two bodies overlap, so a 1x1 expansion must not pay for a second test.
	bool expanded = (count_a * count_b) > 1;

	for (int j = 0; j != count_b; ++j)
	{
		int shape_idx_b = compound_b ? (root_b.child_offset + j) : root_idx_b;
		BBox bbox_b = g_shapes[shape_idx_b].rb_bbox.Transform(b2a);

		for (int i = 0; i != count_a; ++i)
		{
			int shape_idx_a = compound_a ? (root_a.child_offset + i) : root_idx_a;
			if (expanded && !g_shapes[shape_idx_a].rb_bbox.IsIntersection(bbox_b))
				continue;

			// Allocate a slot in the collision pairs buffer atomically.
			uint slot;
			InterlockedAdd(g_counters[0].pair_count, 1, slot);
			if (slot >= g.max_pair_count)
				return false;

			// Write the pair. The leaf indices are the stable half of the contact identity that
			// distinguishes leaves of the same body pair from one another.
			GpuCollisionPair pair;
			pair.body_idx_a = idxA;
			pair.body_idx_b = idxB;
			pair.shape_idx_a = shape_idx_a;
			pair.shape_idx_b = shape_idx_b;
			pair.b2a = b2a;
			pair.child_idx_a = i;
			pair.child_idx_b = j;
			pair.pad0 = 0;
			pair.pad1 = 0;
			g_collision_pairs[slot] = pair;
		}
	}
	return true;
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
		
		// If intersection on all three axes, add a pair to the output buffer.
		// Canonicalise pair so lower body index is always 'a'.
		if (CachedBoundsOverlap(rbA_idx, rbB_idx))
		{
			if (!StorePair(rbA_idx, rbB_idx, rb, other_rb))
				return;
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
