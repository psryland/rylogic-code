//*********************************************
// Physics Engine — GPU Sort and Sweep Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// This shader performs the 'sweep' step of a sort and sweep.
// It takes a sorted index list, performs AABB overlap tests,
// and outputs a list of potential collision pairs.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "pr/hlsl/bounding_box.hlsli"
#include "src/compute/rigid_body_dynamics.hlsli"
#include "src/compute/collision_types.hlsli"

// Shader parameters
cbuffer cbSweep : register(b0)
{
	int g_max_pair_count; // The maximum length of the g_collision_pairs buffer
	int g_pad0;
	int g_pad1;
	int g_pad2;
};

// Shader Resources
RWStructuredBuffer<GpuCollisionCounters> g_counters : register(u0);
RWStructuredBuffer<GpuCollisionPair> g_collision_pairs : register(u1);
RWStructuredBuffer<DispatchArguments> g_dispatch_args : register(u2);
StructuredBuffer<RigidBodyDynamics> g_bodies : register(t0);
StructuredBuffer<int> g_aabb_idx : register(t1);

[numthreads(SweepThreadCount, 1, 1)]
void CSSweep(int3 dtid : SV_DispatchThreadID)
{
	// 'aabb_idx' is a list of encoded body indices sorted on some axes.
	// There is one thread per aabb_idx array element. Indexes that are 'end'
	// bounds, return early. For indexex that are start bounds, search forward
	// in the index buffer adding pairs for overlap that is a full AABB overlap.
	// Note: indices are encoded as:
	//   start = (body_index << 1) | 0
	//   end = (body_index << 1) | 1
	
	// The number of bounds in the 'g_aabb_idx' buffer.
	int bounds_count = 2 * g_counters[0].body_count;

	// The index into 'g_aabb_idx'
	int idx = dtid.x;
	if (idx >= bounds_count)
		return;

	// Only threads on 'start' bounds are needed
	if ((g_aabb_idx[idx] & 1) == 1)
		return;

	// Get the object we're testing against the other objects
	int rbA_idx = g_aabb_idx[idx] >> 1;
	RigidBodyDynamics rb = g_bodies[rbA_idx];
	
	// Get the index value that ends our search
	int end_idx = g_aabb_idx[idx] | 1;

	// The world-space bbox of the object we're testing
	BBox ws_bbox = Transform(rb.os_bbox, rb.o2w);
	
	// Search for overlaps on this axis
	for (++idx; idx != bounds_count && g_aabb_idx[idx] != end_idx; ++idx)
	{
		int rbB_idx = g_aabb_idx[idx] >> 1;
		
		RigidBodyDynamics other_rb = g_bodies[rbB_idx];
		BBox other_ws_bbox = Transform(other_rb.os_bbox, other_rb.o2w);
		
		// If intersection on all three axes, add a pair to the output buffer
		if (BBox_IsIntersection(ws_bbox, other_ws_bbox))
		{
			// Allocate a slot in the collision pairs buffer atomically
			uint slot;
			InterlockedAdd(g_counters[0].pair_count, 1, slot);
			if (slot >= g_max_pair_count)
				return;

			// Write the contact
			GpuCollisionPair pair;
			pair.shape_idx_a = g_bodies[rbA_idx].shape_id;
			pair.shape_idx_b = g_bodies[rbB_idx].shape_id;
			pair.pair_index = slot;
			pair.pad0 = 0;
			pair.b2a = mul(other_rb.o2w, InvertOrthonormal(rb.o2w));
			g_collision_pairs[slot] = pair;
		}
	}
}

// This shader is dispatched with 1 thread. It calculates the number of thread groups needed for the collision detection
// shader based on the number of pairs found in the sweep step, and writes that to the dispatch arguments buffer.
[numthreads(1,1,1)]
void CSCalcCDDispatch(int3 dtid : SV_DispatchThreadID)
{
	uint pair_count = g_counters[0].pair_count;
	g_dispatch_args[0].ThreadGroupCountX = (pair_count + CollideThreadCount) / CollideThreadCount;
	g_dispatch_args[0].ThreadGroupCountY = 1;
	g_dispatch_args[0].ThreadGroupCountZ = 1;
}