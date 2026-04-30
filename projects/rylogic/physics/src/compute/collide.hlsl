//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One thread per broad phase pair. Dispatches to specialised collision tests
// for simple/curved shape pairs, or falls through to generic GJK + EPA for
// purely polyhedral pairs.
//
	// Outputs one GpuResolveContact per colliding pair. The narrow phase returns a
	// manifold contact; contact_point is kept as the centroid for the point-based resolver.
//
// Buffer layout:
//   b0: cbuffer with max contacts
//   u0: RWStructuredBuffer<GpuCollisionCounters> — counters (read pair_count, write contact_count)
//   u1: RWStructuredBuffer<GpuResolveContact>    — output contacts for the resolve shader
//   u2: RWStructuredBuffer<DispatchArguments>    — dispatch args (unused by this shader, bound for root sig compatibility)
//   t0: StructuredBuffer<GpuCollisionPair>       — broadphase overlap pairs
//   t1: StructuredBuffer<GpuShape>               — all unique shapes in the scene
//   t2: StructuredBuffer<float4>                 — shared vertex buffer (polytope/triangle)
//   t3: StructuredBuffer<GpuMaterial>            — material properties
//   u3: RWStructuredBuffer<GpuPairDiag>          — per-pair diagnostic output (PR_COLLISION_DIAGNOSTICS only)
//
// Compile-time switches:
//   PR_COLLISION_DIAGNOSTICS — enable per-pair iteration count output to u3
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/collision.hlsli"
#include "physics/src/compute/gjk.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Shader parameters
struct cbCollision
{
	uint max_contacts;
	uint pad0;
	uint pad1;
	uint pad2;
};

ConstantBuffer<cbCollision> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuResolveContact> resource(g_contacts, u1);
RWStructuredBuffer<DispatchArguments> resource(g_dispatch_args, u2);
StructuredBuffer<GpuCollisionPair> resource(g_pairs, t0);
StructuredBuffer<GpuShape> resource(g_shapes, t1);
StructuredBuffer<float4> resource(g_verts, t2);
#if PR_COLLISION_DIAGNOSTICS
RWStructuredBuffer<GpuPairDiag> resource(g_diag, u3);
#endif

numthreads(CSCollide, CollideThreadCount, 1, 1)
void CSCollide(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= g_counters[0].pair_count)
		return;

	GpuCollisionPair pair = g_pairs[dtid.x];
	GpuShape shape_a = g_shapes[pair.shape_idx_a];
	GpuShape shape_b = g_shapes[pair.shape_idx_b];

	// Build the transforms for each shape.
	// Shape A is at identity (collision runs in A's space).
	// Shape B is at b2a (the relative transform from B to A).
	// "World" here means RigidBody-A's local space.
	float4x4 a2w = Identity4x4();
	float4x4 b2w = pair.b2a;

	// Dispatch to specialised collision tests for shape pairs
	GpuContact col;
	bool hit = CollideShapes(shape_a, a2w, shape_b, b2w, g_verts, col);

	// Write per-pair diagnostics (every pair, not just colliding ones)
	#if PR_COLLISION_DIAGNOSTICS
	{
		GpuPairDiag diag;
		diag.body_idx_a = pair.body_idx_a;
		diag.body_idx_b = pair.body_idx_b;
		diag.shape_type_a = shape_a.type;
		diag.shape_type_b = shape_b.type;
		diag.hit = hit ? 1 : 0;
		diag.pad0 = 0;
		g_diag[dtid.x] = diag;
	}
	#endif

	if (!hit)
		return;

	uint slot;
	InterlockedAdd(g_counters[0].contact_count, 1, slot);
	if (slot >= g.max_contacts)
		return;

	GpuResolveContact contact;
	contact.axis = col.axis;
	contact.contact_point = ContactCentroid(col);
	for (int i = 0; i != GpuContactMaxPoints; ++i)
		contact.manifold[i] = col.manifold[i];
	contact.b2a = pair.b2a;
	contact.body_idx_a = pair.body_idx_a;
	contact.body_idx_b = pair.body_idx_b;
	contact.mat_id_a = shape_a.material_id;
	contact.mat_id_b = shape_b.material_id;
	contact.depth = col.depth;
	contact.collision_time = 0;
	contact.feature = col.feature;
	contact.pad1 = 0;
	g_contacts[slot] = contact;
}

// Calculates the number of thread groups needed for the resolve shader
// based on the number of contacts found, and writes to the dispatch arguments buffer.
numthreads(CSCalcResolveDispatch, 1,1,1)
void CSCalcResolveDispatch(int3 dtid : SV_DispatchThreadID)
{
	// Ensure that there is always at least one thread group dispatched in the resolve shader,
	// so that it always resets the 'colour_used' and contact times
	int count = g_counters[0].contact_count;
	g_dispatch_args[0].ThreadGroupCountX = max(1, (count + ResolveThreadCount - 1) / ResolveThreadCount);
	g_dispatch_args[0].ThreadGroupCountY = 1;
	g_dispatch_args[0].ThreadGroupCountZ = 1;
}

#ifdef __cplusplus
}
#endif
