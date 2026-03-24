//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One thread per broad phase pair. Dispatches to specialised collision tests
// for simple/curved shape pairs, or falls through to generic GJK + EPA for
// purely polyhedral pairs.
//
// Outputs GpuResolveContact directly (no intermediate GpuContact). The collide
// shader has access to the pair data (b2a, body indices) and the materials buffer,
// so it can build the complete resolve contact inline.
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
#include "src/compute/physics_types.hlsli"
#include "src/compute/collision.hlsli"
#include "src/compute/gjk.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Shader parameters
cbuffer cbCollision : register(b0)
{
	uint g_max_contacts;
	uint g_pad0;
	uint g_pad1;
	uint g_pad2;
};

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
	float4x4 a2w = shape_a.s2rb;
	float4x4 b2w = mul(shape_b.s2rb, pair.b2a);

	// Dispatch to specialised collision tests for shape pairs involving
	// implicit curved surfaces (sphere, thick line), or fall through to
	// generic GJK + EPA for purely polyhedral pairs.
	float4 col_axis;
	float4 col_point;
	float depth;
	int gjk_iters = 0;
	int epa_iters = 0;

	// Canonicalise the pair so the "simpler" shape type is always 'sa'.
	// This reduces the number of dispatch branches: sphere < line < box < triangle < polytope.
	// When we swap, negate the contact axis on output.
	bool swapped = false;
	GpuShape sa = shape_a, sb = shape_b;
	float4x4 wa = a2w, wb = b2w;
	if (sa.type > sb.type)
	{
		sa = shape_b; sb = shape_a;
		wa = b2w; wb = a2w;
		swapped = true;
	}

	// sa.type <= sb.type is guaranteed
	bool hit = false;
	switch (sa.type)
	{
		case SHAPE_SPHERE:
		{
			switch (sb.type)
			{
			case SHAPE_SPHERE:   hit = SphereVsSphere(sa, wa, sb, wb, col_axis, col_point, depth); break;
			case SHAPE_BOX:      hit = SphereVsBox(sa, wa, sb, wb, col_axis, col_point, depth); break;
			case SHAPE_LINE:     hit = SphereVsLine(sa, wa, sb, wb, col_axis, col_point, depth); break;
			case SHAPE_TRIANGLE: hit = SphereVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
			case SHAPE_POLYTOPE: hit = SphereVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
			}
			break;
		}
		case SHAPE_BOX:
		{
			switch (sb.type)
			{
			case SHAPE_BOX:      hit = BoxVsBox(sa, wa, sb, wb, col_axis, col_point, depth); break;
			case SHAPE_LINE:     hit = LineVsBox(sb, wb, sa, wa, col_axis, col_point, depth); break;
			case SHAPE_TRIANGLE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
			case SHAPE_POLYTOPE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
			}
			break;
		}
		case SHAPE_LINE:
		{
			switch (sb.type)
			{
			case SHAPE_LINE:     hit = LineVsLine(sa, wa, sb, wb, col_axis, col_point, depth); break;
			case SHAPE_TRIANGLE: hit = LineVsTriangle(sa, wa, sb, wb, g_verts, col_axis, col_point, depth); break;
			case SHAPE_POLYTOPE: hit = LineVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
			}
			break;
		}
		case SHAPE_TRIANGLE:
		{
			switch (sb.type)
			{
			case SHAPE_TRIANGLE: hit = TriangleVsTriangle(sa, wa, sb, wb, g_verts, col_axis, col_point, depth); break;
			case SHAPE_POLYTOPE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
			}
			break;
		}
		case SHAPE_POLYTOPE:
		{
			hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters);
			break;
		}
	}

	// If we swapped A and B, negate the contact axis
	if (swapped && hit)
		col_axis = -col_axis;

	// Write per-pair diagnostics (every pair, not just colliding ones)
	#if PR_COLLISION_DIAGNOSTICS
	{
		GpuPairDiag diag;
		diag.body_idx_a = pair.body_idx_a;
		diag.body_idx_b = pair.body_idx_b;
		diag.shape_type_a = shape_a.type;
		diag.shape_type_b = shape_b.type;
		diag.gjk_iters = gjk_iters;
		diag.epa_iters = epa_iters;
		diag.hit = hit ? 1 : 0;
		diag.pad0 = 0;
		g_diag[dtid.x] = diag;
	}
	#endif

	if (!hit)
		return;

	// Allocate a slot in the contact buffer atomically
	uint slot;
	InterlockedAdd(g_counters[0].contact_count, 1, slot);
	if (slot >= g_max_contacts)
		return;

	// Write the resolve contact directly (no intermediate GpuContact)
	GpuResolveContact contact;
	contact.axis = col_axis;
	contact.contact_point = col_point;
	contact.b2a = pair.b2a;
	contact.body_idx_a = pair.body_idx_a;
	contact.body_idx_b = pair.body_idx_b;
	contact.mat_id_a = shape_a.material_id;
	contact.mat_id_b = shape_b.material_id;
	contact.depth = depth;
	contact.collision_time = 0;
	contact.pad0 = 0;
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
