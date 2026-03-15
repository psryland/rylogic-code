//*********************************************
// Physics Engine — GPU Narrow Phase Collision Compute Shader
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One thread per broadphase pair. Dispatches to specialised collision tests
// for simple/curved shape pairs, or falls through to generic GJK + EPA for
// purely polyhedral pairs.
//
// Buffer layout:
//   t0: StructuredBuffer<GpuShape>           — all unique shapes in the scene
//   t1: StructuredBuffer<GpuCollisionPair>   — broadphase overlap pairs
//   t2: StructuredBuffer<float4>             — shared vertex buffer (polytope/triangle)
//   u0: RWStructuredBuffer<GpuContact>       — collision results (written atomically)
//   u1: RWStructuredBuffer<uint>             — atomic contact count at index 0
//   u2: RWStructuredBuffer<GpuPairDiag>      — per-pair diagnostic output (COLLISION_DIAGNOSTICS only)
//   b0: cbuffer with pair count
//
// Compile-time switches:
//   COLLISION_DIAGNOSTICS — enable per-pair iteration count output to u2
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"

// ---- Shape type enum (matches C++ EShape) ----
static const int SHAPE_SPHERE   = 0;
static const int SHAPE_BOX      = 1;
static const int SHAPE_LINE     = 2;
static const int SHAPE_TRIANGLE = 3;
static const int SHAPE_POLYTOPE = 4;

// ---- GPU data structures (must match C++ layout exactly) ----
struct GpuShape
{
	row_major float4x4 s2p; // shape-to-parent transform
	int type;
	int vert_offset;
	int vert_count;
	int material_id;
	float4 data;            // type-specific: sphere(r), box(half_xyz), line(half_len,thickness)
};
struct GpuCollisionPair
{
	int shape_idx_a;
	int shape_idx_b;
	int pair_index;
	int pad0;
	row_major float4x4 b2a; // transform B into A's space
};
struct GpuContact
{
	float4 axis;
	float4 pt;
	float depth;
	int pair_index;
	int mat_id_a;
	int mat_id_b;
};

#ifdef COLLISION_DIAGNOSTICS
struct GpuPairDiag
{
	int pair_index;
	int shape_type_a;
	int shape_type_b;
	int gjk_iters;
	int epa_iters;
	int hit;
	int pad0;
	int pad1;
};
#endif

// ---- Bindings ----
cbuffer cbCollision : register(b0)
{
	uint g_pair_count;
	uint g_pad0;
	uint g_pad1;
	uint g_pad2;
};

StructuredBuffer<GpuShape>         g_shapes   : register(t0);
StructuredBuffer<GpuCollisionPair> g_pairs    : register(t1);
StructuredBuffer<float4>           g_verts    : register(t2);
RWStructuredBuffer<GpuContact>     g_contacts : register(u0);
RWStructuredBuffer<uint>           g_counters : register(u1);
#ifdef COLLISION_DIAGNOSTICS
RWStructuredBuffer<GpuPairDiag>    g_diag     : register(u2);
#endif

// Include collision algorithms after struct definitions (they reference GpuShape, etc.)
#include "pr/physics/compute/gjk.hlsli"
#include "pr/physics/compute/collision.hlsli"

// ---- Compute shader entry point ----
[numthreads(32, 1, 1)]
void CSCollisionDetect(uint3 ThreadID : SV_DispatchThreadID)
{
	if (ThreadID.x >= g_pair_count)
		return;

	GpuCollisionPair pair = g_pairs[ThreadID.x];
	GpuShape shape_a = g_shapes[pair.shape_idx_a];
	GpuShape shape_b = g_shapes[pair.shape_idx_b];

	// Build the world transforms for each shape.
	// Shape A is at identity (collision runs in A's space).
	// Shape B is at b2a (the relative transform from B to A).
	// The s2p transform is pre-baked into the world transforms.
	float4x4 a2w = shape_a.s2p;
	float4x4 b2w = mul(shape_b.s2p, pair.b2a);

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
		switch (sb.type)
		{
		case SHAPE_SPHERE:   hit = SphereVsSphere(sa, wa, sb, wb, col_axis, col_point, depth); break;
		case SHAPE_BOX:      hit = SphereVsBox(sa, wa, sb, wb, col_axis, col_point, depth); break;
		case SHAPE_LINE:     hit = SphereVsLine(sa, wa, sb, wb, col_axis, col_point, depth); break;
		case SHAPE_TRIANGLE: hit = SphereVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
		case SHAPE_POLYTOPE: hit = SphereVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
		}
		break;

	case SHAPE_BOX:
		switch (sb.type)
		{
		case SHAPE_BOX:      hit = BoxVsBox(sa, wa, sb, wb, col_axis, col_point, depth); break;
		case SHAPE_LINE:     hit = LineVsBox(sb, wb, sa, wa, col_axis, col_point, depth); break;
		case SHAPE_TRIANGLE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
		case SHAPE_POLYTOPE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
		}
		break;

	case SHAPE_LINE:
		switch (sb.type)
		{
		case SHAPE_LINE:     hit = LineVsLine(sa, wa, sb, wb, col_axis, col_point, depth); break;
		case SHAPE_TRIANGLE: hit = LineVsTriangle(sa, wa, sb, wb, g_verts, col_axis, col_point, depth); break;
		case SHAPE_POLYTOPE: hit = LineVsConvex(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters); break;
		}
		break;

	case SHAPE_TRIANGLE:
		switch (sb.type)
		{
		case SHAPE_TRIANGLE: hit = TriangleVsTriangle(sa, wa, sb, wb, g_verts, col_axis, col_point, depth); break;
		case SHAPE_POLYTOPE: hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters); break;
		}
		break;

	case SHAPE_POLYTOPE:
		// Polytope vs Polytope
		hit = GjkCollide(sa, wa, sb, wb, g_verts, col_axis, col_point, depth, gjk_iters, epa_iters);
		break;
	}

	// If we swapped A and B, negate the contact axis
	if (swapped && hit)
		col_axis = -col_axis;

	// Write per-pair diagnostics (every pair, not just colliding ones)
#ifdef COLLISION_DIAGNOSTICS
	{
		GpuPairDiag diag;
		diag.pair_index = pair.pair_index;
		diag.shape_type_a = shape_a.type;
		diag.shape_type_b = shape_b.type;
		diag.gjk_iters = gjk_iters;
		diag.epa_iters = epa_iters;
		diag.hit = hit ? 1 : 0;
		diag.pad0 = 0;
		diag.pad1 = 0;
		g_diag[ThreadID.x] = diag;
	}
#endif

	if (!hit)
		return;

	// Allocate a slot in the contact buffer atomically
	uint slot;
	InterlockedAdd(g_counters[0], 1, slot);
	if (slot >= g_pair_count)
		return;

	// Write the contact
	GpuContact contact;
	contact.axis = col_axis;
	contact.pt = col_point;
	contact.depth = depth;
	contact.pair_index = pair.pair_index;
	contact.mat_id_a = shape_a.material_id;
	contact.mat_id_b = shape_b.material_id;
	g_contacts[slot] = contact;
}
