//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU narrow phase collision detection.
//
// The broad phase emits a flat list of potentially colliding pairs. This shader
// first classifies those pairs into exact shape-pair bins, builds one indirect
// dispatch per bin, then runs a specialised kernel for each bin. The CPU records
// the complete graph every step; empty bins produce zero-sized dispatches.
//
// Buffer layout:
//   b0: cbuffer with contact and pair capacities
//   u0: RWStructuredBuffer<GpuCollisionCounters> - counters (read pair_count, write contact_count)
//   u1: RWStructuredBuffer<GpuResolveContact>    - output contacts for the resolve shader
//   u2: RWStructuredBuffer<DispatchArguments>    - resolve dispatch args
//   u3: RWStructuredBuffer<uint>                 - per-bin pair counts
//   u4: RWStructuredBuffer<uint>                 - per-bin broadphase pair indices
//   u5: RWStructuredBuffer<DispatchArguments>    - per-bin collide dispatch args
//   t0: StructuredBuffer<GpuCollisionPair>       - broadphase overlap pairs
//   t1: StructuredBuffer<GpuShape>               - all unique shapes in the scene
//   t2: StructuredBuffer<float4>                 - shared vertex buffer (polytope/triangle)
//   t3: StructuredBuffer<GpuPolytopeFace>        - shared polytope face topology
//   t4: StructuredBuffer<GpuPolytopeEdge>        - shared polytope edge topology
//
// Runtime order, as recorded by GpuCollisionDetector:
//   1. CSClearCollisionBins
//   2. CSBinCollisionPairs, dispatched with the broadphase pair-count indirect args
//   3. CSBuildCollisionBinDispatch
//   4. One CSCollide* kernel per exact bin, each dispatched from its own indirect args slot
//   5. CSCalcResolveDispatch
//
// The pair bins are deliberately exact shape pairs rather than algorithm-family buckets.
// This keeps each collide entry point small enough that simple pair kernels don't carry
// the bytecode and register pressure of unrelated GJK/EPA paths.
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
	uint max_pair_count;
	uint pad0;
	uint pad1;
};

// Resources
ConstantBuffer<cbCollision> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_counters, u0);
RWStructuredBuffer<GpuResolveContact> resource(g_contacts, u1);
RWStructuredBuffer<DispatchArguments> resource(g_dispatch_args, u2);
RWStructuredBuffer<uint> resource(g_bin_counts, u3);
RWStructuredBuffer<uint> resource(g_bin_pair_indices, u4);
RWStructuredBuffer<DispatchArguments> resource(g_bin_dispatch_args, u5);
StructuredBuffer<GpuCollisionPair> resource(g_pairs, t0);
StructuredBuffer<GpuShape> resource(g_shapes, t1);
StructuredBuffer<float4> resource(g_verts, t2);
StructuredBuffer<GpuPolytopeFace> resource(g_faces, t3);
StructuredBuffer<GpuPolytopeEdge> resource(g_edges, t4);

// Map an unordered pair of shape types to the exact bin that owns it. The broadphase preserves body order so that contacts are stored as A->B,
// but the binning only cares about the shape type combination.
int CollisionBinId(int type_a, int type_b)
{
	// The collision helper naming convention is highest-shape-type vs lowest-shape-type, e.g. BoxVsSphere, TriangleVsLine, PolytopeVsBox.
	int type_lo = min(type_a, type_b);
	int type_hi = max(type_a, type_b);

	switch (type_hi)
	{
		case SHAPE_SPHERE:
		{
			switch (type_lo)
			{
				case SHAPE_SPHERE: return COLLISION_BIN_SPHERE_VS_SPHERE;
			}
			break;
		}
		case SHAPE_BOX:
		{
			switch (type_lo)
			{
				case SHAPE_SPHERE: return COLLISION_BIN_BOX_VS_SPHERE;
				case SHAPE_BOX:    return COLLISION_BIN_BOX_VS_BOX;
			}
			break;
		}
		case SHAPE_LINE:
		{
			switch (type_lo)
			{
				case SHAPE_SPHERE: return COLLISION_BIN_LINE_VS_SPHERE;
				case SHAPE_BOX:    return COLLISION_BIN_LINE_VS_BOX;
				case SHAPE_LINE:   return COLLISION_BIN_LINE_VS_LINE;
			}
			break;
		}
		case SHAPE_TRIANGLE:
		{
			switch (type_lo)
			{
				case SHAPE_SPHERE:   return COLLISION_BIN_TRIANGLE_VS_SPHERE;
				case SHAPE_BOX:      return COLLISION_BIN_TRIANGLE_VS_BOX;
				case SHAPE_LINE:     return COLLISION_BIN_TRIANGLE_VS_LINE;
				case SHAPE_TRIANGLE: return COLLISION_BIN_TRIANGLE_VS_TRIANGLE;
			}
			break;
		}
		case SHAPE_POLYTOPE:
		{
			switch (type_lo)
			{
				case SHAPE_SPHERE:   return COLLISION_BIN_POLYTOPE_VS_SPHERE;
				case SHAPE_BOX:      return COLLISION_BIN_POLYTOPE_VS_BOX;
				case SHAPE_LINE:     return COLLISION_BIN_POLYTOPE_VS_LINE;
				case SHAPE_TRIANGLE: return COLLISION_BIN_POLYTOPE_VS_TRIANGLE;
				case SHAPE_POLYTOPE: return COLLISION_BIN_POLYTOPE_VS_POLYTOPE;
			}
			break;
		}
		default:
		{
			break;
		}
	}

	return COLLISION_BIN_COUNT;
}

// Return the pair in shape-type order without changing the original broadphase pair. This is used only for selecting the correct specialised
// helper call. The final contact still has to be flipped back to the original body order before it is stored.
void CanonicalisePair(
	in_(GpuShape) shape_a, float4x4 a2w,
	in_(GpuShape) shape_b, float4x4 b2w,
	out_(GpuShape) shape_lo, out_(float4x4) lo2w,
	out_(GpuShape) shape_hi, out_(float4x4) hi2w)
{
	if (shape_a.type > shape_b.type)
	{
		shape_lo = shape_b;
		lo2w = b2w;
		shape_hi = shape_a;
		hi2w = a2w;
	}
	else
	{
		shape_lo = shape_a;
		lo2w = a2w;
		shape_hi = shape_b;
		hi2w = b2w;
	}
}

// Load the broadphase pair and convert it into the object-space convention used by all narrow-phase helpers:
// shape A lives at identity, shape B is transformed by pair.b2a into A's space.
void LoadPair(uint pair_index, out_(GpuCollisionPair) pair, out_(GpuShape) shape_a, out_(GpuShape) shape_b, out_(float4x4) a2w, out_(float4x4) b2w)
{
	pair = g_pairs[pair_index];
	shape_a = g_shapes[pair.shape_idx_a];
	shape_b = g_shapes[pair.shape_idx_b];
	a2w = Identity4x4();
	b2w = pair.b2a;
}

// Append one resolve contact. The collision pass can discover contacts from multiple bins concurrently, so the contact slot is atomically allocated
// from the shared counter. Overflow increments the counter for readback, but writes are clipped to the configured capacity.
void StoreContact(in_(GpuCollisionPair) pair, in_(GpuShape) shape_a, in_(GpuShape) shape_b, in_(GpuContact) col)
{
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

// Append a resolver contact for actual hits.
void StoreCollisionResult(in_(GpuCollisionPair) pair, in_(GpuShape) shape_a, in_(GpuShape) shape_b, bool hit, in_(GpuContact) col)
{
	if (!hit) return;
	StoreContact(pair, shape_a, shape_b, col);
}

// Collision helpers that are named HigherVsLower return contacts in the helper's argument order. If the broadphase pair was LowerVsHigher, flip the
// generated manifold back so the stored GpuResolveContact always points from body_idx_a toward body_idx_b.
void StoreCanonicalResult(in_(GpuCollisionPair) pair, in_(GpuShape) shape_a, in_(GpuShape) shape_b, bool hit, inout_(GpuContact) col)
{
	if (shape_a.type < shape_b.type && hit)
		ContactFlip(col);

	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}

// Reset all per-bin state before classifying this frame's broadphase pairs.
// The dispatch args are reset too so empty bins naturally become zero-sized ExecuteIndirect calls.
numthreads(CSClearCollisionBins, COLLISION_BIN_COUNT, 1, 1)
void CSClearCollisionBins(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= COLLISION_BIN_COUNT)
		return;

	g_bin_counts[dtid.x] = 0;
	g_bin_dispatch_args[dtid.x].ThreadGroupCountX = 0;
	g_bin_dispatch_args[dtid.x].ThreadGroupCountY = 1;
	g_bin_dispatch_args[dtid.x].ThreadGroupCountZ = 1;
}

// Compact the flat broadphase pair stream into exact pair-type bins. Each bin row stores broadphase pair indices, not copied
// pair records, so the pair data remains single-source and the bin buffer is just COLLISION_BIN_COUNT * max_pair_count uints.
numthreads(CSBinCollisionPairs, CollideThreadCount, 1, 1)
void CSBinCollisionPairs(int3 dtid : SV_DispatchThreadID)
{
	uint pair_index = dtid.x;
	if (pair_index >= g_counters[0].pair_count || pair_index >= g.max_pair_count)
		return;

	GpuCollisionPair pair = g_pairs[pair_index];
	GpuShape shape_a = g_shapes[pair.shape_idx_a];
	GpuShape shape_b = g_shapes[pair.shape_idx_b];
	int bin_id = CollisionBinId(shape_a.type, shape_b.type);
	if (bin_id == COLLISION_BIN_COUNT)
		return;

	uint bin_slot;
	InterlockedAdd(g_bin_counts[bin_id], 1, bin_slot);
	if (bin_slot >= g.max_pair_count)
		return;

	g_bin_pair_indices[bin_id * g.max_pair_count + bin_slot] = pair_index;
}

// Convert each bin count into an indirect dispatch record. This keeps Engine::Step GPU-resident: the CPU records all
// exact-pair kernels every frame, and the GPU controls which bins do real work by writing zero or non-zero group counts.
numthreads(CSBuildCollisionBinDispatch, COLLISION_BIN_COUNT, 1, 1)
void CSBuildCollisionBinDispatch(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= COLLISION_BIN_COUNT)
		return;

	uint pair_count = min(g_bin_counts[dtid.x], g.max_pair_count);
	g_bin_dispatch_args[dtid.x].ThreadGroupCountX = (pair_count + CollideThreadCount - 1) / CollideThreadCount;
	g_bin_dispatch_args[dtid.x].ThreadGroupCountY = 1;
	g_bin_dispatch_args[dtid.x].ThreadGroupCountZ = 1;
}

// ---- Exact pair wrappers ----
// Each wrapper loads the original broadphase pair, calls one narrow-phase helper, then stores the result in the original A->B body order. The
// wrappers are intentionally repetitive: each one compiles to a tight shader entry point containing only the code needed for that exact pair type.
void CollidePairSphereVsSphere(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b;
	float4x4 a2w, b2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);

	GpuContact col;
	bool hit = SphereVsSphere(shape_a, a2w, shape_b, b2w, col);
	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairBoxVsSphere(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// In this bin, shape_hi is Box and shape_lo is Sphere.
	GpuContact col;
	bool hit = BoxVsSphere(shape_hi, hi2w, shape_lo, lo2w, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairBoxVsBox(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b;
	float4x4 a2w, b2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);

	GpuContact col;
	bool hit = BoxVsBox(shape_a, a2w, shape_b, b2w, col);
	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairLineVsSphere(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// In this bin, shape_hi is Line and shape_lo is Sphere.
	GpuContact col;
	bool hit = LineVsSphere(shape_hi, hi2w, shape_lo, lo2w, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairLineVsBox(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// In this bin, shape_hi is Line and shape_lo is Box.
	GpuContact col;
	bool hit = LineVsBox(shape_hi, hi2w, shape_lo, lo2w, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairLineVsLine(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b;
	float4x4 a2w, b2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);

	GpuContact col;
	bool hit = LineVsLine(shape_a, a2w, shape_b, b2w, col);
	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairTriangleVsSphere(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// Triangle/polytope vs sphere uses closest-point GJK with an analytic sphere margin instead of EPA against a curved Minkowski boundary.
	int gjk_iters = 0;
	GpuContact col;
	bool hit = ConvexVsSphere(shape_hi, hi2w, shape_lo, lo2w, g_verts, col, gjk_iters);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairTriangleVsBox(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	GpuContact col;
	bool hit = TriangleVsBox(shape_hi, hi2w, shape_lo, lo2w, g_verts, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairTriangleVsLine(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// In this bin, shape_hi is Triangle and shape_lo is Line.
	GpuContact col;
	bool hit = TriangleVsLine(shape_hi, hi2w, shape_lo, lo2w, g_verts, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairTriangleVsTriangle(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b;
	float4x4 a2w, b2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);

	GpuContact col;
	bool hit = TriangleVsTriangle(shape_a, a2w, shape_b, b2w, g_verts, col);
	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairPolytopeVsSphere(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// Triangle/polytope vs sphere uses closest-point GJK with an analytic sphere margin instead of EPA against a curved Minkowski boundary.
	int gjk_iters = 0;
	GpuContact col;
	bool hit = ConvexVsSphere(shape_hi, hi2w, shape_lo, lo2w, g_verts, col, gjk_iters);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairPolytopeVsBox(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	GpuContact col;
	bool hit = PolytopeVsBox(shape_hi, hi2w, shape_lo, lo2w, g_verts, g_faces, g_edges, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairPolytopeVsLine(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	// Polytope vs line uses closest-point GJK against the capsule core, then adds the line radius analytically.
	int gjk_iters = 0;
	GpuContact col;
	bool hit = ConvexVsLine(shape_hi, hi2w, shape_lo, lo2w, g_verts, col, gjk_iters);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairPolytopeVsTriangle(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b, shape_lo, shape_hi;
	float4x4 a2w, b2w, lo2w, hi2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);
	CanonicalisePair(shape_a, a2w, shape_b, b2w, shape_lo, lo2w, shape_hi, hi2w);

	GpuContact col;
	bool hit = PolytopeVsTriangle(shape_hi, hi2w, shape_lo, lo2w, g_verts, g_faces, g_edges, col);
	StoreCanonicalResult(pair, shape_a, shape_b, hit, col);
}
void CollidePairPolytopeVsPolytope(uint pair_index)
{
	GpuCollisionPair pair;
	GpuShape shape_a, shape_b;
	float4x4 a2w, b2w;
	LoadPair(pair_index, pair, shape_a, shape_b, a2w, b2w);

	GpuContact col;

	// Same-type bins keep the original broadphase ordering. PolytopeVsPolytope returns the axis from its first shape toward its second, so reversing the
	// call here would produce a B->A axis while the stored contact still uses body_idx_a/body_idx_b.
	bool hit = PolytopeVsPolytope(shape_a, a2w, shape_b, b2w, g_verts, g_faces, g_edges, col);
	StoreCollisionResult(pair, shape_a, shape_b, hit, col);
}

// The entry point macro is only the dispatch shell. All real pair-specific work lives in the CollidePair* wrappers above, keeping the bin kernels
// easy to map back to COLLISION_BIN_* ids while avoiding a second layer of switch/branching inside the hot path.
#define COLLISION_BIN_ENTRY(kernel_name, bin_id, collide_func) \
	numthreads(kernel_name, CollideThreadCount, 1, 1) \
	void kernel_name(int3 dtid : SV_DispatchThreadID) \
	{ \
		if (dtid.x >= g_bin_counts[bin_id] || dtid.x >= g.max_pair_count) return; \
		uint pair_index = g_bin_pair_indices[bin_id * g.max_pair_count + dtid.x]; \
		collide_func(pair_index); \
	}

COLLISION_BIN_ENTRY(CSCollideSphereVsSphere,     COLLISION_BIN_SPHERE_VS_SPHERE,     CollidePairSphereVsSphere)
COLLISION_BIN_ENTRY(CSCollideBoxVsSphere,        COLLISION_BIN_BOX_VS_SPHERE,        CollidePairBoxVsSphere)
COLLISION_BIN_ENTRY(CSCollideBoxVsBox,           COLLISION_BIN_BOX_VS_BOX,           CollidePairBoxVsBox)
COLLISION_BIN_ENTRY(CSCollideLineVsSphere,       COLLISION_BIN_LINE_VS_SPHERE,       CollidePairLineVsSphere)
COLLISION_BIN_ENTRY(CSCollideLineVsBox,          COLLISION_BIN_LINE_VS_BOX,          CollidePairLineVsBox)
COLLISION_BIN_ENTRY(CSCollideLineVsLine,         COLLISION_BIN_LINE_VS_LINE,         CollidePairLineVsLine)
COLLISION_BIN_ENTRY(CSCollideTriangleVsSphere,   COLLISION_BIN_TRIANGLE_VS_SPHERE,   CollidePairTriangleVsSphere)
COLLISION_BIN_ENTRY(CSCollideTriangleVsBox,      COLLISION_BIN_TRIANGLE_VS_BOX,      CollidePairTriangleVsBox)
COLLISION_BIN_ENTRY(CSCollideTriangleVsLine,     COLLISION_BIN_TRIANGLE_VS_LINE,     CollidePairTriangleVsLine)
COLLISION_BIN_ENTRY(CSCollideTriangleVsTriangle, COLLISION_BIN_TRIANGLE_VS_TRIANGLE, CollidePairTriangleVsTriangle)
COLLISION_BIN_ENTRY(CSCollidePolytopeVsSphere,   COLLISION_BIN_POLYTOPE_VS_SPHERE,   CollidePairPolytopeVsSphere)
COLLISION_BIN_ENTRY(CSCollidePolytopeVsBox,      COLLISION_BIN_POLYTOPE_VS_BOX,      CollidePairPolytopeVsBox)
COLLISION_BIN_ENTRY(CSCollidePolytopeVsLine,     COLLISION_BIN_POLYTOPE_VS_LINE,     CollidePairPolytopeVsLine)
COLLISION_BIN_ENTRY(CSCollidePolytopeVsTriangle, COLLISION_BIN_POLYTOPE_VS_TRIANGLE, CollidePairPolytopeVsTriangle)
COLLISION_BIN_ENTRY(CSCollidePolytopeVsPolytope, COLLISION_BIN_POLYTOPE_VS_POLYTOPE, CollidePairPolytopeVsPolytope)
#undef COLLISION_BIN_ENTRY

// Calculates the number of thread groups needed for the resolve shader
// based on the number of contacts found, and writes to the dispatch arguments buffer.
numthreads(CSCalcResolveDispatch, 1, 1, 1)
void CSCalcResolveDispatch(int3 dtid : SV_DispatchThreadID)
{
	// Ensure that there is always at least one thread group dispatched in the resolve shader,
	// so that it always resets the 'colour_used' and contact times.
	int count = min(g_counters[0].contact_count, g.max_contacts);
	g_dispatch_args[0].ThreadGroupCountX = max(1, (count + ResolveThreadCount - 1) / ResolveThreadCount);
	g_dispatch_args[0].ThreadGroupCountY = 1;
	g_dispatch_args[0].ThreadGroupCountZ = 1;
}

#ifdef __cplusplus
}
#endif
