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

// Keep analytic/simple pairs and generic GJK/EPA pairs in separate entry points. The monolithic dispatcher creates very large
// bytecode, and untaken generic paths can still affect execution of simple box/box-style pairs on some drivers.
bool IsGenericCollisionPair(int type_a, int type_b)
{
	int type_lo = min(type_a, type_b);
	int type_hi = max(type_a, type_b);
	switch (type_hi)
	{
		case SHAPE_TRIANGLE:
		{
			return type_lo == SHAPE_SPHERE || type_lo == SHAPE_BOX;
		}
		case SHAPE_POLYTOPE:
		{
			return
				type_lo == SHAPE_SPHERE ||
				type_lo == SHAPE_BOX ||
				type_lo == SHAPE_LINE ||
				type_lo == SHAPE_TRIANGLE ||
				type_lo == SHAPE_POLYTOPE;
		}
		default:
		{
			return false;
		}
	}
}

bool CollideShapesSimple(
	in_(GpuShape) a, float4x4 a2w,
	in_(GpuShape) b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	GpuShape sa, sb;
	float4x4 wa, wb;
	bool swapped = a.type > b.type;
	if (swapped) { sa = b; sb = a; wa = b2w; wb = a2w; }
	else         { sa = a; sb = b; wa = a2w; wb = b2w; }

	bool hit = false;
	switch (sb.type)
	{
		case SHAPE_SPHERE:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE: hit = SphereVsSphere(sa, wa, sb, wb, out_contact); break;
			}
			break;
		}
		case SHAPE_BOX:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE: hit = BoxVsSphere(sb, wb, sa, wa, out_contact); break;
				case SHAPE_BOX:    hit = BoxVsBox(sa, wa, sb, wb, out_contact); break;
			}
			break;
		}
		case SHAPE_LINE:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE: hit = LineVsSphere(sb, wb, sa, wa, out_contact); break;
				case SHAPE_BOX:    hit = LineVsBox(sb, wb, sa, wa, out_contact); break;
				case SHAPE_LINE:   hit = LineVsLine(sa, wa, sb, wb, out_contact); break;
			}
			break;
		}
		case SHAPE_TRIANGLE:
		{
			switch (sa.type)
			{
				case SHAPE_LINE:     hit = TriangleVsLine(sb, wb, sa, wa, verts, out_contact); break;
				case SHAPE_TRIANGLE: hit = TriangleVsTriangle(sa, wa, sb, wb, verts, out_contact); break;
			}
			break;
		}
	}

	if (a.type < b.type && hit)
		ContactFlip(out_contact);

	return hit;
}

bool CollideShapesGeneric(
	in_(GpuShape) a, float4x4 a2w,
	in_(GpuShape) b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	GpuShape sa, sb;
	float4x4 wa, wb;
	bool swapped = a.type > b.type;
	if (swapped) { sa = b; sb = a; wa = b2w; wb = a2w; }
	else         { sa = a; sb = b; wa = a2w; wb = b2w; }

	bool need_gjk = false;
	bool need_cv_sphere = false;
	bool need_cv_line = false;
	switch (sb.type)
	{
		case SHAPE_TRIANGLE:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE: need_cv_sphere = true; break;
				case SHAPE_BOX:    need_gjk = true; break;
			}
			break;
		}
		case SHAPE_POLYTOPE:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE:   need_cv_sphere = true; break;
				case SHAPE_BOX:      need_gjk = true; break;
				case SHAPE_LINE:     need_cv_line = true; break;
				case SHAPE_TRIANGLE: need_gjk = true; break;
				case SHAPE_POLYTOPE: need_gjk = true; break;
			}
			break;
		}
	}

	int gjk_iters = 0;
	int epa_iters = 0;
	bool hit = false;
	if (need_gjk)
		hit = GjkCollide(sb, wb, sa, wa, verts, out_contact, gjk_iters, epa_iters);
	else if (need_cv_sphere)
		hit = ConvexVsSphere(sb, wb, sa, wa, verts, out_contact, gjk_iters);
	else if (need_cv_line)
		hit = ConvexVsLine(sb, wb, sa, wa, verts, out_contact, gjk_iters);

	if (a.type < b.type && hit)
		ContactFlip(out_contact);

	return hit;
}

#if PR_COLLISION_DIAGNOSTICS
void StoreDiag(uint pair_index, in_(GpuCollisionPair) pair, in_(GpuShape) shape_a, in_(GpuShape) shape_b, bool hit)
{
	GpuPairDiag diag;
	diag.body_idx_a = pair.body_idx_a;
	diag.body_idx_b = pair.body_idx_b;
	diag.shape_type_a = shape_a.type;
	diag.shape_type_b = shape_b.type;
	diag.hit = hit ? 1 : 0;
	diag.pad0 = 0;
	g_diag[pair_index] = diag;
}
#endif

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

numthreads(CSCollideSimple, CollideThreadCount, 1, 1)
void CSCollideSimple(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= g_counters[0].pair_count)
		return;

	GpuCollisionPair pair = g_pairs[dtid.x];
	GpuShape shape_a = g_shapes[pair.shape_idx_a];
	GpuShape shape_b = g_shapes[pair.shape_idx_b];
	if (IsGenericCollisionPair(shape_a.type, shape_b.type))
		return;

	float4x4 a2w = Identity4x4();
	float4x4 b2w = pair.b2a;

	GpuContact col;
	bool hit = CollideShapesSimple(shape_a, a2w, shape_b, b2w, g_verts, col);

#if PR_COLLISION_DIAGNOSTICS
	StoreDiag(dtid.x, pair, shape_a, shape_b, hit);
#endif

	if (!hit)
		return;

	StoreContact(pair, shape_a, shape_b, col);
}

numthreads(CSCollideGeneric, CollideThreadCount, 1, 1)
void CSCollideGeneric(int3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= g_counters[0].pair_count)
		return;

	GpuCollisionPair pair = g_pairs[dtid.x];
	GpuShape shape_a = g_shapes[pair.shape_idx_a];
	GpuShape shape_b = g_shapes[pair.shape_idx_b];
	if (!IsGenericCollisionPair(shape_a.type, shape_b.type))
		return;

	float4x4 a2w = Identity4x4();
	float4x4 b2w = pair.b2a;

	GpuContact col;
	bool hit = CollideShapesGeneric(shape_a, a2w, shape_b, b2w, g_verts, col);
#if PR_COLLISION_DIAGNOSTICS
	StoreDiag(dtid.x, pair, shape_a, shape_b, hit);
#endif

	if (!hit)
		return;

	StoreContact(pair, shape_a, shape_b, col);
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
