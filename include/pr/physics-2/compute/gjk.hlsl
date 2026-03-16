//*********************************************
// Physics Engine — GJK + EPA Compute Shader
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU narrow phase collision detection using the Gilbert-Johnson-Keerthi (GJK)
// algorithm with Expanding Polytope Algorithm (EPA) for penetration depth.
//
// One thread per broadphase pair. Each thread runs GJK to determine if two shapes
// overlap, then EPA to find the contact normal, depth, and point.
//
// Inputs:
//   g_shapes  — StructuredBuffer of GpuShape (all unique shapes in the scene)
//   g_pairs   — StructuredBuffer of GpuCollisionPair (broadphase overlap pairs)
//   g_verts   — StructuredBuffer of float4 (shared vertex buffer for Triangle/Polytope shapes)
//
// Outputs:
//   g_contacts — RWStructuredBuffer of GpuContact (collision results, written atomically)
//   g_counters — RWStructuredBuffer of uint (atomic contact count at index 0)
//
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"

// ---- Constants ----
static const int MaxGjkIter = 32;
static const int MaxEpaVerts = 24;
static const int MaxEpaFaces = 48;
static const int MaxEpaEdges = 96;
static const float GjkEps = 1e-8f;
static const float EpaEps = 1e-6f;

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

// ---- Vector helpers ----

float4 Normalise3(float4 v)
{
	float len = length(v.xyz);
	return len > GjkEps ? v / len : float4(1, 0, 0, 0);
}

// Transform a point by a row_major float4x4 (point has w=1)
float4 TransformPoint(float4x4 m, float4 p)
{
	return mul(float4(p.xyz, 1), m);
}

// Transform a direction by a row_major float4x4 (direction has w=0)
float4 TransformDir(float4x4 m, float4 d)
{
	return float4(mul(float4(d.xyz, 0), m).xyz, 0);
}

// ---- Support vertex functions ----
// Each function returns the furthest point on the shape boundary in the given direction.
// Direction is in shape-local space (pre-transformed by inverse of s2p).
float4 SupportVertex_Sphere(GpuShape shape, float4 dir)
{
	// Sphere support: centre + radius * normalised direction
	float4 centre = shape.s2p[3]; // position = last row of row_major matrix
	float radius = shape.data.x;
	return centre + radius * Normalise3(dir);
}

float4 SupportVertex_Box(GpuShape shape, float4 dir)
{
	// Box support: project direction onto each axis, pick sign, scale by half-extent
	float3 half_ext = shape.data.xyz;
	float4 result = shape.s2p[3]; // start at centre (position)

	// Each row of the row_major s2p matrix is a basis vector
	float4 ax = float4(shape.s2p[0].xyz, 0);
	float4 ay = float4(shape.s2p[1].xyz, 0);
	float4 az = float4(shape.s2p[2].xyz, 0);

	result += (dot(dir.xyz, ax.xyz) > 0 ? half_ext.x : -half_ext.x) * ax;
	result += (dot(dir.xyz, ay.xyz) > 0 ? half_ext.y : -half_ext.y) * ay;
	result += (dot(dir.xyz, az.xyz) > 0 ? half_ext.z : -half_ext.z) * az;
	return result;
}

float4 SupportVertex_Line(GpuShape shape, float4 dir)
{
	// Line support: endpoint in direction, plus hemispherical thickness offset
	float half_len = shape.data.x;
	float thickness = shape.data.y;
	float4 centre = shape.s2p[3];
	float4 z_axis = float4(shape.s2p[2].xyz, 0); // line direction (Z axis of s2p)

	float d = dot(dir.xyz, z_axis.xyz);
	float4 result = centre;
	result += (d > 0 ? half_len : -half_len) * z_axis;

	// Hemispherical end-cap offset for thick lines
	if (thickness > 0)
	{
		float len_sq = length_sq(dir.xyz);
		if (len_sq > GjkEps * GjkEps)
			result += thickness * dir / sqrt(len_sq);
	}
	return result;
}

float4 SupportVertex_Triangle(GpuShape shape, float4 dir, StructuredBuffer<float4> verts)
{
	// Triangle support: return the vertex with maximum projection onto direction.
	// Vertices are stored as offsets (w=0) in the shared vertex buffer.
	float4 v0 = verts[shape.vert_offset + 0];
	float4 v1 = verts[shape.vert_offset + 1];
	float4 v2 = verts[shape.vert_offset + 2];

	// Transform vertices by s2p (they're offsets, so transform as points with w=1)
	float4 p0 = TransformPoint(shape.s2p, v0);
	float4 p1 = TransformPoint(shape.s2p, v1);
	float4 p2 = TransformPoint(shape.s2p, v2);

	float d0 = dot(dir.xyz, p0.xyz);
	float d1 = dot(dir.xyz, p1.xyz);
	float d2 = dot(dir.xyz, p2.xyz);

	if (d0 >= d1 && d0 >= d2) return p0;
	if (d1 >= d0 && d1 >= d2) return p1;
	return p2;
}

float4 SupportVertex_Polytope(GpuShape shape, float4 dir, StructuredBuffer<float4> verts)
{
	// Polytope support: brute-force linear scan over all vertices.
	// No adjacency data needed — GPU-friendly parallel scan.
	float best_dot = -1e30f;
	float4 best_vert = float4(0, 0, 0, 1);

	for (int i = 0; i < shape.vert_count; ++i)
	{
		float4 v = TransformPoint(shape.s2p, verts[shape.vert_offset + i]);
		float d = dot(dir.xyz, v.xyz);
		if (d > best_dot)
		{
			best_dot = d;
			best_vert = v;
		}
	}
	return best_vert;
}

// Unified support vertex dispatcher
float4 SupportVertex(GpuShape shape, float4 dir, StructuredBuffer<float4> verts)
{
	switch (shape.type)
	{
	case SHAPE_SPHERE:   return SupportVertex_Sphere(shape, dir);
	case SHAPE_BOX:      return SupportVertex_Box(shape, dir);
	case SHAPE_LINE:     return SupportVertex_Line(shape, dir);
	case SHAPE_TRIANGLE: return SupportVertex_Triangle(shape, dir, verts);
	case SHAPE_POLYTOPE: return SupportVertex_Polytope(shape, dir, verts);
	default:             return float4(0, 0, 0, 1);
	}
}

// ---- Minkowski difference support ----
// Tracks both the difference point and the original shape vertices for EPA contact extraction.
struct MkSup
{
	float4 w; // Minkowski difference point (a - b), w=0
	float4 a; // Support vertex on shape A (world space), w=1
	float4 b; // Support vertex on shape B (world space), w=1
};

MkSup MkSupport(
	GpuShape shape_a, float4x4 a2w, float4x4 w2a,
	GpuShape shape_b, float4x4 b2w, float4x4 w2b,
	float4 dir, StructuredBuffer<float4> verts)
{
	// Support on A in direction +dir, support on B in direction -dir
	float4 dir_a = TransformDir(w2a, +dir);
	float4 dir_b = TransformDir(w2b, -dir);

	float4 va = TransformPoint(a2w, SupportVertex(shape_a, dir_a, verts));
	float4 vb = TransformPoint(b2w, SupportVertex(shape_b, dir_b, verts));
	
	// This is not a fix
	//// Support on A in direction +dir, support on B in direction -dir.
	//// Support functions return in world space (they apply s2p internally),
	//// so no additional transform is needed here.
	//float4 va = SupportVertex(shape_a, +dir, verts);
	//float4 vb = SupportVertex(shape_b, -dir, verts);

	MkSup s;
	s.w = float4((va - vb).xyz, 0);
	s.a = float4(va.xyz, 1);
	s.b = float4(vb.xyz, 1);
	return s;
}

// ---- GJK Simplex ----
// Up to 4 support points. Newest point is always at index 0.
struct Simplex
{
	MkSup s[4];
	int n;
};

void SimplexPush(inout Simplex sx, MkSup p)
{
	for (int i = sx.n; i > 0; --i)
		sx.s[i] = sx.s[i - 1];

	sx.s[0] = p;
	sx.n++;
}

// ---- Simplex reduction cases ----
// Each function updates the simplex to the closest feature to the origin,
// sets the new search direction, and returns true if the origin is enclosed.
bool SimplexLine(inout Simplex sx, inout float4 dir)
{
	// A = newest (sx.s[0]), B = previous (sx.s[1])
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;

	if (dot(ab.xyz, ao.xyz) > 0)
	{
		// Origin projects onto the segment interior — search perpendicular
		dir = float4(cross(cross(ab.xyz, ao.xyz), ab.xyz), 0);
		if (length_sq(dir.xyz) < GjkEps)
			dir = float4(Perpendicular(ab.xyz), 0);
	}
	else
	{
		// Origin is past A — reduce to just point A
		sx.n = 1;
		dir = ao;
	}
	return false;
}

bool SimplexTri(inout Simplex sx, inout float4 dir)
{
	// A = newest (sx.s[0]), B = sx.s[1], C = sx.s[2]
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ac = sx.s[2].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;
	float4 n = float4(cross(ab.xyz, ac.xyz), 0); // triangle normal

	// Test Voronoi regions of the triangle edges
	if (dot(cross(n.xyz, ac.xyz), ao.xyz) > 0)
	{
		// Origin is outside edge AC
		if (dot(ac.xyz, ao.xyz) > 0)
		{
			// Closest to edge AC
			sx.s[1] = sx.s[2];
			sx.n = 2;
			dir = float4(cross(cross(ac.xyz, ao.xyz), ac.xyz), 0);
			if (length_sq(dir.xyz) < GjkEps)
				dir = float4(Perpendicular(ac.xyz), 0);
		}
		else
		{
			// Fall through to AB edge check
			sx.n = 2;
			return SimplexLine(sx, dir);
		}
	}
	else if (dot(cross(ab.xyz, n.xyz), ao.xyz) > 0)
	{
		// Origin is outside edge AB
		sx.n = 2;
		return SimplexLine(sx, dir);
	}
	else
	{
		// Origin is inside the triangle prism — pick the correct face
		if (dot(n.xyz, ao.xyz) > 0)
		{
			dir = n; // above the triangle
		}
		else
		{
			// Flip winding
			MkSup tmp = sx.s[1];
			sx.s[1] = sx.s[2];
			sx.s[2] = tmp;
			dir = -n; // below the triangle
		}
	}
	return false;
}

bool SimplexTetra(inout Simplex sx, inout float4 dir)
{
	// A = newest (sx.s[0]), B = sx.s[1], C = sx.s[2], D = sx.s[3]
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ac = sx.s[2].w - sx.s[0].w;
	float4 ad = sx.s[3].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;

	// Face normals, oriented outward (away from the opposite vertex)
	float4 abc = float4(cross(ab.xyz, ac.xyz), 0); if (dot(abc.xyz, ad.xyz) > 0) abc = -abc;
	float4 acd = float4(cross(ac.xyz, ad.xyz), 0); if (dot(acd.xyz, ab.xyz) > 0) acd = -acd;
	float4 adb = float4(cross(ad.xyz, ab.xyz), 0); if (dot(adb.xyz, ac.xyz) > 0) adb = -adb;

	// Check if origin is outside any face
	if (dot(abc.xyz, ao.xyz) > 0)
	{
		sx.n = 3; // reduce to triangle ABC
		return SimplexTri(sx, dir);
	}
	if (dot(acd.xyz, ao.xyz) > 0)
	{
		sx.s[1] = sx.s[2]; sx.s[2] = sx.s[3]; sx.n = 3; // triangle ACD
		return SimplexTri(sx, dir);
	}
	if (dot(adb.xyz, ao.xyz) > 0)
	{
		MkSup tmp = sx.s[1]; sx.s[1] = sx.s[3]; sx.s[2] = tmp; sx.n = 3; // triangle ADB
		return SimplexTri(sx, dir);
	}

	// Origin is inside the tetrahedron
	return true;
}

bool DoSimplex(inout Simplex sx, inout float4 dir)
{
	switch (sx.n)
	{
	case 2: return SimplexLine(sx, dir);
	case 3: return SimplexTri(sx, dir);
	case 4: return SimplexTetra(sx, dir);
	}
	return false;
}

// ---- EPA (Expanding Polytope Algorithm) ----
// Finds penetration depth, normal, and contact point from a GJK simplex
// that contains the origin.

struct EpaFace
{
	int i0, i1, i2; // vertex indices
	float4 normal;  // outward-facing unit normal
	float dist;     // distance from origin to face plane
};

struct EpaEdge
{
	int a, b;
};

bool Epa(
	GpuShape shape_a, float4x4 a2w, float4x4 w2a,
	GpuShape shape_b, float4x4 b2w, float4x4 w2b,
	Simplex gjk_sx, StructuredBuffer<float4> verts,
	out float4 out_normal, out float out_depth, out float4 out_ptA, out float4 out_ptB)
{
	out_normal = float4(0, 0, 0, 0);
	out_depth = 0;
	out_ptA = float4(0, 0, 0, 1);
	out_ptB = float4(0, 0, 0, 1);

	if (gjk_sx.n < 4) return false;

	// EPA vertex and face buffers (per-thread, in registers/local memory)
	MkSup epa_verts[MaxEpaVerts];
	EpaFace epa_faces[MaxEpaFaces];
	int nv = 4, nf = 0;

	for (int i = 0; i < 4; ++i)
		epa_verts[i] = gjk_sx.s[i];

	// Ensure consistent tetrahedron winding
	float4 n012 = float4(cross(epa_verts[1].w.xyz - epa_verts[0].w.xyz, epa_verts[2].w.xyz - epa_verts[0].w.xyz), 0);
	if (dot(n012.xyz, (epa_verts[3].w - epa_verts[0].w).xyz) > 0)
	{
		MkSup tmp = epa_verts[0];
		epa_verts[0] = epa_verts[1];
		epa_verts[1] = tmp;
	}

	// Helper: add a face with validated outward normal.
	// Returns the index of the new face, or -1 on failure.
	// (Inlined as a macro-like pattern since HLSL doesn't support nested function refs to arrays)
	// We'll use a flat add_face sequence instead.

	// Build initial tetrahedron faces (4 faces)
	// Indices: {0,1,2}, {0,3,1}, {0,2,3}, {1,3,2}
	{
		int fi_a[4] = {0, 0, 0, 1};
		int fi_b[4] = {1, 3, 2, 3};
		int fi_c[4] = {2, 1, 3, 2};
		for (int fi = 0; fi < 4; ++fi)
		{
			int ia = fi_a[fi];
			int ib = fi_b[fi];
			int ic = fi_c[fi];

		float4 fab = epa_verts[ib].w - epa_verts[ia].w;
		float4 fac = epa_verts[ic].w - epa_verts[ia].w;
		float4 fn = float4(cross(fab.xyz, fac.xyz), 0);
		float flen = length(fn.xyz);
		if (flen < GjkEps) continue;
		fn /= flen;
		float fd = dot(fn.xyz, epa_verts[ia].w.xyz);

		// Ensure normal points outward (away from origin)
		if (fd < 0)
		{
			fn = -fn;
			fd = -fd;
			// Swap b and c
			int tmp_i = ib; ib = ic; ic = tmp_i;
		}

		epa_faces[nf].i0 = ia;
		epa_faces[nf].i1 = ib;
		epa_faces[nf].i2 = ic;
		epa_faces[nf].normal = fn;
		epa_faces[nf].dist = fd;
		nf++;
		}
	}

	// EPA main loop
	for (int iter = 0; iter < MaxGjkIter; ++iter)
	{
		// Find the face closest to the origin
		int ci = 0;
		for (int i = 1; i < nf; ++i)
		{
			if (epa_faces[i].dist < epa_faces[ci].dist)
				ci = i;
		}

		float4 cf_normal = epa_faces[ci].normal;
		float cf_dist = epa_faces[ci].dist;
		int cf_i0 = epa_faces[ci].i0;
		int cf_i1 = epa_faces[ci].i1;
		int cf_i2 = epa_faces[ci].i2;

		// Get new support in the closest face's normal direction
		MkSup sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, cf_normal, verts);
		float d = dot(sup.w.xyz, cf_normal.xyz);

		// Convergence: new support doesn't extend the polytope significantly
		if (d - cf_dist < EpaEps || nv >= MaxEpaVerts)
		{
			out_normal = cf_normal;
			out_depth = cf_dist;

			// Barycentric interpolation on the closest face to find contact points
			MkSup va = epa_verts[cf_i0];
			MkSup vb = epa_verts[cf_i1];
			MkSup vc = epa_verts[cf_i2];
			float4 proj = cf_dist * cf_normal; // closest point on face to origin
			float4 e0 = vb.w - va.w;
			float4 e1 = vc.w - va.w;
			float4 e2 = proj - va.w;
			float d00 = dot(e0.xyz, e0.xyz), d01 = dot(e0.xyz, e1.xyz), d11 = dot(e1.xyz, e1.xyz);
			float d20 = dot(e2.xyz, e0.xyz), d21 = dot(e2.xyz, e1.xyz);
			float denom = d00 * d11 - d01 * d01;

			if (abs(denom) > GjkEps)
			{
				float u = (d11 * d20 - d01 * d21) / denom;
				float v = (d00 * d21 - d01 * d20) / denom;
				float w = 1.0f - u - v;
				out_ptA = w * va.a + u * vb.a + v * vc.a;
				out_ptB = w * va.b + u * vb.b + v * vc.b;
			}
			else
			{
				out_ptA = va.a;
				out_ptB = va.b;
			}
			return true;
		}

		// Add new vertex
		epa_verts[nv] = sup;
		int ni = nv++;

		// Remove faces visible from the new point, collecting horizon edges
		EpaEdge edges[MaxEpaEdges];
		int ne = 0;

		for (int i = nf - 1; i >= 0; --i)
		{
			float4 face_to_new = sup.w - epa_verts[epa_faces[i].i0].w;
			if (dot(epa_faces[i].normal.xyz, face_to_new.xyz) <= 0)
				continue;

			// Face is visible — collect its 3 edges
			int fi_arr[3] = { epa_faces[i].i0, epa_faces[i].i1, epa_faces[i].i2 };
			for (int j = 0; j < 3; ++j)
			{
				int ea = fi_arr[j];
				int eb = fi_arr[(j + 1) % 3];

				// Check if the reverse edge already exists (interior edge)
				bool is_shared = false;
				for (int k = ne - 1; k >= 0; --k)
				{
					if (edges[k].a == eb && edges[k].b == ea)
					{
						edges[k] = edges[--ne]; // cancel interior edge
						is_shared = true;
						break;
					}
				}
				if (!is_shared && ne < MaxEpaEdges)
				{
					edges[ne].a = ea;
					edges[ne].b = eb;
					ne++;
				}
			}

			// Remove the visible face (swap with last)
			epa_faces[i] = epa_faces[--nf];
		}

		// Create new faces from horizon edges to the new vertex
		for (int i = 0; i < ne && nf < MaxEpaFaces; ++i)
		{
			int ia = edges[i].a;
			int ib = edges[i].b;
			int ic = ni;

			float4 fab = epa_verts[ib].w - epa_verts[ia].w;
			float4 fac = epa_verts[ic].w - epa_verts[ia].w;
			float4 fn = float4(cross(fab.xyz, fac.xyz), 0);
			float flen = length(fn.xyz);
			if (flen < GjkEps) continue;
			fn /= flen;
			float fd = dot(fn.xyz, epa_verts[ia].w.xyz);

			if (fd < 0)
			{
				fn = -fn;
				fd = -fd;
				int tmp_i = ib; ib = ic; ic = tmp_i;
			}

			epa_faces[nf].i0 = ia;
			epa_faces[nf].i1 = ib;
			epa_faces[nf].i2 = ic;
			epa_faces[nf].normal = fn;
			epa_faces[nf].dist = fd;
			nf++;
		}
	}

	return false; // EPA did not converge
}

// ---- GJK + EPA entry point ----
// Returns true if shapes collide, filling out contact data.
bool GjkCollide(
	GpuShape shape_a, float4x4 a2w,
	GpuShape shape_b, float4x4 b2w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	// Compute shape-to-world and world-to-shape transforms.
	// For the GPU path, shapes are already positioned (s2p baked into a2w/b2w by caller),
	// so the shape's s2p is baked into a2w/b2w.
	float4x4 w2a = InvertOrthonormal(a2w);
	float4x4 w2b = InvertOrthonormal(b2w);

	// Initial search direction: from B's centre toward A's centre
	float4 centre_a = a2w[3]; // position row
	float4 centre_b = b2w[3];
	float4 dir = float4((centre_a - centre_b).xyz, 0);
	if (length_sq(dir.xyz) < GjkEps)
		dir = float4(1, 0, 0, 0);

	// Seed the simplex with the first support point
	Simplex sx;
	sx.n = 0;
	MkSup sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, dir, verts);
	SimplexPush(sx, sup);
	dir = -sup.w;

	// GJK main loop
	for (int iter = 0; iter < MaxGjkIter; ++iter)
	{
		if (length_sq(dir.xyz) < GjkEps)
			break;

		sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, dir, verts);

		// If the new support point doesn't pass the origin, shapes are separated
		if (dot(sup.w.xyz, dir.xyz) < 0)
			return false;

		SimplexPush(sx, sup);

		if (DoSimplex(sx, dir))
		{
			// Origin enclosed — shapes overlap. Run EPA for penetration info.
			float4 normal, ptA, ptB;
			float depth;
			if (!Epa(shape_a, a2w, w2a, shape_b, b2w, w2b, sx, verts, normal, depth, ptA, ptB))
				return false;

			// Orient axis from A toward B (convention: axis points A→B)
			float pa = dot(normal.xyz, centre_a.xyz);
			float pb = dot(normal.xyz, centre_b.xyz);
			float sign = (pa < pb) ? 1.0f : -1.0f;
			out_axis = sign * normal;
			out_depth = depth;
			out_point = float4(((ptA + ptB) * 0.5f).xyz, 1);
			return true;
		}
	}

	return false; // GJK did not converge
}

// ---- Compute shader entry point ----
// One thread per broadphase pair. Tests collision via GJK + EPA.
// Colliding pairs write their contact to g_contacts atomically.
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
	float4x4 a2w = shape_a.s2p; // A is in its own local space
	float4x4 b2w = mul(shape_b.s2p, pair.b2a); // B transformed into A's space

	// Run GJK + EPA
	float4 col_axis;
	float4 col_point;
	float depth;
	if (!GjkCollide(shape_a, a2w, shape_b, b2w, g_verts, col_axis, col_point, depth))
		return;

	// Allocate a slot in the contact buffer atomically.
	// Bounds check prevents writing past the buffer if more pairs collide than expected.
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
