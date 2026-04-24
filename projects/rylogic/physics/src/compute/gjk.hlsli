//*********************************************
// Physics Engine — GJK + EPA Algorithm
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Gilbert-Johnson-Keerthi (GJK) distance/overlap algorithm with
// Expanding Polytope Algorithm (EPA) for penetration depth extraction.
//
// WARNING — EPA limitations with implicit curved surfaces:
//   EPA approximates the Minkowski boundary with a convex polytope that is
//   iteratively expanded. When one or both shapes are implicitly curved (sphere,
//   thick line), the Minkowski difference boundary is also curved. EPA cannot
//   represent this curved surface accurately with a limited number of vertices
//   and faces, leading to:
//     - Inaccurate contact normals (off by many degrees for sphere-sphere)
//     - Poor convergence (hitting iteration limits, causing GPU TDR)
//     - Energy drift in collision response
//
//   For pairs involving spheres or thick lines, use specialised analytic tests
//   (see collision.hlsli) or the "GJK with margins" technique: run GJK on the
//   core convex shape vs the sphere centre (a point), then add the radius
//   analytically. This avoids EPA entirely and gives exact normals.
//
//   GJK + EPA is safe for purely polyhedral pairs (box-box, box-polytope,
//   polytope-polytope) where the Minkowski boundary is also polyhedral.
//
// Matrix convention: same as integrate.hlsl (row-vector / DirectX-style).
//   HLSL 'row_major float4x4' rows = C++ columns = basis vectors.
//   mul(v, M) for vector transforms, mul(A, B) = A then B in row-vector convention.
#ifndef PR_PHYSICS_GJK_HLSLI
#define PR_PHYSICS_GJK_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// ---- Constants ----
static const int MaxGjkIter = 32;
static const int MaxEpaVerts = 24;
static const int MaxEpaFaces = 48;
static const int MaxEpaEdges = 96;
static const float GjkEps = 1e-8f;
static const float EpaEps = 1e-6f;

// ---- Support vertex functions ----
// Each function returns the furthest point on the shape boundary in the given direction.
inline float4 SupportVertex_Sphere(in_(GpuShape) shape, float4 dir)
{
	float4 centre = shape.s2rb[3];
	float radius = shape.data.x;
	return centre + radius * normalize(dir);
}
inline float4 SupportVertex_Box(in_(GpuShape) shape, float4 dir)
{
	float3 half_ext = shape.data.xyz;
	float4 result = shape.s2rb[3];
	float4 ax = float4(shape.s2rb[0].xyz, 0);
	float4 ay = float4(shape.s2rb[1].xyz, 0);
	float4 az = float4(shape.s2rb[2].xyz, 0);
	result += (dot(dir.xyz, ax.xyz) > 0 ? half_ext.x : -half_ext.x) * ax;
	result += (dot(dir.xyz, ay.xyz) > 0 ? half_ext.y : -half_ext.y) * ay;
	result += (dot(dir.xyz, az.xyz) > 0 ? half_ext.z : -half_ext.z) * az;
	return result;
}
inline float4 SupportVertex_Line(in_(GpuShape) shape, float4 dir)
{
	float half_len = shape.data.x;
	float thickness = shape.data.y;
	float4 centre = shape.s2rb[3];
	float4 z_axis = float4(shape.s2rb[2].xyz, 0);
	float d = dot(dir.xyz, z_axis.xyz);
	float4 result = centre;
	result += (d > 0 ? half_len : -half_len) * z_axis;
	if (thickness > 0)
	{
		float len_sq = length_sq(dir.xyz);
		if (len_sq > GjkEps * GjkEps)
			result += thickness * dir / sqrt(len_sq);
	}
	return result;
}
inline float4 SupportVertex_Triangle(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	float4 v0 = verts[shape.vert_offset + 0];
	float4 v1 = verts[shape.vert_offset + 1];
	float4 v2 = verts[shape.vert_offset + 2];
	float4 p0 = mul(v0, shape.s2rb);
	float4 p1 = mul(v1, shape.s2rb);
	float4 p2 = mul(v2, shape.s2rb);
	float d0 = dot(dir.xyz, p0.xyz);
	float d1 = dot(dir.xyz, p1.xyz);
	float d2 = dot(dir.xyz, p2.xyz);
	if (d0 >= d1 && d0 >= d2) return p0;
	if (d1 >= d0 && d1 >= d2) return p1;
	return p2;
}
inline float4 SupportVertex_Polytope(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	float best_dot = -1e30f;
	float4 best_vert = float4(0, 0, 0, 1);
	for (int i = 0; i < shape.vert_count; ++i)
	{
		float4 v = mul(verts[shape.vert_offset + i], shape.s2rb);
		float d = dot(dir.xyz, v.xyz);
		if (d > best_dot)
		{
			best_dot = d;
			best_vert = v;
		}
	}
	return best_vert;
}
inline float4 SupportVertex(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
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

// ---- Support face centroid functions ----
// For face-on contact, the "support point" is degenerate: many vertices tie for the
// maximum dot product. The centroid of those tied vertices is the centre of the
// support face — a stable contact point. For non-degenerate (vertex/edge) contact,
// only one vertex ties and the result equals the standard SupportVertex.
// 'dir' is in the shape's rigid-body frame (same convention as SupportVertex).
inline float4 SupportFaceCentre_Box(in_(GpuShape) shape, float4 dir)
{
	const float TieEps = 1e-4f;
	float3 half_ext = shape.data.xyz;
	float4 result = shape.s2rb[3];
	float4 ax = float4(shape.s2rb[0].xyz, 0);
	float4 ay = float4(shape.s2rb[1].xyz, 0);
	float4 az = float4(shape.s2rb[2].xyz, 0);
	float dx = dot(dir.xyz, ax.xyz);
	float dy = dot(dir.xyz, ay.xyz);
	float dz = dot(dir.xyz, az.xyz);
	result += (abs(dx) < TieEps ? 0.0f : (dx > 0 ? half_ext.x : -half_ext.x)) * ax;
	result += (abs(dy) < TieEps ? 0.0f : (dy > 0 ? half_ext.y : -half_ext.y)) * ay;
	result += (abs(dz) < TieEps ? 0.0f : (dz > 0 ? half_ext.z : -half_ext.z)) * az;
	return result;
}
inline float4 SupportFaceCentre_Polytope(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	const float TieEps = 1e-4f;
	float best_dot = -1e30f;
	for (int i = 0; i < shape.vert_count; ++i)
	{
		float4 v = mul(verts[shape.vert_offset + i], shape.s2rb);
		float d = dot(dir.xyz, v.xyz);
		if (d > best_dot) best_dot = d;
	}
	float4 sum = float4(0, 0, 0, 0);
	int count = 0;
	for (int j = 0; j < shape.vert_count; ++j)
	{
		float4 v = mul(verts[shape.vert_offset + j], shape.s2rb);
		float d = dot(dir.xyz, v.xyz);
		if (d >= best_dot - TieEps)
		{
			sum += v;
			++count;
		}
	}
	return float4(sum.xyz / max((float)count, 1.0f), 1);
}
inline float4 SupportFaceCentre(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	switch (shape.type)
	{
		case SHAPE_BOX:      return SupportFaceCentre_Box(shape, dir);
		case SHAPE_POLYTOPE: return SupportFaceCentre_Polytope(shape, dir, verts);
		default:             return float4(SupportVertex(shape, dir, verts).xyz, 1);
	}
}

// ---- Minkowski difference support ----
struct MkSup
{
	float4 w; // Minkowski difference point (a - b), w=0
	float4 a; // Support vertex on shape A (world space), w=1
	float4 b; // Support vertex on shape B (world space), w=1
};
inline MkSup MkSupport(
	in_(GpuShape) shape_a, float4x4 a2w, float4x4 w2a,
	in_(GpuShape) shape_b, float4x4 b2w, float4x4 w2b,
	float4 dir, in_(StructuredBuffer<float4>) verts)
{
	float4 dir_a = mul(+dir, w2a);
	float4 dir_b = mul(-dir, w2b);
	float4 va = mul(SupportVertex(shape_a, dir_a, verts), a2w);
	float4 vb = mul(SupportVertex(shape_b, dir_b, verts), b2w);
	MkSup s;
	s.w = float4((va - vb).xyz, 0);
	s.a = float4(va.xyz, 1);
	s.b = float4(vb.xyz, 1);
	return s;
}

// ---- GJK Simplex ----
struct Simplex
{
	MkSup s[4];
	int n;
};
inline void SimplexPush(inout_(Simplex) sx, MkSup p)
{
	for (int i = sx.n; i > 0; --i)
		sx.s[i] = sx.s[i - 1];
	
	sx.s[0] = p;
	sx.n++;
}

// ---- Simplex reduction cases ----
inline bool SimplexLine(inout_(Simplex) sx, inout_(float4) dir)
{
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;
	if (dot(ab.xyz, ao.xyz) > 0)
	{
		dir = float4(cross(cross(ab.xyz, ao.xyz), ab.xyz), 0);
		if (length_sq(dir.xyz) < GjkEps)
			dir = float4(Perpendicular(ab.xyz), 0);
	}
	else
	{
		sx.n = 1;
		dir = ao;
	}
	return false;
}
inline bool SimplexTri(inout_(Simplex) sx, inout_(float4) dir)
{
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ac = sx.s[2].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;
	float4 n = float4(cross(ab.xyz, ac.xyz), 0);
	if (dot(cross(n.xyz, ac.xyz), ao.xyz) > 0)
	{
		if (dot(ac.xyz, ao.xyz) > 0)
		{
			sx.s[1] = sx.s[2];
			sx.n = 2;
			dir = float4(cross(cross(ac.xyz, ao.xyz), ac.xyz), 0);
			if (length_sq(dir.xyz) < GjkEps)
				dir = float4(Perpendicular(ac.xyz), 0);
		}
		else
		{
			sx.n = 2;
			return SimplexLine(sx, dir);
		}
	}
	else if (dot(cross(ab.xyz, n.xyz), ao.xyz) > 0)
	{
		sx.n = 2;
		return SimplexLine(sx, dir);
	}
	else
	{
		if (dot(n.xyz, ao.xyz) > 0)
		{
			dir = n;
		}
		else
		{
			MkSup tmp = sx.s[1];
			sx.s[1] = sx.s[2];
			sx.s[2] = tmp;
			dir = -n;
		}
	}
	return false;
}
inline bool SimplexTetra(inout_(Simplex) sx, inout_(float4) dir)
{
	float4 ab = sx.s[1].w - sx.s[0].w;
	float4 ac = sx.s[2].w - sx.s[0].w;
	float4 ad = sx.s[3].w - sx.s[0].w;
	float4 ao = -sx.s[0].w;
	float4 abc = float4(cross(ab.xyz, ac.xyz), 0); if (dot(abc.xyz, ad.xyz) > 0) abc = -abc;
	float4 acd = float4(cross(ac.xyz, ad.xyz), 0); if (dot(acd.xyz, ab.xyz) > 0) acd = -acd;
	float4 adb = float4(cross(ad.xyz, ab.xyz), 0); if (dot(adb.xyz, ac.xyz) > 0) adb = -adb;
	if (dot(abc.xyz, ao.xyz) > 0) { sx.n = 3; return SimplexTri(sx, dir); }
	if (dot(acd.xyz, ao.xyz) > 0) { sx.s[1] = sx.s[2]; sx.s[2] = sx.s[3]; sx.n = 3; return SimplexTri(sx, dir); }
	if (dot(adb.xyz, ao.xyz) > 0) { MkSup tmp = sx.s[1]; sx.s[1] = sx.s[3]; sx.s[2] = tmp; sx.n = 3; return SimplexTri(sx, dir); }
	return true;
}
inline bool DoSimplex(inout_(Simplex) sx, inout_(float4) dir)
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
struct EpaFace
{
	int i0, i1, i2;
	float4 normal;
	float dist;
};
struct EpaEdge
{
	int a, b;
};
inline bool Epa(
	in_(GpuShape) shape_a, float4x4 a2w, float4x4 w2a,
	in_(GpuShape) shape_b, float4x4 b2w, float4x4 w2b,
	in_(Simplex) gjk_sx, in_(StructuredBuffer<float4>) verts,
	out_(float4) out_normal, out_(float) out_depth, out_(float4) out_ptA, out_(float4) out_ptB,
	out_(int) out_epa_iters)
{
	out_normal = float4(0, 0, 0, 0);
	out_depth = 0;
	out_ptA = float4(0, 0, 0, 1);
	out_ptB = float4(0, 0, 0, 1);
	out_epa_iters = 0;

	if (gjk_sx.n < 4)
		return false;

	MkSup epa_verts[MaxEpaVerts];
	EpaFace epa_faces[MaxEpaFaces];
	int nv = 4, nf = 0;
	int i;
	
	for (i = 0; i < 4; ++i)
		epa_verts[i] = gjk_sx.s[i];

	// Ensure consistent tetrahedron winding
	float4 n012 = float4(cross(epa_verts[1].w.xyz - epa_verts[0].w.xyz, epa_verts[2].w.xyz - epa_verts[0].w.xyz), 0);
	if (dot(n012.xyz, (epa_verts[3].w - epa_verts[0].w).xyz) > 0)
	{
		MkSup tmp = epa_verts[0];
		epa_verts[0] = epa_verts[1];
		epa_verts[1] = tmp;
	}

	// Build initial tetrahedron faces
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

	// EPA main loop
	for (int iter = 0; iter < MaxGjkIter; ++iter)
	{
		out_epa_iters = iter + 1;

		int ci = 0;
		for (i = 1; i < nf; ++i)
		{
			if (epa_faces[i].dist < epa_faces[ci].dist)
				ci = i;
		}

		float4 cf_normal = epa_faces[ci].normal;
		float cf_dist = epa_faces[ci].dist;

		MkSup sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, cf_normal, verts);
		float d = dot(sup.w.xyz, cf_normal.xyz);

		if (d - cf_dist < EpaEps || nv >= MaxEpaVerts)
		{
			out_normal = cf_normal;
			out_depth = cf_dist;

			// For face-on contact (e.g. cube-vs-cube), many EPA triangles tie for the
			// minimum distance and the EPA polytope only sparsely samples the contact
			// face. Single-triangle barycentric witness points are biased toward whichever
			// triangle won the tie. To produce a stable, geometrically meaningful contact
			// point, query the support face centroid on each shape along the contact
			// normal — averaging all vertices tied for the maximum dot product gives the
			// true face centre. For non-degenerate (vertex/edge) contact, only one vertex
			// ties and this reduces to the standard support point.
			float4 dir_a = mul(+cf_normal, w2a);
			float4 dir_b = mul(-cf_normal, w2b);
			float4 ca_local = SupportFaceCentre(shape_a, dir_a, verts);
			float4 cb_local = SupportFaceCentre(shape_b, dir_b, verts);
			out_ptA = float4(mul(ca_local, a2w).xyz, 1);
			out_ptB = float4(mul(cb_local, b2w).xyz, 1);
			return true;
		}

		epa_verts[nv] = sup;
		int ni = nv++;

		EpaEdge edges[MaxEpaEdges];
		int ne = 0;
		for (i = nf - 1; i >= 0; --i)
		{
			float4 face_to_new = sup.w - epa_verts[epa_faces[i].i0].w;
			if (dot(epa_faces[i].normal.xyz, face_to_new.xyz) <= 0)
				continue;

			int fi_arr[3] = { epa_faces[i].i0, epa_faces[i].i1, epa_faces[i].i2 };
			for (int j = 0; j < 3; ++j)
			{
				int ea = fi_arr[j];
				int eb = fi_arr[(j + 1) % 3];
				bool is_shared = false;
				for (int k = ne - 1; k >= 0; --k)
				{
					if (edges[k].a == eb && edges[k].b == ea)
					{
						edges[k] = edges[--ne];
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

			epa_faces[i] = epa_faces[--nf];
		}

		for (i = 0; i < ne && nf < MaxEpaFaces; ++i)
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
inline bool GjkCollide(
	in_(GpuShape) shape_a, float4x4 a2w,
	in_(GpuShape) shape_b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	out_(float4) out_axis, out_(float4) out_point, out_(float) out_depth,
	out_(int) out_gjk_iters, out_(int) out_epa_iters)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;
	out_gjk_iters = 0;
	out_epa_iters = 0;

	float4x4 w2a = InvertOrthonormal(a2w);
	float4x4 w2b = InvertOrthonormal(b2w);

	float4 centre_a = a2w[3];
	float4 centre_b = b2w[3];
	float4 dir = float4((centre_a - centre_b).xyz, 0);
	if (length_sq(dir.xyz) < GjkEps)
		dir = float4(1, 0, 0, 0);

	Simplex sx;
	sx.n = 0;
	MkSup sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, dir, verts);
	SimplexPush(sx, sup);
	dir = -sup.w;

	for (int iter = 0; iter < MaxGjkIter; ++iter)
	{
		out_gjk_iters = iter + 1;

		if (length_sq(dir.xyz) < GjkEps)
			break;

		sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, dir, verts);
		if (dot(sup.w.xyz, dir.xyz) < 0)
			return false;

		SimplexPush(sx, sup);

		if (DoSimplex(sx, dir))
		{
			float4 normal, ptA, ptB;
			float depth;
			int epa_iters;
			if (!Epa(shape_a, a2w, w2a, shape_b, b2w, w2b, sx, verts, normal, depth, ptA, ptB, epa_iters))
			{
				out_epa_iters = epa_iters;
				return false;
			}
			out_epa_iters = epa_iters;

			float pa = dot(normal.xyz, centre_a.xyz);
			float pb = dot(normal.xyz, centre_b.xyz);
			float sign = (pa < pb) ? 1.0f : -1.0f;
			out_axis = sign * normal;
			out_depth = depth;
			out_point = float4(((ptA + ptB) * 0.5f).xyz, 1);
			return true;
		}
	}

	return false;
}

#ifdef __cplusplus
}
#endif
#endif
