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
#include "src/compute/collision_types.hlsli"

// ---- Constants ----
static const int MaxGjkIter = 32;
static const int MaxEpaVerts = 24;
static const int MaxEpaFaces = 48;
static const int MaxEpaEdges = 96;
static const float GjkEps = 1e-8f;
static const float EpaEps = 1e-6f;

// ---- Vector helpers ----

//float4 Normalise3(float4 v)
//{
//	float len = length(v.xyz);
//	return len > GjkEps ? v / len : float4(1, 0, 0, 0);
//}

//// Transform a point by a row_major float4x4 (point has w=1)
//float4 TransformPoint(float4x4 m, float4 p)
//{
//	return mul(float4(p.xyz, 1), m);
//}

//// Transform a direction by a row_major float4x4 (direction has w=0)
//float4 TransformDir(float4x4 m, float4 d)
//{
//	return float4(mul(float4(d.xyz, 0), m).xyz, 0);
//}

// ---- Support vertex functions ----
// Each function returns the furthest point on the shape boundary in the given direction.
float4 SupportVertex_Sphere(GpuShape shape, float4 dir)
{
	float4 centre = shape.s2rb[3];
	float radius = shape.data.x;
	return centre + radius * normalize(dir);
}

float4 SupportVertex_Box(GpuShape shape, float4 dir)
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

float4 SupportVertex_Line(GpuShape shape, float4 dir)
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

float4 SupportVertex_Triangle(GpuShape shape, float4 dir, StructuredBuffer<float4> verts)
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

float4 SupportVertex_Polytope(GpuShape shape, float4 dir, StructuredBuffer<float4> verts)
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

void SimplexPush(inout Simplex sx, MkSup p)
{
	for (int i = sx.n; i > 0; --i)
		sx.s[i] = sx.s[i - 1];
	sx.s[0] = p;
	sx.n++;
}

// ---- Simplex reduction cases ----
bool SimplexLine(inout Simplex sx, inout float4 dir)
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

bool SimplexTri(inout Simplex sx, inout float4 dir)
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

bool SimplexTetra(inout Simplex sx, inout float4 dir)
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

bool Epa(
	GpuShape shape_a, float4x4 a2w, float4x4 w2a,
	GpuShape shape_b, float4x4 b2w, float4x4 w2b,
	Simplex gjk_sx, StructuredBuffer<float4> verts,
	out float4 out_normal, out float out_depth, out float4 out_ptA, out float4 out_ptB,
	out int out_epa_iters)
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
		int cf_i0 = epa_faces[ci].i0;
		int cf_i1 = epa_faces[ci].i1;
		int cf_i2 = epa_faces[ci].i2;

		MkSup sup = MkSupport(shape_a, a2w, w2a, shape_b, b2w, w2b, cf_normal, verts);
		float d = dot(sup.w.xyz, cf_normal.xyz);

		if (d - cf_dist < EpaEps || nv >= MaxEpaVerts)
		{
			out_normal = cf_normal;
			out_depth = cf_dist;

			// Barycentric interpolation for contact points
			MkSup va = epa_verts[cf_i0];
			MkSup vb = epa_verts[cf_i1];
			MkSup vc = epa_verts[cf_i2];
			float4 proj = cf_dist * cf_normal;
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
bool GjkCollide(
	GpuShape shape_a, float4x4 a2w,
	GpuShape shape_b, float4x4 b2w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth,
	out int out_gjk_iters, out int out_epa_iters)
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

#endif
