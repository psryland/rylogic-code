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
#include "pr/hlsl/closest_point.hlsli"
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
static const int GpuFeatureMaxPoints = 16;
static const int GpuManifoldMaxCorners = 64;
static const float GjkEps = 1e-8f;
static const float EpaEps = 1e-6f;

#ifdef __cplusplus
#define PR_HLSL_BRANCH
#else
#define PR_HLSL_BRANCH [branch]
#endif

// ---- Local data structures ----
struct GpuFeature
{
	int count;
	float4 points[GpuFeatureMaxPoints];
};
struct GpuClipEdge
{
	float t0;
	float t1;
	int valid;
	int pad0;
};
struct GpuClipEdgeSet
{
	GpuClipEdge edges[GpuFeatureMaxPoints];
};
struct GpuCornerSet
{
	int count;
	float4 points[GpuManifoldMaxCorners];
};

// ---- Utility helpers ----
inline float4 NormaliseSafe(float4 v, float4 fallback)
{
	float len_sq = dot(v.xyz, v.xyz);
	return len_sq > GjkEps * GjkEps ? float4(v.xyz * rsqrt(len_sq), 0) : fallback;
}

// ---- Support vertex queries ----
// Furthest boundary point in a shape-space direction. Shape-specific overloads follow EShape order.
inline float4 SupportVertex_Sphere(in_(GpuShape) shape, float4 dir)
{
	float radius = shape.data.x;
	return float4(radius * NormaliseSafe(dir, float4(1, 0, 0, 0)).xyz, 1);
}
inline float4 SupportVertex_Box(in_(GpuShape) shape, float4 dir)
{
	float3 half_ext = shape.data.xyz;
	return float4(
		dir.x > 0 ? +half_ext.x : -half_ext.x,
		dir.y > 0 ? +half_ext.y : -half_ext.y,
		dir.z > 0 ? +half_ext.z : -half_ext.z,
		1);
}
inline float4 SupportVertex_Line(in_(GpuShape) shape, float4 dir)
{
	float half_len = shape.data.x;
	float thickness = shape.data.y;
	float4 result = float4(0, 0, dir.z > 0 ? +half_len : -half_len, 1);
	if (thickness > 0)
	{
		float len_sq = length_sq(dir.xyz);
		if (len_sq > GjkEps * GjkEps)
			result += float4(thickness * dir.xyz * rsqrt(len_sq), 0);
	}
	return result;
}
inline float4 SupportVertex_Triangle(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	float4 v0 = verts[shape.vert_offset + 0];
	float4 v1 = verts[shape.vert_offset + 1];
	float4 v2 = verts[shape.vert_offset + 2];
	float d0 = dot(dir.xyz, v0.xyz);
	float d1 = dot(dir.xyz, v1.xyz);
	float d2 = dot(dir.xyz, v2.xyz);
	if (d0 >= d1 && d0 >= d2) return float4(v0.xyz, 1);
	if (d1 >= d0 && d1 >= d2) return float4(v1.xyz, 1);
	return float4(v2.xyz, 1);
}
inline float4 SupportVertex_Polytope(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	float best_dot = -1e30f;
	float4 best_vert = float4(0, 0, 0, 1);
	for (int i = 0; i < shape.vert_count; ++i)
	{
		float4 v = float4(verts[shape.vert_offset + i].xyz, 1);
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

// ---- Feature accumulation helpers ----
inline void FeatureClear(out_(GpuFeature) feature)
{
	feature.count = 0;
	for (int i = 0; i != GpuFeatureMaxPoints; ++i)
		feature.points[i] = float4(0, 0, 0, 1);
}
inline void FeatureAddUnique(inout_(GpuFeature) feature, float4 pt)
{
	const float TolSq = 1e-8f;
	if (feature.count >= GpuFeatureMaxPoints)
		return;

	for (int i = 0; i != feature.count; ++i)
	{
		if (length_sq((pt - feature.points[i]).xyz) < TolSq)
			return;
	}

	feature.points[feature.count++] = float4(pt.xyz, 1);
}
inline float4 FeatureCentroid(in_(GpuFeature) feature)
{
	float4 centre = float4(0, 0, 0, 0);
	for (int i = 0; i != feature.count; ++i)
		centre += feature.points[i];

	return float4(centre.xyz / max((float)feature.count, 1.0f), 1);
}
inline bool FeatureIsPlanar(in_(GpuFeature) feature, float4 axis)
{
	if (feature.count < 3)
		return true;

	const float Tol = 1e-4f;
	const float TolSq = Tol * Tol;
	float4 normal = float4(0, 0, 0, 0);
	for (int i = 1; i != feature.count - 1; ++i)
	{
		float4 e0 = feature.points[i] - feature.points[0];
		float4 e1 = feature.points[i + 1] - feature.points[0];
		normal = float4(cross(e0.xyz, e1.xyz), 0);
		float len_sq = length_sq(normal.xyz);
		if (len_sq > TolSq)
		{
			normal = float4(normal.xyz * rsqrt(len_sq), 0);
			break;
		}
	}
	if (length_sq(normal.xyz) <= TolSq)
		return false;

	float4 axis_norm = NormaliseSafe(axis, float4(1, 0, 0, 0));
	if (abs(dot(normal.xyz, axis_norm.xyz)) < 0.5f)
		return false;

	for (int j = 0; j != feature.count; ++j)
	{
		if (abs(dot((feature.points[j] - feature.points[0]).xyz, normal.xyz)) > Tol)
			return false;
	}
	return true;
}
inline void FeatureSortAroundAxis(inout_(GpuFeature) feature, float4 axis)
{
	if (feature.count < 3)
		return;

	float4 centre = FeatureCentroid(feature);
	float4 norm = NormaliseSafe(axis, float4(1, 0, 0, 0));
	float4 basis_x = float4(0, 0, 0, 0);
	const float TolSq = 1e-8f;

	for (int i = 0; i != feature.count; ++i)
	{
		float4 radial = float4((feature.points[i] - centre).xyz, 0);
		radial -= dot(radial.xyz, norm.xyz) * norm;
		float len_sq = length_sq(radial.xyz);
		if (len_sq > TolSq)
		{
			basis_x = float4(radial.xyz * rsqrt(len_sq), 0);
			break;
		}
	}
	if (length_sq(basis_x.xyz) <= TolSq)
		return;

	float4 basis_y = float4(cross(norm.xyz, basis_x.xyz), 0);
	for (int i = 0; i != feature.count - 1; ++i)
	{
		for (int j = i + 1; j != feature.count; ++j)
		{
			float4 di = float4((feature.points[i] - centre).xyz, 0);
			float4 dj = float4((feature.points[j] - centre).xyz, 0);
			float ai = atan2(dot(di.xyz, basis_y.xyz), dot(di.xyz, basis_x.xyz));
			float aj = atan2(dot(dj.xyz, basis_y.xyz), dot(dj.xyz, basis_x.xyz));
			if (aj < ai)
			{
				float4 tmp = feature.points[i];
				feature.points[i] = feature.points[j];
				feature.points[j] = tmp;
			}
		}
	}
}

// ---- Support feature queries ----
inline void SupportFeature_Box(in_(GpuShape) shape, float4 axis, out_(GpuFeature) feature)
{
	FeatureClear(feature);
	feature.count = 1;
	feature.points[0] = float4(0, 0, 0, 1);

	float3 radius = shape.data.xyz;
	for (int i = 0; i != 3; ++i)
	{
		float4 basis = float4(0, 0, 0, 0);
		basis[i] = 1.0f;

		float d = axis[i];
		if (d > +GjkEps)
		{
			for (int f = 0; f != feature.count; ++f)
				feature.points[f] += basis * radius[i];
		}
		else if (d < -GjkEps)
		{
			for (int f = 0; f != feature.count; ++f)
				feature.points[f] -= basis * radius[i];
		}
		else if (feature.count == 1)
		{
			feature.points[1] = feature.points[0];
			feature.points[0] += basis * radius[i];
			feature.points[1] -= basis * radius[i];
			feature.count = 2;
		}
		else if (feature.count == 2)
		{
			feature.points[3] = feature.points[0];
			feature.points[2] = feature.points[1];
			feature.points[0] += basis * radius[i];
			feature.points[1] += basis * radius[i];
			feature.points[2] -= basis * radius[i];
			feature.points[3] -= basis * radius[i];
			if (dot(axis.xyz, cross((feature.points[1] - feature.points[0]).xyz, (feature.points[2] - feature.points[0]).xyz)) < 0)
			{
				float4 tmp = feature.points[1];
				feature.points[1] = feature.points[3];
				feature.points[3] = tmp;
			}
			feature.count = 4;
		}
	}
}
inline void SupportFeature_Line(in_(GpuShape) shape, float4 axis, out_(GpuFeature) feature)
{
	FeatureClear(feature);
	float half_len = shape.data.x;
	float radius = shape.data.y;
	float4 r = float4(0, 0, half_len, 0);
	float4 thickness_offset = float4(0, 0, 0, 0);
	float len_sq = length_sq(axis.xyz);
	if (radius > 0 && len_sq > GjkEps * GjkEps)
		thickness_offset = float4(radius * axis.xyz * rsqrt(len_sq), 0);

	const float Tol = 1e-3f;
	if (axis.z > +Tol)
	{
		feature.count = 1;
		feature.points[0] = float4((r + thickness_offset).xyz, 1);
	}
	else if (axis.z < -Tol)
	{
		feature.count = 1;
		feature.points[0] = float4((-r + thickness_offset).xyz, 1);
	}
	else
	{
		feature.count = 2;
		feature.points[0] = float4((-r + thickness_offset).xyz, 1);
		feature.points[1] = float4((+r + thickness_offset).xyz, 1);
	}
}
inline void SupportFeature_Triangle(in_(GpuShape) shape, float4 axis, in_(StructuredBuffer<float4>) verts, out_(GpuFeature) feature)
{
	FeatureClear(feature);
	float4 v0 = float4(verts[shape.vert_offset + 0].xyz, 1);
	float4 v1 = float4(verts[shape.vert_offset + 1].xyz, 1);
	float4 v2 = float4(verts[shape.vert_offset + 2].xyz, 1);
	float d0 = dot(axis.xyz, v0.xyz);
	float d1 = dot(axis.xyz, v1.xyz);
	float d2 = dot(axis.xyz, v2.xyz);
	float dmax = max(max(d0, d1), d2) - GjkEps;

	if (d0 >= dmax) FeatureAddUnique(feature, v0);
	if (d1 >= dmax) FeatureAddUnique(feature, v1);
	if (d2 >= dmax) FeatureAddUnique(feature, v2);
	if (feature.count == 3)
	{
		if (dot(axis.xyz, cross((feature.points[1] - feature.points[0]).xyz, (feature.points[2] - feature.points[0]).xyz)) < 0)
		{
			float4 tmp = feature.points[1];
			feature.points[1] = feature.points[2];
			feature.points[2] = tmp;
		}
	}
}
inline void SupportFeature_Polytope(in_(GpuShape) shape, float4 axis, in_(StructuredBuffer<float4>) verts, out_(GpuFeature) feature)
{
	FeatureClear(feature);
	float best_dist = -1e30f;
	for (int i = 0; i != shape.vert_count; ++i)
	{
		float4 pt = float4(verts[shape.vert_offset + i].xyz, 1);
		best_dist = max(best_dist, dot(axis.xyz, pt.xyz));
	}
	for (int j = 0; j != shape.vert_count; ++j)
	{
		float4 pt = float4(verts[shape.vert_offset + j].xyz, 1);
		if (dot(axis.xyz, pt.xyz) >= best_dist - 1e-4f)
			FeatureAddUnique(feature, pt);
	}
	FeatureSortAroundAxis(feature, axis);
}
inline void SupportFeature(in_(GpuShape) shape, float4 axis, in_(StructuredBuffer<float4>) verts, out_(GpuFeature) feature)
{
	switch (shape.type)
	{
		case SHAPE_SPHERE:   { FeatureClear(feature); feature.count = 1; feature.points[0] = SupportVertex_Sphere(shape, axis); return; }
		case SHAPE_BOX:      { SupportFeature_Box(shape, axis, feature); return; }
		case SHAPE_LINE:     { SupportFeature_Line(shape, axis, feature); return; }
		case SHAPE_TRIANGLE: { SupportFeature_Triangle(shape, axis, verts, feature); return; }
		case SHAPE_POLYTOPE: { SupportFeature_Polytope(shape, axis, verts, feature); return; }
		default:             { FeatureClear(feature); return; }
	}
}
inline void TransformFeature(inout_(GpuFeature) feature, float4x4 s2w)
{
	for (int i = 0; i != feature.count; ++i)
		feature.points[i] = float4(mul(feature.points[i], s2w).xyz, 1);
}

// ---- Contact record helpers ----
inline void ContactClear(out_(GpuContact) contact)
{
	contact.axis = float4(0, 0, 0, 0);
	contact.feature = FEATURE_NONE;
	contact.depth = 0;
	contact.pad0 = 0;
	contact.pad1 = 0;
	for (int i = 0; i != GpuContactMaxPoints; ++i)
		contact.manifold[i] = float4(0, 0, 0, 1);
}
inline int ContactCount(in_(GpuContact) contact)
{
	return clamp(contact.feature, FEATURE_NONE, GpuContactMaxPoints);
}
inline float4 ContactCentroid(in_(GpuContact) contact)
{
	float4 centre = float4(0, 0, 0, 0);
	int count = ContactCount(contact);
	for (int i = 0; i != count; ++i)
		centre += contact.manifold[i];

	return count != 0 ? float4(centre.xyz / (float)count, 1) : float4(0, 0, 0, 1);
}
inline void ContactSetPoint(out_(GpuContact) contact, float4 axis, float4 pt, float depth)
{
	ContactClear(contact);
	contact.axis = float4(NormaliseSafe(axis, float4(1, 0, 0, 0)).xyz, 0);
	contact.manifold[0] = float4(pt.xyz, 1);
	contact.feature = FEATURE_VERT;
	contact.depth = depth;
}
inline void ContactSetManifold(out_(GpuContact) contact, float4 axis, float4 manifold[GpuContactMaxPoints], int feature, float depth)
{
	ContactClear(contact);
	contact.axis = float4(NormaliseSafe(axis, float4(1, 0, 0, 0)).xyz, 0);
	contact.feature = clamp(feature, FEATURE_NONE, GpuContactMaxPoints);
	contact.depth = depth;
	for (int i = 0; i != contact.feature; ++i)
		contact.manifold[i] = float4(manifold[i].xyz, 1);
}
inline void ContactFlip(inout_(GpuContact) contact)
{
	contact.axis = -contact.axis;
	int count = ContactCount(contact);
	for (int i = 0; i != count / 2; ++i)
	{
		float4 tmp = contact.manifold[i];
		contact.manifold[i] = contact.manifold[count - 1 - i];
		contact.manifold[count - 1 - i] = tmp;
	}
}

// ---- Manifold clipping helpers ----
inline void CornerSetClear(out_(GpuCornerSet) corners)
{
	corners.count = 0;
	for (int i = 0; i != GpuManifoldMaxCorners; ++i)
		corners.points[i] = float4(0, 0, 0, 1);
}
inline void CornerSetAdd(inout_(GpuCornerSet) corners, float4 pt)
{
	if (corners.count < GpuManifoldMaxCorners)
		corners.points[corners.count++] = float4(pt.xyz, 1);
}
inline void ClipEdgeClear(inout_(GpuClipEdge) edge)
{
	edge.t0 = 0.0f;
	edge.t1 = 1.0f;
	edge.valid = 1;
	edge.pad0 = 0;
}
inline void ClipEdgeSetClear(GpuClipEdgeSet edgeset)
{
	for (int i = 0; i != GpuFeatureMaxPoints; ++i)
		ClipEdgeClear(edgeset.edges[i]);
}
inline void ClipFeatureEdges(in_(GpuFeature) points0, in_(GpuFeature) points1, inout_(GpuClipEdgeSet) edges1, float sign, float4 axis)
{
	const float Tol = 1e-5f;
	for (int i = 0; i != points0.count; ++i)
	{
		float4 as = points0.points[i];
		float4 ae = points0.points[(i + 1) % points0.count];
		float4 n = sign * float4(cross(axis.xyz, (ae - as).xyz), 0);
		for (int j = 0; j != points1.count; ++j)
		{
			if (edges1.edges[j].valid == 0)
				continue;

			float4 bs = points1.points[j];
			float4 be = points1.points[(j + 1) % points1.count];
			float d0 = dot(n.xyz, (bs - as).xyz);
			float d1 = dot(n.xyz, (be - as).xyz);
			if (d0 < -Tol && d1 < -Tol)
			{
				edges1.edges[j].valid = 0;
				continue;
			}
			if (d0 >= -Tol && d1 >= -Tol)
				continue;

			float t = d0 / (d0 - d1);
			if (d0 < -Tol)
				edges1.edges[j].t0 = max(edges1.edges[j].t0, t);
			else
				edges1.edges[j].t1 = min(edges1.edges[j].t1, t);

			if (edges1.edges[j].t0 > edges1.edges[j].t1 + Tol)
				edges1.edges[j].valid = 0;
		}
	}
}
inline void AppendClippedEdge(inout_(GpuCornerSet) corners, in_(GpuFeature) points, int edge_index, GpuClipEdge edge, float4 offset)
{
	if (edge.valid == 0 || edge.t0 > edge.t1 + 1e-5f)
		return;

	float4 pt0 = points.points[edge_index];
	float4 pt1 = points.points[(edge_index + 1) % points.count];
	CornerSetAdd(corners, pt0 + edge.t0 * (pt1 - pt0) + offset);
	CornerSetAdd(corners, pt0 + edge.t1 * (pt1 - pt0) + offset);
}

// Build the best representative point for simple feature pairs without invoking the polygon clipper.
inline bool ContactTrySetPoint(in_(GpuFeature) featA, in_(GpuFeature) featB, float4 axis, float depth, out_(GpuContact) contact)
{
	ContactClear(contact);
	int countA = featA.count;
	int countB = featB.count;
	PR_HLSL_BRANCH if (countA == 0 || countB == 0)
		return true;

	PR_HLSL_BRANCH if (countA == 1 && countB == 1)
	{
		ContactSetPoint(contact, axis, 0.5f * (featA.points[0] + featB.points[0]), depth);
		return true;
	}
	PR_HLSL_BRANCH if (countA == 1)
	{
		ContactSetPoint(contact, axis, featA.points[0] + axis * (0.5f * dot(axis.xyz, (featB.points[0] - featA.points[0]).xyz)), depth);
		return true;
	}
	PR_HLSL_BRANCH if (countB == 1)
	{
		ContactSetPoint(contact, axis, featB.points[0] + axis * (0.5f * dot(axis.xyz, (featA.points[0] - featB.points[0]).xyz)), depth);
		return true;
	}
	PR_HLSL_BRANCH if (countA == 2 && countB == 2)
	{
		float4 da = featA.points[1] - featA.points[0];
		float4 db = featB.points[1] - featB.points[0];
		float4 ca = 0.5f * (featA.points[0] + featA.points[1]);
		float4 cb = 0.5f * (featB.points[0] + featB.points[1]);
		float2 t = ClosestPoint_SegmentToSegment(ca, NormaliseSafe(da, float4(1, 0, 0, 0)), 0.5f * length(da.xyz), cb, NormaliseSafe(db, float4(1, 0, 0, 0)), 0.5f * length(db.xyz));
		ContactSetPoint(contact, axis, 0.5f * (ca + t.x * NormaliseSafe(da, float4(1, 0, 0, 0)) + cb + t.y * NormaliseSafe(db, float4(1, 0, 0, 0))), depth);
		return true;
	}
	return false;
}

// Reduce clipped corner candidates to the largest point/edge/triangle/quad representative manifold.
inline void ContactReduceManifold(in_(GpuCornerSet) corners, float4 axis, out_(GpuContact) contact)
{
	ContactClear(contact);
	const float Tol = 1e-5f;
	const float TolSq = Tol * Tol;

	if (corners.count == 0)
		return;
	if (corners.count == 1)
	{
		ContactSetPoint(contact, axis, corners.points[0], 0);
		return;
	}
	if (corners.count == 2)
	{
		if (length_sq((corners.points[1] - corners.points[0]).xyz) < TolSq)
		{
			ContactSetPoint(contact, axis, corners.points[0], 0);
			return;
		}

		float4 manifold[GpuContactMaxPoints];
		manifold[0] = corners.points[0];
		manifold[1] = corners.points[1];
		ContactSetManifold(contact, axis, manifold, FEATURE_EDGE, 0);
		return;
	}
	if (corners.count == 3)
	{
		float4 e0 = corners.points[1] - corners.points[0];
		float4 e1 = corners.points[2] - corners.points[1];
		float4 e2 = corners.points[0] - corners.points[2];
		float l0 = length_sq(e0.xyz);
		float l1 = length_sq(e1.xyz);
		float l2 = length_sq(e2.xyz);
		float sign = dot(axis.xyz, cross(e0.xyz, e1.xyz));

		if (l0 < TolSq && l1 < TolSq && l2 < TolSq)
		{
			ContactSetPoint(contact, axis, corners.points[0], 0);
			return;
		}
		if (l0 < TolSq || l1 < TolSq || l2 < TolSq || abs(sign) < TolSq)
		{
			float4 manifold[GpuContactMaxPoints];
			int i0 = 0, i1 = 1;
			if (l1 >= l0 && l1 >= l2) { i0 = 1; i1 = 2; }
			else if (l2 >= l0 && l2 >= l1) { i0 = 2; i1 = 0; }
			manifold[0] = corners.points[i0];
			manifold[1] = corners.points[i1];
			ContactSetManifold(contact, axis, manifold, FEATURE_EDGE, 0);
			return;
		}

		float4 manifold[GpuContactMaxPoints];
		manifold[0] = corners.points[0];
		manifold[1] = sign > 0 ? corners.points[1] : corners.points[2];
		manifold[2] = sign > 0 ? corners.points[2] : corners.points[1];
		ContactSetManifold(contact, axis, manifold, FEATURE_TRI, 0);
		return;
	}

	int corner_count = min(corners.count, GpuManifoldMaxCorners);
	int i0 = 0, i1 = 1;
	float max_dist_sq = length_sq((corners.points[i1] - corners.points[i0]).xyz);
	for (int step = 0, i = 2; step != 2 * corner_count; ++step, i = (i + 1) % corner_count)
	{
		if (i == i0)
			break;

		float dist_sq = length_sq((corners.points[i] - corners.points[i0]).xyz);
		if (dist_sq > max_dist_sq)
		{
			i1 = i0;
			i0 = i;
			max_dist_sq = dist_sq;
		}
	}
	if (max_dist_sq < TolSq)
	{
		ContactSetPoint(contact, axis, corners.points[i0], 0);
		return;
	}

	int j0 = i0, j1 = i0;
	float max_dist0 = -Tol;
	float max_dist1 = +Tol;
	float4 bi_norm = NormaliseSafe(float4(cross(axis.xyz, (corners.points[i1] - corners.points[i0]).xyz), 0), float4(1, 0, 0, 0));
	for (int j = 0; j != corner_count; ++j)
	{
		if (j == i0 || j == i1)
			continue;

		float dist = dot((corners.points[j] - corners.points[i0]).xyz, bi_norm.xyz);
		if (dist < max_dist0) { j0 = j; max_dist0 = dist; }
		if (dist > max_dist1) { j1 = j; max_dist1 = dist; }
	}

	float4 manifold[GpuContactMaxPoints];
	manifold[0] = corners.points[i0];
	if (j0 == i0 && j1 == i0)
	{
		manifold[1] = corners.points[i1];
		ContactSetManifold(contact, axis, manifold, FEATURE_EDGE, 0);
	}
	else if (j0 == i0)
	{
		manifold[1] = corners.points[i1];
		manifold[2] = corners.points[j1];
		ContactSetManifold(contact, axis, manifold, FEATURE_TRI, 0);
	}
	else if (j1 == i0)
	{
		manifold[1] = corners.points[j0];
		manifold[2] = corners.points[i1];
		ContactSetManifold(contact, axis, manifold, FEATURE_TRI, 0);
	}
	else
	{
		manifold[1] = corners.points[j0];
		manifold[2] = corners.points[i1];
		manifold[3] = corners.points[j1];
		ContactSetManifold(contact, axis, manifold, FEATURE_QUAD, 0);
	}
}
inline void ContactFallback(in_(GpuFeature) featA, in_(GpuFeature) featB, float4 axis, float depth, out_(GpuContact) contact)
{
	ContactSetPoint(contact, axis, 0.5f * (FeatureCentroid(featA) + FeatureCentroid(featB)), depth);
}

// Clip support features in world space, then shift the surviving boundary to the contact mid-plane.
inline void FindContactManifold(in_(GpuFeature) featA, in_(GpuFeature) featB, float4 axis, float depth, out_(GpuContact) contact)
{
	if (ContactTrySetPoint(featA, featB, axis, depth, contact))
		return;

	int countA = featA.count;
	int countB = featB.count;
	PR_HLSL_BRANCH if (!FeatureIsPlanar(featA, axis) || !FeatureIsPlanar(featB, axis))
	{
		ContactFallback(featA, featB, axis, depth, contact);
		return;
	}

	GpuClipEdgeSet edgesA;
	GpuClipEdgeSet edgesB;
	for (int i = 0; i != GpuFeatureMaxPoints; ++i)
	{
		edgesA.edges[i].t0 = 0.0f; edgesA.edges[i].t1 = 1.0f; edgesA.edges[i].valid = 1; edgesA.edges[i].pad0 = 0;
		edgesB.edges[i].t0 = 0.0f; edgesB.edges[i].t1 = 1.0f; edgesB.edges[i].valid = 1; edgesB.edges[i].pad0 = 0;
	}

	GpuCornerSet corners;
	CornerSetClear(corners);

	if (countA == 2)
	{
		ClipFeatureEdges(featB, featA, edgesA, -1.0f, axis);
		AppendClippedEdge(corners, featA, 0, edgesA.edges[0], -(0.5f * depth) * axis);
		ContactReduceManifold(corners, axis, contact);
		contact.axis = float4(NormaliseSafe(axis, float4(1, 0, 0, 0)).xyz, 0);
		contact.depth = depth;
		if (contact.feature == FEATURE_NONE)
			ContactFallback(featA, featB, axis, depth, contact);
		return;
	}
	if (countB == 2)
	{
		ClipFeatureEdges(featA, featB, edgesB, +1.0f, axis);
		AppendClippedEdge(corners, featB, 0, edgesB.edges[0], +(0.5f * depth) * axis);
		ContactReduceManifold(corners, axis, contact);
		contact.axis = float4(NormaliseSafe(axis, float4(1, 0, 0, 0)).xyz, 0);
		contact.depth = depth;
		if (contact.feature == FEATURE_NONE)
			ContactFallback(featA, featB, axis, depth, contact);
		return;
	}

	ClipFeatureEdges(featA, featB, edgesB, +1.0f, axis);
	ClipFeatureEdges(featB, featA, edgesA, -1.0f, axis);
	for (int j = 0; j != countA; ++j)
		AppendClippedEdge(corners, featA, j, edgesA.edges[j], -(0.5f * depth) * axis);
	for (int k = 0; k != countB; ++k)
		AppendClippedEdge(corners, featB, k, edgesB.edges[k], +(0.5f * depth) * axis);

	ContactReduceManifold(corners, axis, contact);
	contact.axis = float4(NormaliseSafe(axis, float4(1, 0, 0, 0)).xyz, 0);
	contact.depth = depth;
	if (contact.feature == FEATURE_NONE)
		ContactFallback(featA, featB, axis, depth, contact);
}
inline void FindContactManifold(
	in_(GpuShape) shape_a, float4x4 a2w,
	in_(GpuShape) shape_b, float4x4 b2w,
	float4 axis, float depth, in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) contact)
{
	GpuFeature featA, featB;
	SupportFeature(shape_a, mul(+axis, InvertOrthonormal(a2w)), verts, featA);
	SupportFeature(shape_b, mul(-axis, InvertOrthonormal(b2w)), verts, featB);
	TransformFeature(featA, a2w);
	TransformFeature(featB, b2w);
	FindContactManifold(featA, featB, axis, depth, contact);
}

// ---- Support face centroid functions ----
// For face-on contact, the "support point" is degenerate: many vertices tie for the
// maximum dot product. The centroid of those tied vertices is the centre of the
// support face — a stable contact point. For non-degenerate (vertex/edge) contact,
// only one vertex ties and the result equals the standard SupportVertex.
// 'dir' is in shape space (same convention as SupportVertex).
inline float4 SupportFaceCentre_Box(in_(GpuShape) shape, float4 dir)
{
	const float TieEps = 1e-4f;
	float3 half_ext = shape.data.xyz;
	float4 result = float4(0, 0, 0, 1);
	result.x = abs(dir.x) < TieEps ? 0.0f : (dir.x > 0 ? half_ext.x : -half_ext.x);
	result.y = abs(dir.y) < TieEps ? 0.0f : (dir.y > 0 ? half_ext.y : -half_ext.y);
	result.z = abs(dir.z) < TieEps ? 0.0f : (dir.z > 0 ? half_ext.z : -half_ext.z);
	return result;
}
inline float4 SupportFaceCentre_Polytope(in_(GpuShape) shape, float4 dir, in_(StructuredBuffer<float4>) verts)
{
	const float TieEps = 1e-4f;
	float best_dot = -1e30f;
	for (int i = 0; i < shape.vert_count; ++i)
	{
		float4 v = float4(verts[shape.vert_offset + i].xyz, 1);
		float d = dot(dir.xyz, v.xyz);
		if (d > best_dot) best_dot = d;
	}
	float4 sum = float4(0, 0, 0, 0);
	int count = 0;
	for (int j = 0; j < shape.vert_count; ++j)
	{
		float4 v = float4(verts[shape.vert_offset + j].xyz, 1);
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
	in_(GpuShape) shape_a, float4x4 a2w_,
	in_(GpuShape) shape_b, float4x4 b2w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact,
	out_(int) out_gjk_iters, out_(int) out_epa_iters)
{
	ContactClear(out_contact);
	float4x4 a2w = mul(shape_a.s2rb, a2w_);
	float4x4 b2w = mul(shape_b.s2rb, b2w_);
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
			float4 axis = sign * normal;

			// Generic EPA normals can produce support polygons that are unstable on the GPU; specialised SAT paths build full manifolds where stable features are available.
			GpuFeature featA, featB;
			SupportFeature(shape_a, mul(+axis, w2a), verts, featA);
			SupportFeature(shape_b, mul(-axis, w2b), verts, featB);
			TransformFeature(featA, a2w);
			TransformFeature(featB, b2w);
			if (!ContactTrySetPoint(featA, featB, axis, depth, out_contact))
				ContactFallback(featA, featB, axis, depth, out_contact);
			return true;
		}
	}

	return false;
}
inline bool GjkCollide(
	in_(GpuShape) shape_a, float4x4 a2w,
	in_(GpuShape) shape_b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	out_(float4) out_axis, out_(float4) out_point, out_(float) out_depth,
	out_(int) out_gjk_iters, out_(int) out_epa_iters)
{
	GpuContact contact;
	bool hit = GjkCollide(shape_a, a2w, shape_b, b2w, verts, contact, out_gjk_iters, out_epa_iters);
	out_axis = contact.axis;
	out_point = ContactCentroid(contact);
	out_depth = contact.depth;
	return hit;
}

#ifdef __cplusplus
}
#endif
#endif
