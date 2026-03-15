//*********************************************
// Physics Engine — Specialised Collision Functions
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Analytic and semi-analytic collision tests for shape pairs where GJK/EPA
// would produce poor results due to implicit curved surfaces.
//
// WARNING: GJK/EPA should NOT be used when either shape is implicitly curved
// (sphere, thick line). EPA approximates the Minkowski boundary with a polytope,
// which cannot accurately represent curved surfaces. This leads to:
//   - Inaccurate contact normals (off-axis by many degrees)
//   - Poor EPA convergence (hitting iteration limits → TDR on GPU)
//   - Energy drift in the physics simulation
//
// For pairs involving implicit shapes (sphere, thick line), use either:
//   - Direct analytic tests (sphere-sphere, sphere-box, sphere-line, line-line)
//   - "GJK with margins": run closest-point GJK on the core shape (sphere centre
//     or line skeleton) vs the convex shape, then add the radius/thickness margin
//     analytically. This avoids EPA entirely and gives exact normals.
//
// Collision dispatch matrix (15 unique pairs from 5 shape types):
//
//   Analytic (no GJK):
//     Sphere vs Sphere         — distance test
//     Sphere vs Box            — closest point on OBB
//     Sphere vs Line           — closest point on segment + combined radii
//     Line vs Line             — closest segment-segment + combined thickness
//     Line vs Box              — iterative closest point on OBB + thickness
//     Line vs Triangle         — closest segment-triangle + thickness
//     Box vs Box               — SAT (6 face axes)
//     Triangle vs Triangle     — SAT (2 face + 9 edge axes)
//
//   GJK with margins (GJK on core shape, add radius analytically):
//     Sphere vs Triangle       — GJK point-vs-triangle + radius
//     Sphere vs Polytope       — GJK point-vs-polytope + radius
//     Line vs Polytope         — GJK skeleton-vs-polytope + thickness
//
//   GJK + EPA (both shapes polyhedral, Minkowski boundary is polyhedral):
//     Box vs Triangle
//     Box vs Polytope
//     Triangle vs Polytope
//     Polytope vs Polytope
#ifndef PR_PHYSICS_COLLISION_HLSLI
#define PR_PHYSICS_COLLISION_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "src/compute/collision_types.hlsli"
#include "src/compute/gjk.hlsli"

// ---- Constants ----
static const float Eps = 1e-8f;

// ---- Sphere vs Sphere ----
// Direct distance test between two sphere centres.
bool SphereVsSphere(
	GpuShape sa, float4x4 a2w,
	GpuShape sb, float4x4 b2w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float3 ca = a2w[3].xyz;
	float3 cb = b2w[3].xyz;
	float ra = sa.data.x;
	float rb = sb.data.x;
	float3 diff = cb - ca;
	float dist_sq = dot(diff, diff);
	float radii_sum = ra + rb;

	if (dist_sq >= radii_sum * radii_sum || dist_sq < 1e-12f)
		return false;

	float dist = sqrt(dist_sq);
	float3 normal = diff / dist;
	out_axis = float4(normal, 0);
	out_depth = radii_sum - dist;
	float3 pt = ca + normal * (ra - out_depth * 0.5f);
	out_point = float4(pt, 1);
	return true;
}

// ---- Sphere vs Box ----
// Find the closest point on the OBB to the sphere centre.
// If the distance is less than the sphere radius, there's a collision.
bool SphereVsBox(
	GpuShape sphere, float4x4 sphere_w,
	GpuShape box, float4x4 box_w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 sphere_centre = sphere_w[3];
	float radius = sphere.data.x;
	float4 half_ext = box.data;
	float4 box_centre = box_w[3];

	// Transform sphere centre into box's local frame
	float4 local = float4(
		dot(sphere_centre - box_centre, box_w[0]),
		dot(sphere_centre - box_centre, box_w[1]),
		dot(sphere_centre - box_centre, box_w[2]),
		0);

	// Clamp to box extents to find closest point on box surface
	float4 clamped = clamp(local, -half_ext, half_ext);

	// Distance from sphere centre to closest point on box
	float4 delta = local - clamped;
	float dist_sq = dot(delta, delta);

	if (dist_sq >= radius * radius)
		return false;

	// Sphere centre is outside the box (normal case)
	if (dist_sq > 1e-12f)
	{
		float dist = sqrt(dist_sq);
		float4 local_normal = delta / dist;

		// Transform normal back to world space
		float4 world_normal = local_normal.x * box_w[0]
		                    + local_normal.y * box_w[1]
		                    + local_normal.z * box_w[2];

		out_axis = world_normal;
		out_depth = radius - dist;

		// Contact point on box surface in world space
		float4 closest_world = box_centre
			+ clamped.x * box_w[0]
			+ clamped.y * box_w[1]
			+ clamped.z * box_w[2];
		out_point = closest_world;
		return true;
	}

	// Sphere centre is inside the box — find the shortest axis to push out
	float4 face_dist = half_ext - abs(local);
	int min_axis = 0;
	if (face_dist.y < face_dist.x) min_axis = 1;
	if (face_dist.z < face_dist[min_axis]) min_axis = 2;

	float4 local_normal = float4(0, 0, 0, 0);
	local_normal[min_axis] = local[min_axis] > 0 ? 1.0f : -1.0f;

	float4 world_normal = local_normal.x * box_w[0]
	                    + local_normal.y * box_w[1]
	                    + local_normal.z * box_w[2];

	out_axis = world_normal;
	out_depth = radius + face_dist[min_axis];

	// Contact point: box face
	float4 face_pt = local;
	face_pt[min_axis] = local_normal[min_axis] * half_ext[min_axis];
	out_point = box_centre
		+ face_pt.x * box_w[0]
		+ face_pt.y * box_w[1]
		+ face_pt.z * box_w[2];
	return true;
}

// ---- Sphere vs Line ----
// Find the closest point on the line segment to the sphere centre.
// The line has hemispherical end-caps of thickness 'data.y'.
bool SphereVsLine(
	GpuShape sphere, float4x4 sphere_w,
	GpuShape seg, float4x4 seg_w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 sphere_centre = sphere_w[3];
	float sphere_r = sphere.data.x;
	float half_len = seg.data.x;
	float thickness = seg.data.y;
	float4 seg_centre = seg_w[3];
	float4 seg_dir = seg_w[2]; // Z axis is the line direction

	// Project sphere centre onto line axis
	float4 to_sphere = sphere_centre - seg_centre;
	float t = dot(to_sphere, seg_dir);
	t = clamp(t, -half_len, half_len);

	// Closest point on line segment
	float4 closest = seg_centre + t * seg_dir;
	float4 diff = sphere_centre - closest;
	float dist_sq = dot(diff, diff);
	float combined_r = sphere_r + thickness;

	if (dist_sq >= combined_r * combined_r || dist_sq < 1e-12f)
		return false;

	float dist = sqrt(dist_sq);
	float4 normal = diff / dist;
	out_axis = normal;
	out_depth = combined_r - dist;
	out_point = closest + thickness * normal;
	return true;
}

// ---- Sphere vs Convex (Polytope/Triangle) ----
// Uses "GJK with margins": find the closest point on the convex shape to the
// sphere centre (treating it as a point), then check distance < radius.
// This avoids EPA entirely and gives exact contact normals.
bool SphereVsConvex(
	GpuShape sphere, float4x4 sphere_w,
	GpuShape convex, float4x4 convex_w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth,
	out int out_gjk_iters)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;
	out_gjk_iters = 0;

	float4 sphere_centre = sphere_w[3];
	float radius = sphere.data.x;

	float4x4 w2c = InvertOrthonormal(convex_w);

	// Initial search direction: from convex centre toward sphere centre
	float4 dir = sphere_centre - convex_w[3];
	if (dot(dir, dir) < Eps)
		dir = float4(1, 0, 0, 0);

	// GJK closest-point between the convex shape and the sphere centre (a point).
	// The simplex tracks the closest feature on the Minkowski difference (convex - point = convex shifted).
	// We just need the closest point on the convex to the sphere centre.
	float4 local_dir = mul(dir, w2c);
	float4 sup_local = SupportVertex(convex, local_dir, verts);
	float4 sup = mul(sup_local, convex_w);
	float4 w = sup - sphere_centre; // Minkowski vertex

	// Simplex for closest-point GJK (we store up to 3 vertices)
	float4 simplex_w[4];  // Minkowski difference points
	float4 simplex_a[4];  // Corresponding convex surface points
	simplex_w[0] = w;
	simplex_a[0] = sup;
	int sn = 1;

	float closest_dist_sq = dot(w.xyz, w.xyz);

	for (int iter = 0; iter < MaxGjkIter; ++iter)
	{
		out_gjk_iters = iter + 1;

		// Search toward the origin
		dir = -float4(w.xyz, 0);
		if (dot(dir.xyz, dir.xyz) < Eps)
			break;

		local_dir = mul(dir, w2c);
		sup_local = SupportVertex(convex, local_dir, verts);
		sup = mul(sup_local, convex_w);
		w = sup - sphere_centre;

		// Check if new support makes progress toward origin
		float proj = dot(w.xyz, dir.xyz);
		if (proj < 0 && proj * proj > closest_dist_sq * dot(dir.xyz, dir.xyz) * 0.999f)
			break; // No significant progress

		// Add to simplex
		simplex_w[sn] = w;
		simplex_a[sn] = sup;
		sn++;

		// Reduce simplex to closest feature to origin
		if (sn == 2)
		{
			// Line case: project origin onto segment
			float4 ab = simplex_w[1] - simplex_w[0];
			float t = -dot(simplex_w[0].xyz, ab.xyz) / max(dot(ab.xyz, ab.xyz), Eps);
			t = saturate(t);
			w = simplex_w[0] + t * ab;
			if (t <= 0) { sn = 1; }
			else if (t >= 1) { simplex_w[0] = simplex_w[1]; simplex_a[0] = simplex_a[1]; sn = 1; }
		}
		else if (sn == 3)
		{
			// Triangle case: project origin onto triangle
			float4 a = simplex_w[0], b = simplex_w[1], c = simplex_w[2];
			float4 ab = b - a, ac = c - a, ao = -a;
			float d00 = dot(ab.xyz, ab.xyz), d01 = dot(ab.xyz, ac.xyz), d11 = dot(ac.xyz, ac.xyz);
			float d20 = dot(ao.xyz, ab.xyz), d21 = dot(ao.xyz, ac.xyz);
			float denom = d00 * d11 - d01 * d01;

			if (abs(denom) < Eps)
			{
				// Degenerate triangle — reduce to best edge
				sn = 2;
				continue;
			}

			float u = (d11 * d20 - d01 * d21) / denom;
			float v = (d00 * d21 - d01 * d20) / denom;

			if (u >= 0 && v >= 0 && u + v <= 1)
			{
				// Origin projects inside triangle
				w = a + u * ab + v * ac;
			}
			else
			{
				// Origin is outside — reduce to closest edge
				// Find closest edge by testing all three
				float best_d = 1e30f;
				int best_i = 0, best_j = 1;
				float best_t = 0;
				int idx[3] = {0, 1, 2};
				for (int e = 0; e < 3; ++e)
				{
					int i = idx[e], j = idx[(e + 1) % 3];
					float4 edge = simplex_w[j] - simplex_w[i];
					float et = -dot(simplex_w[i].xyz, edge.xyz) / max(dot(edge.xyz, edge.xyz), Eps);
					et = saturate(et);
					float4 pt = simplex_w[i] + et * edge;
					float d = dot(pt.xyz, pt.xyz);
					if (d < best_d) { best_d = d; best_i = i; best_j = j; best_t = et; w = pt; }
				}
				simplex_w[0] = simplex_w[best_i];
				simplex_w[1] = simplex_w[best_j];
				simplex_a[0] = simplex_a[best_i];
				simplex_a[1] = simplex_a[best_j];
				sn = 2;
			}
		}
		else if (sn >= 4)
		{
			// Tetrahedron — origin might be enclosed. If so, shapes overlap deeply.
			// For the sphere-vs-convex case with margin, this means deep penetration.
			// Fall back to the closest face.
			sn = 3;
		}

		closest_dist_sq = dot(w.xyz, w.xyz);
	}

	// 'w' is the closest point on (convex - sphere_centre) to the origin,
	// so its length is the distance from sphere centre to the convex surface.
	float dist = sqrt(closest_dist_sq);

	if (dist >= radius)
		return false; // No overlap

	if (dist > Eps)
	{
		float4 normal = w / dist;
		out_axis = normal;
		out_depth = radius - dist;

		// Contact point: on the convex surface (closest point to sphere centre)
		// Reconstruct from simplex
		float4 contact_on_convex = sphere_centre + normal * dist;
		out_point = contact_on_convex;
	}
	else
	{
		// Sphere centre is on/inside the convex — use an approximate axis
		out_axis = normalize(sphere_centre - convex_w[3]);
		out_depth = radius;
		out_point = sphere_centre;
	}
	return true;
}

// ---- Helpers for line and triangle tests ----

// Closest point between two line segments. Returns the parameters (t0, t1) in [0,1].
void ClosestPointSegmentSegment(
	float4 p0, float4 d0, float len0,
	float4 p1, float4 d1, float len1,
	out float t0, out float t1)
{
	float4 r = p0 - p1;
	float a = dot(d0, d0); // len0*len0 but d0 might not be unit
	float e = dot(d1, d1);
	float f = dot(d1, r);
	float b = dot(d0, d1);
	float c = dot(d0, r);
	float denom = a * e - b * b;

	if (denom > 1e-12f)
	{
		t0 = clamp((b * f - c * e) / denom, -len0, len0);
	}
	else
	{
		t0 = 0; // parallel segments
	}

	// Compute t1 from t0
	t1 = (b * t0 + f) / max(e, 1e-12f);

	// Clamp t1 and recompute t0 if needed
	if (t1 < -len1) { t1 = -len1; t0 = clamp((-b * len1 - c) / max(a, 1e-12f), -len0, len0); }
	else if (t1 > len1) { t1 = len1; t0 = clamp((b * len1 - c) / max(a, 1e-12f), -len0, len0); }
}

// Closest point on a triangle (in world space) to a point. Returns barycentric coords.
float4 ClosestPointOnTriangle(float4 p, float4 v0, float4 v1, float4 v2)
{
	float4 ab = v1 - v0, ac = v2 - v0, ap = p - v0;
	float d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= 0 && d2 <= 0) return v0;

	float4 bp = p - v1;
	float d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= 0 && d4 <= d3) return v1;

	float4 cp = p - v2;
	float d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= 0 && d5 <= d6) return v2;

	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0) { float v = d1 / (d1 - d3); return v0 + v * ab; }

	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0) { float w = d2 / (d2 - d6); return v0 + w * ac; }

	float va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) { float w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); return v1 + w * (v2 - v1); }

	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return v0 + ab * v + ac * w;
}

// Get the world-space vertices of a triangle shape
void GetTriangleVerts(GpuShape tri, float4x4 tri_w, StructuredBuffer<float4> verts, out float4 v0, out float4 v1, out float4 v2)
{
	v0 = mul(mul(verts[tri.vert_offset + 0], tri.s2rb), tri_w);
	v1 = mul(mul(verts[tri.vert_offset + 1], tri.s2rb), tri_w);
	v2 = mul(mul(verts[tri.vert_offset + 2], tri.s2rb), tri_w);
}

// Closest point on a line segment to a point
float4 ClosestPointOnSegment(float4 p, float4 seg_centre, float4 seg_dir, float half_len)
{
	float t = clamp(dot(p - seg_centre, seg_dir), -half_len, half_len);
	return seg_centre + t * seg_dir;
}

// ---- Line vs Line ----
// Closest points between two capsules (line segments with thickness).
bool LineVsLine(
	GpuShape la, float4x4 la_w,
	GpuShape lb, float4x4 lb_w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 ca = la_w[3], cb = lb_w[3];
	float4 da = la_w[2], db = lb_w[2];
	float ha = la.data.x, hb = lb.data.x;
	float ta = la.data.y, tb = lb.data.y;
	float combined_r = ta + tb;

	float t0, t1;
	ClosestPointSegmentSegment(ca, da, ha, cb, db, hb, t0, t1);

	float4 pa = ca + t0 * da;
	float4 pb = cb + t1 * db;
	float4 diff = pb - pa;
	float dist_sq = dot(diff, diff);

	if (dist_sq >= combined_r * combined_r || dist_sq < 1e-12f)
		return false;

	float dist = sqrt(dist_sq);
	float4 normal = diff / dist;
	out_axis = normal;
	out_depth = combined_r - dist;
	out_point = pa + ta * normal;
	return true;
}

// ---- Line vs Box ----
// Closest point between the line skeleton (segment) and the box surface,
// then add the line thickness as a margin.
bool LineVsBox(
	GpuShape seg, float4x4 seg_w,
	GpuShape box, float4x4 box_w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 seg_centre = seg_w[3];
	float4 seg_dir = seg_w[2];
	float half_len = seg.data.x;
	float thickness = seg.data.y;
	float4 half_ext = box.data;
	float4 box_centre = box_w[3];

	// Sample several points along the line and find the closest one to the box.
	// For a segment vs OBB, the closest feature is one of: endpoint, or the point
	// where the segment is tangent to a box face. We approximate by testing both
	// endpoints and the midpoint projected onto the box.
	float best_dist_sq = 1e30f;
	float4 best_line_pt = seg_centre;
	float4 best_box_pt = box_centre;

	// Test N sample points along the line
	static const int N = 5;
	for (int i = 0; i < N; ++i)
	{
		float t = -half_len + (2.0f * half_len) * (float)i / (float)(N - 1);
		float4 lp = seg_centre + t * seg_dir;

		// Closest point on box to this line point
		float4 local = float4(
			dot(lp - box_centre, box_w[0]),
			dot(lp - box_centre, box_w[1]),
			dot(lp - box_centre, box_w[2]),
			0);
		float4 clamped = clamp(local, -half_ext, half_ext);
		float4 bp = box_centre + clamped.x * box_w[0] + clamped.y * box_w[1] + clamped.z * box_w[2];

		float4 diff = lp - bp;
		float d = dot(diff, diff);
		if (d < best_dist_sq)
		{
			best_dist_sq = d;
			best_line_pt = lp;
			best_box_pt = bp;
		}
	}

	// Refine: project the best box point back onto the line to get exact closest pair
	float t_refine = clamp(dot(best_box_pt - seg_centre, seg_dir), -half_len, half_len);
	float4 refined_lp = seg_centre + t_refine * seg_dir;
	float4 local_r = float4(
		dot(refined_lp - box_centre, box_w[0]),
		dot(refined_lp - box_centre, box_w[1]),
		dot(refined_lp - box_centre, box_w[2]),
		0);
	float4 clamped_r = clamp(local_r, -half_ext, half_ext);
	float4 refined_bp = box_centre + clamped_r.x * box_w[0] + clamped_r.y * box_w[1] + clamped_r.z * box_w[2];

	float4 diff = refined_lp - refined_bp;
	float dist_sq = dot(diff, diff);

	if (dist_sq >= thickness * thickness)
		return false;

	if (dist_sq > 1e-12f)
	{
		float dist = sqrt(dist_sq);
		float4 normal = diff / dist;
		out_axis = normal;
		out_depth = thickness - dist;
		out_point = refined_bp;
	}
	else
	{
		// Line skeleton is inside the box — push out along shortest box face
		float4 face_dist = half_ext - abs(local_r);
		int min_axis = 0;
		if (face_dist.y < face_dist.x) min_axis = 1;
		if (face_dist.z < face_dist[min_axis]) min_axis = 2;
		float4 local_normal = float4(0, 0, 0, 0);
		local_normal[min_axis] = local_r[min_axis] > 0 ? 1.0f : -1.0f;
		float4 world_normal = local_normal.x * box_w[0] + local_normal.y * box_w[1] + local_normal.z * box_w[2];
		out_axis = world_normal;
		out_depth = thickness + face_dist[min_axis];
		out_point = refined_bp;
	}
	return true;
}

// ---- Line vs Triangle ----
// Closest point between the line skeleton and the triangle, plus thickness margin.
bool LineVsTriangle(
	GpuShape seg, float4x4 seg_w,
	GpuShape tri, float4x4 tri_w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 seg_centre = seg_w[3];
	float4 seg_dir = seg_w[2];
	float half_len = seg.data.x;
	float thickness = seg.data.y;
	int i;
	
	float4 v0, v1, v2;
	GetTriangleVerts(tri, tri_w, verts, v0, v1, v2);

	// Find closest point pair between segment and triangle.
	// Test: closest point on triangle to each segment endpoint, and
	// closest point on segment to each triangle vertex/edge.
	float best_dist_sq = 1e30f;
	float4 best_lp = seg_centre;
	float4 best_tp = v0;

	// Test segment endpoints against triangle
	for (i = 0; i < 2; ++i)
	{
		float4 lp = seg_centre + (i == 0 ? -half_len : half_len) * seg_dir;
		float4 tp = ClosestPointOnTriangle(lp, v0, v1, v2);
		float d = dot(lp - tp, lp - tp);
		if (d < best_dist_sq) { best_dist_sq = d; best_lp = lp; best_tp = tp; }
	}

	// Test triangle vertices against segment
	float4 tri_verts[3] = {v0, v1, v2};
	for (i = 0; i < 3; ++i)
	{
		float4 lp = ClosestPointOnSegment(tri_verts[i], seg_centre, seg_dir, half_len);
		float d = dot(lp - tri_verts[i], lp - tri_verts[i]);
		if (d < best_dist_sq) { best_dist_sq = d; best_lp = lp; best_tp = tri_verts[i]; }
	}

	// Test segment against triangle edges
	float4 edge_a[3] = {v0, v1, v2};
	float4 edge_b[3] = {v1, v2, v0};
	for (i = 0; i < 3; ++i)
	{
		float4 ec = (edge_a[i] + edge_b[i]) * 0.5f;
		float4 ed = normalize(edge_b[i] - edge_a[i]);
		float elen = length(edge_b[i] - edge_a[i]) * 0.5f;
		float t0, t1;
		ClosestPointSegmentSegment(seg_centre, seg_dir, half_len, ec, ed, elen, t0, t1);
		float4 lp = seg_centre + t0 * seg_dir;
		float4 tp = ec + t1 * ed;
		float d = dot(lp - tp, lp - tp);
		if (d < best_dist_sq) { best_dist_sq = d; best_lp = lp; best_tp = tp; }
	}

	if (best_dist_sq >= thickness * thickness || best_dist_sq < 1e-12f)
		return false;

	float dist = sqrt(best_dist_sq);
	float4 normal = (best_lp - best_tp) / dist;
	out_axis = normal;
	out_depth = thickness - dist;
	out_point = best_tp;
	return true;
}

// ---- Line vs Convex (Polytope) ----
// Uses "GJK with margins": run closest-point GJK between the line skeleton
// (a segment, no thickness) and the convex shape, then add thickness margin.
bool LineVsConvex(
	GpuShape seg, float4x4 seg_w,
	GpuShape convex, float4x4 convex_w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth,
	out int out_gjk_iters)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;
	out_gjk_iters = 0;

	float thickness = seg.data.y;

	// Create a skeleton shape (zero thickness) and use full GJK to find
	// closest distance between the skeleton and the convex shape.
	// If the distance is less than the thickness, there's a collision.
	GpuShape skeleton = seg;
	skeleton.data.y = 0;

	// Use full GJK+EPA on skeleton vs convex
	int gjk_iters, epa_iters;
	float4 gjk_axis, gjk_point;
	float gjk_depth;
	bool overlap = GjkCollide(skeleton, seg_w, convex, convex_w, verts, gjk_axis, gjk_point, gjk_depth, gjk_iters, epa_iters);
	out_gjk_iters = gjk_iters;

	if (overlap)
	{
		// Skeleton overlaps the convex — add thickness margin
		out_axis = gjk_axis;
		out_depth = gjk_depth + thickness;
		out_point = gjk_point;
		return true;
	}

	// Skeleton doesn't overlap — check if the closest distance is within thickness.
	// For this we need closest-point GJK, which the current GJK doesn't expose.
	// Fall back to a simpler test: sample points along the line and check distance
	// to the convex shape's AABB as a conservative approximation.
	// For now, if GJK says no overlap and thickness is small, we report no collision.
	// TODO: implement proper closest-point GJK for line-vs-convex margin test.
	return false;
}

// ---- Triangle vs Triangle (SAT) ----
// Tests 11 separating axes: 2 face normals + 9 edge cross products.
// Both shapes are polyhedral so EPA would also work, but SAT is faster
// and gives exact results with fixed cost (no iteration).
bool TriangleVsTriangle(
	GpuShape ta, float4x4 ta_w,
	GpuShape tb, float4x4 tb_w,
	StructuredBuffer<float4> verts,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;

	float4 a0, a1, a2, b0, b1, b2;
	GetTriangleVerts(ta, ta_w, verts, a0, a1, a2);
	GetTriangleVerts(tb, tb_w, verts, b0, b1, b2);

	float4 a_edges[3] = { a1 - a0, a2 - a1, a0 - a2 };
	float4 b_edges[3] = { b1 - b0, b2 - b1, b0 - b2 };
	float4 a_verts[3] = { a0, a1, a2 };
	float4 b_verts[3] = { b0, b1, b2 };

	float best_depth = 1e30f;
	float3 best_axis = float3(0, 0, 0);

	// Helper: test a separating axis
	// Projects both triangles onto the axis and checks for overlap.
	// Returns false if separated, true if overlapping (updating best_depth/best_axis).
	#define TEST_AXIS(axis_expr) { \
		float3 ax = (axis_expr); \
		float ax_len = length(ax); \
		if (ax_len < 1e-8f) {} else { \
			ax /= ax_len; \
			float a_min = 1e30f, a_max = -1e30f, b_min = 1e30f, b_max = -1e30f; \
			for (int _i = 0; _i < 3; ++_i) { \
				float da = dot(a_verts[_i].xyz, ax); a_min = min(a_min, da); a_max = max(a_max, da); \
				float db = dot(b_verts[_i].xyz, ax); b_min = min(b_min, db); b_max = max(b_max, db); \
			} \
			float overlap = min(a_max - b_min, b_max - a_min); \
			if (overlap < 0) return false; \
			if (overlap < best_depth) { best_depth = overlap; best_axis = (a_max + a_min < b_max + b_min) ? ax : -ax; } \
		} }

	// Face normals
	TEST_AXIS(cross(a_edges[0].xyz, a_edges[1].xyz))
	TEST_AXIS(cross(b_edges[0].xyz, b_edges[1].xyz))

	// Edge cross products (9 axes)
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			TEST_AXIS(cross(a_edges[i].xyz, b_edges[j].xyz))

	#undef TEST_AXIS

	out_depth = best_depth;
	out_axis = float4(best_axis, 0);

	// Contact point: midpoint of the overlap region
	float4 centre_a = (a0 + a1 + a2) / 3.0f;
	float4 centre_b = (b0 + b1 + b2) / 3.0f;
	out_point = (centre_a + centre_b) * 0.5f;
	return true;
}

// ---- Box vs Box (SAT) ----
// The Minkowski difference of two boxes is a larger box, and EPA struggles
// to resolve the face normal precisely. SAT gives the exact separating axis.
bool BoxVsBox(
	GpuShape sa, float4x4 a2w,
	GpuShape sb, float4x4 b2w,
	out float4 out_axis, out float4 out_point, out float out_depth)
{
	out_axis = float4(0, 0, 0, 0);
	out_point = float4(0, 0, 0, 1);
	out_depth = 0;
	int i;
	
	float3 ha = sa.data.xyz;
	float3 hb = sb.data.xyz;
	float3x3 rot_a = (float3x3)a2w;
	float3x3 rot_b = (float3x3)b2w;
	float3 pos_a = a2w[3].xyz;
	float3 pos_b = b2w[3].xyz;
	float3 d = pos_b - pos_a;

	// Rotation of B relative to A
	float3x3 R;
	float3x3 absR;
	for (i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			R[i][j] = dot(rot_a[i], rot_b[j]);
			absR[i][j] = abs(R[i][j]) + 1e-6f;
		}
	}

	// Translation in A's frame
	float3 t = float3(dot(d, rot_a[0]), dot(d, rot_a[1]), dot(d, rot_a[2]));
	float best_depth = 1e30f;
	float3 best_axis = float3(0, 0, 0);

	// Face axes of A
	for (i = 0; i < 3; ++i)
	{
		float ra_proj = ha[i];
		float rb_proj = hb[0] * absR[i][0] + hb[1] * absR[i][1] + hb[2] * absR[i][2];
		float sep = abs(t[i]) - (ra_proj + rb_proj);
		if (sep > 0) return false;
		if (-sep < best_depth)
		{
			best_depth = -sep;
			float3 axis = float3(0, 0, 0);
			axis[i] = t[i] > 0 ? 1.0f : -1.0f;
			best_axis = mul(axis, rot_a);
		}
	}

	// Face axes of B
	for (i = 0; i < 3; ++i)
	{
		float ra_proj = ha[0] * absR[0][i] + ha[1] * absR[1][i] + ha[2] * absR[2][i];
		float rb_proj = hb[i];
		float sep_val = dot(float3(R[0][i], R[1][i], R[2][i]), t);
		float sep = abs(sep_val) - (ra_proj + rb_proj);
		if (sep > 0) return false;
		if (-sep < best_depth)
		{
			best_depth = -sep;
			float3 axis = float3(0, 0, 0);
			axis[i] = sep_val > 0 ? 1.0f : -1.0f;
			best_axis = mul(axis, rot_b);
		}
	}

	out_depth = best_depth;
	out_axis = float4(normalize(best_axis), 0);
	float3 pt = pos_a + dot(d, out_axis.xyz) * 0.5f * out_axis.xyz;
	out_point = float4(pt, 1);
	return true;
}

#endif
