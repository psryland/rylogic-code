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
//   - Direct analytic tests (sphere-sphere, box-sphere, line-sphere, line-line)
//   - "GJK with margins": run closest-point GJK on the core shape (sphere centre
//     or line skeleton) vs the convex shape, then add the radius/thickness margin
//     analytically. This avoids EPA entirely and gives exact normals.
//
// Collision dispatch matrix (15 unique pairs from 5 shape types):
//
//   Analytic (no GJK):
//     Sphere vs Sphere         — distance test
//     Box vs Sphere            — closest point on OBB
//     Line vs Sphere           — closest point on segment + combined radii
//     Line vs Line             — closest segment-segment + combined thickness
//     Line vs Box              — iterative closest point on OBB + thickness
//     Triangle vs Line         — closest segment-triangle + thickness
//     Box vs Box               — SAT (15 axes)
//     Triangle vs Box          — SAT (13 axes)
//     Triangle vs Triangle     — SAT (2 face + 9 edge axes)
//
//   GJK with margins (GJK on core shape, add radius analytically):
//     Triangle vs Sphere       — GJK point-vs-triangle + radius
//     Polytope vs Sphere       — GJK point-vs-polytope + radius
//     Polytope vs Line         — GJK skeleton-vs-polytope + thickness
//
//   Topology SAT:
//     Polytope vs Box
//     Polytope vs Triangle
//     Polytope vs Polytope
#ifndef PR_PHYSICS_COLLISION_HLSLI
#define PR_PHYSICS_COLLISION_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/closest_point.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/gjk.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// ---- Constants ----
static const float Eps = 1e-8f;
static const float ContactStrictFeatureTol = 1e-4f;
static const float ContactFaceFeatureTol = 2e-2f;
static const float ContactParallelFaceTol = 1.0f - 0.5f * ContactFaceFeatureTol * ContactFaceFeatureTol;

// ---- Helpers ----
// Get the world-space vertices of a triangle shape
odr void GetTriangleVerts(in_(GpuShape) tri, float4x4 tri_w, in_(StructuredBuffer<float4>) verts, out_(float4) v0, out_(float4) v1, out_(float4) v2)
{
	v0 = mul(float4(verts[tri.vert_offset + 0].xyz, 1), tri_w);
	v1 = mul(float4(verts[tri.vert_offset + 1].xyz, 1), tri_w);
	v2 = mul(float4(verts[tri.vert_offset + 2].xyz, 1), tri_w);
}

// Clip a line segment's parametric range [t0,t1] against a half-plane.
// Half-plane: dot(P - plane_pt, plane_n) >= 0. Returns false if fully clipped.
odr bool ClipSegmentToHalfPlane(float3 seg_s, float3 seg_e, float3 plane_pt, float3 plane_n, inout_(float) t0, inout_(float) t1)
{
	if (t0 >= t1)
		return false;

	float ds = dot(seg_s - plane_pt, plane_n);
	float de = dot(seg_e - plane_pt, plane_n);

	if (ds >= 0 && de >= 0) return true;
	if (ds < 0 && de < 0) { t1 = t0; return false; }

	float t_cross = ds / (ds - de);
	if (ds < 0)
		t0 = max(t0, t_cross);
	else
		t1 = min(t1, t_cross);

	return t0 < t1;
}

// Determine the support feature of a box in a given direction.
// Returns the feature vertex count (1=vert, 2=edge, 4=quad) and fills 'pts'.
// Mirrors the CPU SupportFeature() in support.h.
odr int BoxSupportFeature(float3 pos, float3x3 rot, float3 h, float3 dir, float feature_tol, arrayout_(float3, pts, 4))
{
	pts[0] = pts[1] = pts[2] = pts[3] = pos;
	int count = 1;

	for (int i = 0; i != 3; ++i)
	{
		float d = dot(rot[i], dir);
		if (d > feature_tol)
		{
			for (int f = 0; f < count; ++f)
				pts[f] += rot[i] * h[i];
		}
		else if (d < -feature_tol)
		{
			for (int f = 0; f < count; ++f)
				pts[f] -= rot[i] * h[i];
		}
		else if (count == 1)
		{
			// Vert → Edge
			pts[1] = pts[0];
			pts[0] += rot[i] * h[i];
			pts[1] -= rot[i] * h[i];
			count = 2;
		}
		else if (count == 2)
		{
			// Edge → Quad
			pts[3] = pts[0];
			pts[2] = pts[1];
			pts[0] += rot[i] * h[i];
			pts[1] += rot[i] * h[i];
			pts[2] -= rot[i] * h[i];
			pts[3] -= rot[i] * h[i];

			// Fix winding so face normal matches 'dir'
			if (dot(dir, cross(pts[1] - pts[0], pts[2] - pts[0])) < 0)
			{
				float3 tmp = pts[1];
				pts[1] = pts[3];
				pts[3] = tmp;
			}
			count = 4;
		}
	}
	return count;
}
odr bool BoxHasSupportFace(float3x3 rot, float3 dir)
{
	float len_sq = dot(dir, dir);
	if (len_sq < Eps * Eps)
		return false;

	dir *= rsqrt(len_sq);
	float3 local_dir = float3(dot(rot[0], dir), dot(rot[1], dir), dot(rot[2], dir));
	return max(max(abs(local_dir.x), abs(local_dir.y)), abs(local_dir.z)) > ContactParallelFaceTol;
}
odr int BoxSupportFeatureContact(float3 pos, float3x3 rot_a, float3x3 rot_b, float3 h, float3 dir, arrayout_(float3, pts, 4))
{
	// Resting face contacts need a little angular slack so tiny rotations do not collapse the manifold to an edge or a vertex.
	// Gate it on a matching face from the other box so arbitrary corner contacts keep the strict support feature.
	float feature_tol = BoxHasSupportFace(rot_a, dir) && BoxHasSupportFace(rot_b, -dir) ? ContactFaceFeatureTol : ContactStrictFeatureTol;
	return BoxSupportFeature(pos, rot_a, h, dir, feature_tol, pts);
}

// Determine the support feature of a line segment (possibly with thickness) in a given direction.
// Returns 1 (vert) or 2 (edge). 'line_dir' should be unit length, 'axis' may be unnormalised.
// Mirrors the CPU SupportFeature(ShapeLine) in support.h.
odr int LineSupportFeature(float3 line_pos, float3 line_dir, float hlength, float line_r, float3 axis, arrayout_(float3, pts, 4))
{
	// Relaxed tolerance for the "axis perpendicular to line" case: the edge feature
	// includes both endpoints, so we want to fall through to it whenever the dot
	// product is small. Matches CPU SupportFeature(ShapeLine) and SupportFeature_Line in gjk.hlsli.
	float threshold = 1e-3f;
	float d = dot(axis, line_dir);

	// Hemispherical thickness offset: add line_r * axis_hat (axis normalised).
	// Note: ShapeLine has cylindrical sides (no hemispherical end-caps), but the support
	// point for SAT still shifts by the thickness in the axis direction.
	float3 thick_off = float3(0, 0, 0);
	if (line_r > 0)
	{
		float len_sq = dot(axis, axis);
		if (len_sq > 1e-12f)
			thick_off = line_r * axis * rsqrt(len_sq);
	}

	if (d > threshold)
	{
		pts[0] = line_pos + hlength * line_dir + thick_off;
		return 1;
	}
	if (d < -threshold)
	{
		pts[0] = line_pos - hlength * line_dir + thick_off;
		return 1;
	}

	// Axis perpendicular to line direction — both endpoints form the edge feature
	pts[0] = line_pos - hlength * line_dir + thick_off;
	pts[1] = line_pos + hlength * line_dir + thick_off;
	return 2;
}

// ---- Sphere vs Sphere ----
// Direct distance test between two sphere centres.
odr bool SphereVsSphere(
	in_(GpuShape) sa, float4x4 a2w_,
	in_(GpuShape) sb, float4x4 b2w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 a2w = mul(sa.s2rb, a2w_);
	float4x4 b2w = mul(sb.s2rb, b2w_);

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
	float depth = radii_sum - dist;
	float3 pt = ca + normal * (ra - depth * 0.5f);
	ContactSetPoint(out_contact, float4(normal, 0), float4(pt, 1), depth);
	return true;
}

// ---- Box vs Sphere ----
// Find the closest point on the OBB to the sphere centre.
// If the distance is less than the sphere radius, there's a collision.
odr bool BoxVsSphere(
	in_(GpuShape) box, float4x4 box_w_,
	in_(GpuShape) sphere, float4x4 sphere_w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 box_w = mul(box.s2rb, box_w_);
	float4x4 sphere_w = mul(sphere.s2rb, sphere_w_);

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

		// Axis from box toward sphere.
		float4 axis = world_normal;
		float depth = radius - dist;

		// Contact point: midpoint between box surface and sphere surface
		float4 closest_world = box_centre
			+ clamped.x * box_w[0]
			+ clamped.y * box_w[1]
			+ clamped.z * box_w[2];
		ContactSetPoint(out_contact, axis, closest_world - (0.5f * depth) * axis, depth);
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

	// Axis from box toward sphere.
	float4 axis = world_normal;
	float depth = radius + face_dist[min_axis];

	// Contact point: midpoint between box face and sphere surface
	float4 face_pt = local;
	face_pt[min_axis] = local_normal[min_axis] * half_ext[min_axis];
	float4 face_world = box_centre
		+ face_pt.x * box_w[0]
		+ face_pt.y * box_w[1]
		+ face_pt.z * box_w[2];
	ContactSetPoint(out_contact, axis, face_world - (0.5f * depth) * axis, depth);
	return true;
}

// ---- Line vs Sphere ----
// Find the closest point on the line segment to the sphere centre.
// The line has hemispherical end-caps of thickness 'data.y'.
odr bool LineVsSphere(
	in_(GpuShape) seg, float4x4 seg_w_,
	in_(GpuShape) sphere, float4x4 sphere_w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 seg_w = mul(seg.s2rb, seg_w_);
	float4x4 sphere_w = mul(sphere.s2rb, sphere_w_);

	float4 sphere_centre = sphere_w[3];
	float sphere_r = sphere.data.x;
	float hlength = seg.data.x;
	float thickness = seg.data.y;
	float4 seg_centre = seg_w[3];
	float4 seg_dir = seg_w[2]; // Z axis is the line direction

	// Project sphere centre onto line axis
	float4 to_sphere = sphere_centre - seg_centre;
	float t = dot(to_sphere, seg_dir);
	t = clamp(t, -hlength, hlength);

	// Closest point on line segment
	float4 closest = seg_centre + t * seg_dir;
	float4 diff = sphere_centre - closest;
	float dist_sq = dot(diff, diff);
	float combined_r = sphere_r + thickness;

	if (dist_sq >= combined_r * combined_r || dist_sq < 1e-12f)
		return false;

	float dist = sqrt(dist_sq);
	// Normal points from line toward sphere.
	float4 normal = diff / dist;
	float depth = combined_r - dist;
	// Contact point is the midpoint between the sphere surface and the line surface.
	//   line_surface   = closest + thickness * normal       (line surface toward sphere)
	//   sphere_surface = sphere_centre - sphere_r * normal  (sphere surface toward line)
	//   midpoint       = 0.5 * (closest + sphere_centre + (thickness - sphere_r) * normal)
	float4 mid = 0.5f * (closest + sphere_centre + (thickness - sphere_r) * normal);
	ContactSetPoint(out_contact, normal, float4(mid.xyz, 1), depth);
	return true;
}

// ---- Line vs Line ----
// SAT-style contact between two capsules (line segments with optional cylindrical thickness).
//
// Algorithm (mirrors CPU col_line_vs_line.h + support.h FindContactManifold):
//   1. Run segment-to-segment closest-point to find the candidate contact axis.
//   2. Determine the contact axis with three fallback levels:
//        a. closest points distinct  → axis = (pb - pa) / dist          (already A→B)
//        b. lines intersect (cross of directions valid) → axis = normalised cross
//        c. parallel coincident      → arbitrary perpendicular to line direction
//   3. Reject if the closest distance exceeds the combined thickness envelope.
//   4. Build support features for both lines via LineSupportFeature (perpendicular axis
//      yields a 2-point edge feature, allowing the manifold builder to emit a parallel
//      overlap segment instead of collapsing to a single midpoint).
//   5. Hand the feature pair to FindContactManifold.
odr bool LineVsLine(
	in_(GpuShape) la, float4x4 la_w_,
	in_(GpuShape) lb, float4x4 lb_w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 la_w = mul(la.s2rb, la_w_);
	float4x4 lb_w = mul(lb.s2rb, lb_w_);

	float4 ca = la_w[3], cb = lb_w[3];
	float4 da = la_w[2], db = lb_w[2];
	float ha = la.data.x, hb = lb.data.x;
	float ta = la.data.y, tb = lb.data.y;

	// Match CPU semantics (col_line_vs_line.h:54-57): zero-thickness lines register near-contact.
	const float tol = 1e-4f;
	float effective_r = max(ta + tb, tol);

	float2 t = ClosestPoint_SegmentToSegment(ca, da, ha, cb, db, hb);
	float4 pa = ca + t.x * da;
	float4 pb = cb + t.y * db;
	float3 diff = (pb - pa).xyz;
	float dist_sq = dot(diff, diff);

	if (dist_sq >= effective_r * effective_r)
		return false;

	float dist = sqrt(dist_sq);
	float depth = effective_r - dist;

	// Determine the contact axis (oriented A→B).
	float3 axis;
	if (dist_sq > 1e-12f)
	{
		axis = diff / dist;
	}
	else
	{
		// Lines intersect — use the cross of the two line directions.
		float3 cr = cross(da.xyz, db.xyz);
		float cr_sq = dot(cr, cr);
		if (cr_sq > 1e-12f)
		{
			axis = cr * rsqrt(cr_sq);

			// Tie-break orientation only when centres are clearly separated along this
			// axis. When they are not, the cross-product sign is itself the canonical
			// answer and centre-based flipping just adds noise.
			float orient = dot(axis, (cb - ca).xyz);
			if (abs(orient) > 1e-4f && orient < 0)
				axis = -axis;
		}
		else
		{
			// Parallel and coincident — pick any perpendicular to the line direction.
			float3 alt = abs(da.x) < 0.9f ? float3(1, 0, 0) : float3(0, 1, 0);
			axis = normalize(cross(da.xyz, alt));
		}
	}

	// Build support features (axis convention: +axis for A's exterior, -axis for B's exterior).
	// LineSupportFeature returns 2 points (the full edge) when the axis is perpendicular to the line,
	// which lets FindContactManifold's edge-edge path emit a parallel-overlap manifold.
	float3 ptsA[4], ptsB[4];
	int countA = LineSupportFeature(ca.xyz, da.xyz, ha, ta, +axis, ptsA);
	int countB = LineSupportFeature(cb.xyz, db.xyz, hb, tb, -axis, ptsB);

	GpuFeature featA, featB;
	FeatureClear(featA);
	FeatureClear(featB);
	featA.count = countA;
	featB.count = countB;
	for (int i = 0; i != countA; ++i) featA.points[i] = float4(ptsA[i], 1);
	for (int j = 0; j != countB; ++j) featB.points[j] = float4(ptsB[j], 1);
	FindContactManifold(featA, featB, float4(axis, 0), depth, out_contact);
	return true;
}

// ---- Triangle Vs Box ----
// SAT contact between a triangle and an OBB, mirrors the CPU col_triangle_vs_box.h algorithm.
//
// Tests 13 candidate separating axes:
//   - 1 triangle face normal
//   - 3 box face normals
//   - 9 triangle-edge x box-edge axes
odr bool TriangleVsBox(
	in_(GpuShape) tri, float4x4 tri_w_,
	in_(GpuShape) box, float4x4 box_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 tri_w = mul(tri.s2rb, tri_w_);
	float4x4 box_w = mul(box.s2rb, box_w_);

	float4 w0, w1, w2;
	GetTriangleVerts(tri, tri_w, verts, w0, w1, w2);

	float3 hb = box.data.xyz;
	float3 box_pos = box_w[3].xyz;
	float3x3 rot_b = (float3x3)box_w;

	float3 d0 = w0.xyz - box_pos;
	float3 d1 = w1.xyz - box_pos;
	float3 d2 = w2.xyz - box_pos;
	float3 v0 = float3(dot(d0, rot_b[0]), dot(d0, rot_b[1]), dot(d0, rot_b[2]));
	float3 v1 = float3(dot(d1, rot_b[0]), dot(d1, rot_b[1]), dot(d1, rot_b[2]));
	float3 v2 = float3(dot(d2, rot_b[0]), dot(d2, rot_b[1]), dot(d2, rot_b[2]));

	float3 tri_edges[3] = { v1 - v0, v2 - v1, v0 - v2 };
	float best_depth = 1e30f;
	float3 best_axis = float3(1, 0, 0);

	#define TEST_AXIS(axis_expr) { \
		float3 ax = (axis_expr); \
		float ax_len_sq = dot(ax, ax); \
		if (ax_len_sq >= 1e-16f) { \
			ax *= rsqrt(ax_len_sq); \
			float p0 = dot(ax, v0); \
			float p1 = dot(ax, v1); \
			float p2 = dot(ax, v2); \
			float tri_min = min(min(p0, p1), p2); \
			float tri_max = max(max(p0, p1), p2); \
			float radius = dot(abs(ax), hb); \
			float depth = min(tri_max + radius, radius - tri_min); \
			if (depth < 0) return false; \
			if (depth < best_depth) { \
				best_depth = depth; \
				best_axis = (tri_min + tri_max <= 0.0f) ? ax : -ax; \
			} \
		} \
	}

	TEST_AXIS(cross(tri_edges[0], tri_edges[1]))

	for (int i = 0; i != 3; ++i)
	{
		float3 box_axis = float3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f);
		TEST_AXIS(box_axis)
	}

	for (int i = 0; i != 3; ++i)
	{
		for (int j = 0; j != 3; ++j)
		{
			float3 box_axis = float3(j == 0 ? 1.0f : 0.0f, j == 1 ? 1.0f : 0.0f, j == 2 ? 1.0f : 0.0f);
			TEST_AXIS(cross(tri_edges[i], box_axis))
		}
	}

	#undef TEST_AXIS

	if (best_depth <= 0)
		return false;

	float3 axis_w = best_axis.x * rot_b[0] + best_axis.y * rot_b[1] + best_axis.z * rot_b[2];
	float4 axis = NormaliseSafe(float4(axis_w, 0), float4(1, 0, 0, 0));
	FindContactManifold(tri, tri_w, box, box_w, axis, best_depth, verts, out_contact);
	return true;
}

// ---- Triangle vs Line ----
// SAT-based contact, mirrors the CPU col_triangle_vs_line.h algorithm.
//
// Tests 4 candidate separating axes:
//   - 1 triangle face normal             (face-vs-anything contact)
//   - 3 triangle edges × line direction  (edge-vs-edge contact; degenerate axes are skipped)
//
// For each axis, projects the triangle's 3 vertices and the line interval
// (centre ± half-length·dir, expanded by the line radius) onto the axis. Any axis with
// negative overlap is a separating axis. Otherwise the axis with the smallest overlap is
// the MTV. The axis is oriented immediately at the test step using the projected interval
// centres — global centroid orientation does NOT match the per-axis SAT result.
//
// Strict positive depth required for contact (matches CPU ContactPenetration semantics).
odr bool TriangleVsLine(
	in_(GpuShape) tri, float4x4 tri_w_,
	in_(GpuShape) seg, float4x4 seg_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 tri_w = mul(tri.s2rb, tri_w_);
	float4x4 seg_w = mul(seg.s2rb, seg_w_);

	float4 v0, v1, v2;
	GetTriangleVerts(tri, tri_w, verts, v0, v1, v2);

	float3 seg_centre = seg_w[3].xyz;
	float3 seg_dir = seg_w[2].xyz; // unit (column of orthonormal transform)
	float hlength = seg.data.x;
	float line_radius = seg.data.y;
	float3 seg_half = hlength * seg_dir;

	float best_depth = 1e30f;
	float3 best_axis = float3(0, 0, 0);

	// Project the triangle and the (thickness-expanded) line interval onto a normalised
	// axis. Update best_depth/best_axis with the minimum overlap and orient the chosen
	// axis from the triangle's projected interval toward the line's projected interval.
	// Returns false (via the caller's return) if the axis fully separates the shapes.
	#define TEST_AXIS(axis_expr) { \
		float3 ax = (axis_expr); \
		float ax_len_sq = dot(ax, ax); \
		if (ax_len_sq >= 1e-16f) { \
			ax *= rsqrt(ax_len_sq); \
			float d0 = dot(ax, v0.xyz); \
			float d1 = dot(ax, v1.xyz); \
			float d2 = dot(ax, v2.xyz); \
			float t_min = min(min(d0, d1), d2); \
			float t_max = max(max(d0, d1), d2); \
			float lm = dot(ax, seg_centre); \
			float lr = abs(dot(ax, seg_half)) + line_radius; \
			float l_min = lm - lr; \
			float l_max = lm + lr; \
			float overlap = min(t_max - l_min, l_max - t_min); \
			if (overlap < 0) return false; \
			if (overlap < best_depth) { \
				best_depth = overlap; \
				best_axis = (t_min + t_max <= l_min + l_max) ? ax : -ax; \
			} \
		} \
	}

	// Triangle face normal
	TEST_AXIS(cross((v1 - v0).xyz, (v2 - v1).xyz))

	// Triangle edges × line direction
	TEST_AXIS(cross((v1 - v0).xyz, seg_dir))
	TEST_AXIS(cross((v2 - v1).xyz, seg_dir))
	TEST_AXIS(cross((v0 - v2).xyz, seg_dir))

	#undef TEST_AXIS

	// Strict positive depth — touching/coplanar zero-overlap is not a contact.
	if (best_depth <= 0)
		return false;

	// Build the manifold via the templated helper. It transforms the world axis into each
	// shape's local space, calls SupportFeature (Triangle and Line), brings the resulting
	// features back to world space, and runs the manifold reducer.
	FindContactManifold(tri, tri_w, seg, seg_w, float4(best_axis, 0), best_depth, verts, out_contact);
	return true;
}

// ---- Triangle vs Triangle (SAT) ----
// Tests 11 separating axes: 2 face normals + 9 edge cross products.
// Both shapes are polyhedral so EPA would also work, but SAT is faster
// and gives exact results with fixed cost (no iteration).
odr bool TriangleVsTriangle(
	in_(GpuShape) ta, float4x4 ta_w_,
	in_(GpuShape) tb, float4x4 tb_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 ta_w = mul(ta.s2rb, ta_w_);
	float4x4 tb_w = mul(tb.s2rb, tb_w_);

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

	// Ensure axis points sa→sb (from triangle A toward triangle B)
	float4 centre_a = (a0 + a1 + a2) / 3.0f;
	float4 centre_b = (b0 + b1 + b2) / 3.0f;
	if (dot(best_axis, (centre_b - centre_a).xyz) < 0)
		best_axis = -best_axis;

	FindContactManifold(ta, ta_w, tb, tb_w, float4(best_axis, 0), best_depth, verts, out_contact);
	return true;
}

// ---- Box vs Box (SAT) ----
// The Minkowski difference of two boxes is a larger box, and EPA struggles
// to resolve the face normal precisely. SAT gives the exact separating axis.
odr bool BoxVsBox(
	in_(GpuShape) sa, float4x4 a2w_,
	in_(GpuShape) sb, float4x4 b2w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 a2w = mul(sa.s2rb, a2w_);
	float4x4 b2w = mul(sb.s2rb, b2w_);

	int i;
	
	float3 ha = sa.data.xyz;
	float3 hb = sb.data.xyz;
	float3x3 rot_a = (float3x3) a2w;
	float3x3 rot_b = (float3x3) b2w;
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
		if (sep > 0)
			return false;
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
		if (sep > 0)
			return false;
		if (-sep < best_depth)
		{
			best_depth = -sep;
			float3 axis = float3(0, 0, 0);
			axis[i] = sep_val > 0 ? -1.0f : 1.0f;
			best_axis = mul(axis, rot_b);
		}
	}

	// Edge-edge cross product axes (9 axes: each edge of A × each edge of B).
	// Required for correct SAT — face axes alone miss edge-edge separating planes.
	for (i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			float3 cross_axis = cross(rot_a[i], rot_b[j]);
			float len_sq = dot(cross_axis, cross_axis);

			// Skip near-parallel edges (degenerate cross product)
			if (len_sq < 1e-6f)
				continue;

			float inv_len = rsqrt(len_sq);
			cross_axis *= inv_len;

			// Project half-extents onto the cross axis
			float ra_proj = 0;
			float rb_proj = 0;
			for (int k = 0; k < 3; ++k)
			{
				ra_proj += ha[k] * abs(dot(rot_a[k], cross_axis));
				rb_proj += hb[k] * abs(dot(rot_b[k], cross_axis));
			}

			float sp = abs(dot(d, cross_axis));
			float sep = sp - (ra_proj + rb_proj);
			if (sep > 0)
				return false;

			// Prefer face axes over edge axes when depths are similar,
			// to avoid noisy edge contacts on nearly-flat surfaces.
			if (-sep + 1e-4f < best_depth)
			{
				best_depth = -sep;
				best_axis = dot(d, cross_axis) > 0 ? cross_axis : -cross_axis;
			}
		}
	}

	// Ensure axis points sa→sb (from box A toward box B)
	if (dot(best_axis, d) < 0)
		best_axis = -best_axis;

	float4 axis = float4(normalize(best_axis), 0);

	// Compute support features for both boxes and derive the contact manifold.
	float3 ptsA[4], ptsB[4];
	int countA = BoxSupportFeatureContact(pos_a, rot_a, rot_b, ha, +axis.xyz, ptsA);
	int countB = BoxSupportFeatureContact(pos_b, rot_b, rot_a, hb, -axis.xyz, ptsB);

	GpuFeature featA, featB;
	FeatureClear(featA);
	FeatureClear(featB);
	featA.count = countA;
	featB.count = countB;
	for (int j = 0; j != countA; ++j) featA.points[j] = float4(ptsA[j], 1);
	for (int k = 0; k != countB; ++k) featB.points[k] = float4(ptsB[k], 1);
	FindContactManifold(featA, featB, axis, best_depth, out_contact);
	return true;
}

// ---- Line vs Box ----
// SAT between a capsule (line segment with cylindrical thickness + spherical end caps) and an OBB.
// Mirrors the CPU LineVsBox() algorithm in col_line_vs_box.h.
//
// For each candidate separating axis 'n' (unit, in box space):
//   box projection radius:      rb = |n.x|*hb.x + |n.y|*hb.y + |n.z|*hb.z
//   segment projection radius:  rs = |n . line_axis| * hlength
//   signed centre separation:   d  = n . mid     (box centre is at origin)
//   penetration depth:          rb + rs + line_radius - |d|
// If any axis gives depth <= 0 the shapes are separated. Otherwise the axis with the smallest
// depth is the MTV.
//
// Candidate axes (7 total):
//   - 3 box face normals           (face contacts)
//   - 3 line_axis x box_axis       (edge-edge contacts; skipped if parallel)
//   - 1 closest-corner axis        (corner contacts; the other 6 can't produce this direction)
odr bool LineVsBox(
	in_(GpuShape) seg, float4x4 seg_w_,
	in_(GpuShape) box, float4x4 box_w_,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 seg_w = mul(seg.s2rb, seg_w_);
	float4x4 box_w = mul(box.s2rb, box_w_);

	float hlength = seg.data.x;
	float line_r = seg.data.y;
	float3 hb = box.data.xyz;

	float3 line_pos_w = seg_w[3].xyz;
	float3 line_dir_w = seg_w[2].xyz;
	float3 box_pos_w = box_w[3].xyz;
	float3x3 rot_b = (float3x3) box_w;

	// Work in box space: box is an AABB centred at the origin, line centre at 'mid' with unit direction 'line_axis'.
	float3 d_w = line_pos_w - box_pos_w;
	float3 mid = float3(dot(d_w, rot_b[0]), dot(d_w, rot_b[1]), dot(d_w, rot_b[2]));
	float3 line_axis = float3(dot(line_dir_w, rot_b[0]), dot(line_dir_w, rot_b[1]), dot(line_dir_w, rot_b[2]));

	float best_depth = 1e30f;
	float3 best_axis = float3(1, 0, 0); // unit, in box space

	// Test one unit axis 'n' (in box space) and track the minimum penetration depth.
	#define TEST_AXIS(n) { \
		float3 _n = (n); \
		float _rb = abs(_n.x) * hb.x + abs(_n.y) * hb.y + abs(_n.z) * hb.z; \
		float _rs = abs(dot(_n, line_axis)) * hlength; \
		float _d  = dot(_n, mid); \
		float _depth = _rb + _rs + line_r - abs(_d); \
		if (_depth < best_depth) { best_depth = _depth; best_axis = _n; } \
	}

	// 3 box face normals
	TEST_AXIS(float3(1, 0, 0));
	TEST_AXIS(float3(0, 1, 0));
	TEST_AXIS(float3(0, 0, 1));

	// 3 cross products of line axis with box axes (edge-edge)
	for (int i = 0; i < 3; ++i)
	{
		float3 e = float3(i == 0 ? 1.0f : 0.0f, i == 1 ? 1.0f : 0.0f, i == 2 ? 1.0f : 0.0f);
		float3 n = cross(line_axis, e);
		float len_sq = dot(n, n);

		// Skip if the line is parallel to this box axis — the face axis already covers this case
		if (len_sq <= 1e-12f) continue;
		TEST_AXIS(n * rsqrt(len_sq));
	}

	// Test an axis from the closest box corner to the line.
	// Identifying the corner without scanning all 8: take the closest point on the segment to the box centre,
	// then pick the corner in that octant. Then form the axis between that corner and the closest segment
	// point to the corner. The extra Clamp on box_pt handles the case where a line end sits near a corner.
	{
		float t0 = clamp(-dot(mid, line_axis), -hlength, hlength);
		float3 seg_pt = mid + t0 * line_axis;
		float3 box_pt = float3(
			seg_pt.x >= 0 ? hb.x : -hb.x,
			seg_pt.y >= 0 ? hb.y : -hb.y,
			seg_pt.z >= 0 ? hb.z : -hb.z);
		float t1 = clamp(dot(box_pt - mid, line_axis), -hlength, hlength);
		seg_pt = mid + t1 * line_axis;
		box_pt = clamp(seg_pt, -hb, hb);
		float3 sep = seg_pt - box_pt;
		float len_sq = dot(sep, sep);
		if (len_sq > 1e-12f)
			TEST_AXIS(sep * rsqrt(len_sq));
	}
	#undef TEST_AXIS

	// Negative depth means a separating axis was found
	if (best_depth <= 0)
		return false;

	// Convert axis to world space and orient it to point box -> line (sign from centre separation)
	float3 axis_w = best_axis.x * rot_b[0] + best_axis.y * rot_b[1] + best_axis.z * rot_b[2];
	float sign_m = dot(mid, best_axis) >= 0 ? 1.0f : -1.0f;
	float3 best_axis_b2a_w = axis_w * sign_m;

	// Contact axis convention: from shape A (line) toward shape B (box) — flip the box-to-line axis.
	float3 contact_axis = -best_axis_b2a_w;
	float4 axis = float4(contact_axis, 0);

	// Derive the contact manifold from support features (same pipeline as the CPU).
	// For each shape, pass the axis pointing *into* that shape's exterior.
	float3 ptsA[4], ptsB[4];
	int countA = LineSupportFeature(line_pos_w, line_dir_w, hlength, line_r, +contact_axis, ptsA);
	int countB = BoxSupportFeature(box_pos_w, rot_b, hb, -contact_axis, ContactStrictFeatureTol, ptsB);

	GpuFeature featA, featB;
	FeatureClear(featA);
	FeatureClear(featB);
	featA.count = countA;
	featB.count = countB;
	for (int i = 0; i != countA; ++i) featA.points[i] = float4(ptsA[i], 1);
	for (int j = 0; j != countB; ++j) featB.points[j] = float4(ptsB[j], 1);
	FindContactManifold(featA, featB, axis, best_depth, out_contact);
	return true;
}

// ---- Polytope vs Polytope ----
odr float4 PolytopeFaceNormal(in_(GpuShape) shape, float4x4 s2w, uint face_index, in_(StructuredBuffer<GpuPolytopeFace>) faces)
{
	GpuPolytopeFace face = faces[shape.face_offset + face_index];
	return NormaliseSafe(float4(mul(float4(face.plane.xyz, 0), s2w).xyz, 0), float4(1, 0, 0, 0));
}
odr void PolytopeProject(in_(GpuShape) shape, float4x4 s2w, float4 axis, in_(StructuredBuffer<float4>) verts, out_(float) min_dist, out_(float) max_dist)
{
	min_dist = +1e30f;
	max_dist = -1e30f;
	for (int i = 0; i != shape.vert_count; ++i)
	{
		float4 vert = float4(verts[shape.vert_offset + i].xyz, 1);
		float dist = dot(axis.xyz, mul(vert, s2w).xyz);
		min_dist = min(min_dist, dist);
		max_dist = max(max_dist, dist);
	}
}
odr bool PolytopeTestAxis(in_(GpuShape) shape_a, float4x4 a2w, in_(GpuShape) shape_b, float4x4 b2w, float4 axis, in_(StructuredBuffer<float4>) verts, inout_(float) best_depth, inout_(float4) best_axis)
{
	float axis_len_sq = length_sq(axis.xyz);
	if (axis_len_sq < 1e-12f)
		return true;

	axis = float4(axis.xyz * rsqrt(axis_len_sq), 0);

	float a_min, a_max, b_min, b_max;
	PolytopeProject(shape_a, a2w, axis, verts, a_min, a_max);
	PolytopeProject(shape_b, b2w, axis, verts, b_min, b_max);

	float depth = min(a_max - b_min, b_max - a_min);
	if (depth < 0)
		return false;

	if (depth < best_depth)
	{
		best_depth = depth;
		best_axis = (a_min + a_max <= b_min + b_max) ? axis : -axis;
	}

	return true;
}
odr void BoxProject(in_(GpuShape) box, float4x4 box2w, float4 axis, out_(float) min_dist, out_(float) max_dist)
{
	float centre = dot(axis.xyz, box2w[3].xyz);
	float radius =
		box.data.x * abs(dot(axis.xyz, box2w[0].xyz)) +
		box.data.y * abs(dot(axis.xyz, box2w[1].xyz)) +
		box.data.z * abs(dot(axis.xyz, box2w[2].xyz));
	min_dist = centre - radius;
	max_dist = centre + radius;
}
odr bool PolytopeBoxTestAxis(in_(GpuShape) poly, float4x4 poly2w, in_(GpuShape) box, float4x4 box2w, float4 axis, in_(StructuredBuffer<float4>) verts, inout_(float) best_depth, inout_(float4) best_axis)
{
	float axis_len_sq = length_sq(axis.xyz);
	if (axis_len_sq < 1e-12f)
		return true;

	axis = float4(axis.xyz * rsqrt(axis_len_sq), 0);

	float poly_min, poly_max, box_min, box_max;
	PolytopeProject(poly, poly2w, axis, verts, poly_min, poly_max);
	BoxProject(box, box2w, axis, box_min, box_max);

	float depth = min(poly_max - box_min, box_max - poly_min);
	if (depth < 0)
		return false;

	if (depth < best_depth)
	{
		best_depth = depth;
		best_axis = (poly_min + poly_max <= box_min + box_max) ? axis : -axis;
	}

	return true;
}
odr void TriangleProject(float4 v0, float4 v1, float4 v2, float4 axis, out_(float) min_dist, out_(float) max_dist)
{
	float d0 = dot(axis.xyz, v0.xyz);
	float d1 = dot(axis.xyz, v1.xyz);
	float d2 = dot(axis.xyz, v2.xyz);
	min_dist = min(min(d0, d1), d2);
	max_dist = max(max(d0, d1), d2);
}
odr bool PolytopeTriangleTestAxis(in_(GpuShape) poly, float4x4 poly2w, float4 v0, float4 v1, float4 v2, float4 axis, in_(StructuredBuffer<float4>) verts, inout_(float) best_depth, inout_(float4) best_axis)
{
	float axis_len_sq = length_sq(axis.xyz);
	if (axis_len_sq < 1e-12f)
		return true;

	axis = float4(axis.xyz * rsqrt(axis_len_sq), 0);

	float poly_min, poly_max, tri_min, tri_max;
	PolytopeProject(poly, poly2w, axis, verts, poly_min, poly_max);
	TriangleProject(v0, v1, v2, axis, tri_min, tri_max);

	float depth = min(poly_max - tri_min, tri_max - poly_min);
	if (depth < 0)
		return false;

	if (depth < best_depth)
	{
		best_depth = depth;
		best_axis = (poly_min + poly_max <= tri_min + tri_max) ? axis : -axis;
	}

	return true;
}
odr bool PolytopeAxisInEdgeNormalCone(in_(GpuShape) shape, float4x4 s2w, in_(GpuPolytopeEdge) edge, float4 axis, in_(StructuredBuffer<GpuPolytopeFace>) faces)
{
	const float Tol = 1e-5f;
	float axis_len_sq = length_sq(axis.xyz);
	if (axis_len_sq < 1e-12f)
		return false;

	axis = float4(axis.xyz * rsqrt(axis_len_sq), 0);
	float4 normal0 = PolytopeFaceNormal(shape, s2w, edge.face0, faces);
	float4 normal1 = PolytopeFaceNormal(shape, s2w, edge.face1, faces);
	float cos_angle = clamp(dot(normal0.xyz, normal1.xyz), -1.0f, +1.0f);
	float det = 1.0f - cos_angle * cos_angle;
	if (det < Tol * Tol)
		return false;

	float dot0 = dot(axis.xyz, normal0.xyz);
	float dot1 = dot(axis.xyz, normal1.xyz);
	float weight0 = (dot0 - cos_angle * dot1) / det;
	float weight1 = (dot1 - cos_angle * dot0) / det;
	return weight0 >= -Tol && weight1 >= -Tol;
}
odr bool PolytopeEdgePairCompatible(in_(GpuShape) shape_a, float4x4 a2w, in_(GpuPolytopeEdge) edge_a, in_(GpuShape) shape_b, float4x4 b2w, in_(GpuPolytopeEdge) edge_b, float4 axis, in_(StructuredBuffer<GpuPolytopeFace>) faces)
{
	return
		(PolytopeAxisInEdgeNormalCone(shape_a, a2w, edge_a, +axis, faces) && PolytopeAxisInEdgeNormalCone(shape_b, b2w, edge_b, -axis, faces)) ||
		(PolytopeAxisInEdgeNormalCone(shape_a, a2w, edge_a, -axis, faces) && PolytopeAxisInEdgeNormalCone(shape_b, b2w, edge_b, +axis, faces));
}
odr bool PolytopeBoxEdgePairCompatible(in_(GpuShape) poly, float4x4 poly2w, in_(GpuPolytopeEdge) edge, float4 axis, in_(StructuredBuffer<GpuPolytopeFace>) faces)
{
	return
		PolytopeAxisInEdgeNormalCone(poly, poly2w, edge, +axis, faces) ||
		PolytopeAxisInEdgeNormalCone(poly, poly2w, edge, -axis, faces);
}
odr bool PolytopeVsPolytope(
	in_(GpuShape) shape_a, float4x4 a2w_,
	in_(GpuShape) shape_b, float4x4 b2w_,
	in_(StructuredBuffer<float4>) verts,
	in_(StructuredBuffer<GpuPolytopeFace>) faces,
	in_(StructuredBuffer<GpuPolytopeEdge>) edges,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	if (shape_a.face_count == 0 || shape_b.face_count == 0 || shape_a.edge_count == 0 || shape_b.edge_count == 0)
		return false;

	float4x4 a2w = mul(shape_a.s2rb, a2w_);
	float4x4 b2w = mul(shape_b.s2rb, b2w_);
	float4x4 a2p = a2w;
	float4x4 b2p = b2w;

	// Recentre only the projection transforms to reduce cancellation; contact generation must still use the unshifted transforms.
	float4 ofs = 0.5f * float4((a2p[3] + b2p[3]).xyz, 0);
	a2p[3] -= ofs;
	b2p[3] -= ofs;

	float best_depth = +1e30f;
	float4 best_axis = float4(1, 0, 0, 0);

	for (int i = 0; i != shape_a.face_count; ++i)
	{
		GpuPolytopeFace face = faces[shape_a.face_offset + i];
		if ((face.flags & POLY_FACE_IGNORE_AXIS) != 0)
			continue;

		float4 axis = mul(float4(face.plane.xyz, 0), a2p);
		if (!PolytopeTestAxis(shape_a, a2p, shape_b, b2p, axis, verts, best_depth, best_axis))
			return false;
	}

	for (int i = 0; i != shape_b.face_count; ++i)
	{
		GpuPolytopeFace face = faces[shape_b.face_offset + i];
		if ((face.flags & POLY_FACE_IGNORE_AXIS) != 0)
			continue;

		float4 axis = mul(float4(face.plane.xyz, 0), b2p);
		if (!PolytopeTestAxis(shape_a, a2p, shape_b, b2p, axis, verts, best_depth, best_axis))
			return false;
	}

	for (int i = 0; i != shape_a.edge_count; ++i)
	{
		GpuPolytopeEdge edge_a = edges[shape_a.edge_offset + i];
		if ((edge_a.flags & POLY_EDGE_IGNORE_AXES) != 0)
			continue;

		float4 dir_a = mul(edge_a.direction, a2p);
		for (int j = 0; j != shape_b.edge_count; ++j)
		{
			GpuPolytopeEdge edge_b = edges[shape_b.edge_offset + j];
			if ((edge_b.flags & POLY_EDGE_IGNORE_AXES) != 0)
				continue;

			float4 dir_b = mul(edge_b.direction, b2p);
			float4 axis = float4(cross(dir_a.xyz, dir_b.xyz), 0);
			if (length_sq(axis.xyz) < 1e-12f)
				continue;

			if (!PolytopeEdgePairCompatible(shape_a, a2p, edge_a, shape_b, b2p, edge_b, axis, faces))
				continue;

			if (!PolytopeTestAxis(shape_a, a2p, shape_b, b2p, axis, verts, best_depth, best_axis))
				return false;
		}
	}

	FindContactManifold(shape_a, a2w, shape_b, b2w, best_axis, best_depth, verts, out_contact);
	return true;
}
odr bool PolytopeVsBox(
	in_(GpuShape) poly, float4x4 poly2w_,
	in_(GpuShape) box, float4x4 box2w_,
	in_(StructuredBuffer<float4>) verts,
	in_(StructuredBuffer<GpuPolytopeFace>) faces,
	in_(StructuredBuffer<GpuPolytopeEdge>) edges,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	if (poly.face_count == 0 || poly.edge_count == 0)
		return false;

	float4x4 poly2w = mul(poly.s2rb, poly2w_);
	float4x4 box2w = mul(box.s2rb, box2w_);
	float4x4 poly2p = poly2w;
	float4x4 box2p = box2w;

	// Recentre only the projection transforms to reduce cancellation; contact generation must still use the unshifted transforms.
	float4 ofs = 0.5f * float4((poly2p[3] + box2p[3]).xyz, 0);
	poly2p[3] -= ofs;
	box2p[3] -= ofs;

	float best_depth = +1e30f;
	float4 best_axis = float4(1, 0, 0, 0);

	for (int i = 0; i != poly.face_count; ++i)
	{
		GpuPolytopeFace face = faces[poly.face_offset + i];
		if ((face.flags & POLY_FACE_IGNORE_AXIS) != 0)
			continue;

		float4 axis = mul(float4(face.plane.xyz, 0), poly2p);
		if (!PolytopeBoxTestAxis(poly, poly2p, box, box2p, axis, verts, best_depth, best_axis))
			return false;
	}

	for (int i = 0; i != 3; ++i)
	{
		float4 axis = float4(box2p[i].xyz, 0);
		if (!PolytopeBoxTestAxis(poly, poly2p, box, box2p, axis, verts, best_depth, best_axis))
			return false;
	}

	for (int i = 0; i != poly.edge_count; ++i)
	{
		GpuPolytopeEdge edge = edges[poly.edge_offset + i];
		if ((edge.flags & POLY_EDGE_IGNORE_AXES) != 0)
			continue;

		float4 poly_dir = mul(edge.direction, poly2p);
		for (int j = 0; j != 3; ++j)
		{
			float4 box_dir = float4(box2p[j].xyz, 0);
			float4 axis = float4(cross(poly_dir.xyz, box_dir.xyz), 0);
			if (length_sq(axis.xyz) < 1e-12f)
				continue;

			if (!PolytopeBoxEdgePairCompatible(poly, poly2p, edge, axis, faces))
				continue;

			if (!PolytopeBoxTestAxis(poly, poly2p, box, box2p, axis, verts, best_depth, best_axis))
				return false;
		}
	}

	FindContactManifold(poly, poly2w, box, box2w, best_axis, best_depth, verts, out_contact);
	return true;
}
odr bool PolytopeVsTriangle(
	in_(GpuShape) poly, float4x4 poly2w_,
	in_(GpuShape) tri, float4x4 tri2w_,
	in_(StructuredBuffer<float4>) verts,
	in_(StructuredBuffer<GpuPolytopeFace>) faces,
	in_(StructuredBuffer<GpuPolytopeEdge>) edges,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	if (poly.face_count == 0 || poly.edge_count == 0)
		return false;

	float4x4 poly2w = mul(poly.s2rb, poly2w_);
	float4x4 tri2w = mul(tri.s2rb, tri2w_);
	float4x4 poly2p = poly2w;
	float4x4 tri2p = tri2w;

	// Recentre only the projection transforms to reduce cancellation; contact generation must still use the unshifted transforms.
	float4 ofs = 0.5f * float4((poly2p[3] + tri2p[3]).xyz, 0);
	poly2p[3] -= ofs;
	tri2p[3] -= ofs;

	float4 v0, v1, v2;
	GetTriangleVerts(tri, tri2p, verts, v0, v1, v2);
	float4 tri_edges[3] = { v1 - v0, v2 - v1, v0 - v2 };

	float best_depth = +1e30f;
	float4 best_axis = float4(1, 0, 0, 0);

	for (int i = 0; i != poly.face_count; ++i)
	{
		GpuPolytopeFace face = faces[poly.face_offset + i];
		if ((face.flags & POLY_FACE_IGNORE_AXIS) != 0)
			continue;

		float4 axis = mul(float4(face.plane.xyz, 0), poly2p);
		if (!PolytopeTriangleTestAxis(poly, poly2p, v0, v1, v2, axis, verts, best_depth, best_axis))
			return false;
	}

	float4 tri_axis = float4(cross(tri_edges[0].xyz, tri_edges[1].xyz), 0);
	if (!PolytopeTriangleTestAxis(poly, poly2p, v0, v1, v2, tri_axis, verts, best_depth, best_axis))
		return false;

	for (int i = 0; i != poly.edge_count; ++i)
	{
		GpuPolytopeEdge edge = edges[poly.edge_offset + i];
		if ((edge.flags & POLY_EDGE_IGNORE_AXES) != 0)
			continue;

		float4 poly_dir = mul(edge.direction, poly2p);
		for (int j = 0; j != 3; ++j)
		{
			float4 axis = float4(cross(poly_dir.xyz, tri_edges[j].xyz), 0);
			if (length_sq(axis.xyz) < 1e-12f)
				continue;

			if (!PolytopeBoxEdgePairCompatible(poly, poly2p, edge, axis, faces))
				continue;

			if (!PolytopeTriangleTestAxis(poly, poly2p, v0, v1, v2, axis, verts, best_depth, best_axis))
				return false;
		}
	}

	FindContactManifold(poly, poly2w, tri, tri2w, best_axis, best_depth, verts, out_contact);
	return true;
}

// ---- Collision Dispatch ----
odr bool CollideShapes(
	in_(GpuShape) a, float4x4 a2w,
	in_(GpuShape) b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	in_(StructuredBuffer<GpuPolytopeFace>) faces,
	in_(StructuredBuffer<GpuPolytopeEdge>) edges,
	out_(GpuContact) out_contact)
{
	// Canonicalise the pair so 'sb' has the higher shape type and can dispatch to the CPU-style HigherVsLower functions.
	GpuShape sa, sb;
	float4x4 wa, wb;
	bool swapped = a.type > b.type;
	if (swapped) { sa = b; sb = a; wa = b2w; wb = a2w; }
	else         { sa = a; sb = b; wa = a2w; wb = b2w; }

	int gjk_iters = 0;

	// sa.type <= sb.type is guaranteed
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
				case SHAPE_SPHERE:   hit = ConvexVsSphere(sb, wb, sa, wa, verts, out_contact, gjk_iters); break;
				case SHAPE_BOX:      hit = TriangleVsBox(sb, wb, sa, wa, verts, out_contact); break;
				case SHAPE_LINE:     hit = TriangleVsLine(sb, wb, sa, wa, verts, out_contact); break;
				case SHAPE_TRIANGLE: hit = TriangleVsTriangle(sa, wa, sb, wb, verts, out_contact); break;
			}
			break;
		}
		case SHAPE_POLYTOPE:
		{
			switch (sa.type)
			{
				case SHAPE_SPHERE:   hit = ConvexVsSphere(sb, wb, sa, wa, verts, out_contact, gjk_iters); break;
				case SHAPE_BOX:      hit = PolytopeVsBox(sb, wb, sa, wa, verts, faces, edges, out_contact); break;
				case SHAPE_LINE:     hit = ConvexVsLine(sb, wb, sa, wa, verts, out_contact, gjk_iters); break;
				case SHAPE_TRIANGLE: hit = PolytopeVsTriangle(sb, wb, sa, wa, verts, faces, edges, out_contact); break;
				case SHAPE_POLYTOPE: hit = PolytopeVsPolytope(sa, wa, sb, wb, verts, faces, edges, out_contact); break;
			}
			break;
		}
	}

	// HigherVsLower functions return contact in canonical order. Flip back if the caller supplied lower-vs-higher.
	if (a.type < b.type && hit)
		ContactFlip(out_contact);

	return hit;
}

#ifdef __cplusplus
}
#endif
#endif
