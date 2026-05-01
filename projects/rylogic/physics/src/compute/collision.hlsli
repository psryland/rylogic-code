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
//     Box vs Box               — SAT (6 face axes)
//     Triangle vs Triangle     — SAT (2 face + 9 edge axes)
//
//   GJK with margins (GJK on core shape, add radius analytically):
//     Triangle vs Sphere       — GJK point-vs-triangle + radius
//     Polytope vs Sphere       — GJK point-vs-polytope + radius
//     Polytope vs Line         — GJK skeleton-vs-polytope + thickness
//
//   GJK + EPA (both shapes polyhedral, Minkowski boundary is polyhedral):
//     Triangle vs Box
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

// ---- Helpers ----
// Get the world-space vertices of a triangle shape
inline void GetTriangleVerts(in_(GpuShape) tri, float4x4 tri_w, in_(StructuredBuffer<float4>) verts, out_(float4) v0, out_(float4) v1, out_(float4) v2)
{
	v0 = mul(float4(verts[tri.vert_offset + 0].xyz, 1), tri_w);
	v1 = mul(float4(verts[tri.vert_offset + 1].xyz, 1), tri_w);
	v2 = mul(float4(verts[tri.vert_offset + 2].xyz, 1), tri_w);
}

// Clip a line segment's parametric range [t0,t1] against a half-plane.
// Half-plane: dot(P - plane_pt, plane_n) >= 0. Returns false if fully clipped.
inline bool ClipSegmentToHalfPlane(float3 seg_s, float3 seg_e, float3 plane_pt, float3 plane_n, inout_(float) t0, inout_(float) t1)
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
inline int BoxSupportFeature(float3 pos, float3x3 rot, float3 h, float3 dir, arrayout_(float3, pts, 4))
{
	float threshold = 1e-4f;
	pts[0] = pts[1] = pts[2] = pts[3] = pos;
	int count = 1;

	for (int i = 0; i != 3; ++i)
	{
		float d = dot(rot[i], dir);
		if (d > threshold)
		{
			for (int f = 0; f < count; ++f)
				pts[f] += rot[i] * h[i];
		}
		else if (d < -threshold)
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

// Determine the support feature of a line segment (possibly with thickness) in a given direction.
// Returns 1 (vert) or 2 (edge). 'line_dir' should be unit length, 'axis' may be unnormalised.
// Mirrors the CPU SupportFeature(ShapeLine) in support.h.
inline int LineSupportFeature(float3 line_pos, float3 line_dir, float hlength, float line_r, float3 axis, arrayout_(float3, pts, 4))
{
	float threshold = 1e-4f;
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
inline bool SphereVsSphere(
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
inline bool BoxVsSphere(
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
inline bool LineVsSphere(
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

// ---- Convex vs Sphere (Triangle/Polytope) ----
// Uses "GJK with margins": find the closest point on the convex shape to the
// sphere centre (treating it as a point), then check distance < radius.
// This avoids EPA entirely and gives exact contact normals.
inline bool ConvexVsSphere(
	in_(GpuShape) convex, float4x4 convex_w_,
	in_(GpuShape) sphere, float4x4 sphere_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact,
	out_(int) out_gjk_iters)
{
	ContactClear(out_contact);
	float4x4 convex_w = mul(convex.s2rb, convex_w_);
	float4x4 sphere_w = mul(sphere.s2rb, sphere_w_);
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
		float depth = radius - dist;

		// Contact point: on the convex surface (closest point to sphere centre)
		// Reconstruct from simplex
		float4 contact_on_convex = sphere_centre + normal * dist;
		ContactSetPoint(out_contact, -normal, contact_on_convex, depth);
	}
	else
	{
		// Sphere centre is on/inside the convex — use an approximate convex-to-sphere axis.
		ContactSetPoint(out_contact, NormaliseSafe(sphere_centre - convex_w[3], float4(1, 0, 0, 0)), sphere_centre, radius);
	}
	return true;
}

// ---- Line vs Line ----
// Closest points between two capsules (line segments with thickness).
inline bool LineVsLine(
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
	float combined_r = ta + tb;

	float2 t = ClosestPoint_SegmentToSegment(ca, da, ha, cb, db, hb);

	float4 pa = ca + t.x * da;
	float4 pb = cb + t.y * db;
	float4 diff = pb - pa;
	float dist_sq = dot(diff, diff);

	if (dist_sq >= combined_r * combined_r || dist_sq < 1e-12f)
		return false;

	float dist = sqrt(dist_sq);
	float4 normal = diff / dist;
	float depth = combined_r - dist;

	// Contact point on the midplane between the two capsule surfaces:
	//   A_surf = pa + ta * normal      (A's surface in the direction of B)
	//   B_surf = pb - tb * normal      (B's surface in the direction of A)
	//   contact = (A_surf + B_surf)/2 = pa + (ta - depth/2) * normal
	ContactSetPoint(out_contact, normal, pa + (ta - 0.5f * depth) * normal, depth);
	return true;
}

// ---- Triangle vs Line ----
// Closest point between the line skeleton and the triangle, plus thickness margin.
inline bool TriangleVsLine(
	in_(GpuShape) tri, float4x4 tri_w_,
	in_(GpuShape) seg, float4x4 seg_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	ContactClear(out_contact);
	float4x4 tri_w = mul(tri.s2rb, tri_w_);
	float4x4 seg_w = mul(seg.s2rb, seg_w_);

	float4 seg_centre = seg_w[3];
	float4 seg_dir = seg_w[2];
	float hlength = seg.data.x;
	float line_radius = seg.data.y;
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
		float4 lp = seg_centre + (i == 0 ? -hlength : hlength) * seg_dir;
		float4 tp = ClosestPoint_PointToTriangle(lp, v0, v1, v2);
		float d = dot(lp - tp, lp - tp);
		if (d < best_dist_sq) { best_dist_sq = d; best_lp = lp; best_tp = tp; }
	}

	// Test triangle vertices against segment
	float4 tri_verts[3] = {v0, v1, v2};
	for (i = 0; i < 3; ++i)
	{
		float4 lp = ClosestPoint_PointToSegment(tri_verts[i], seg_centre, seg_dir, hlength);
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
		float2 t = ClosestPoint_SegmentToSegment(seg_centre, seg_dir, hlength, ec, ed, elen);
		float4 lp = seg_centre + t.x * seg_dir;
		float4 tp = ec + t.y * ed;
		float d = dot(lp - tp, lp - tp);
		if (d < best_dist_sq) { best_dist_sq = d; best_lp = lp; best_tp = tp; }
	}

	if (best_dist_sq >= line_radius * line_radius || best_dist_sq < 1e-12f)
		return false;

	float dist = sqrt(best_dist_sq);
	float4 normal = (best_lp - best_tp) / dist;
	ContactSetPoint(out_contact, normal, best_tp, line_radius - dist);
	return true;
}

// ---- Polytope vs Line ----
// Uses "GJK with margins": run closest-point GJK between the line skeleton
// (a segment, no thickness) and the convex shape, then add thickness margin.
inline bool PolytopeVsLine(
	in_(GpuShape) polytope, float4x4 polytope_w_,
	in_(GpuShape) seg, float4x4 seg_w_,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact,
	out_(int) out_gjk_iters)
{
	ContactClear(out_contact);
	out_gjk_iters = 0;

	float line_radius = seg.data.y;

	// Create a skeleton shape (zero thickness) and use full GJK to find
	// closest distance between the skeleton and the convex shape.
	// If the distance is less than the thickness, there's a collision.
	GpuShape skeleton = seg;
	skeleton.data.y = 0;

	// Use full GJK+EPA on polytope vs skeleton, preserving the higher-vs-lower contact convention.
	int gjk_iters, epa_iters;
	GpuContact gjk_contact;
	bool overlap = GjkCollide(polytope, polytope_w_, skeleton, seg_w_, verts, gjk_contact, gjk_iters, epa_iters);
	out_gjk_iters = gjk_iters;

	if (overlap)
	{
		// Skeleton overlaps the convex — add thickness margin
		out_contact = gjk_contact;
		out_contact.depth = gjk_contact.depth + line_radius;
		return true;
	}

	// Skeleton doesn't overlap — check if the closest distance is within thickness.
	// For this we need closest-point GJK, which the current GJK doesn't expose.
	// Fall back to a simpler test: sample points along the line and check distance
	// to the convex shape's AABB as a conservative approximation.
	// For now, if GJK says no overlap and thickness is small, we report no collision.
	// TODO: implement proper closest-point GJK for polytope-vs-line margin test.
	return false;
}

// ---- Triangle vs Triangle (SAT) ----
// Tests 11 separating axes: 2 face normals + 9 edge cross products.
// Both shapes are polyhedral so EPA would also work, but SAT is faster
// and gives exact results with fixed cost (no iteration).
inline bool TriangleVsTriangle(
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
inline bool BoxVsBox(
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
	int countA = BoxSupportFeature(pos_a, rot_a, ha, +axis.xyz, ptsA);
	int countB = BoxSupportFeature(pos_b, rot_b, hb, -axis.xyz, ptsB);

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
inline bool LineVsBox(
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
	int countB = BoxSupportFeature(box_pos_w, rot_b, hb, -contact_axis, ptsB);

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

// ---- Collision Dispatch ----
inline bool CollideShapes(
	in_(GpuShape) a, float4x4 a2w,
	in_(GpuShape) b, float4x4 b2w,
	in_(StructuredBuffer<float4>) verts,
	out_(GpuContact) out_contact)
{
	// Canonicalise the pair so 'sb' has the higher shape type and can dispatch to the CPU-style HigherVsLower functions.
	GpuShape sa, sb;
	float4x4 wa, wb;
	bool swapped = a.type > b.type;
	if (swapped) { sa = b; sb = a; wa = b2w; wb = a2w; }
	else         { sa = a; sb = b; wa = a2w; wb = b2w; }

	int gjk_iters = 0;
	int epa_iters = 0;

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
				case SHAPE_BOX:      hit = GjkCollide(sb, wb, sa, wa, verts, out_contact, gjk_iters, epa_iters); break;
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
				case SHAPE_BOX:      hit = GjkCollide(sb, wb, sa, wa, verts, out_contact, gjk_iters, epa_iters); break;
				case SHAPE_LINE:     hit = PolytopeVsLine(sb, wb, sa, wa, verts, out_contact, gjk_iters); break;
				case SHAPE_TRIANGLE: hit = GjkCollide(sb, wb, sa, wa, verts, out_contact, gjk_iters, epa_iters); break;
				case SHAPE_POLYTOPE: hit = GjkCollide(sa, wa, sb, wb, verts, out_contact, gjk_iters, epa_iters); break;
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
