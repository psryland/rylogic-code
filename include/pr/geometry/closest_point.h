//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/geometry/common.h"

// TODO:
// - Convert these to return parametric values rather than 'points'. Parametric values are more versatile.
// - Don't fret about performance, the optimiser will remove any redundant calculations.

namespace pr::geometry::closest_point
{
	// Returns the point closest to 'point' on 'plane'
	inline v4 pr_vectorcall PointToPlane(v4 point, Plane plane)
	{
		return point - distance::PointToPlane(point, plane) * plane.direction();
	}
	inline v4 pr_vectorcall PointToPlane(v4 point, v4 a, v4 b, v4 c)
	{
		return PointToPlane(point, Plane::FromTriangle(a, b, c));
	}

	// Returns the parametric value of the closest point on 'line'
	inline v4 pr_vectorcall PointToRay(v4 point, v4 s, v4 d, void*, float& t)
	{
		assert(point.w == 1 && s.w == 1.0f && d.w == 0.0f);
		assert(d != v4::Zero());
		t = Dot(point - s, d) / LengthSq(d);
		return s + t * d;
	}
	inline v4 pr_vectorcall PointToRay(v4 point, v4 s, v4 d, void*)
	{
		float t;
		return PointToRay(point, s, d, 0, t);
	}

	// Returns the parametric value of the closest point on 'line'
	inline v4 pr_vectorcall PointToLine(v4 point, v4 s, v4 e, float& t)
	{
		assert(point.w == 1.0f && s.w == 1.0f && e.w == 1.0f);
		auto line = e - s;

		// Project 'point' onto 'line', but defer divide by 'line.LengthSq()'
		t = Dot(point - s, line);
		if (t <= 0.0f)
		{
			// 'point' projects outside 'line', clamp to 0.0f
			t = 0.0f;
			return s;
		}

		auto denom = LengthSq(line);
		if (t >= denom)
		{
			// 'point' projects outside 'line', clamp to 1.0f
			t = 1.0f;
			return e;
		}

		// 'point' projects inside 'line', do deferred divide now
		t = t / denom;
		return s + t * line;
	}
	inline v4 pr_vectorcall PointToLine(v4 point, v4 s, v4 e)
	{
		float t;
		return PointToLine(point, s, e, t);
	}

	// Returns the point on an AABB that is closest to 'point'
	// if 'surface_only' is true, the point is projected onto the bbox surface,
	// otherwise points within the bounding box are counted as the closest point
	inline v4 pr_vectorcall PointToBoundingBox(v4 point, BBox bbox, bool surface_only)
	{
		v4 result;
		v4 lower = bbox.Lower();
		v4 upper = bbox.Upper();
		if (!surface_only || !IsWithin(bbox, point))
		{
			result.x = Clamp(point.x, lower.x, upper.x);
			result.y = Clamp(point.y, lower.y, upper.y);
			result.z = Clamp(point.z, lower.z, upper.z);
		}
		else
		{
			result = point;
			auto centre = bbox.Centre();
			auto upr = upper - point;
			auto lwr = point - lower;
			auto i0 = MinElementIndex(upr.xyz);
			auto i1 = MinElementIndex(lwr.xyz);
			if (upr[i0] < lwr[i1]) result[i0] = upper[i0];
			else                   result[i1] = lower[i1];
		}
		result.w = 1.0f;
		return result;
	}

	// Returns the closest point on an ellipse to 'x','y'.
	// 'x','y' are a point in ellipse space
	// 'major' is the size of the major radius of the ellipse (along the x axis)
	// 'minor' is the size of the minor radius of the ellipse (along the y axis)
	// Note: this is only an approximation, the true solution involves finding the
	// largest root of a quartic equation.
	inline v2 pr_vectorcall PointToEllipse(float x, float y, float major, float minor)
	{
		assert(major >= 0.0f && minor >= 0.0f && major >= minor);

		// Special case minor axis lengths of zero
		if (minor < maths::tiny<float>)
			return v2(Clamp(x, -major, major), 0.0f);

		auto ratio = Sign(y) * minor / (major + maths::tiny<float>); // Add an epsilon to prevent divide by zero
		auto a = Sqr(major), b = Sqr(minor);
		v2 pt(x, y);
		v2 nearest;

		// Binary search along X for the nearest
		float bounds[2] = { -major * (x < 0.0f), major * (x >= 0.0f) };
		do
		{
			nearest.x = 0.5f * (bounds[0] + bounds[1]);
			nearest.y = ratio * Sqrt(a - Sqr(nearest.x));
			v2 tang(nearest.y / b, -nearest.x / a);

			auto d = Sign(y) * Dot(tang, pt - nearest);
			bounds[d < 0.0f] = nearest.x;
		} while (!FEql(bounds[0], bounds[1]));
		return nearest;
	}

	// Returns the closest point on a triangle to 'point'. From "Real time collision detection" by 'Christer Ericson'
	inline v4 pr_vectorcall PointToTriangle(v4 p, v4 a, v4 b, v4 c, v4& barycentric)
	{
		assert(p.w == 1.0f && a.w == 1.0f && b.w == 1.0f && c.w == 1.0f);

		// Check if P in vertex region outside A
		auto ab = b - a;
		auto ac = c - a;
		auto ap = p - a;
		auto d1 = Dot3(ab, ap);
		auto d2 = Dot3(ac, ap);
		if (d1 <= 0.0f && d2 <= 0.0f)
		{
			barycentric = v4(1.0f, 0.0f, 0.0f, 0.0f);
			return a;
		}

		// Check if P in vertex region outside B
		auto bp = p - b;
		auto d3 = Dot3(ab, bp);
		auto d4 = Dot3(ac, bp);
		if (d3 >= 0.0f && d4 <= d3)
		{
			barycentric = v4(0.0f, 1.0f, 0.0f, 0.0f);
			return b;
		}

		// Check if P in edge region of AB, if so return projection of P onto AB
		auto vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		{
			// Barycentric coordinates (1-v, v, 0)
			auto v = d1 / (d1 - d3);
			barycentric = v4(1.0f - v, v, 0.0f, 0.0f);
			return a + v * ab;
		}

		// Check if P in vertex region outside C
		auto cp = p - c;
		auto d5 = Dot3(ab, cp);
		auto d6 = Dot3(ac, cp);
		if (d6 >= 0.0f && d5 <= d6)
		{
			barycentric = v4(0.0f, 0.0f, 1.0f, 0.0f);
			return c;
		}

		// Check if P in edge region of AC, if so return projection of P onto AC
		auto vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		{
			// Barycentric coordinates (1-w, 0, w)
			auto w = d2 / (d2 - d6);
			barycentric = v4(1.0f - w, 0.0f, w, 0.0f);
			return a + w * ac;
		}

		// Check if P in edge region of BC, if so return projection of P onto BC
		auto va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
		{
			// Barycentric coordinates (0, 1-w, w)
			auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			barycentric = v4(0.0f, 1.0f - w, w, 0.0f);
			return b + w * (c - b);
		}

		// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
		//'  = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
		auto denom = 1.0f / (va + vb + vc);
		auto v = vb * denom;
		auto w = vc * denom;
		barycentric = v4(1.0f - v - w, v, w, 0.0f);
		return a + ab * v + ac * w;
	}
	inline v4 pr_vectorcall PointToTriangle(v4 point, v4 a, v4 b, v4 c)
	{
		v4 barycentric;
		return PointToTriangle(point, a, b, c, barycentric);
	}
	inline v4 pr_vectorcall PointToTriangle(v4 point, const v4* tri, v4& barycentric)
	{
		return PointToTriangle(point, tri[0], tri[1], tri[2], barycentric);
	}
	inline v4 pr_vectorcall PointToTriangle(v4 point, const v4* tri)
	{
		v4 barycentric;
		return PointToTriangle(point, tri[0], tri[1], tri[2], barycentric);
	}

	// Returns the closest point on a tetrahedron to 'point'. From "Real time collision detection" by 'Christer Ericson'
	inline v4 pr_vectorcall PointToTetrahedron(v4 p, v4 a, v4 b, v4 c, v4 d, v4& barycentric)
	{
		assert(p.w == 1.0f && a.w == 1.0f && b.w == 1.0f && c.w == 1.0f && d.w == 1.0f);

		// Start out assuming point inside all halfspaces, so closest to itself
		v4 closest_point = p;
		float best_dist_sq = limits<float>::max();
		bool point_is_inside = true;

		// If point outside face abc then compute closest point on abc
		if (PointInFrontOfPlane(p, a, b, c)) // Test face abc
		{
			v4 bary;
			v4 q = PointToTriangle(p, a, b, c, bary);
			float dist_sq = LengthSq(q - p);
			if (dist_sq < best_dist_sq) { best_dist_sq = dist_sq; closest_point = q; barycentric = v4(bary.x, bary.y, bary.z, 0.0f); point_is_inside = false; }
		}
		if (PointInFrontOfPlane(p, a, c, d)) // Test face acd
		{
			v4 bary;
			v4 q = PointToTriangle(p, a, c, d, bary);
			float dist_sq = LengthSq(q - p);
			if (dist_sq < best_dist_sq) { best_dist_sq = dist_sq; closest_point = q; barycentric = v4(bary.x, 0.0f, bary.y, bary.z); point_is_inside = false; }
		}
		if (PointInFrontOfPlane(p, a, d, b)) // Test face adb
		{
			v4 bary;
			v4 q = PointToTriangle(p, a, d, b, bary);
			float dist_sq = LengthSq(q - p);
			if (dist_sq < best_dist_sq) { best_dist_sq = dist_sq; closest_point = q; barycentric = v4(bary.x, bary.z, 0.0f, bary.y); point_is_inside = false; }
		}
		if (PointInFrontOfPlane(p, d, c, b)) // Test face dcb
		{
			v4 bary;
			v4 q = PointToTriangle(p, d, c, b, bary);
			float dist_sq = LengthSq(q - p);
			if (dist_sq < best_dist_sq) { best_dist_sq = dist_sq; closest_point = q; barycentric = v4(0.0f, bary.z, bary.y, bary.x); point_is_inside = false; }
		}
		if (point_is_inside)
		{
			barycentric = v4(0.25f, 0.25f, 0.25f, 0.25f); // This is wrong but it shouldn't be needed
		}
		return closest_point;
	}
	inline v4 pr_vectorcall PointToTetrahedron(v4 point, v4 a, v4 b, v4 c, v4 d)
	{
		v4 barycentric;
		return PointToTetrahedron(point, a, b, c, d, barycentric);
	}
	inline v4 pr_vectorcall PointToTetrahedron(v4 point, const v4* tetra, v4& barycentric)
	{
		return PointToTetrahedron(point, tetra[0], tetra[1], tetra[2], tetra[3], barycentric);
	}
	inline v4 pr_vectorcall PointToTetrahedron(v4 point, const v4* tetra)
	{
		v4 barycentric;
		return PointToTetrahedron(point, tetra[0], tetra[1], tetra[2], tetra[3], barycentric);
	}

	// Returns the parametric values of the closest points on two rays
	// Closest points are: p0 = s0 + return.x * d0, p1 = s1 + return.y * d1
	inline v2 pr_vectorcall RayToRay(v4 s0, v4 d0, v4 s1, v4 d1)
	{
		// Degenerate lines should not be passed to this function
		assert(d0 != v4::Zero());
		assert(d1 != v4::Zero());
		assert(s0.w == 1.0f && d0.w == 0.0f && s1.w == 1.0f && d1.w == 0.0f);

		v4 r = s0 - s1;
		float a = Dot3(d0, d0);
		float b = Dot3(d0, d1);
		float e = Dot3(d1, d1);
		float d = a * e - b * b;
		if (d == 0.0f) // The lines are parallel, return the start of line0 as the nearest point
			return v2{ 0.0f, -Dot(r, d0) / a };

		float c = Dot(d0, r);
		float f = Dot(d1, r);
		return v2{
			(b * f - c * e) / d,
			(a * f - b * c) / d,
		};
	}

	// Finds the closest points between two line segments and also
	// the parametric values on each line.
	// From "Real time collision detection" by 'Christer Ericson'
	inline void pr_vectorcall LineToLine(v4 s0, v4 e0, v4 s1, v4 e1, float& t0, float& t1)
	{
		assert(s0.w == 1.0f && e0.w == 1.0f && s1.w == 1.0f && e1.w == 1.0f);

		auto line0 = e0 - s0;
		auto line1 = e1 - s1;
		auto sep = s0 - s1;
		auto len_sq0 = LengthSq(line0);
		auto len_sq1 = LengthSq(line1);
		auto f = Dot3(line1, sep);
		auto c = Dot3(line0, sep);

		// Check if either or both segments are degenerate
		if (FEql(len_sq0, 0.f) && FEql(len_sq1, 0.f)) { t0 = 0.0f; t1 = 0.0f; return; }
		if (FEql(len_sq0, 0.f)) { t0 = 0.0f; t1 = Clamp(+f / len_sq1, 0.0f, 1.0f); return; }
		if (FEql(len_sq1, 0.f)) { t1 = 0.0f; t0 = Clamp(-c / len_sq0, 0.0f, 1.0f); return; }

		// The general non-degenerate case starts here
		auto b = Dot3(line0, line1);
		auto denom = len_sq0 * len_sq1 - b * b; // Always non-negative

		// If segments not parallel, calculate closest point on infinite line 'line0'
		// to infinite line 'line1', and clamp to segment 1. Otherwise pick arbitrary t0
		t0 = denom != 0.0f ? Clamp((b * f - c * len_sq1) / denom, 0.0f, 1.0f) : 0.0f;

		// Calculate point on infinite line 'line1' closest to segment 'line0' at t0
		// using t1 = Dot3(pt0 - s1, line1) / len_sq1 = (b*t0 + f) / len_sq1
		t1 = (b * t0 + f) / len_sq1;

		// If t1 in [0,1] then done. Otherwise, clamp t1, recompute t0 for the new value
		// of t1 using t0 = Dot3(pt1 - s0, line0) / len_sq0 = (b*t1 - c) / len_sq0
		// and clamped t0 to [0, 1]
		if (t1 < 0.0f)
		{
			t1 = 0.0f;
			t0 = Clamp((0 - c) / len_sq0, 0.0f, 1.0f);
		}
		else if (t1 > 1.0f)
		{
			t1 = 1.0f;
			t0 = Clamp((b - c) / len_sq0, 0.0f, 1.0f);
		}
	}
	inline void pr_vectorcall LineToLine(v4 s0, v4 e0, v4 s1, v4 e1, v4& pt0, v4& pt1)
	{
		float t0, t1;
		LineToLine(s0, e0, s1, e1, t0, t1);
		pt0 = (1.0f - t0) * s0 + t0 * e0;
		pt1 = (1.0f - t1) * s1 + t1 * e1;
	}
	inline void pr_vectorcall LineToLine(v4 s0, v4 e0, v4 s1, v4 e1, v4& pt0, v4& pt1, float& t0, float& t1)
	{
		LineToLine(s0, e0, s1, e1, t0, t1);
		pt0 = (1.0f - t0) * s0 + t0 * e0;
		pt1 = (1.0f - t1) * s1 + t1 * e1;
	}
	inline void pr_vectorcall LineToLine(v4 s0, v4 e0, v4 s1, v4 e1, float& dist_sq)
	{
		float t0, t1;
		LineToLine(s0, e0, s1, e1, t0, t1);
		v4 pt0 = (1.0f - t0) * s0 + t0 * e0;
		v4 pt1 = (1.0f - t1) * s1 + t1 * e1;
		dist_sq = LengthSq(pt1 - pt0);
	}

	// Returns the parametric values of the closest point between a line segment '(s0,e0)'
	// and an infinite line '(s1,line1)'. From "Real time collision detection" by 'Christer Ericson'
	inline void pr_vectorcall LineToRay(v4 s0, v4 e0, v4 s1, v4 line1, float& t0, float& t1)
	{
		assert(s0.w == 1.0f && e0.w == 1.0f && s1.w == 1.0f && line1.w == 0.0f);
		assert(line1 != v4::Zero() && "The infinite line should not be degenerate");

		auto line0 = e0 - s0;
		auto line0_length_sq = LengthSq(line0);
		auto line1_length_sq = LengthSq(line1);
		auto separation = s0 - s1;
		auto s1_on_line0 = -Dot3(separation, line0);
		auto s0_on_line1 = Dot3(separation, line1);

		// Check if the segment is degenerate
		if (FEql(line0_length_sq, 0.f))
		{
			t0 = 0.0f;
			t1 = s0_on_line1 / line1_length_sq; // t0 = 0 => t1 = (b*t0 + f) / line1_length_sq = f / line1_length_sq
			return;
		}

		// The general non-degenerate case starts here
		float b = Dot3(line0, line1);
		float denom = line0_length_sq * line1_length_sq - b * b; // Always non-negative

		// If segments not parallel, calculate closest point on infinite line 'line0'
		// to infinite line 'line1', and clamp to the segment. Otherwise pick arbitrary t0
		t0 = denom != 0.0f ? Clamp<float>((b * s0_on_line1 + s1_on_line0 * line1_length_sq) / denom, 0.0f, 1.0f) : 0.0f;

		// Calculate point on infinite line 'line1' closest to segment 'line0' at t0
		// using t1 = Dot3(pt0 - s1, line1) / line1_length_sq = (b*t0 + f) / line1_length_sq
		t1 = (b * t0 + s0_on_line1) / line1_length_sq;
	}
	inline void pr_vectorcall LineToRay(v4 s0, v4 e0, v4 s1, v4 line1, float& t0, float& t1, float& dist_sq)
	{
		LineToRay(s0, e0, s1, line1, t0, t1);
		v4 pt0 = (1.0f - t0) * s0 + t0 * e0;
		v4 pt1 = s1 + t1 * line1;
		dist_sq = LengthSq(pt0 - pt1);
	}

	// Returns the minimum distance of a line segment '(s,e)' to the AABB 'bbox'
	inline MinSeparation pr_vectorcall LineToBBox(v4 s, v4 e, BBox bbox)
	{
		// Note: This code is basically the same as col_box_vs_line.h.
		// Make sure to maintain both

		// Line segment mid-point
		auto mid = (s + e) * 0.5f;

		// Line segment "radius" plus an epsilon term to counteract arithmetic
		// errors when the segment is (near) parallel to a coordinate axis.
		auto half = e - mid;
		auto rad = Abs(half) + v4::TinyF();

		// Translate box and segment to origin
		mid = mid - bbox.m_centre;

		// Records the minimum penetration (position depth means overlap)
		auto sep = MinSeparation{};

		// Try world coordinate axes
		sep(bbox.m_radius.x + rad.x - Abs(mid.x), Sign(mid.x) * v4::XAxis());
		sep(bbox.m_radius.y + rad.y - Abs(mid.y), Sign(mid.y) * v4::YAxis());
		sep(bbox.m_radius.z + rad.z - Abs(mid.z), Sign(mid.z) * v4::ZAxis());

		// Lambda for returning a separating axis with the correct sign
		auto sep_axis = [&](v4 sa) { return Sign(Dot(mid, sa)) * sa; };

		// Try cross products of the segment direction with the coordinate axes.
		sep(rad.z * bbox.m_radius.y + rad.y * bbox.m_radius.z - rad.z * Abs(mid.y) - rad.y * Abs(mid.z), sep_axis(Cross(v4::XAxis(), half)));
		sep(rad.z * bbox.m_radius.x + rad.x * bbox.m_radius.z - rad.x * Abs(mid.z) - rad.z * Abs(mid.x), sep_axis(Cross(v4::YAxis(), half)));
		sep(rad.y * bbox.m_radius.x + rad.x * bbox.m_radius.y - rad.y * Abs(mid.x) - rad.x * Abs(mid.y), sep_axis(Cross(v4::ZAxis(), half)));

		return sep;
	}
	inline MinSeparation pr_vectorcall LineToBBox(v4 s, v4 e, BBox bbox, float& t)
	{
		// Returns the parametric value of the closest point on the line segment '(s,e)' to the AABB 'bbox' and the distance.
		auto sep = LineToBBox(s, e, bbox);
		auto axis = sep.SeparatingAxis();

		// Get the feature on the box in the direction of 'axis'
		v4 points[4] = {}; auto n = 1;
		for (int i = 0; i != 3; ++i)
		{
			if (axis[i] > 0)
			{
				for (int j = 0; j != n; ++j)
					points[j] += m4x4::Identity()[i] * bbox.m_radius[i];
			}
			else if (axis[i] < 0)
			{
				for (int j = 0; j != n; ++j)
					points[j] -= m4x4::Identity()[i] * bbox.m_radius[i];
			}
			else
			{
				for (int j = 0; j != n; ++j)
				{
					points[j + n] = points[j] + m4x4::Identity()[i] * bbox.m_radius[i];
					points[j] = points[j] - m4x4::Identity()[i] * bbox.m_radius[i];
				}
				n *= 2;
			}
		}

		auto line_s = (s - bbox.m_centre).w1();
		auto line_e = (e - bbox.m_centre).w1();

		// Box corner to line segment
		if (n == 1)
		{
			PointToLine(points[0].w1(), line_s, line_e, t);
			return sep;
		}

		// Box edge to line segment
		if (n == 2)
		{
			float t2;
			LineToLine(line_s, line_e, points[0].w1(), points[1].w1(), t, t2);
			return sep;
		}

		// Box face to line segment
		auto d = Dot(line_e - line_s, axis);
		if (d > 0) { t = 1.0f; return sep; }
		if (d < 0) { t = 0.0f; return sep; }

		// Box face vs. line edge
		//throw std::exception("not implemented");
		//LineSegmentToPlane
		assert("not implemented" && false);
		return sep;
	}
	inline MinSeparation pr_vectorcall LineToBBox(v4 s, v4 e, BBox bbox, v4& pt0, v4& pt1)
	{
		float t;
		auto sep = LineToBBox(s, e, bbox, t);
		pt0 = s + t * (e - s);
		if (t == 0.0f || t == 1.0f)
			pt1 = PointToBoundingBox(pt0, bbox, true);
		else
			pt1 = pt0 + sep.Depth() * sep.SeparatingAxis();

		return sep;
	}

	// Returns the parametric values of the closest point between a ray 's -> s+d' and a triangle 'a,b,c'
	// The closest point on the triangle is at: 'BaryPoint(a, b, c, return.xyz)'
	// The closest point on the ray is at: 's + return.w * d'
	inline v4 pr_vectorcall RayToTriangle(v4 s, v4 d, v4 a, v4 b, v4 c)
	{
		assert(s.w == 1.0f && d.w == 0.0f);
		assert(a.w == 1.0f && b.w == 1.0f && c.w == 1.0f);
		assert(d != v4::Zero());

		// If the ray intersects the triangle, then the intersection point is the closest point
		v4 bary = {};
		if (intersect::RayVsTriangle(s, d, 0, a, b, c, bary))
		{
			// Find the parameteric value on the ray
			auto pt = BaryPoint(a, b, c, bary);
			auto t = Dot(pt - s, d) / LengthSq(d);
			return v4(bary.xyz, t);
		}

		// Otherwise, find the closest point between the ray and the triangle edges/vertices
		else
		{
			struct Edge { v4 s; v4 d; };
			Edge edges[3] = { {a, b - a}, {b, c - b}, {c, a - c} };

			v2 best_t = {};
			int best_edge = -1;
			float best_dist_sq = limits<float>::max();

			// Check distance to each edge
			for (int i = 0; i != 3; ++i)
			{
				auto const& edge = edges[i];
				auto t = closest_point::RayToRay(s, d, edge.s, edge.d);
				t.y = Clamp(t.y, 0.0f, 1.0f); // Clamp t.edge to [0, 1] to stay on the edge segment

				auto pt_on_ray = s + t.x * d;
				auto pt_on_edge = edge.s + t.y * edge.d;
				auto dist_sq = LengthSq(pt_on_ray - pt_on_edge);
				if (dist_sq < best_dist_sq)
				{
					best_dist_sq = dist_sq;
					best_edge = i;
					best_t = t;
				}
			}

			// Find the barycentric coords of the closest point on the triangle
			auto pt = edges[best_edge].s + best_t.y * edges[best_edge].d;
			bary = Barycentric(pt, a, b, c);
			return v4(bary.xyz, best_t.x);
		}
	}
}

// See 'unit_tests.h'