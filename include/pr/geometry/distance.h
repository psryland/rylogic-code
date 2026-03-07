//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry::distance
{
	// Return the distance that 'point' is from the infinite plane: 'plane'
	inline float pr_vectorcall PointToPlane(v4 point, v4 a, v4 b, v4 c)
	{
		assert(point.w == 1.0f);
		v4 plane = Normalise(Cross(b - a, c - a));
		plane.w = -Dot3(plane, a);
		return Dot(plane, point);
	}
	inline float pr_vectorcall PointToPlane(v4 point, Plane plane)
	{
		assert(point.w == 1.0f);
		return Dot(plane, point);
	}

	// Return the distance that 'point' is from the infinite line: 'line'
	inline float pr_vectorcall PointToRay(v4 point, v4 start, v4 end)
	{
		v4 line     = end   - start;
		v4 to_point = point - start;
		float p_dot_l = Dot3(to_point, line);
		return Sqrt(LengthSq(to_point) - Sqr(p_dot_l) / LengthSq(line));
	}

	// Return the minimum distance between two infinite lines
	inline float pr_vectorcall RayToRay(v4 s0, v4 line0, v4 s1, v4 line1)
	{
		v4 a = s1 - s0;
		float a_len_sq = LengthSq(a);
		if (a_len_sq == 0.0f)
			return 0.0f;

		v4 b = Cross(line0, line1);
		if (FEql(b, v4::Zero()))
			return Sqrt(a_len_sq - Sqr(Dot3(a, line0)) / LengthSq(line0));
		else
			return Dot3(a, b) / LengthSq(b);
	}

	// Returns the squared distance from 'point' to 'line'
	inline float pr_vectorcall PointToRaySq(v4 point, v4 s, v4 d)
	{
		auto sp   = point - s;
		auto d_sq = Dot3(d,d);
		assert(d_sq != 0.0f && "divide by zero in DistanceSq_PointToInfiniteLine");
		return Dot3(sp,sp) - Sqr(Dot3(sp,d))/d_sq;
	}

	// Returns the squared distance from 'point' to 'line'
	inline float pr_vectorcall PointToLineSq(v4 point, v4 s, v4 e)
	{
		auto a = point - s;
		auto d = e - s;

		float ad = Dot3(a,d);
		if (ad <= 0.0f)
			return LengthSq(a);

		float dd = Dot3(d,d);
		if (ad >= dd)
			return LengthSq(point - e);

		return LengthSq(a) - Sqr(ad) / dd;
	}

	// Returns the squared distance from 'point' to 'bbox'
	inline float pr_vectorcall PointToBoundingBoxSq(v4 point, BBox const& bbox)
	{
		float dist_sq = 0.0f;
		v4 lower = bbox.Lower();
		v4 upper = bbox.Upper();
		if      (point.x < lower.x) dist_sq += Sqr(lower.x - point.x);
		else if (point.x > upper.x) dist_sq += Sqr(point.x - upper.x);
		if      (point.y < lower.y) dist_sq += Sqr(lower.y - point.y);
		else if (point.y > upper.y) dist_sq += Sqr(point.y - upper.y);
		if      (point.z < lower.z) dist_sq += Sqr(lower.z - point.z);
		else if (point.z > upper.z) dist_sq += Sqr(point.z - upper.z);
		return dist_sq;
	}

	// Returns the squared distance from 'point' to a 'triangle'
	inline float pr_vectorcall PointToTriangleSq(v4 point, v4 a, v4 b, v4 c)
	{
		v4 bary;
		auto pt = closest_point::PointToTriangle(point, a, b, c, bary);
		return LengthSq(pt - point);
	}

	// Returns the signed minimum distance between a line segment '(s,e)' and an AABB 'bbox'.
	// 's' and 'e' must be in the same space as 'bbox'. A negative value means the line segment intersects the AABB.
	inline float pr_vectorcall LineToBBox(v4 s, v4 e, BBox bbox)
	{
		auto pen = closest_point::LineToBBox(s, e, bbox);
		return -pen.Depth(); // 'depth' is positive for penetration, in this case we want the opposite.
	}
}

