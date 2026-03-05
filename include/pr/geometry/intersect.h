//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/geometry/common.h"

// Notes:
//  - GPU versions of these functions prefer 'Intercept' because branching is not preferred.
//  - It isn't possible to represent 'no-intercept' if you only return parametric values though, so 'bool' return values are used here.

namespace pr::geometry::intersect
{
	// Given a 2D line that passes through 'p0' and 'd0' and another that passes through 'p1' and 'd1'.
	// Returns true if the lines intersect, false if they don't.
	// Point of intersection is giving by: p0 + intersect.x * d0, p1 + intersect.y * d1
	// Note: returns false if parallel, *even* if colinear.
	inline bool RayVsRay(v2 p0, v2 d0, v2 p1, v2 d1)
	{
		v2 intersect;
		return RayVsRay(p0, d0 - p0, p1, d1 - p1, intersect);
	}
	inline bool RayVsRay(v2 p0, v2 d0, v2 p1, v2 d1, v2& intersect)
	{
		auto da = d0 - p0;
		auto db = d1 - p1;
		auto denom = Cross(da, db);
		if (FEql(denom, 0.0f)) return false;
		auto i = Cross(p0, d0);
		auto j = Cross(p1, d1);
		intersect.x = (da.x * j - db.x * i) / denom;
		intersect.y = (da.y * j - db.y * i) / denom;
		return true;
	}

	// Find the intersection of two 2D line segments
	inline bool LineVsLine(v2 a0, v2 a1, v2 b0, v2 b1, float& ta, float& tb)
	{
		// 'a' and 'b' intersect if there exists values for 'ta' and 'tb' such that: a0 + ta*da == b0 + tb*db
		// Solve for 'ta'
		//  => (a0 + ta*da) x db == (b0 + tb*db) x db        ; cross both sides by 'db' noting db x db == 0
		//    a0 x db + ta*da x db == b0 x db + tb*db x db
		//    ta*da x db == (b0 - a0) x db
		//    ta = (b0 - a0) x db / (da x db)
		//    tb = (b0 - a0) x da / (da x db)    ; similarly, using (da x db) = -(db x da)
		// Four cases:
		//  (da x db) == 0, (b0 - a0) x da == 0 => lines are colinear => degenerates to 1D overlap
		//  (da x db) == 0, (b0 - a0) x da != 0 => lines are parallal but not colinear => no intersect
		//  (da x db) != 0, ta in [0,1] and tb in [0,1] => intersection
		//  no intersection
		// In the 1D overlap case, return 'ta' as the minimum overlap along 'da', and 'tb' as the maximum
		// overlap along 'da' expressed as a point on 'b'. This means the points calculated using 'ta' and 'tb'
		// give the overlap interval.
		auto ab = b0 - a0;
		auto da = a1 - a0;
		auto db = b1 - b0;
		auto denom = Cross(da, db);
		if (!FEql(denom, 0.f)) // not parallel
		{
			ta = Cross(ab, db) / denom;
			tb = Cross(ab, da) / denom;
			return ta >= 0 && ta <= 1 && tb >= 0 && tb <= 1;
		}

		auto numer = Cross(ab, da);
		if (!FEql(numer, 0.f)) // not colinear
			return false;

		auto dd = Dot(da, db);
		auto da_sq = Dot(da, da);
		auto db_sq = Dot(db, db);
		if (FEql(dd, 0.f)) // one or both of 'a' and 'b' are points
		{
			ta = FEql(da_sq, 0.f) ? 0 : Dot(b0 - a0, da) / da_sq;
			tb = FEql(db_sq, 0.f) ? 0 : Dot(a0 - b0, db) / db_sq;
		}
		else if (dd > 0) // 'da' and 'db' in the same direction
		{
			ta = std::max(0.0f, Dot(b0 - a0, da) / da_sq);
			tb = std::min(1.0f, Dot(a1 - b0, db) / db_sq);
		}
		else // 'da' and 'db' are in opposite directions
		{
			ta = std::min(1.0f, Dot(b1 - a0, da) / da_sq);
			tb = std::max(0.0f, Dot(a1 - b0, db) / db_sq);
		}

		return (ta == 0 && tb == 0)
			? FEql(a0, b0)                                  // both 'a' and 'b' are points
			: (ta >= 0 && ta <= 1) && (tb >= 0 && tb <= 1); // one of 'a' or 'b' is a point
	}

	// Clip a 2D line segment to a 2D bounding box.
	// Returns the clipped line in 'A,B'
	inline bool LineVsBBox(v2 a, v2 b, v2 bbox_min, v2 bbox_max, v2& A, v2& B)
	{
		// Cohen–Sutherland clipping algorithm clips a line from
		// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with 
		// diagonal from (xmin, ymin) to (xmax, ymax).

		enum Zone
		{
			Inside = 0,
			Left = 1 << 0,
			Right = 1 << 1,
			Bottom = 1 << 2,
			Top = 1 << 3,
		};

		// Compute the bit code for a point (x, y) using the clip
		// bounded diagonally by (xmin, ymin), and (xmax, ymax)
		auto GetZone = [=](float x, float y) -> int
		{
			int code = Zone::Inside;  // initialised as being inside of [[clip window]]
			code |= (x < bbox_min.x) * Zone::Left;
			code |= (x > bbox_max.x) * Zone::Right;
			code |= (y < bbox_min.y) * Zone::Bottom;
			code |= (y > bbox_max.y) * Zone::Top;
			return code;
		};

		A = a;
		B = b;

		// Get the zones for the start and end of the line
		auto zone0 = GetZone(A.x, A.y);
		auto zone1 = GetZone(B.x, B.y);

		for (;;)
		{
			// Bitwise OR is zero => both points inside the rectangle.
			if ((zone0 | zone1) == 0)
				return true;

			// Bitwise AND is not 0 => both points share an outside zone, so both must be outside window
			if ((zone0 & zone1) != 0)
				return false;

			// At least one end is outside the rectangle; pick it.
			auto zone_out = std::max(zone1, zone0);

			// Find the intersection point using formulas:
			//   slope = (y1 - y0) / (x1 - x0)
			//   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
			//   y = y0 + slope * (xm - x0), where xm is xmin or xmax
			// No need to worry about divide-by-zero because, in each case, the
			// outcode bit being tested guarantees the denominator is non-zero
			float x, y;
			auto dx = B.x - A.x;
			auto dy = B.y - A.y;

			// Point is above the clip window
			if ((zone_out & Zone::Top) != 0)
			{
				x = A.x + dx * (bbox_max.y - A.y) / dy;
				y = bbox_max.y;
			}
			// Point is below the clip window
			else if (zone_out & Zone::Bottom)
			{
				x = A.x + dx * (bbox_min.y - A.y) / dy;
				y = bbox_min.y;
			}
			// Point is to the right of clip window
			else if (zone_out & Zone::Right)
			{
				y = A.y + dy * (bbox_max.x - A.x) / dx;
				x = bbox_max.x;
			}
			// Point is to the left of clip window
			else if (zone_out & Zone::Left)
			{
				y = A.y + dy * (bbox_min.x - A.x) / dx;
				x = bbox_min.x;
			}

			// Move outside point to intersection point to clip and get ready for next pass.
			if (zone_out == zone0)
			{
				A.x = x;
				A.y = y;
				zone0 = GetZone(A.x, A.y);
			}
			else
			{
				B.x = x;
				B.y = y;
				zone1 = GetZone(B.x, B.y);
			}
		}
	}

	// Given a line passing through 's' with direction 'd', and a CCW triangle 'a', 'b', 'c',
	// Returns true if the line pierces triangle.
	// Returns the barycentric coordinates (u,v,w) of the intersection point.
	// If the line pierces from front to back then 'front_to_back' will be 1.0f
	// If the line pierces from back to front then 'front_to_back' will be -1.0f
	// Note about floating point accuracy: always ensure that the line direction and
	// the triangle edges provided to this function have the same direction each time.
	// This ensures the returned results are consistent
	inline bool pr_vectorcall RayVsTriangle(v4 s, v4 d, int dummy, v4 a, v4 b, v4 c)
	{
		v4 bary;
		return RayVsTriangle(s, d, dummy, a, b, c, bary);
	}
	inline bool pr_vectorcall RayVsTriangle(v4 s, v4 d, int dummy, v4 a, v4 b, v4 c, v4& bary)
	{
		float front_to_back;
		return RayVsTriangle(s, d, dummy, a, b, c, front_to_back, bary);
	}
	inline bool pr_vectorcall RayVsTriangle(v4 s, v4 d, int dummy, v4 a, v4 b, v4 c, float& front_to_back, v4& bary)
	{
		(void)dummy;
		v4 sa = a - s;
		v4 sb = b - s;
		v4 sc = c - s;

		// Test if 'line' is on or inside the edges ab, bc, and ca. Done by testing
		// that the signed tetrahedral volumes are all positive
		bary.x = Triple(d, sc, sb);
		bary.y = Triple(d, sa, sc);
		bary.z = Triple(d, sb, sa);

		// Compute the barycentric coordinates (u, v, w) determining the
		// intersection point r, r = u*a + v*b + w*c. Note: If the line lies
		// in the plane of the triangle then 'sum' will be zero
		float sum = bary.x + bary.y + bary.z;
		if (FEql(sum, 0.f))
			return false;

		float denom = 1.0f / sum;
		bary.x *= denom;
		bary.y *= denom;
		bary.z *= denom; // w = 1.0f - u - v;
		front_to_back = (denom > 0.0f) * 2.0f - 1.0f;
		return bary.x > -maths::tiny<float> && bary.y > -maths::tiny<float> && bary.z > -maths::tiny<float>;
	}

	// DEPRECATED
	// Given a line that passes through 's' with direction 'd', and triangle 'abc'.
	// Returns true if the line intersects the triangle and if so, also returns the
	// barycentric coordinates 'u,v,w' and parametric value 't' of the intersection point.
	inline bool pr_vectorcall RayVsTriangle(v4 s, v4 d, int dummy, v4 a, v4 b, v4 c, float* t = nullptr, v4* bary = nullptr, float* f2b = nullptr, float tmin = 0.0f, float tmax = 1.0f)
	{
		(void)dummy;
		v4 ab = b - a;
		v4 ac = c - a;
		v4 es = -d;

		// Compute triangle normal.
		v4 n = Cross(ab, ac);

		// Compute denominator d. If d == 0, the line is parallel to the triangle, so exit early
		float denom = Dot3(es, n);
		if (denom == 0.0f)
			return false;
		
		float sign = Sign(denom);
		denom = sign * denom;

		// Compute intersection 't' value of 'se' with plane of triangle.
		// A ray intersects iff 0 <= t.
		// Segment intersects iff 0 <= t <= 1.
		// Delay dividing by d until intersection has been found to pierce triangle
		v4 as = s - a;
		float T = sign * Dot3(as, n);
		if (T < denom * tmin) return false;
		if (T > denom * tmax) return false;

		// Compute barycentric coordinate components and test if within bounds
		v4 f = Cross(es, as);
		v4 Bary;
		Bary.y = +sign * Dot3(ac, f); if (Bary.y < 0.0f || Bary.y          > denom) return false;
		Bary.z = -sign * Dot3(ab, f); if (Bary.z < 0.0f || Bary.y + Bary.z > denom) return false;

		// Line/segment/ray intersects triangle.
		// Perform delayed division and compute the last barycentric coordinate component
		float ood = 1.0f / denom;
		if (t) { T *= ood; *t = T; }
		if (bary) { Bary.y *= ood; Bary.z *= ood; Bary.x = 1.0f - Bary.y - Bary.z; *bary = Bary.w0(); }
		if (f2b) { *f2b = sign; }
		return true;
	}

	// Given a line passing through 's' with direction 'd', and initial parametric range '[tmin,tmax]',
	// returns true if the line pierces the sphere within the initial range.
	// The sphere is centred on the origin, 's' and 'd' should be in sphere space
	// 'tmin' and 'tmax' should be initialised to -FLT_MAX and FLT_MAX respectively for infinite line intersection.
	// Returns the parametric values of the intersection points.
	inline bool pr_vectorcall RayVsSphere(v4 s, v4 d, float radius, float& tmin, float& tmax)
	{
		auto d_sq = Dot(d, d);
		if (d_sq < maths::tiny<float>)
			return false; // zero length line

		// Find the closest point to the line
		auto c = s - d * (Dot(d, s) / d_sq);
		auto c_sq = Dot(c.xyz, c.xyz);

		// If the closest point is not within the sphere then there is no intersection
		auto rad_sq = radius * radius;
		if (rad_sq < c_sq)
			return false;

		// Get the distance from the closest point to the intersection with the boundary of the sphere
		auto x = Sqrt((rad_sq - c_sq) / d_sq); // include the normalising 1/d in x

		// Get the parametric values of the intersection
		auto offset = d * x;
		auto lstart = c - offset;
		auto lend = c + offset;
		tmin = std::max(tmin, Dot(d, lstart - s) / d_sq);
		tmax = std::min(tmax, Dot(d, lend - s) / d_sq);
		return true;
	}

	// Given a line passing through 's' with direction 'd', and initial parametric range '[tmin,tmax]',
	// returns true if the line pierces the axis aligned box within the initial range.
	// 'tmin' and 'tmax' should be initialised to -FLT_MAX and FLT_MAX respectively for infinite line intersection.
	// Returns the parametric values of the intersection points.
	inline bool pr_vectorcall RayVsBBox(v4 s, v4 d, BBox box, float& tmin, float& tmax)
	{
		auto bb_min = box.Lower();
		auto bb_max = box.Upper();

		// For all three slabs
		for (int i = 0; i != 3; ++i)
		{
			// If the line is parallel to the slab, then no hit if origin not within slab
			if (FEql(d[i], 0.0f))
			{
				if (s[i] < bb_min[i] || s[i] > bb_max[i])
					return false;
			}
			else
			{
				// Compute intersection t value of ray with near and far plane of slab
				auto ood = 1.0f / d[i];
				auto t1 = (bb_min[i] - s[i]) * ood;
				auto t2 = (bb_max[i] - s[i]) * ood;

				// Make t1 be intersection with near plane, t2 with far plane
				if (t1 > t2) std::swap(t1, t2);

				// Compute the intersection of slab intersection intervals
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;

				// Exit with no collision as soon as slab intersection becomes empty
				if (tmin > tmax)
					return false;
			}
		}
		return true;
	}

	// Intersect the line passing through 's' with direction 'd' to 'frustum' returning parametric values 't0' and 't1'.
	// 's' and 'd' must be in 'frustum' space where the frustum apex is at (0,0,0) and grows down the -z axis. (i.e. camera space).
	// Note: this is an accumulative function, 't0' and 't1' must be initialised.
	// Returns true if t0 < t1 i.e. some of the line is within the frustum.
	inline bool pr_vectorcall RayVsFrustum(v4 s, v4 d, Frustum const& frustum, bool accumulative, float& t0, float& t1, bool include_zfar)
	{
		return frustum.clip(s, d, accumulative, t0, t1, include_zfar);
	}

	// Returns true if the ray that passes through 's' and has direction 'd' passes through the infinite plane 'plane'
	// (i.e. returns false if the line and plane are parallel but not coincident
	// Also returns the parametric value of the intercept 't'. 'plane' does not have to be a normalised plane.
	inline bool pr_vectorcall RayVsPlane(v4 s, v4 d, Plane plane, float& t, float tmin, float tmax)
	{
		// Find the distances to the plane for the start and end of the line
		float d0 = distance::PointToPlane(s, plane);
		float d1 = distance::PointToPlane(s+d, plane);

		if (Abs(d0) > maths::tiny<float>)
		{
			float dist = d1 - d0;
			if (Abs(dist) < maths::tiny<float>)
				return false; // Line and plane are parallel

			t = -d0 / dist; // Use similar triangles to find 't'
		}
		return t >= tmin && t < tmax;
	}

	// Test if the line segment starting at 's' and ending at 'e' with initial
	// parametric values 't0' and 't1' to the infinite plane described by 'plane'.
	// The portion of the line on the positive side of the plane is returned, described
	// by updated 't0' and 't1' values. 'plane' does not have to be a normalised plane.
	// Returns true if the interval [t0,t1] is not zero.
	inline bool pr_vectorcall LineVsPlane(Plane plane, v4 s, v4 e, float& t0, float& t1)
	{
		// Find the distances to the plane for the start and end of the line
		float d0 = distance::PointToPlane(s, plane);
		float d1 = distance::PointToPlane(e, plane);
		if (d0 <= 0.0f && d1 <= 0.0f)
			return false;
		if (d0 > 0.0f && d1 > 0.0f)
			return true;

		// Calculate the parametric value at the intercept
		float t = d0 / (d0 - d1);
		if (d0 < 0.0f && t > t0) t0 = t; // Move the start point of the line onto the plane
		if (d0 > 0.0f && t < t1) t1 = t; // Move the end point onto the plane
		return t0 < t1;
	}

	// Test if the line segment starting at 's' and ending at 'e' intersects the infinite plane 'plane'.
	// Returns true if any part of the line is on the positive side of the plane.
	// Parameter aliasing is allowed, i.e. &s_out == &s is allowed
	inline bool pr_vectorcall LineVsPlane(Plane plane, v4 s, v4 e, v4& s_out, v4& e_out)
	{
		float d0 = distance::PointToPlane(s, plane);
		float d1 = distance::PointToPlane(e, plane);
		if (d0 <= 0.0f && d1 <= 0.0f) { s_out = s; e_out = s; return false; }
		if (d0 > 0.0f && d1 > 0.0f) { s_out = s; e_out = e; return true; }

		float t = d0 / (d0 - d1);
		v4 intersept = (e - s) * t;
		if (d0 < 0.0f) s_out = s + intersept; // Move the start point onto the plane
		if (d0 > 0.0f) e_out = s + intersept; // Move the end point onto the plane
		return true;
	}

	// Test if a line segment specified by points 's' and 'e' intersects AABB b
	inline bool pr_vectorcall LineVsBoundingBox(v4 s, v4 e, BBox bbox)
	{
		v4 lineM = (s + e) * 0.5f;     // Line segment midpoint
		v4 lineH = e - lineM;          // Line segment half length vector
		lineM = lineM - bbox.m_centre; // Translate box and segment to origin

		// Try world coordinate axes as separating axes
		float adx = Abs(lineH.x); if (Abs(lineM.x) > bbox.m_radius.x + adx) return false;
		float ady = Abs(lineH.y); if (Abs(lineM.y) > bbox.m_radius.y + ady) return false;
		float adz = Abs(lineH.z); if (Abs(lineM.z) > bbox.m_radius.z + adz) return false;

		// Add in an epsilon term to counteract arithmetic errors when segment is
		// (near) parallel to a coordinate axis
		adx += maths::tiny<float>;
		ady += maths::tiny<float>;
		adz += maths::tiny<float>;

		// Try cross products of segment direction vector with coordinate axes
		// This might be wrong. Compare with 'ClosestPoint_LineSegmentToBBox'
		if (Abs(lineM.y * lineH.z - lineM.z * lineH.y) > bbox.m_radius.y * adz + bbox.m_radius.z * ady) return false;
		if (Abs(lineM.z * lineH.x - lineM.x * lineH.z) > bbox.m_radius.x * adz + bbox.m_radius.z * adx) return false;
		if (Abs(lineM.x * lineH.y - lineM.y * lineH.x) > bbox.m_radius.x * ady + bbox.m_radius.y * adx) return false;

		// No separating axis found; segment must be overlapping AABB
		return true;
	}

	// Clip a line segment to between two parallel planes.
	// 'dist1' is the near plane distance, 'dist2' is the far plane distance
	// Returns true if any part of the line segment is within the slab
	// Parameter aliasing is allowed, i.e. &s_out == &s is allowed
	inline bool pr_vectorcall LineVsSlab(v4 norm, float dist1, float dist2, v4 s, v4 e, v4& s_out, v4& e_out)
	{
		assert(dist1 <= dist2);
		auto plane = Plane(norm, dist1);

		float slab_width = dist2 - dist1;
		float d1 = distance::PointToPlane(s, plane);
		float d2 = distance::PointToPlane(e, plane);
		if (d1 < 0.0f && d2 < 0.0f) { e_out = s_out = s; return false; }
		if (d1 > slab_width && d2 > slab_width) { e_out = s_out = s; return false; }

		v4 start = s;
		v4 line = e - s;
		float dsum = d1 - d2;
		if (d1 < 0.0f) { float p = d1 / dsum;                s_out = start + line * p; } // Intercept with the near plane
		else if (d1 > slab_width) { float p = (d1 - slab_width) / dsum; s_out = start + line * p; } // Intercept with the far plane
		if (d2 < 0.0f) { float p = d1 / dsum;                e_out = start + line * p; } // Intercept with the near plane
		else if (d2 > slab_width) { float p = (d1 - slab_width) / dsum; e_out = start + line * p; } // Intercept with the far plane
		return true;
	}

	// Returns true if 'bbox' intersects 'plane'
	inline bool pr_vectorcall BBoxVsPlane(BBox bbox, Plane plane)
	{
		// Project the box onto the plane normal
		auto r = Dot(Abs(plane.direction()), bbox.m_radius);

		// Compute distance of box center from plane
		auto s = Dot(plane, bbox.m_centre);

		// Intersection occurs when distance s falls within [-r,+r] interval
		return Abs(s) <= r;
	}

	// Returns true if 'lhs' and 'rhs' intersect
	inline bool pr_vectorcall BBoxVsBBox(BBox lhs, BBox rhs)
	{
		return
			Abs(lhs.m_centre.x - rhs.m_centre.x) <= (lhs.m_radius.x + rhs.m_radius.x) &&
			Abs(lhs.m_centre.y - rhs.m_centre.y) <= (lhs.m_radius.y + rhs.m_radius.y) &&
			Abs(lhs.m_centre.z - rhs.m_centre.z) <= (lhs.m_radius.z + rhs.m_radius.z);
	}

	// Returns true if 'lhs' and 'rhs' intersect
	inline bool pr_vectorcall OBoxVsOBox(OBox const& lhs, OBox const& rhs)
	{
		auto l2w = lhs.m_box_to_world;
		auto r2w = rhs.m_box_to_world;

		// Compute a transform for 'rhs' in 'lhs's frame
		auto r2l = InvertAffine(l2w) * r2w;

		// Compute common sub expressions. Add in an epsilon term to counteract arithmetic
		// errors when two edges are parallel and their cross product is (near) 0
		auto r2l_abs = Abs(r2l.rot) + m3x4(maths::tiny<float>);

		// Lambda for returning a separating axis with the correct sign
		auto sep_axis = [&](v4 sa) { return Sign(Dot(r2l.pos, sa)) * sa; };

		float ra, rb, sp;

		// Test axes L = lhs.x, L = lhs.y, L = lhs.z
		for (int i = 0; i != 3; ++i)
		{
			ra = lhs.m_radius[i];
			rb = rhs.m_radius.x * r2l_abs.x[i] + rhs.m_radius.y * r2l_abs.y[i] + rhs.m_radius.z * r2l_abs.z[i];
			sp = Abs(r2l.pos[i]);
			if (Sign(ra + rb - sp) < 0)
				return false;
		}

		// Test axes L = rhs.x, L = rhs.y, L = rhs.z
		for (int i = 0; i != 3; ++i)
		{
			ra = Dot3(lhs.m_radius, r2l_abs[i]);
			rb = rhs.m_radius[i];
			sp = Abs(Dot3(r2l.pos, r2l[i]));
			if (Sign(ra + rb - sp) < 0)
				return false;
		}

		// Test axis L = lhs.x X rhs.x
		ra = lhs.m_radius.y * r2l_abs.x.z + lhs.m_radius.z * r2l_abs.x.y;
		rb = rhs.m_radius.y * r2l_abs.z.x + rhs.m_radius.z * r2l_abs.y.x;
		sp = Abs(r2l.pos.z * r2l.x.y - r2l.pos.y * r2l.x.z);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.x X rhs.y
		ra = lhs.m_radius.y * r2l_abs.y.z + lhs.m_radius.z * r2l_abs.y.y;
		rb = rhs.m_radius.x * r2l_abs.z.x + rhs.m_radius.z * r2l_abs.x.x;
		sp = Abs(r2l.pos.z * r2l.y.y - r2l.pos.y * r2l.y.z);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.x X rhs.z
		ra = lhs.m_radius.y * r2l_abs.z.z + lhs.m_radius.z * r2l_abs.z.y;
		rb = rhs.m_radius.x * r2l_abs.y.x + rhs.m_radius.y * r2l_abs.x.x;
		sp = Abs(r2l.pos.z * r2l.z.y - r2l.pos.y * r2l.z.z);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.y X rhs.x
		ra = lhs.m_radius.x * r2l_abs.x.z + lhs.m_radius.z * r2l_abs.x.x;
		rb = rhs.m_radius.y * r2l_abs.z.y + rhs.m_radius.z * r2l_abs.y.y;
		sp = Abs(r2l.pos.x * r2l.x.z - r2l.pos.z * r2l.x.x);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.y X rhs.y
		ra = lhs.m_radius.x * r2l_abs.y.z + lhs.m_radius.z * r2l_abs.y.x;
		rb = rhs.m_radius.x * r2l_abs.z.y + rhs.m_radius.z * r2l_abs.x.y;
		sp = Abs(r2l.pos.x * r2l.y.z - r2l.pos.z * r2l.y.x);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.y X rhs.z
		ra = lhs.m_radius.x * r2l_abs.z.z + lhs.m_radius.z * r2l_abs.z.x;
		rb = rhs.m_radius.x * r2l_abs.y.y + rhs.m_radius.y * r2l_abs.x.y;
		sp = Abs(r2l.pos.x * r2l.z.z - r2l.pos.z * r2l.z.x);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.z X rhs.x
		ra = lhs.m_radius.x * r2l_abs.x.y + lhs.m_radius.y * r2l_abs.x.x;
		rb = rhs.m_radius.y * r2l_abs.z.z + rhs.m_radius.z * r2l_abs.y.z;
		sp = Abs(r2l.pos.y * r2l.x.x - r2l.pos.x * r2l.x.y);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.z X rhs.y
		ra = lhs.m_radius.x * r2l_abs.y.y + lhs.m_radius.y * r2l_abs.y.x;
		rb = rhs.m_radius.x * r2l_abs.z.z + rhs.m_radius.z * r2l_abs.x.z;
		sp = Abs(r2l.pos.y * r2l.y.x - r2l.pos.x * r2l.y.y);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// Test axis L = lhs.z X rhs.z
		ra = lhs.m_radius.x * r2l_abs.z.y + lhs.m_radius.y * r2l_abs.z.x;
		rb = rhs.m_radius.x * r2l_abs.y.z + rhs.m_radius.y * r2l_abs.x.z;
		sp = Abs(r2l.pos.y * r2l.z.x - r2l.pos.x * r2l.z.y);
		if (Sign(ra + rb - sp) < 0)
			return false;

		// No separating axis found, must be intersecting
		return true;
	}

	// Find the region of intersection between two convex polygons.
	// 'out' receives the vertices of the intersection polygon, in winding order
	inline bool pr_vectorcall ConvexPolygonVsConvexPolygon(v4 const* poly0, int count0, v4 const* poly1, int count1, v4 norm, std::invocable<v4> auto& out)
	{
		#if 0
		pr::v4 last_out;

		// Determine whether we're starting inside or outside of 'poly1'
		auto inside = PointWithinConvexPolygon(poly0[0], poly1, count1);
		if (inside)
			out(last_out = poly0[0])

			// Generate the planes for polyA
			auto planes = PR_ALLOCA_POD(pr::Plane, count0);
		for (auto i = 0; i != count0; ++i)
		{
			planes[i] = Cross(norm, poly0[i] - poly0[i - 1]);
			planes[i].w = Dot3(planes[i]
		}

		// For each edge of poly0, clip against edges of poly1
		for (auto i0 = 0; i0 != count0; ++i0)
		{

			for (auto i1 = 0; i1 != count1; ++i1)
			{
				Intersect_LineSegmentToPlane()
			}
		}

		auto polyA = poly0;
		auto polyB = poly1;

		if (count0 < count1)
		{

		}
		return false;
		#else
		throw std::exception("Not implemented");
		#endif
	}
}

// See 'unit_tests.h'