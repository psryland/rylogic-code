//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shapes.h"

namespace pr::collision
{
	// Support features of a collision shape
	enum class EFeature :int
	{
		// Note
		//  - int(EFeature) is used as the number of points returned from 'SupportFeature()'
		Vert = 1,
		Edge = 2,
		Tri  = 3,
		Quad = 4,
		// higher order faces are supported
	};
	static constexpr int EFeatureBits = 3;
	static constexpr int EFeatureMask = (1 << EFeatureBits) - 1;
	static constexpr int FeaturePolygonMaxSides = 8;

	// Returns a support vertex for a shape for a given direction.
	// Assumes 'direction' is in the shape's root parent space (i.e. transformed
	// by Invert(shape2world) but not 'shape.m_s2p' or any nested shapes)
	inline v4 SupportVertex(ShapeSphere const& shape, v4 direction, EFeature& feature_type)
	{
		assert(IsNormalised(direction));
		feature_type = EFeature::Vert;
		return shape.m_base.m_s2p.pos + shape.m_radius * direction;
	}
	inline v4 SupportVertex(ShapeBox const& shape, v4 direction, EFeature& feature_type)
	{
		feature_type = EFeature::Vert;

		auto vert = shape.m_base.m_s2p.pos;
		for (int i = 0; i != 3; ++i)
		{
			float d = Dot3(direction, shape.m_base.m_s2p[i]);

			if      (d > +math::tiny<float>) vert += shape.m_base.m_s2p[i] * shape.m_radius[i];
			else if (d < -math::tiny<float>) vert -= shape.m_base.m_s2p[i] * shape.m_radius[i];
			else feature_type = EFeature(int(feature_type) << 1);
		}
		return vert;
	}
	inline v4 SupportVertex(ShapeLine const& shape, v4 direction, EFeature& feature_type)
	{
		auto d = Dot(direction, shape.m_base.m_s2p.z);
		auto r = shape.m_base.m_s2p.z * shape.m_radius;

		feature_type = EFeature::Vert;
		auto vert = shape.m_base.m_s2p.pos;
		if      (d > +math::tiny<float>) vert += r;
		else if (d < -math::tiny<float>) vert -= r;
		else feature_type = EFeature::Edge;

		// Hemispherical end-cap offset for thick lines
		if (shape.m_thickness > 0)
		{
			auto len_sq = LengthSq(direction);
			if (len_sq > Sqr(math::tiny<float>))
				vert += shape.m_thickness * direction / Sqrt(len_sq);
		}
		return vert;
	}
	inline v4 SupportVertex(ShapeTriangle const& shape, v4 direction, EFeature& feature_type)
	{
		// Triangle vertices are stored as offsets (w=0), return as positions (w=1)
		v4 d(Dot3(direction, shape.m_v.x), Dot3(direction, shape.m_v.y), Dot3(direction, shape.m_v.z), 0.0f);
		feature_type = EFeature::Vert;
		return shape.m_v[MaxElementIndex(d.xyz)].w1();
	}
	template <ShapeType TShape>
	inline v4 SupportVertex(TShape const& shape, v4 direction)
	{
		auto feature_type = EFeature{};
		return SupportVertex(shape, direction, feature_type);
	}

	// Return the feature of the shape in a given direction.
	// 'points' returns the feature polygon. The number of sides equals 'int(feature_type)'
	// Assumes 'axis' is in the shape's root parent space (i.e. transformed by Invert(shape2world) but not 'shape.m_s2p' or any nested shapes)
	// When a face is returned, the points should be in order such that the face normal == 'axis'
	inline void SupportFeature(ShapeSphere const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		points[0] = SupportVertex(shape, axis, feature_type);
		assert(feature_type == EFeature::Vert);
	}
	inline void SupportFeature(ShapeBox const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		feature_type = EFeature::Vert;
		*points = shape.m_base.m_s2p.pos;
		for (int i = 0; i != 3; ++i)
		{
			float d = Dot3(axis, shape.m_base.m_s2p[i]);
			if (d > +math::tiny<float>)
			{
				for (int f = 0; f != int(feature_type); ++f)
					points[f] += shape.m_base.m_s2p[i] * shape.m_radius[i];
			}
			else if (d < -math::tiny<float>)
			{
				for (int f = 0; f != int(feature_type); ++f)
					points[f] -= shape.m_base.m_s2p[i] * shape.m_radius[i];
			}
			else
			{
				switch (feature_type)
				{
				case EFeature::Vert:
					feature_type = EFeature::Edge;
					points[1]  = points[0];
					points[0] += shape.m_base.m_s2p[i] * shape.m_radius[i];
					points[1] -= shape.m_base.m_s2p[i] * shape.m_radius[i];
					break;
				case EFeature::Edge:
					feature_type = EFeature::Quad;
					points[3]  = points[0];
					points[2]  = points[1];
					points[0] += shape.m_base.m_s2p[i] * shape.m_radius[i];
					points[1] += shape.m_base.m_s2p[i] * shape.m_radius[i];
					points[2] -= shape.m_base.m_s2p[i] * shape.m_radius[i];
					points[3] -= shape.m_base.m_s2p[i] * shape.m_radius[i];
					if (Triple(axis, points[1] - points[0], points[2] - points[0]) < 0)
						std::swap(points[1],points[3]); // Flip the winding order
					break;
				}
			}
		}
	}
	inline void SupportFeature(ShapeLine const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		auto d = Dot(axis, shape.m_base.m_s2p.z);
		auto r = shape.m_base.m_s2p.z * shape.m_radius;

		// Hemispherical end-cap offset for thick lines
		auto thickness_offset = v4::Zero();
		if (shape.m_thickness > 0)
		{
			auto len_sq = LengthSq(axis);
			if (len_sq > Sqr(math::tiny<float>))
				thickness_offset = shape.m_thickness * axis / Sqrt(len_sq);
		}

		if (d > +math::tiny<float>)
		{
			// Line points in the direction of the axis, return the end point
			feature_type = EFeature::Vert;
			points[0] = shape.m_base.m_s2p.pos + r + thickness_offset;
		}
		else if (d < -math::tiny<float>)
		{
			// Line points against the direction of the axis, return the start point
			feature_type = EFeature::Vert;
			points[0] = shape.m_base.m_s2p.pos - r + thickness_offset;
		}
		else
		{
			// Line is perpendicular to the axis, return the line (both endpoints offset by thickness)
			feature_type = EFeature::Edge;
			points[0] = shape.m_base.m_s2p.pos - r + thickness_offset;
			points[1] = shape.m_base.m_s2p.pos + r + thickness_offset;
		}
	}
	inline void SupportFeature(ShapeTriangle const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		// Project each vertex onto the axis
		auto d0 = Dot3(axis, shape.m_v.x);
		auto d1 = Dot3(axis, shape.m_v.y);
		auto d2 = Dot3(axis, shape.m_v.z);
		auto d_max = std::max({d0, d1, d2});

		// Count how many vertices are at the maximum projection (within tolerance).
		// 1 vertex  → Vert feature, 2 vertices → Edge feature, 3 vertices → Tri (face) feature.
		auto tol = math::tiny<float>;
		int count = 0;
		int indices[3] = {};
		if (d0 >= d_max - tol) indices[count++] = 0;
		if (d1 >= d_max - tol) indices[count++] = 1;
		if (d2 >= d_max - tol) indices[count++] = 2;

		// Triangle vertices are stored as offsets (w=0), return as positions (w=1)
		feature_type = EFeature(count);
		for (int i = 0; i != count; ++i)
			points[i] = shape.m_v[indices[i]].w1();

		// For the face case, ensure winding order matches the axis direction
		if (count == 3 && Triple(axis, points[1] - points[0], points[2] - points[0]) < 0)
			std::swap(points[1], points[2]);
	}

	// Returns the *single* point of contact between two shapes, 'lhs' and 'rhs'.
	// 'axis' is the collision separating axis.
	// 'pen' is the depth of penetration
	// 'l2w' and 'r2w' transform 'lhs' and 'rhs' into the same space as 'axis' and the space
	// that the contact point is returned in (typically world space).
	template <typename = void>
	v4 FindContactPoint(v4* pointA, EFeature featA, v4* pointB, EFeature featB, m4x4 const& l2w, m4x4 const& r2w, v4 axis, float pen)
	{
		auto countA = int(featA);
		auto countB = int(featB);

		// Transform the contact points to world space
		for (int i = 0; i != countA; ++i) pointA[i] = l2w * pointA[i];
		for (int i = 0; i != countB; ++i) pointB[i] = r2w * pointB[i];

		// Generally, we want to project the points of featureA/B onto 'axis' to find the
		// average position along the axis as the "single point of collision". Since the feature is 
		// perpendicular to the separating axis, the distance along 'axis' will be halfway between
		// the first point from each feature (in the direction of 'axis'. Still need to find the
		// average position perpendicular to 'axis').

		// For features with area, check that the polygon is facing the correct direction, +ve for featA, -ve for featB
		assert((featA <= EFeature::Edge || (Dot(Plane::FromBestFit({ pointA, size_t(countA) }), axis) > 0)) && "Contact polygon has incorrect winding order");
		assert((featB <= EFeature::Edge || (Dot(Plane::FromBestFit({ pointB, size_t(countB) }), axis) < 0)) && "Contact polygon has incorrect winding order");

		// If both shapes contact at a vert, then the separating axis passes through their average position
		if (featA == EFeature::Vert && featB == EFeature::Vert)
			return (pointA[0] + pointB[0]) * 0.5f;

		// If one shape is contacting at a vert, then the separating axis must pass through this vert
		if (featA == EFeature::Vert)
			return pointA[0] + axis * (0.5f * Dot3(axis, pointB[0] - pointA[0]));
		if (featB == EFeature::Vert)
			return pointB[0] + axis * (0.5f * Dot3(axis, pointA[0] - pointB[0]));

		// If this is edge-edge contact, then the separating axis passes through the closest points
		if (featA == EFeature::Edge && featB == EFeature::Edge)
		{
			v4 pt0, pt1;
			geometry::closest_point::LineToLine(pointA[0], pointA[1], pointB[0], pointB[1], pt0, pt1);
			return (pt0 + pt1) * 0.5f;
		}

		// Face-Face or Face-Edge contacts require clipping.
		// Find the geometric intersection of the two polygons (in the plane of 'axis').
		// Return the average position of the remaining verts

		// Generate a container of edges for each feature
		struct Edge { float t0, t1; operator bool() const { return t0 < t1; } };
		auto edgesA = PR_ALLOCA_POD(Edge, countA);
		auto edgesB = PR_ALLOCA_POD(Edge, countB);
		for (int i = 0; i != countA; ++i) { edgesA[i].t0 = 0.0f; edgesA[i].t1 = 1.0f; }
		for (int i = 0; i != countB; ++i) { edgesB[i].t0 = 0.0f; edgesB[i].t1 = 1.0f; }

		// Clip the edges of '1' against the edges of '0'.
		// Note: the winding order for polygon '1' is always the opposite of the winding
		// order of polygon '0'. This is because the SupportFeature() function returns the
		// face in the direction of the support axis which for featB is -axis.
		auto clip = [&](v4 const* point0, int count0, v4 const* point1, int count1, Edge* edges1, float sign)
		{
			for (int i = 0; i != count0; ++i)
			{
				auto& as = point0[ i          ];
				auto& ae = point0[(i+1)%count0];
				auto n = Plane{sign * Cross(axis, ae - as)};
				for (int j = 0; j != count1; ++j)
				{
					auto& edge = edges1[j];
					if (!edge) continue; // already clipped

					auto& bs = point1[ j          ];
					auto& be = point1[(j+1)%count1];
					if (!geometry::intersect::LineVsPlane(n, bs - as.w0(), be - as.w0(), edge.t0, edge.t1))
						edge.t1 = edge.t0;
				}
			}
			#if 0 // Show the clipped polygon
			{
				string<> s;
				for (int i = 0; i != count1; ++i)
				{
					if (!edges1[i]) continue;
					ldr::Line(s, "e", 0xFF0000FF, point1[i], point1[(i+1)%count1], edges1[i].t0, edges1[i].t1);
				}
				ldr::Write(s, "collision_debug2.ldr");
			}
			#endif
		};

		// If this is edge-face contact, then clip the edge against the face
		if (featA == EFeature::Edge)
		{
			clip(pointB, countB, pointA, countA, edgesA, -1.0f);
			return pointA[0] + (0.5f*(edgesA[0].t0 + edgesA[0].t1))*(pointA[1] - pointA[0]) - (0.5f * pen)*axis;
		}
		if (featB == EFeature::Edge)
		{
			clip(pointA, countA, pointB, countB, edgesB, +1.0f);
			return pointB[0] + (0.5f*(edgesB[0].t0 + edgesB[0].t1))*(pointB[1] - pointB[0]) - (0.5f * pen)*axis;
		}

		// Face to face contact, i.e featA >= EFeature::Tri && featB >= EFeature::Tri
		clip(pointA, countA, pointB, countB, edgesB, +1.0f);
		clip(pointB, countB, pointA, countA, edgesA, -1.0f);

		// Find the average point
		auto avr = [&](v4 const* point, int count, Edge const* edges)
		{
			auto total = 0.0f;
			auto centre = v4::Zero();
			for (int i = 0; i != count; ++i)
			{
				if (!edges[i]) continue;
				auto& s = point[i];
				auto& e = point[(i+1)%count];
				auto  d = e - s;
				centre += s + 0.5f * (edges[i].t0 + edges[i].t1) * d;
				total += 1.0f;
			}

			// Fallback: if all edges were clipped out (degenerate case, e.g. faces
			// share boundaries), use the centroid of the original polygon instead.
			if (total == 0.0f)
			{
				for (int i = 0; i != count; ++i)
				{
					centre += point[i];
					total += 1.0f;
				}
			}
			return centre / total;
		};
		auto centreA = avr(pointA, countA, edgesA);
		auto centreB = avr(pointB, countB, edgesB);

		// Shift centre to the halfway point between the faces
		return (0.5f * (centreA + centreB)).w1();
	}
	template <ShapeType Shape0, ShapeType Shape1>
	v4 FindContactPoint(Shape0 const& lhs, m4x4 const& l2w, Shape1 const& rhs, m4x4 const& r2w, v4 axis, float pen)
	{
		// Find the support feature on each shape (in each shape's space)
		auto featA = EFeature{};
		auto featB = EFeature{};
		v4 pointA[FeaturePolygonMaxSides] = {};
		v4 pointB[FeaturePolygonMaxSides] = {};
		SupportFeature(lhs, InvertOrthonormal(l2w) * +axis, featA, pointA);
		SupportFeature(rhs, InvertOrthonormal(r2w) * -axis, featB, pointB);

		return FindContactPoint(pointA, featA, pointB, featB, l2w, r2w, axis, pen);
	}
}
