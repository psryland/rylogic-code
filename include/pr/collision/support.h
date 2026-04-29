//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shapes.h"
#include "pr/collision/penetration.h"

namespace pr::collision
{
	using ManifoldCorners = vector<v4, 4 * FeaturePolygonMaxSides>;

	inline std::tuple<Contact::Manifold, EFeature> ReduceContactManifold(std::span<v4> corners, v4 axis);

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
		auto r = shape.m_base.m_s2p.z * shape.m_hlength;

		feature_type = EFeature::Vert;
		auto vert = shape.m_base.m_s2p.pos;
		if      (d > +math::tiny<float>) vert += r;
		else if (d < -math::tiny<float>) vert -= r;
		else feature_type = EFeature::Edge;

		// Hemispherical end-cap offset for thick lines
		if (shape.m_radius > 0)
		{
			auto len_sq = LengthSq(direction);
			if (len_sq > Sqr(math::tiny<float>))
				vert += shape.m_radius * direction / Sqrt(len_sq);
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
	inline v4 SupportVertex(ShapePolytope const& shape, v4 direction, EFeature& feature_type)
	{
		assert("Invalid polytope" && shape.m_vert_count != 0);

		feature_type = EFeature::Vert;
		auto s2p = shape.m_base.m_s2p;
		auto best = (s2p * shape.vertex(0)).w1();
		auto best_dist = Dot3(direction, best);
		for (int i = 1; i != shape.m_vert_count; ++i)
		{
			auto point = (s2p * shape.vertex(i)).w1();
			auto dist = Dot3(direction, point);
			if (dist > best_dist)
			{
				best = point;
				best_dist = dist;
			}
		}
		return best;
	}
	inline v4 SupportVertex(Shape const& shape, v4 direction, EFeature& feature_type)
	{
		switch (shape.m_type)
		{
		case EShape::Sphere:   { return SupportVertex(shape_cast<ShapeSphere>(shape), direction, feature_type); }
		case EShape::Box:      { return SupportVertex(shape_cast<ShapeBox>(shape), direction, feature_type); }
		case EShape::Line:     { return SupportVertex(shape_cast<ShapeLine>(shape), direction, feature_type); }
		case EShape::Triangle: { return SupportVertex(shape_cast<ShapeTriangle>(shape), direction, feature_type); }
		case EShape::Polytope: { return SupportVertex(shape_cast<ShapePolytope>(shape), direction, feature_type); }
		default:               { throw std::runtime_error("Shape type does not support SupportVertex"); }
		}
	}
	inline v4 SupportVertex(ShapeType auto const& shape, v4 direction)
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
		auto r = shape.m_base.m_s2p.z * shape.m_hlength;

		// Hemispherical end-cap offset for thick lines
		auto thickness_offset = v4::Zero();
		if (shape.m_radius > 0)
		{
			auto len_sq = LengthSq(axis);
			if (len_sq > Sqr(math::tiny<float>))
				thickness_offset = shape.m_radius * axis / Sqrt(len_sq);
		}

		// Allow a more relaxed tolerance for perpendicular 'axis because the edge
		// feature includes the end points, but not the other way round.
		constexpr float tol = 1e-3f;
		if (d > +tol)
		{
			// Line points in the direction of the axis, return the end point
			feature_type = EFeature::Vert;
			points[0] = shape.m_base.m_s2p.pos + r + thickness_offset;
		}
		else if (d < -tol)
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
		constexpr auto tol = 1e-5f;
		constexpr auto tol_sq = Sqr(tol);

		// Project each vertex onto the axis
		auto d0 = Dot3(axis, shape.m_v.x);
		auto d1 = Dot3(axis, shape.m_v.y);
		auto d2 = Dot3(axis, shape.m_v.z);
		auto d_max = std::max({d0, d1, d2});

		// Count how many vertices are at the maximum projection (within tolerance).
		// 1 vertex → Vert feature, 2 vertices → Edge feature, 3 vertices → Tri (face) feature.
		int count = 0;
		int indices[3] = {};
		d_max -= math::tiny<float>;
		if (d0 >= d_max) indices[count++] = 0;
		if (d1 >= d_max) indices[count++] = 1;
		if (d2 >= d_max) indices[count++] = 2;

		// Return the support feature, while handling degenerate vertices in the triangle.
		switch (count)
		{
			case 0:
			{
				feature_type = EFeature::None;
				return;
			}
			case 1:
			{
				feature_type = EFeature::Vert;
				points[0] = shape.m_v[indices[0]];
				return;
			}
			case 2:
			{
				points[0] = shape.m_v[indices[0]];
				points[1] = shape.m_v[indices[1]];
				feature_type = LengthSq(points[1] - points[0]) < tol_sq ? EFeature::Vert : EFeature::Edge;
				return;
			}
			case 3:
			{
				points[0] = shape.m_v.x;
				points[1] = shape.m_v.y;
				points[2] = shape.m_v.z;

				auto e0 = points[1] - points[0];
				auto e1 = points[2] - points[1];
				auto e2 = points[0] - points[2];
				auto l0 = LengthSq(e0);
				auto l1 = LengthSq(e1);
				auto l2 = LengthSq(e2);
		
				// If any edge is degenerate, or the verts are collinear, return the longest edge as the feature instead of the face.
				if (l0 < tol_sq || l1 < tol_sq || l2 < tol_sq || LengthSq(Cross(e0, e1)) < tol_sq)
				{
					// Find the longest edge
					auto p0 = points[0];
					auto p1 = points[1];
					auto len_sq = l0;
					if (l1 >= len_sq)
					{
						p0 = points[1];
						p1 = points[2];
						len_sq = l1;
					}
					if (l2 >= len_sq)
					{
						p0 = points[2];
						p1 = points[0];
						len_sq = l2;
					}
					points[0] = p0;
					points[1] = p1;
					feature_type = len_sq < tol_sq ? EFeature::Vert : EFeature::Edge;
					return;
				}

				// Valid face feature. Ensure the points are in the correct winding order (CCW when viewed from the direction of 'axis').
				feature_type = EFeature::Tri;
				if (Triple(axis, points[1] - points[0], points[2] - points[0]) < 0)
					std::swap(points[1], points[2]);
				return;
			}
		}
	}
	inline void SupportFeature(ShapePolytope const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		assert("Invalid polytope" && shape.m_vert_count != 0);

		constexpr auto tol = 1e-4f;
		constexpr auto tol_sq = Sqr(tol);
		auto s2p = shape.m_base.m_s2p;
		auto local_axis = (InvertOrthonormal(s2p) * axis.w0()).w0();

		auto best_dist = -limits<float>::max();
		for (int i = 0; i != shape.m_vert_count; ++i)
			best_dist = Max(best_dist, Dot3(local_axis, shape.vertex(i)));

		std::array<v4, 256> support_points;
		assert("Polytope has too many vertices" && shape.m_vert_count <= static_cast<int>(support_points.size()));

		auto support_count = 0;
		for (int i = 0; i != shape.m_vert_count; ++i)
		{
			if (Dot3(local_axis, shape.vertex(i)) < best_dist - tol)
				continue;

			auto point = (s2p * shape.vertex(i)).w1();
			auto duplicate = false;
			for (int j = 0; j != support_count; ++j)
			{
				if (LengthSq(point - support_points[j]) < tol_sq)
				{
					duplicate = true;
					break;
				}
			}
			if (!duplicate)
				support_points[support_count++] = point;
		}

		if (support_count == 0)
		{
			feature_type = EFeature::None;
			return;
		}
		if (support_count != 1)
		{
			auto centre = v4::Zero();
			for (int i = 0; i != support_count; ++i)
				centre += support_points[i];
			centre = (centre / static_cast<float>(support_count)).w1();

			auto norm = Normalise(axis);
			auto basis_x = v4::Zero();
			for (int i = 0; i != support_count; ++i)
			{
				auto radial = (support_points[i] - centre).w0();
				radial -= Dot3(radial, norm) * norm;
				auto len_sq = LengthSq(radial);
				if (len_sq > tol_sq)
				{
					basis_x = radial / Sqrt(len_sq);
					break;
				}
			}

			if (LengthSq(basis_x) > tol_sq)
			{
				auto basis_y = Cross(norm, basis_x);
				std::sort(support_points.begin(), support_points.begin() + support_count, [&](v4 const& lhs, v4 const& rhs)
				{
					auto dl = (lhs - centre).w0();
					auto dr = (rhs - centre).w0();
					auto al = Atan2(Dot3(dl, basis_y), Dot3(dl, basis_x));
					auto ar = Atan2(Dot3(dr, basis_y), Dot3(dr, basis_x));
					return al < ar;
				});
			}
		}

		if (support_count > FeaturePolygonMaxSides)
		{
			auto [manifold, feature] = ReduceContactManifold(std::span<v4>{support_points.data(), static_cast<size_t>(support_count)}, axis);
			feature_type = feature;
			for (int i = 0, iend = int(feature); i != iend; ++i)
				points[i] = manifold[i];
			return;
		}

		feature_type = static_cast<EFeature>(support_count);
		for (int i = 0, iend = int(feature_type); i != iend; ++i)
			points[i] = support_points[i];
	}
	inline void SupportFeature(Shape const& shape, v4 axis, EFeature& feature_type, v4 (&points)[FeaturePolygonMaxSides])
	{
		switch (shape.m_type)
		{
		case EShape::Sphere:   { return SupportFeature(shape_cast<ShapeSphere>(shape), axis, feature_type, points); }
		case EShape::Box:      { return SupportFeature(shape_cast<ShapeBox>(shape), axis, feature_type, points); }
		case EShape::Line:     { return SupportFeature(shape_cast<ShapeLine>(shape), axis, feature_type, points); }
		case EShape::Triangle: { return SupportFeature(shape_cast<ShapeTriangle>(shape), axis, feature_type, points); }
		case EShape::Polytope: { return SupportFeature(shape_cast<ShapePolytope>(shape), axis, feature_type, points); }
		default:               { throw std::runtime_error("Shape type does not support SupportFeature"); }
		}
	}

	// Returns the centroid of a set of points
	inline v4 Centroid(std::span<v4 const> points)
	{
		auto centre = v4::Zero();
		for (auto const& point : points)
			centre += point;

		centre /= static_cast<float>(points.size());
		return centre.w1();
	};

	// Reduce an ordered contact polygon to a solver-sized manifold.
	inline std::tuple<Contact::Manifold, EFeature> ReduceContactManifold(std::span<v4> corners, v4 axis)
	{
		using R = std::tuple<Contact::Manifold, EFeature>;
		constexpr auto tol = 1e-5f;
		constexpr auto tol_sq = Sqr(tol);

		if (corners.empty())
		{
			return R{ {}, EFeature::None };
		}
		if (corners.size() == 1)
		{
			return R{ { corners[0] }, EFeature::Vert };
		}
		if (corners.size() == 2)
		{
			auto e0 = corners[1] - corners[0];
			auto x0 = LengthSq(e0) < tol_sq;
			return
				x0 ? R{ { corners[0] }, EFeature::Vert } :
				R{ { corners[0], corners[1] }, EFeature::Edge };
		}
		if (corners.size() == 3)
		{
			auto e0 = corners[1] - corners[0];
			auto e1 = corners[2] - corners[1];
			auto e2 = corners[0] - corners[2];
			auto l0 = LengthSq(e0);
			auto l1 = LengthSq(e1);
			auto l2 = LengthSq(e2);
			auto x0 = l0 < tol_sq;
			auto x1 = l1 < tol_sq;
			auto x2 = l2 < tol_sq;
			auto sign = Dot(axis, Cross(e0, e1));

			if (x0 && x1 && x2)
				return R{ { corners[0] }, EFeature::Vert };

			if (x0 || x1 || x2 || Abs(sign) < tol_sq)
				return
					l0 >= l1 && l0 >= l2 ? R{ { corners[0], corners[1] }, EFeature::Edge } :
					l1 >= l2             ? R{ { corners[1], corners[2] }, EFeature::Edge } :
					R{ { corners[2], corners[0] }, EFeature::Edge };

			return sign > 0
				? R{ { corners[0], corners[1], corners[2] }, EFeature::Tri }
				: R{ { corners[0], corners[2], corners[1] }, EFeature::Tri };
		}

		// Find two distant points. This won't find the global maximum pair, but should be good enough for a seed edge
		int i0 = 0, i1 = 1;
		auto max_dist_sq = LengthSq(corners[i1] - corners[i0]);
		for (int i = 2, iend = isize(corners); ; i = (i + 1) % iend)
		{
			if (i == i0) break; // Loop until we've tested everything against i0
			auto dist_sq = LengthSq(corners[i] - corners[i0]);
			if (dist_sq > max_dist_sq)
			{
				i1 = i0;
				i0 = i;
				max_dist_sq = dist_sq;
			}
		}

		// All points are coincident, return a single point
		if (max_dist_sq < Sqr(tol))
		{
			return R{ { corners[i0] }, EFeature::Vert };
		}

		// Find two more points that are the max in the transverse direction to the seed edge. This should give a good initial polygon for the reduction.
		int j0 = i0, j1 = i0;
		auto max_dist0 = -tol;
		auto max_dist1 = +tol;
		auto bi_norm = Normalise(Cross(axis, corners[i1] - corners[i0])); // Note: guaranteed to be non-zero since the corners are in a plane perpendicular to the axis
		for (int j = 0, jend = isize(corners); j != jend; ++j)
		{
			if (j == i0 || j == i1) continue;
			auto dist = Dot(corners[j] - corners[i0], bi_norm);
			if (dist < max_dist0) { j0 = j; max_dist0 = dist; }
			if (dist > max_dist1) { j1 = j; max_dist1 = dist; }
		}

		return
			j0 == i0 && j1 == i0 ? R{ { corners[i0], corners[i1] }, EFeature::Edge } :
			j0 == i0 ? R{ { corners[i0], corners[i1], corners[j1] }, EFeature::Tri } :
			j1 == i0 ? R{ { corners[i0], corners[j0], corners[i1] }, EFeature::Tri } :
			R{ { corners[i0], corners[j0], corners[i1], corners[j1] }, EFeature::Quad };
	}

	// Returns the contact manifold between two shapes, 'lhs' and 'rhs'. 'axis' is the collision separating axis. 'pen' is the depth of penetration.
	inline std::tuple<Contact::Manifold, EFeature> FindContactManifold(std::span<v4 const> pointsA, EFeature featA, std::span<v4 const> pointsB, EFeature featB, v4 axis, float pen)
	{
		auto countA = int(featA);
		auto countB = int(featB);
		pointsA = pointsA.subspan(0, countA);
		pointsB = pointsB.subspan(0, countB);

		// Generally, the contact lies half way between the support features along 'axis'. Vertex and
		// edge cases can be represented directly; face contacts are clipped below to preserve the patch area.

		// For features with area, check that the polygon is facing the correct direction, +ve for featA, -ve for featB
		assert((featA <= EFeature::Edge || (Dot(Plane::FromBestFit(pointsA), axis) > 0)) && "Contact polygon has incorrect winding order");
		assert((featB <= EFeature::Edge || (Dot(Plane::FromBestFit(pointsB), axis) < 0)) && "Contact polygon has incorrect winding order");

		// If both shapes contact at a vert, then the separating axis passes through their average position
		if (featA == EFeature::Vert && featB == EFeature::Vert)
		{
			auto point = (pointsA[0] + pointsB[0]) * 0.5f;
			return { { point }, EFeature::Vert };
		}

		// If one shape is contacting at a vert, then the separating axis must pass through this vert
		if (featA == EFeature::Vert)
		{
			auto point = pointsA[0] + axis * (0.5f * Dot3(axis, pointsB[0] - pointsA[0]));
			return { { point }, EFeature::Vert };
		}
		if (featB == EFeature::Vert)
		{
			auto point = pointsB[0] + axis * (0.5f * Dot3(axis, pointsA[0] - pointsB[0]));
			return { { point }, EFeature::Vert };
		}

		// If this is edge-edge contact, then the separating axis passes through the closest points
		if (featA == EFeature::Edge && featB == EFeature::Edge)
		{
			v4 pt0, pt1;
			geometry::closest_point::LineToLine(pointsA[0], pointsA[1], pointsB[0], pointsB[1], pt0, pt1);

			auto point = (pt0 + pt1) * 0.5f;
			return { { point }, EFeature::Vert }; // Ignoring parallel edges for now
		}

		// Face-Face or Face-Edge contacts require clipping.
		// Find the geometric intersection of the two polygons (in the plane of 'axis').
		// Return the intersection as the contact manifold

		// Generate a container of edges for each feature
		struct Edge
		{
			float t0 = 0.0f;
			float t1 = 1.0f;
			bool valid = true;
			explicit operator bool() const
			{
				return valid && t0 <= t1 + 1e-5f;
			}
		};
		vector<Edge, FeaturePolygonMaxSides> edgesA(countA);
		vector<Edge, FeaturePolygonMaxSides> edgesB(countB);
		ManifoldCorners corners;

		// Clip the edges of 'points1' against the edges of 'points0'.
		// Note: the winding order for polygon 'points1' is always the opposite of the winding order of polygon 'points0'.
		// This is because the SupportFeature() function returns the face in the direction of the support axis which for featB is -axis.
		auto clip = [](std::span<v4 const> points0, std::span<v4 const> points1, std::span<Edge> edges1, float sign, v4 axis)
		{
			constexpr auto tol = 1e-5f;
			for (int i = 0, in = s_cast<int>(points0.size()); i != in; ++i)
			{
				auto& as = points0[i];
				auto& ae = points0[(i + 1) % in];
				auto n = sign * Cross(axis, ae - as);
				for (int j = 0, jn = s_cast<int>(points1.size()); j != jn; ++j)
				{
					auto& edge = edges1[j];
					if (!edge) continue; // already clipped

					auto& bs = points1[j];
					auto& be = points1[(j + 1) % jn];
					auto d0 = Dot3(n, bs - as);
					auto d1 = Dot3(n, be - as);
					if (d0 < -tol && d1 < -tol)
					{
						edge.valid = false;
						continue;
					}
					if (d0 >= -tol && d1 >= -tol)
						continue;

					auto t = d0 / (d0 - d1);
					if (d0 < -tol)
						edge.t0 = Max(edge.t0, t);
					else
						edge.t1 = Min(edge.t1, t);

					if (edge.t0 > edge.t1 + tol)
						edge.valid = false;
				}
			}
		};

		// Append the clipped edge (if valid) to the list of candidate contact points, applying an offset along 'axis' to ensure the contact patch has area.
		auto append_clipped_edge = [](ManifoldCorners& corners, std::span<v4 const> points, int edge_index, Edge const& edge, v4 offset)
		{
			if (!edge) return;
			auto const& pt0 = points[edge_index];
			auto const& pt1 = points[(edge_index + 1) % points.size()];
			auto beg = pt0 + edge.t0 * (pt1 - pt0) + offset;
			auto end = pt0 + edge.t1 * (pt1 - pt0) + offset;
			corners.push_back(beg);
			corners.push_back(end);
		};

		// If all edges are clipped, fall back to the average of the centroids as the contact point.
		auto fallback = [&]
		{
			Contact::Manifold manifold = {};
			manifold[0] = (0.5f * (Centroid(pointsA) + Centroid(pointsB))).w1();
			return std::tuple{ manifold, EFeature::Vert };
		};

		// If this is edge-face contact, then clip the edge against the face
		if (featA == EFeature::Edge)
		{
			clip(pointsB, pointsA, edgesA, -1.0f, axis);

			append_clipped_edge(corners, pointsA, 0, edgesA[0], -(0.5f * pen) * axis);
			auto manifold = ReduceContactManifold(corners, axis);
			return std::get<1>(manifold) != EFeature::None ? manifold : fallback();
		}
		if (featB == EFeature::Edge)
		{
			clip(pointsA, pointsB, edgesB, +1.0f, axis);
			append_clipped_edge(corners, pointsB, 0, edgesB[0], +(0.5f * pen) * axis);
			auto manifold = ReduceContactManifold(corners, axis);
			return std::get<1>(manifold) != EFeature::None ? manifold : fallback();
		}

		// Face to face contact, i.e featA >= EFeature::Tri && featB >= EFeature::Tri

		// Clip the edges of each face against the other face.
		clip(pointsA, pointsB, edgesB, +1.0f, axis);
		clip(pointsB, pointsA, edgesA, -1.0f, axis);

		// Append the clipped edges from both polygons to the manifold
		for (int i = 0; i != countA; ++i)
			append_clipped_edge(corners, pointsA, i, edgesA[i], -(0.5f * pen) * axis);
		for (int i = 0; i != countB; ++i)
			append_clipped_edge(corners, pointsB, i, edgesB[i], +(0.5f * pen) * axis);

		// Reduce the manifold down to the number of contact points supported by the solver
		auto manifold = ReduceContactManifold(corners, axis);
		return std::get<1>(manifold) != EFeature::None ? manifold : fallback();
	}
	inline std::tuple<Contact::Manifold, EFeature> FindContactManifold(ShapeType auto const& lhs, m4x4 const& l2w, ShapeType auto const& rhs, m4x4 const& r2w, v4 axis, float pen)
	{
		// Find the support feature on each shape (in each shape's space)
		auto featA = EFeature{};
		auto featB = EFeature{};
		v4 pointA[FeaturePolygonMaxSides] = {};
		v4 pointB[FeaturePolygonMaxSides] = {};
		SupportFeature(lhs, InvertOrthonormal(l2w) * +axis, featA, pointA);
		SupportFeature(rhs, InvertOrthonormal(r2w) * -axis, featB, pointB);

		// Transform the contact points to world space
		for (auto& pt : pointA) pt = l2w * pt;
		for (auto& pt : pointB) pt = r2w * pt;
		return FindContactManifold(pointA, featA, pointB, featB, axis, pen);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::collision::tests
{
	PRUnitTestClass(ContactManifoldTests)
	{
		PRUnitTestMethod(ReduceManifoldDegeneratesTest)
		{
			auto axis = v4::ZAxis();

			auto expect_feature = [&](std::span<v4> corners, EFeature expected)
			{
				auto [manifold, feature] = ReduceContactManifold(corners, axis);
				PR_EXPECT(feature == expected);
				return manifold;
			};
			auto expect_non_degenerate_edge = [&](std::span<v4> corners)
			{
				auto manifold = expect_feature(corners, EFeature::Edge);
				PR_EXPECT(LengthSq(manifold[1] - manifold[0]) > Sqr(1e-5f));
				return manifold;
			};
			auto expect_positive_area = [&](std::span<v4> corners, EFeature expected)
			{
				auto manifold = expect_feature(corners, expected);
				PR_EXPECT(Dot(axis, Cross(manifold[1] - manifold[0], manifold[2] - manifold[0])) > 0);
				return manifold;
			};

			auto empty = std::span<v4>{};
			expect_feature(empty, EFeature::None);

			v4 one[] = { v4{0, 0, 0, 1} };
			expect_feature(one, EFeature::Vert);

			v4 two_distinct[] = { v4{0, 0, 0, 1}, v4{1, 0, 0, 1} };
			expect_non_degenerate_edge(two_distinct);

			v4 two_same[] = { v4{0, 0, 0, 1}, v4{0, 0, 0, 1} };
			expect_feature(two_same, EFeature::Vert);

			v4 three_same[] = { v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{0, 0, 0, 1} };
			expect_feature(three_same, EFeature::Vert);

			v4 three_two_same[] = { v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{1, 0, 0, 1} };
			expect_non_degenerate_edge(three_two_same);

			v4 three_collinear[] = { v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{2, 0, 0, 1} };
			auto collinear_edge = expect_non_degenerate_edge(three_collinear);
			PR_EXPECT(FEql(LengthSq(collinear_edge[1] - collinear_edge[0]), 4.0f));

			v4 tri_ccw[] = { v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1} };
			expect_positive_area(tri_ccw, EFeature::Tri);

			v4 tri_cw[] = { v4{0, 0, 0, 1}, v4{0, 1, 0, 1}, v4{1, 0, 0, 1} };
			expect_positive_area(tri_cw, EFeature::Tri);

			v4 four_same[] = { v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{0, 0, 0, 1} };
			expect_feature(four_same, EFeature::Vert);

			v4 four_collinear[] = { v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{2, 0, 0, 1}, v4{3, 0, 0, 1} };
			expect_non_degenerate_edge(four_collinear);

			v4 quad_with_duplicate[] = { v4{-1, -1, 0, 1}, v4{+1, -1, 0, 1}, v4{+1, +1, 0, 1}, v4{+1, +1, 0, 1}, v4{-1, +1, 0, 1} };
			expect_positive_area(quad_with_duplicate, EFeature::Quad);

			v4 quad[] = { v4{-1, -1, 0, 1}, v4{+1, -1, 0, 1}, v4{+1, +1, 0, 1}, v4{-1, +1, 0, 1} };
			expect_positive_area(quad, EFeature::Quad);
		}

		PRUnitTestMethod(DegenerateTriangleSupportFeatureTest)
		{
			auto axis = v4::ZAxis();
			auto expect_feature = [&](ShapeTriangle const& tri, EFeature expected)
			{
				auto feature = EFeature{};
				v4 points[FeaturePolygonMaxSides] = {};
				SupportFeature(tri, axis, feature, points);
				PR_EXPECT(feature == expected);
				for (int i = 0, iend = int(feature); i != iend; ++i)
					PR_EXPECT(points[i].w == 1.0f);
				
				auto manifold = Contact::Manifold{};
				for (int i = 0, iend = int(feature); i != iend; ++i)
					manifold[i] = points[i];
				return std::tuple{ feature, manifold };
			};

			{
				auto tri = ShapeTriangle{ v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{0, 0, 0, 1} };
				expect_feature(tri, EFeature::Vert);
			}
			{
				auto tri = ShapeTriangle{ v4{0, 0, 0, 1}, v4{0, 0, 0, 1}, v4{1, 0, 0, 1} };
				auto [feature, points] = expect_feature(tri, EFeature::Edge);
				PR_EXPECT(LengthSq(points[1] - points[0]) > Sqr(1e-5f));
			}
			{
				auto tri = ShapeTriangle{ v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{2, 0, 0, 1} };
				auto [feature, points] = expect_feature(tri, EFeature::Edge);
				PR_EXPECT(FEql(LengthSq(points[1] - points[0]), 4.0f));
			}
			{
				auto tri = ShapeTriangle{ v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1} };
				auto [feature, points] = expect_feature(tri, EFeature::Tri);
				PR_EXPECT(Dot(axis, Cross(points[1] - points[0], points[2] - points[0])) > 0);
			}
		}
	};
}
#endif
