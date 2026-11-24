//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_cylinder.h"
#include "pr/collision/shape_array.h"

namespace pr::collision
{
	// The number of straight segments used for each full circle of a curved outline.
	inline constexpr int WireframeCircleSegments = 16;

	// Outline 'shape' with straight line segments. 'p2w' is the transform of the frame that 'shape.m_s2r' is relative to (normally a body's
	// object-to-world). 'emit(a, b)' receives each segment's end points, transformed by 'p2w'. Curved surfaces are drawn with circles and
	// arcs, so the outline shows the true extent of the shape but not every surface point. Compound children are outlined individually.
	template <typename EmitLine> requires std::invocable<EmitLine&, v4 const&, v4 const&>
	void ShapeWireframe(Shape const& shape, m4x4 const& p2w, EmitLine&& emit)
	{
		// Compound children hold transforms in the compound's parent frame, so they share the parent transform.
		if (shape.m_type == EShape::Array)
		{
			auto const& arr = shape_cast<ShapeArray>(shape);
			for (auto const* child = arr.begin(), *end = arr.end(); child != end; child = next(child))
				ShapeWireframe(*child, p2w, emit);

			return;
		}

		// Every leaf outline is built in its own shape space, then moved to world space.
		auto s2w = p2w * shape.m_s2r;
		auto line = [&](v4 const& a, v4 const& b)
		{
			// Transform one shape-space segment into the output frame.
			emit(s2w * a, s2w * b);
		};

		// Outline an arc of 'radius' about 'centre' in the plane spanned by the unit axes 'u' and 'v', from angle 'a0' to 'a1' radians.
		auto arc = [&](v4 const& centre, v4 const& u, v4 const& v, float radius, float a0, float a1)
		{
			// Use enough segments that a full circle has 'WireframeCircleSegments' sides.
			auto count = std::max(1, static_cast<int>(std::ceil(WireframeCircleSegments * Abs(a1 - a0) / math::constants<float>::tau)));
			auto prev = centre + radius * (std::cos(a0) * u + std::sin(a0) * v);
			for (auto i = 1; i <= count; ++i)
			{
				auto angle = a0 + (a1 - a0) * i / count;
				auto curr = centre + radius * (std::cos(angle) * u + std::sin(angle) * v);
				line(prev, curr);
				prev = curr;
			}
		};

		// Outline the leaf according to its type.
				switch (shape.m_type)
		{
			case EShape::Sphere:
			{
				// Three orthogonal great circles show the sphere's extent along every axis.
				auto const& s = shape_cast<ShapeSphere>(shape);
				arc(v4::Origin(), v4::XAxis(), v4::YAxis(), s.m_radius, 0, math::constants<float>::tau);
				arc(v4::Origin(), v4::YAxis(), v4::ZAxis(), s.m_radius, 0, math::constants<float>::tau);
				arc(v4::Origin(), v4::ZAxis(), v4::XAxis(), s.m_radius, 0, math::constants<float>::tau);
				break;
			}
			case EShape::Box:
			{
				// The twelve edges of the box, from the eight corners at +/- the half extents.
				auto const& s = shape_cast<ShapeBox>(shape);
				auto r = s.m_radius;
				auto corner = [&](int i)
				{
					// Bit 0, 1 and 2 of 'i' select the sign of x, y and z.
					return v4((i & 1) ? +r.x : -r.x, (i & 2) ? +r.y : -r.y, (i & 4) ? +r.z : -r.z, 1);
				};
				for (auto i = 0; i != 8; ++i)
				{
					// Join each corner to the corners that differ from it in exactly one higher axis bit.
					for (auto bit = 1; bit != 8; bit <<= 1)
					{
						if ((i & bit) == 0)
							line(corner(i), corner(i | bit));
					}
				}
				break;
			}
			case EShape::Line:
			{
				// A thin line is a single segment along Z. A line with a radius is a capsule: end circles, side lines, and hemisphere arcs.
				auto const& s = shape_cast<ShapeLine>(shape);
				auto top = v4(0, 0, +s.m_hlength, 1);
				auto bot = v4(0, 0, -s.m_hlength, 1);
				if (s.m_radius == 0)
				{
					line(bot, top);
					break;
				}

				// The cylindrical section.
				arc(top, v4::XAxis(), v4::YAxis(), s.m_radius, 0, math::constants<float>::tau);
				arc(bot, v4::XAxis(), v4::YAxis(), s.m_radius, 0, math::constants<float>::tau);
				line(bot + v4(+s.m_radius, 0, 0, 0), top + v4(+s.m_radius, 0, 0, 0));
				line(bot + v4(-s.m_radius, 0, 0, 0), top + v4(-s.m_radius, 0, 0, 0));
				line(bot + v4(0, +s.m_radius, 0, 0), top + v4(0, +s.m_radius, 0, 0));
				line(bot + v4(0, -s.m_radius, 0, 0), top + v4(0, -s.m_radius, 0, 0));

				// The hemispherical end caps, as two half circles each.
				arc(top, v4::XAxis(), v4::ZAxis(), s.m_radius, 0, math::constants<float>::tau_by_2);
				arc(top, v4::YAxis(), v4::ZAxis(), s.m_radius, 0, math::constants<float>::tau_by_2);
				arc(bot, v4::XAxis(), -v4::ZAxis(), s.m_radius, 0, math::constants<float>::tau_by_2);
				arc(bot, v4::YAxis(), -v4::ZAxis(), s.m_radius, 0, math::constants<float>::tau_by_2);
				break;
			}
			case EShape::Cylinder:
			{
				// Flat-ended, so only the end circles and side lines.
				auto const& s = shape_cast<ShapeCylinder>(shape);
				auto top = v4(0, 0, +s.m_hlength, 1);
				auto bot = v4(0, 0, -s.m_hlength, 1);
				arc(top, v4::XAxis(), v4::YAxis(), s.m_radius, 0, math::constants<float>::tau);
				arc(bot, v4::XAxis(), v4::YAxis(), s.m_radius, 0, math::constants<float>::tau);
				line(bot + v4(+s.m_radius, 0, 0, 0), top + v4(+s.m_radius, 0, 0, 0));
				line(bot + v4(-s.m_radius, 0, 0, 0), top + v4(-s.m_radius, 0, 0, 0));
				line(bot + v4(0, +s.m_radius, 0, 0), top + v4(0, +s.m_radius, 0, 0));
				line(bot + v4(0, -s.m_radius, 0, 0), top + v4(0, -s.m_radius, 0, 0));
				break;
			}
			case EShape::Triangle:
			{
				// The three edges of the triangle.
				auto const& s = shape_cast<ShapeTriangle>(shape);
				line(s.m_v.x, s.m_v.y);
				line(s.m_v.y, s.m_v.z);
				line(s.m_v.z, s.m_v.x);
				break;
			}
			case EShape::Polytope:
			{
				// The polytope stores its hull edges, so each is drawn once.
				auto const& s = shape_cast<ShapePolytope>(shape);
				for (auto const& edge : s.edges())
					line(s.vertex(edge.m_v0), s.vertex(edge.m_v1));

				break;
			}
			default:
			{
				throw std::runtime_error("Unknown shape type");
			}
		}
	}
}
