//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Triangle vs Line segment collision detection.
//
// Algorithm:
//  Both shapes are zero-volume. The triangle is a flat face and the
//  line is a Z-axis aligned segment. We test potential separating axes:
//   - 1 axis:  triangle normal (the line must cross the triangle plane)
//   - 3 axes:  cross products of 3 triangle edges × line direction
//
//  If no separating axis is found, the line pierces the triangle.
//  Penetration depth along the triangle normal is how far the line
//  extends through the plane of the triangle.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between a triangle and a line segment, with generic penetration collection.
	// 'lhs' is the triangle (3), 'rhs' is the line (2) (tri-table order: Triangle=3, Line=2).
	template <typename Penetration>
	void pr_vectorcall TriangleVsLine(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& tri = shape_cast<ShapeTriangle>(lhs_);
		auto& line = shape_cast<ShapeLine>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Transform to triangle space for simpler geometry
		auto t2w_inv = InvertOrthonormal(l2w);

		// Triangle vertices (in triangle space, as positions w=1)
		auto a = tri.m_v.x.w1();
		auto b = tri.m_v.y.w1();
		auto c = tri.m_v.z.w1();

		// Line segment endpoints in triangle space
		auto line_dir_ws = line.m_radius * r2w.z; // half-extent vector in world space
		auto line_mid = t2w_inv * r2w.pos;         // line midpoint in triangle space
		auto line_half = t2w_inv * line_dir_ws;     // half-extent vector in triangle space (direction only)

		// Triangle edges
		auto e0 = b - a;
		auto e1 = c - b;
		auto e2 = a - c;

		// Triangle normal in triangle space
		auto tri_norm = tri.m_v.w; // already computed as Normalise(Cross(b-a, c-b))

		// Lambda: project triangle and line onto an axis and compute overlap.
		// For thick lines, the collision envelope adds m_thickness * |axis| to the line's projection.
		auto test_axis = [&](v4 axis) -> float
		{
			// Project triangle vertices onto axis
			auto da = Dot3(axis, a);
			auto db = Dot3(axis, b);
			auto dc = Dot3(axis, c);
			auto t_min = std::min({da, db, dc});
			auto t_max = std::max({da, db, dc});

			// Project line segment onto axis, plus thickness envelope
			auto lm = Dot3(axis, line_mid);
			auto lr = Abs(Dot3(axis, line_half)) + line.m_thickness * Length(axis);
			auto l_min = lm - lr;
			auto l_max = lm + lr;

			// Overlap = min of the two maximum intrusions
			return std::min(t_max - l_min, l_max - t_min);
		};

		// Lambda for returning a separating axis with the correct sign (in world space)
		auto make_sep_axis = [&](v4 sa_trispace)
		{
			auto sa = l2w * sa_trispace;
			auto tri_ctr = l2w * ((a + b + c) / 3.0f);
			auto line_ctr = r2w.pos;
			return Bool2SignF(Dot3(sa, tri_ctr) < Dot3(sa, line_ctr)) * sa;
		};

		// --- Axis 1: Triangle normal ---
		{
			auto depth = test_axis(tri_norm);
			if (!pen(depth, [&]{ return make_sep_axis(tri_norm); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}

		// --- Axes 2-4: Cross products of triangle edges × line direction ---
		v4 edges[] = { e0, e1, e2 };
		for (int i = 0; i != 3; ++i)
		{
			auto axis = Cross(edges[i], line_half);

			// Skip degenerate axes (parallel edge and line direction)
			auto axis_len_sq = LengthSq(axis);
			if (axis_len_sq < Sqr(math::tiny<float>))
				continue;

			auto depth = test_axis(axis);
			if (!pen(depth, [&]{ return make_sep_axis(axis); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}
	}

	// Returns true if the triangle and line segment intersect
	inline bool pr_vectorcall TriangleVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		TriangleVsLine(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the triangle and line segment are intersecting, with contact details
	inline bool pr_vectorcall TriangleVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		TriangleVsLine(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeLine>(rhs), r2w, contact.m_axis, contact.m_depth);
		contact.m_mat_idA = p.m_mat_idA;
		contact.m_mat_idB = p.m_mat_idB;
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/ldraw.h"

namespace pr::collision::tests
{
	PRUnitTestClass(TriangleVsLineTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = m4x4::Random(rng, v4::Origin(), 0.5f);
				m4x4 r2w = m4x4::Random(rng, v4::Origin(), 0.5f);

				Builder builder;
				{ auto& g = builder.Group("tri", 0x30FF0000); AddShape(g, tri); g.o2w(l2w); }
				{ auto& g = builder.Group("line", 0x3000FF00); AddShape(g, line); g.o2w(r2w); }
				if (TriangleVsLine(tri, l2w, line, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.005f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.005f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Line piercing through the triangle
		PRUnitTestMethod(LinePiercesTriangle)
		{
			// Triangle on XY plane
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f}; // along Z from -1 to +1

			// Line passes vertically through the centre of the triangle
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));
		}

		// Line parallel to triangle, in the plane
		PRUnitTestMethod(LineInTrianglePlane)
		{
			auto tri = ShapeTriangle{v4{-2, -1, 0, 0}, v4{2, -1, 0, 0}, v4{0, 2, 0, 0}};

			// Line along X-axis (rotated from Z to X), in the XY plane
			auto line = ShapeLine{2.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());

			// Line lies in the triangle's plane, overlapping → contact
			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));
		}

		// Line parallel to triangle but offset: should not collide
		PRUnitTestMethod(LineParallelSeparated)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f};

			// Line along X-axis, offset 2 units above the triangle
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 2, 1});

			PR_EXPECT(!TriangleVsLine(tri, l2w, line, r2w));
		}

		// Line endpoint touching the triangle
		PRUnitTestMethod(EndpointTouchesTriangle)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f}; // half-length = 1, along Z

			// Place line so its -Z end is at the triangle (z=0)
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 1.0f, 0}); // line from z=0 to z=2

			// Line endpoint just touches the triangle plane
			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));
		}

		// Line misses the triangle entirely: passes beside it
		PRUnitTestMethod(LineMissesTriangle)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f};

			// Line along Z but offset far in X, passing beside the triangle
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{5, 0, 0, 0});

			PR_EXPECT(!TriangleVsLine(tri, l2w, line, r2w));
		}

		// Line along a triangle edge: coplanar edge contact
		PRUnitTestMethod(LineAlongTriangleEdge)
		{
			auto tri = ShapeTriangle{v4{-1, 0, 0, 0}, v4{1, 0, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f}; // half-length = 1

			// Rotate line to align with X-axis, place on the bottom edge
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());

			// Line from (-1,0,0) to (+1,0,0) coincides with triangle edge
			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));
		}

		// Thick line: collision detected when thickness bridges the gap to the triangle
		PRUnitTestMethod(ThickLinePiercesTriangle)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{2.0f, 0.4f}; // half-length=1, half-thickness=0.2

			// Line along Z, offset 0.15 above the XY plane: zero-thickness pierces but
			// the thick envelope should give a larger penetration depth.
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 0.15f, 0});

			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));

			Contact c;
			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w, c));
			// Penetration should be greater than zero-thickness case
			PR_EXPECT(c.m_depth > 0.0f);
		}

		// Thick line: parallel to triangle, within thickness envelope
		PRUnitTestMethod(ThickLineParallelNearTriangle)
		{
			auto tri = ShapeTriangle{v4{-2, -2, 0, 0}, v4{2, -2, 0, 0}, v4{0, 2, 0, 0}};
			auto line = ShapeLine{1.0f, 0.4f}; // half-length=0.5, half-thickness=0.2

			// Line along X-axis, 0.1 above the triangle: zero-thickness line doesn't intersect
			// (parallel), but thick line's envelope (0.2) reaches the triangle plane.
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 0.1f, 1});

			PR_EXPECT(TriangleVsLine(tri, l2w, line, r2w));
		}

		// Thick line: parallel but too far away
		PRUnitTestMethod(ThickLineParallelSeparated)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto line = ShapeLine{1.0f, 0.2f}; // half-thickness=0.1

			// Line along X-axis, 0.5 above the triangle: thickness 0.1 can't bridge 0.5 gap
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 0.5f, 1});

			PR_EXPECT(!TriangleVsLine(tri, l2w, line, r2w));
		}
	};
}
#endif
