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
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

		// Transform to triangle space for simpler geometry
		auto t2w_inv = InvertOrthonormal(l2w);

		// Triangle vertices (in triangle space, as positions w=1)
		auto a = tri.m_v.x.w1();
		auto b = tri.m_v.y.w1();
		auto c = tri.m_v.z.w1();

		// Line segment endpoints in triangle space
		auto line_dir_ws = line.m_hlength * r2w.z; // half-extent vector in world space
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
		auto test_axis = [&](v4 axis)
		{
			// Project triangle vertices onto axis
			auto da = Dot3(axis, a);
			auto db = Dot3(axis, b);
			auto dc = Dot3(axis, c);
			auto t_min = std::min({da, db, dc});
			auto t_max = std::max({da, db, dc});

			// Project line segment onto axis, plus thickness envelope
			auto lm = Dot3(axis, line_mid);
			auto lr = Abs(Dot3(axis, line_half)) + line.m_radius * Length(axis);
			auto l_min = lm - lr;
			auto l_max = lm + lr;

			// Overlap = min of the two maximum intrusions
			auto depth = std::min(t_max - l_min, l_max - t_min);
			return pen(depth, [&]{ return Bool2SignF(t_min + t_max <= l_min + l_max) * (l2w * axis); }, lhs_.m_material_id, rhs_.m_material_id);
		};

		// --- Axis 1: Triangle normal ---
		if (!test_axis(tri_norm))
			return;

		// --- Axes 2-4: Cross products of triangle edges × line direction ---
		v4 edges[] = { e0, e1, e2 };
		for (int i = 0; i != 3; ++i)
		{
			auto axis = Cross(edges[i], line_half);

			// Skip degenerate axes (parallel edge and line direction)
			auto axis_len_sq = LengthSq(axis);
			if (axis_len_sq < Sqr(math::tiny<float>))
				continue;

			if (!test_axis(axis))
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

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeLine>(rhs), r2w, sep_axis, depth);

		contact.m_depth = depth;
		contact.m_axis = sep_axis;
		contact.m_manifold = manifold;
		contact.m_feature = feature;
		contact.m_mat_idA = p.m_mat_idA;
		contact.m_mat_idB = p.m_mat_idB;
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/unittest_helpers.h"

namespace pr::collision::tests
{
	PRUnitTestClass(TriangleVsLineTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Line piercing through the triangle
		PRUnitTestMethod(LinePiercesTriangle)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(+0.89442718f, +0.44721359f, 0, 0),
				.m_manifold = {
					v4(+0.200000003f, +0.100000001f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.44721359f,
			}));
		}

		// Line parallel to triangle, in the plane
		PRUnitTestMethod(LineInTrianglePlane)
		{
			auto lhs = ShapeTriangle{v4{-2, -1, 0, 1}, v4{2, -1, 0, 1}, v4{0, 2, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Line parallel to triangle but offset: should not collide
		PRUnitTestMethod(LineParallelSeparated)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 2, 1});

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Line endpoint touching the triangle
		PRUnitTestMethod(EndpointTouchesTriangle)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 1.0f);

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Line misses the triangle entirely: passes beside it
		PRUnitTestMethod(LineMissesTriangle)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(5, 0, 0);

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Line along a triangle edge: coplanar edge contact
		PRUnitTestMethod(LineAlongTriangleEdge)
		{
			auto lhs = ShapeTriangle{v4{-1, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Thick line: collision detected when thickness bridges the gap to the triangle
		PRUnitTestMethod(ThickLinePiercesTriangle)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{2.0f, 0.4f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 0.15f);

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(+0.89442718f, +0.44721359f, 0, 0),
				.m_manifold = {
					v4(+0.0211145f, +0.0105573f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.847213626f,
			}));
		}

		// Thick line: parallel to triangle, within thickness envelope
		PRUnitTestMethod(ThickLineParallelNearTriangle)
		{
			auto lhs = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}};
			auto rhs = ShapeLine{1.0f, 0.4f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 0.1f, 1});

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0.5f, 0, -0.15f, 1),
					v4(-0.5f, 0, -0.15f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.3f,
			}));
		}

		// Thick line: parallel but too far away
		PRUnitTestMethod(ThickLineParallelSeparated)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeLine{1.0f, 0.2f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0, 0.5f, 1});

			Contact c;
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Triangle-vs-Line with s2r transforms 
		PRUnitTestMethod(TriangleVsLineWithS2R) 
		{ 
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) };
			auto rhs = ShapeLine{1.0f, 0.2f, m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) };
			auto l2w = m4x4::Identity(); 
			auto r2w = m4x4::Identity(); 
 
			Contact c; 
			auto r = TriangleVsLine(lhs, l2w, rhs, r2w, c); 
			Visualise(lhs, l2w, rhs, r2w, c); 
 
			PR_EXPECT(r); 
			PR_EXPECT(CheckContact(c, Contact{ 
				.m_axis = v4(0.940947f,-0.19229f,-0.278645f,0),
				.m_manifold = { 
					v4(0.98419f,-0.0020251f,0.0147336f,1),
				}, 
				.m_feature = EFeature::Vert,
				.m_depth = 0.362814397f,
			})); 
		}
	};
}
#endif
