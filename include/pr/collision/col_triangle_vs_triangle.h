//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Triangle vs Triangle collision detection using the Separating Axis Theorem.
//
// Algorithm:
//  Test 11 potential separating axes:
//   - 1 axis:  normal of triangle A
//   - 1 axis:  normal of triangle B
//   - 9 axes:  cross products of 3 edges of A × 3 edges of B
//
//  Both triangles are zero-volume (flat faces). A separating axis exists
//  if and only if the projected intervals on any axis do not overlap.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between two triangles, with generic penetration collection.
	template <typename Penetration>
	void pr_vectorcall TriangleVsTriangle(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapeTriangle>(lhs_);
		auto& rhs = shape_cast<ShapeTriangle>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

		// Test triangles in world space, but near the origin
		auto ofs = 0.5f * (l2w.pos + r2w.pos).w0();
		l2w.pos -= ofs;
		r2w.pos -= ofs;

		// Transform all vertices to world space
		auto a0 = l2w * lhs.m_v.x;
		auto a1 = l2w * lhs.m_v.y;
		auto a2 = l2w * lhs.m_v.z;
		auto b0 = r2w * rhs.m_v.x;
		auto b1 = r2w * rhs.m_v.y;
		auto b2 = r2w * rhs.m_v.z;

		// Edges in world space
		auto ea0 = a1 - a0;
		auto ea1 = a2 - a1;
		auto ea2 = a0 - a2;
		auto eb0 = b1 - b0;
		auto eb1 = b2 - b1;
		auto eb2 = b0 - b2;

		// Test a separating axis. Projects both triangles onto the axis and returns the overlap depth.
		auto test_axis = [&](v4 axis)
		{
			// Project triangle A vertices
			auto da0 = Dot3(axis, a0);
			auto da1 = Dot3(axis, a1);
			auto da2 = Dot3(axis, a2);
			auto a_min = std::min({da0, da1, da2});
			auto a_max = std::max({da0, da1, da2});

			// Project triangle B vertices
			auto db0 = Dot3(axis, b0);
			auto db1 = Dot3(axis, b1);
			auto db2 = Dot3(axis, b2);
			auto b_min = std::min({db0, db1, db2});
			auto b_max = std::max({db0, db1, db2});

			// Overlap = min of the two maximum intrusions
			auto depth = std::min(a_max - b_min, b_max - a_min);
			return pen(depth, [&]{ return Bool2SignF(a_min + a_max <= b_min + b_max) * axis; }, lhs_.m_material_id, rhs_.m_material_id);
		};

		// Note: degenerate triangles have a normal of zero so don't collide
		// --- Axis 1: Normal of triangle A ---
		{
			auto norm_a = l2w * lhs.m_v.w; // normal in world space
			if (!test_axis(norm_a))
				return;
		}

		// --- Axis 2: Normal of triangle B ---
		{
			auto norm_b = r2w * rhs.m_v.w;
			if (!test_axis(norm_b))
				return;
		}

		// --- Axes 3-11: Cross products of edges ---
		v4 edges_a[] = { ea0, ea1, ea2 };
		v4 edges_b[] = { eb0, eb1, eb2 };
		for (int i = 0; i != 3; ++i)
		{
			for (int j = 0; j != 3; ++j)
			{
				auto axis = Cross(edges_a[i], edges_b[j]);

				// Skip degenerate axes (parallel edges)
				auto axis_len_sq = LengthSq(axis);
				if (axis_len_sq < Sqr(math::tiny<float>))
					continue;

				axis /= Sqrt(axis_len_sq);
				if (!test_axis(axis))
					return;
			}
		}
	}

	// Returns true if the two triangles intersect
	inline bool pr_vectorcall TriangleVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		TriangleVsTriangle(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the two triangles are intersecting, with contact details
	inline bool pr_vectorcall TriangleVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		TriangleVsTriangle(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeTriangle>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(TriangleVsTriangleTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Two coplanar overlapping triangles
		PRUnitTestMethod(CoplanarOverlapping, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeTriangle{v4{0, -1, 0, 1}, v4{2, -1, 0, 1}, v4{1, 1, 0, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Two intersecting triangles (crossing like an X)
		PRUnitTestMethod(CrossingTriangles, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeTriangle{v4{-1, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0, 0, 1, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-5f,0,0);

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.816497f,0.408248f,-0.408248f, 0),
				.m_manifold = {
					v4(0, 0.166665f, 0.166665f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.816488445f,
			}));
		}

		// Two separated triangles: parallel but offset
		PRUnitTestMethod(ParallelSeparated, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 2);

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Edge-to-edge touching: triangles share a common edge
		PRUnitTestMethod(SharedEdge, Quick)
		{
			auto lhs = ShapeTriangle{v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeTriangle{v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, -1, 0, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Vertex touching: one triangle's vertex touches the other's face
		PRUnitTestMethod(VertexTouchesFace, Quick)
		{
			auto lhs = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}};
			auto rhs = ShapeTriangle{v4{0, 0, -0.00001f, 1}, v4{1, 0, 1, 1}, v4{0, 1, 1, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0,0,1,0),
				.m_manifold = {
					v4(0, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.0f,
			}));
		}

		// Separated in all axes: no projection overlap
		PRUnitTestMethod(ClearlySeparated, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(10, 10, 10);

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Perpendicular triangles intersecting: T-junction
		PRUnitTestMethod(TJunction, Quick)
		{
			auto lhs = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}};
			auto rhs = ShapeTriangle{v4{0, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0.5f, 0, 1, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-5f,0,0);

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(+0.872871518f, +0.436435759f, -0.21821788f, 0),
				.m_manifold = {
					v4(+0.571429f, +0.142857f, +0.0714286f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.654653668f,
			}));
		}

		// Intersecting degenerates
		PRUnitTestMethod(Degenerates, Quick)
		{
			auto lhs = ShapeTriangle{v4{0, -1, 0, 1}, v4{0, 0, 0, 1}, v4{0, +1, 0, 1}};
			auto rhs = ShapeTriangle{v4{ 0, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0.5f, 0, 1, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Tri-vs-Tri with s2r transforms
		PRUnitTestMethod(TriVsTriWithS2R, Quick)
		{
			auto lhs = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) };
			auto rhs = ShapeTriangle{v4{0, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0.5f, 0, 1, 1}, m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) };
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.929562f,-0.12738f,-0.345962f,0),
				.m_manifold = {
					v4(1.11464f,-0.00566204f,-0.415233f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.318721f,
			}));
		}
	};
}
#endif
