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
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

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

		// Centroids for sign determination
		auto ctr_a = (a0 + a1 + a2) / 3.0f;
		auto ctr_b = (b0 + b1 + b2) / 3.0f;

		// Test a separating axis. Projects both triangles onto the axis and returns the overlap depth.
		auto test_axis = [&](v4 axis) -> float
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
			return std::min(a_max - b_min, b_max - a_min);
		};

		// Lambda for returning a separating axis with the correct sign
		auto make_sep_axis = [&](v4 sa)
		{
			return Bool2SignF(Dot3(sa, ctr_a) < Dot3(sa, ctr_b)) * sa;
		};

		// Note: degenerate triangles have a normal of zero so don't collide
		// --- Axis 1: Normal of triangle A ---
		{
			auto norm_a = l2w * lhs.m_v.w; // normal in world space
			auto depth = test_axis(norm_a);
			if (!pen(depth, [&]{ return make_sep_axis(norm_a); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}

		// --- Axis 2: Normal of triangle B ---
		{
			auto norm_b = r2w * rhs.m_v.w;
			auto depth = test_axis(norm_b);
			if (!pen(depth, [&]{ return make_sep_axis(norm_b); }, lhs_.m_material_id, rhs_.m_material_id))
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
				auto depth = test_axis(axis);
				if (!pen(depth, [&]{ return make_sep_axis(axis); }, lhs_.m_material_id, rhs_.m_material_id))
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
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		sep_axis = Bool2SignF(p0 < p1) * sep_axis;

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
		inline static constexpr bool CreateVisuals = true;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Two coplanar overlapping triangles
		PRUnitTestMethod(CoplanarOverlapping)
		{
			// Two triangles on the XY plane, overlapping
			auto tri_a = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{0, -1, 0, 1}, v4{2, -1, 0, 1}, v4{1, 1, 0, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			// Coplanar zero-thickness overlap has no positive penetration depth.
			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(!r);
		}

		// Two intersecting triangles (crossing like an X)
		PRUnitTestMethod(CrossingTriangles)
		{
			// Triangle A on XY plane, Triangle B on XZ plane, both passing through origin
			auto tri_a = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{-1, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0, 0, 1, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.00001f,0,0);

			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

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
		PRUnitTestMethod(ParallelSeparated)
		{
			auto tri_a = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 2); // 2 units apart in Z

			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(!r);
		}

		// Edge-to-edge touching: triangles share a common edge
		PRUnitTestMethod(SharedEdge)
		{
			auto tri_a = ShapeTriangle{v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{0, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, -1, 0, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			// Shared edge along (0,0,0)→(1,0,0), but zero-thickness contact has no positive penetration depth.
			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(!r);
		}

		// Vertex touching: one triangle's vertex touches the other's face
		PRUnitTestMethod(VertexTouchesFace)
		{
			// Large triangle on XY plane
			// Small triangle with vertex at origin, tilted
			auto tri_a = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}};
			auto tri_b = ShapeTriangle{v4{0, 0, -0.00001f, 1}, v4{1, 0, 1, 1}, v4{0, 1, 1, 1}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			// tri_b vertex (0,0,0) is on tri_a's face, but point contact has no positive penetration depth.
			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0,0,1,0),
				.m_manifold = {
					v4(0, -0.5f, -0.5f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.0f,
			}));
		}

		// Separated in all axes: no projection overlap
		PRUnitTestMethod(ClearlySeparated)
		{
			auto tri_a = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(10, 10, 10); // far away

			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(!r);
		}

		// Perpendicular triangles intersecting: T-junction
		PRUnitTestMethod(TJunction)
		{
			// Triangle A on XY plane
			auto tri_a = ShapeTriangle{v4{-2, -2, 0, 1}, v4{2, -2, 0, 1}, v4{0, 2, 0, 1}};
			// Triangle B vertical (XZ plane), cutting through A
			auto tri_b = ShapeTriangle{v4{0, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0.5f, 0, 1, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(-0.872871518f, -0.436435759f, 0.21821788f, 0),
				.m_manifold = {
					v4(-0.5f, -1, -0.5f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.654653668f,
			}));
		}

		// Intersecting degenerates
		PRUnitTestMethod(Degenerates)
		{
			auto tri_a = ShapeTriangle{v4{0, -1, 0, 1}, v4{0, 0, 0, 1}, v4{0, +1, 0, 1}};
			auto tri_b = ShapeTriangle{v4{ 0, 0, -1, 1}, v4{1, 0, -1, 1}, v4{0.5f, 0, 1, 1}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsTriangle(tri_a, l2w, tri_b, r2w, c);
			Visualise(tri_a, l2w, tri_b, r2w, c);

			PR_EXPECT(!r);
		}
	};
}
#endif
