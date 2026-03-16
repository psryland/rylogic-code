//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Triangle vs Sphere collision detection.
//
// Algorithm:
//  Find the closest point on the triangle to the sphere centre.
//  If the distance is less than the sphere radius, they overlap.
//  The separating axis is the direction from closest point to sphere centre.
//
// ShapeTriangle stores vertices in m_v: x=vert0, y=vert1, z=vert2, w=normal.
// Vertices have w=0 (they are offsets in shape space, not positions).
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between a triangle and a sphere, with generic penetration collection.
	// 'lhs' is the triangle, 'rhs' is the sphere (tri-table order: Triangle=3, Sphere=0).
	template <typename Penetration>
	void pr_vectorcall TriangleVsSphere(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& tri = shape_cast<ShapeTriangle>(lhs_);
		auto& sph = shape_cast<ShapeSphere>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Transform the sphere centre into triangle space
		auto s2t = InvertOrthonormal(l2w) * r2w.pos - v4::Origin();

		// Find the closest point on the triangle to the sphere centre (in triangle space)
		auto closest = geometry::closest_point::PointToTriangle(s2t.w1(), tri.m_v.x.w1(), tri.m_v.y.w1(), tri.m_v.z.w1());

		// Vector from closest point to sphere centre
		auto delta = s2t - (closest - v4::Origin());
		auto dist_sq = LengthSq(delta);
		auto dist = Sqrt(dist_sq + math::tiny<float>);

		// Penetration depth: positive means overlap
		auto depth = sph.m_radius - dist;

		pen(depth, [&]
		{
			// Separating axis: from triangle surface toward sphere centre (in world space).
			if (dist_sq > Sqr(math::tiny<float>))
				return l2w * (delta / dist);

			// Sphere centre is exactly on the triangle surface — use the triangle normal
			return l2w * tri.m_v.w;
		}, lhs_.m_material_id, rhs_.m_material_id);
	}

	// Returns true if the triangle intersects the sphere
	inline bool pr_vectorcall TriangleVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		TriangleVsSphere(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the triangle and sphere are intersecting, with contact details
	inline bool pr_vectorcall TriangleVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		TriangleVsSphere(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTestClass(TriangleVsSphereTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			// Triangle on the XY plane centred at origin
			auto tri = ShapeTriangle{v4{-1,  0, 0, 0}, v4{1, 0, 0, 0}, v4{0, 1, 0, 0}};
			auto sph = ShapeSphere{0.3f};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = m4x4::Random(rng, v4::Origin(), 0.5f);
				m4x4 r2w = m4x4::Random(rng, v4::Origin(), 1.0f);

				Builder builder;
				{ auto& g = builder.Group("tri", 0x30FF0000); AddShape(g, tri); g.o2w(l2w); }
				{ auto& g = builder.Group("sph", 0x3000FF00); AddShape(g, sph); g.o2w(r2w); }
				if (TriangleVsSphere(tri, l2w, sph, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Sphere directly above triangle centre: face collision
		PRUnitTestMethod(SphereFaceCollision)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto sph = ShapeSphere{0.5f};

			// Sphere 0.3 above the XY-plane triangle → depth = 0.5 - 0.3 = 0.2
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 0.3f, 0});

			Contact c;
			PR_EXPECT(TriangleVsSphere(tri, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.2f, 0.02f));
		}

		// Sphere touching a triangle edge
		PRUnitTestMethod(SphereEdgeCollision)
		{
			// Triangle on XY plane
			auto tri = ShapeTriangle{v4{0, 0, 0, 0}, v4{2, 0, 0, 0}, v4{1, 2, 0, 0}};
			auto sph = ShapeSphere{0.5f};

			// Place sphere at (1, -0.3, 0) — near the bottom edge, 0.3 away
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.0f, -0.3f, 0, 0});

			// Closest point on edge (0,0)→(2,0) to (1,-0.3) is (1,0). Distance = 0.3.
			PR_EXPECT(TriangleVsSphere(tri, l2w, sph, r2w));

			Contact c;
			PR_EXPECT(TriangleVsSphere(tri, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.2f, 0.02f));
		}

		// Sphere touching a triangle vertex
		PRUnitTestMethod(SphereVertexCollision)
		{
			auto tri = ShapeTriangle{v4{0, 0, 0, 0}, v4{2, 0, 0, 0}, v4{1, 2, 0, 0}};
			auto sph = ShapeSphere{0.5f};

			// Place sphere at (-0.3, 0, 0) — near vertex (0,0,0)
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{-0.3f, 0, 0, 0});

			// Closest point is vertex (0,0,0). Distance = 0.3. Depth = 0.2.
			PR_EXPECT(TriangleVsSphere(tri, l2w, sph, r2w));
		}

		// Sphere clearly separated: below the triangle, beyond radius
		PRUnitTestMethod(Separated)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 1.0f, 0});

			PR_EXPECT(!TriangleVsSphere(tri, l2w, sph, r2w));
		}

		// Degenerate triangle (collinear vertices) — should not crash
		PRUnitTestMethod(DegenerateTriangle)
		{
			// All three vertices on the X-axis → degenerate triangle (zero area)
			auto tri = ShapeTriangle{v4{-1, 0, 0, 0}, v4{0, 0, 0, 0}, v4{1, 0, 0, 0}};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0.3f, 0, 0});

			// Should not crash; closest point on degenerate tri is still well-defined
			auto result = TriangleVsSphere(tri, l2w, sph, r2w);
			(void)result; // just verify no crash
		}

		// Sphere centred on the triangle surface
		PRUnitTestMethod(SphereCentreOnSurface)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity(); // sphere at origin, on triangle surface

			// Depth = sphere radius = 0.5
			Contact c;
			PR_EXPECT(TriangleVsSphere(tri, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.5f, 0.02f));
		}
	};
}
#endif
