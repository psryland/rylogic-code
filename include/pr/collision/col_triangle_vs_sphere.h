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
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

		// Transform the sphere centre into triangle space
		auto s2t = InvertOrthonormal(l2w) * r2w.pos;

		// Find the closest point on the triangle to the sphere centre (in triangle space)
		auto closest = geometry::closest_point::PointToTriangle(s2t, tri.m_v.x, tri.m_v.y, tri.m_v.z);

		// Vector from closest point to sphere centre
		auto delta = s2t - closest;
		auto dist_sq = LengthSq(delta);
		auto dist = Sqrt(dist_sq);

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

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(TriangleVsSphereTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Sphere directly above triangle centre: face collision
		PRUnitTestMethod(SphereFaceCollision, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 0.3f);

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0, 0, -0.1f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.199994445f,
			}));
		}

		// Sphere touching a triangle edge
		PRUnitTestMethod(SphereEdgeCollision, Quick)
		{
			auto lhs = ShapeTriangle{v4{0, 0, 0, 1}, v4{2, 0, 0, 1}, v4{1, 2, 0, 1}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.0f, -0.3f, 0);

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, -1, 0, 0),
				.m_manifold = {
					v4(1, 0.1f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.2f,
			}));
		}

		// Sphere touching a triangle vertex
		PRUnitTestMethod(SphereVertexCollision, Quick)
		{
			auto lhs = ShapeTriangle{v4{0, 0, 0, 1}, v4{2, 0, 0, 1}, v4{1, 2, 0, 1}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(-0.3f, 0, 0);

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(-1, 0, 0, 0),
				.m_manifold = {
					v4(0.1f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.2f,
			}));
		}

		// Sphere clearly separated: below the triangle, beyond radius
		PRUnitTestMethod(Separated, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 1.0f);

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Degenerate triangle (collinear vertices) — should not crash
		PRUnitTestMethod(DegenerateTriangle, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, 0, 0, 1}, v4{0, 0, 0, 1}, v4{1, 0, 0, 1}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0.3f, 0);

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 1, 0, 0),
				.m_manifold = {
					v4(0, -0.1f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.2f,
			}));
		}

		// Sphere centred on the triangle surface
		PRUnitTestMethod(SphereCentreOnSurface, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0, 0, -0.25f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.5f,
			}));
		} 

		// Triangle-vs-Sphere with s2r transforms 
		PRUnitTestMethod(TriVsSphereWithS2R, Quick)
		{ 
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) }; 
			auto rhs = ShapeSphere{0.5f, m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) }; 
			auto l2w = m4x4::Identity(); 
			auto r2w = m4x4::Identity(); 
 
			Contact c; 
			auto r = TriangleVsSphere(lhs, l2w, rhs, r2w, c); 
			Visualise(lhs, l2w, rhs, r2w, c); 
 
			PR_EXPECT(r); 
			PR_EXPECT(CheckContact(c, Contact{ 
				.m_axis = v4(0.353553f,-0.707107f,0.612373f,0),
				.m_manifold = { 
					v4(0.880362f,0.239276f,-0.207219f,1),
				}, 
				.m_feature = EFeature::Vert,
				.m_depth = 0.323223f,
			})); 
		} 
	};
}
#endif
