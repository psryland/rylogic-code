//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Polytope-vs-triangle SAT collision detection using a triangle-as-polytope view.
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/col_polytope_vs_polytope.h"
#include "pr/collision/col_polytope_view.h"

namespace pr::collision
{
	// Returns true if 'lhs' and 'rhs' are intersecting.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Penetration& pen)
	{
		auto tri = polytope::MakeView(shape_cast<ShapeTriangle>(rhs));
		PolytopeVsPolytope(lhs, l2w, tri.m_shape, r2w, pen);
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		PolytopeVsTriangle(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		auto tri = polytope::MakeView(shape_cast<ShapeTriangle>(rhs));
		return PolytopeVsPolytope(lhs, l2w, tri.m_shape, r2w, contact);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/unittest_helpers.h"
namespace pr::collision::tests
{
	PRUnitTestClass(PolytopeVsTriangleTests)
	{
		inline static constexpr bool CreateVisuals = false;

		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		byte_data<16> MakeCube(float radius = 1.0f)
		{
			v4 cube_pts[] = {
				v4{-radius, -radius, -radius, 1}, v4{+radius, -radius, -radius, 1},
				v4{-radius, +radius, -radius, 1}, v4{+radius, +radius, -radius, 1},
				v4{-radius, -radius, +radius, 1}, v4{+radius, -radius, +radius, 1},
				v4{-radius, +radius, +radius, 1}, v4{+radius, +radius, +radius, 1},
			};
			return BuildPolytopeFromPoints(cube_pts);
		}

		PRUnitTestMethod(FaceContact, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto tri = ShapeTriangle{
				v4{-0.5f, -0.5f, 0.0f, 1},
				v4{+0.5f, -0.5f, 0.0f, 1},
				v4{+0.0f, +0.5f, 0.0f, 1}};
			auto p2w = m4x4::Identity();
			auto t2w = m4x4::Translation(0, 0, 0.9f);

			Contact c;
			auto r = PolytopeVsTriangle(poly, p2w, tri, t2w, c);
			Visualise(poly, p2w, tri, t2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(+0.0f, +0.5f, 0.95f, 1),
					v4(-0.5f, -0.5f, 0.95f, 1),
					v4(+0.5f, -0.5f, 0.95f, 1),
				},
				.m_feature = EFeature::Tri,
				.m_depth = 0.1f,
			}));
		}

		PRUnitTestMethod(Separated, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto tri = ShapeTriangle{
				v4{-0.5f, -0.5f, 0.0f, 1},
				v4{+0.5f, -0.5f, 0.0f, 1},
				v4{+0.0f, +0.5f, 0.0f, 1}};

			Contact c;
			auto r = PolytopeVsTriangle(poly, m4x4::Identity(), tri, m4x4::Translation(0, 0, 1.1f), c);

			PR_EXPECT(!r);
			PR_EXPECT(!c.contact());
		}

		PRUnitTestMethod(TriangleWithS2R, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto tri = ShapeTriangle{
				v4{-0.5f, -0.5f, 0.0f, 1},
				v4{+0.5f, -0.5f, 0.0f, 1},
				v4{+0.0f, +0.5f, 0.0f, 1},
				m4x4::TransformDeg(30, 10, -20, v4{0.2f, 0.1f, 0.4f, 1})};

			Contact c;
			auto r = PolytopeVsTriangle(poly, m4x4::Identity(), tri, m4x4::Identity(), c);
			Visualise(poly, m4x4::Identity(), tri, m4x4::Identity(), c);

			PR_EXPECT(r);
			PR_EXPECT(c.contact());
		}
	};
}
#endif
