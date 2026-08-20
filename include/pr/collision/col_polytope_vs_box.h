//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Polytope-vs-box SAT collision detection using a box-as-polytope view.
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/col_polytope_vs_polytope.h"
#include "pr/collision/col_polytope_view.h"

namespace pr::collision
{
	// Returns true if 'lhs' and 'rhs' are intersecting.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Penetration& pen)
	{
		auto box = polytope::MakeView(shape_cast<ShapeBox>(rhs));
		PolytopeVsPolytope(lhs, l2w, box.m_shape, r2w, pen);
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		PolytopeVsBox(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		auto box = polytope::MakeView(shape_cast<ShapeBox>(rhs));
		return PolytopeVsPolytope(lhs, l2w, box.m_shape, r2w, contact);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/unittest_helpers.h"
namespace pr::collision::tests
{
	PRUnitTestClass(PolytopeVsBoxTests)
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

		PRUnitTestMethod(FaceFaceContact, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto box = ShapeBox{v4{2, 2, 2, 0}};
			auto p2w = m4x4::Identity();
			auto b2w = m4x4::Translation(1.5f, 0, 0);

			Contact c;
			auto r = PolytopeVsBox(poly, p2w, box, b2w, c);
			Visualise(poly, p2w, box, b2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.75f, +1.0f, +1.0f, 1),
					v4(0.75f, -1.0f, +1.0f, 1),
					v4(0.75f, -1.0f, -1.0f, 1),
					v4(0.75f, +1.0f, -1.0f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 0.5f,
			}));
		}

		PRUnitTestMethod(Separated, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto box = ShapeBox{v4{2, 2, 2, 0}};

			Contact c;
			auto r = PolytopeVsBox(poly, m4x4::Identity(), box, m4x4::Translation(3.1f, 0, 0), c);

			PR_EXPECT(!r);
			PR_EXPECT(!c.contact());
		}

		PRUnitTestMethod(BoxWithS2R, Quick)
		{
			auto buf = MakeCube();
			auto& poly = buf.as<ShapePolytope>();
			auto box = ShapeBox{v4{2, 2, 2, 0}, m4x4::TransformDeg(0, 0, 45, v4{1.5f, 0, 0, 1})};

			Contact c;
			auto r = PolytopeVsBox(poly, m4x4::Identity(), box, m4x4::Identity(), c);
			Visualise(poly, m4x4::Identity(), box, m4x4::Identity(), c);

			PR_EXPECT(r);
			PR_EXPECT(c.contact());
		}
	};
}
#endif
