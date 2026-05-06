//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Polytope-vs-triangle SAT collision detection using a triangle-as-polytope view.
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/col_polytope_vs_polytope.h"

namespace pr::collision
{
	struct TriangleAsPolytope
	{
		ShapePolytope m_poly;
		std::array<ShapePolyVert,3>  m_vert;
		std::array<ShapePolyFace,1>  m_face;
		std::array<ShapePolyEdge,3>  m_edge;

		TriangleAsPolytope(ShapeTriangle const& tri)
			: m_poly(tri.m_base.m_s2r, tri.m_base.m_material_id, tri.m_base.m_flags)
			, m_vert({ tri.m_v.x.w1(), tri.m_v.y.w1(), tri.m_v.z.w1() })
			, m_face({ { Plane(tri.normal(), 0.0f), {0u,1u,2u}, ShapePolytope::EFaceFlags::None } })
			, m_edge({ edge(tri, 0, 1), edge(tri, 1, 2), edge(tri, 2, 0) })
		{
			m_poly.Complete(3, 1, 3);
		}
		static ShapePolyEdge edge(ShapeTriangle const& tri, int v0, int v1)
		{
			return ShapePolyEdge{
				.m_direction = Normalise(tri.m_v[v1].w1() - tri.m_v[v0].w1(), v4::Zero()),
				.m_v0 = v0,
				.m_v1 = v1,
				.m_face0 = 0,
				.m_face1 = 0,
				.m_flags = ShapePolytope::EEdgeFlags::None,
			};
		}
	};

	// Returns true if 'lhs' and 'rhs' are intersecting.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsTriangle(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Penetration& pen)
	{
		TriangleAsPolytope tri(shape_cast<ShapeTriangle>(rhs));
		PolytopeVsPolytope(lhs, l2w, tri.m_poly, r2w, pen);
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
		TriangleAsPolytope tri(shape_cast<ShapeTriangle>(rhs));
		return PolytopeVsPolytope(lhs, l2w, tri.m_poly, r2w, contact);
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

		PRUnitTestMethod(FaceContact)
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

		PRUnitTestMethod(Separated)
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

		PRUnitTestMethod(TriangleWithS2R)
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
