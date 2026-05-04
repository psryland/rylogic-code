//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Brute-force polytope-vs-polytope SAT reference.
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	namespace polytope
	{
		inline std::pair<float, float> Project(ShapePolytope const& shape, m4x4 const& s2w, v4 axis)
		{
			auto min_dist = +limits<float>::max();
			auto max_dist = -limits<float>::max();
			for (auto const& vert : shape.verts())
			{
				auto dist = Dot3(axis, s2w * vert);
				min_dist = Min(min_dist, dist);
				max_dist = Max(max_dist, dist);
			}
			return { min_dist, max_dist };
		}

		template <typename Penetration>
		bool TestAxis(ShapePolytope const& lhs, m4x4 const& l2w, ShapePolytope const& rhs, m4x4 const& r2w, v4 axis, Penetration& pen)
		{
			auto axis_len_sq = LengthSq(axis);
			if (axis_len_sq < Sqr(math::tiny<float>))
				return true;

			axis /= Sqrt(axis_len_sq);

			auto [a_min, a_max] = Project(lhs, l2w, axis);
			auto [b_min, b_max] = Project(rhs, r2w, axis);
			auto depth = Min(a_max - b_min, b_max - a_min);

			return pen(
				depth,
				[=]
				{
					return Bool2SignF(a_min + a_max <= b_min + b_max) * axis;
				},
				lhs.m_base.m_material_id,
				rhs.m_base.m_material_id);
		}
	}

	// Brute-force SAT reference for convex polytope pairs.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapePolytope>(lhs_);
		auto& rhs = shape_cast<ShapePolytope>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

		// Test in world space, but near the pair centre to reduce projection cancellation.
		auto ofs = 0.5f * (l2w.pos + r2w.pos).w0();
		l2w.pos -= ofs;
		r2w.pos -= ofs;

		for (auto const& face : lhs.faces())
		{
			auto axis = l2w * face.m_plane.direction();
			if (!polytope::TestAxis(lhs, l2w, rhs, r2w, axis, pen))
				return;
		}

		for (auto const& face : rhs.faces())
		{
			auto axis = r2w * face.m_plane.direction();
			if (!polytope::TestAxis(lhs, l2w, rhs, r2w, axis, pen))
				return;
		}

		for (auto const& edge_a : lhs.edges())
		{
			auto dir_a = l2w * edge_a.m_direction;
			for (auto const& edge_b : rhs.edges())
			{
				auto dir_b = r2w * edge_b.m_direction;
				auto axis = Cross(dir_a, dir_b);
				if (!polytope::TestAxis(lhs, l2w, rhs, r2w, axis, pen))
					return;
			}
		}
	}

	// Returns true if polytopes 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		PolytopeVsPolytope_BruteSAT(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		PolytopeVsPolytope_BruteSAT(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapePolytope>(lhs), l2w, shape_cast<ShapePolytope>(rhs), r2w, sep_axis, depth);

		contact.m_axis = sep_axis;
		contact.m_depth = depth;
		contact.m_manifold = manifold;
		contact.m_feature = feature;
		contact.m_mat_idA = p.m_mat_idA;
		contact.m_mat_idB = p.m_mat_idB;
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/col_gjk.h"
#include "pr/collision/unittest_helpers.h"
namespace pr::collision::tests
{
	PRUnitTestClass(PolytopeVsPolytopeBruteSATTests)
	{
		inline static constexpr bool CreateVisuals = true;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		PRUnitTestMethod(CubeOverlap)
		{
			v4 cube_pts[] = {
				v4{-1, -1, -1, 1}, v4{+1, -1, -1, 1},
				v4{-1, +1, -1, 1}, v4{+1, +1, -1, 1},
				v4{-1, -1, +1, 1}, v4{+1, -1, +1, 1},
				v4{-1, +1, +1, 1}, v4{+1, +1, +1, 1},
			};

			auto buf_a = BuildPolytopeFromPoints(cube_pts);
			auto buf_b = BuildPolytopeFromPoints(cube_pts);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(1.5f, 0, 0);
			PR_EXPECT(PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c));
			Visualise(pa, a2w, pb, b2w, c);
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

		PRUnitTestMethod(CubeSeparated)
		{
			v4 cube_pts[] = {
				v4{-1, -1, -1, 1}, v4{+1, -1, -1, 1},
				v4{-1, +1, -1, 1}, v4{+1, +1, -1, 1},
				v4{-1, -1, +1, 1}, v4{+1, -1, +1, 1},
				v4{-1, +1, +1, 1}, v4{+1, +1, +1, 1},
			};

			auto buf_a = BuildPolytopeFromPoints(cube_pts);
			auto buf_b = BuildPolytopeFromPoints(cube_pts);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(3.0f, 0, 0);
			PR_EXPECT(!PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c));
			Visualise(pa, a2w, pb, b2w, c);
			PR_EXPECT(CheckContact(c, nullptr));
		}

		PRUnitTestMethod(AgreesWithGjkOnSimpleCases)
		{
			v4 pts_a[] = {
				v4{-1, -1, -1, 1}, v4{+1, -1, -1, 1},
				v4{0, +1, -1, 1}, v4{0, 0, +1, 1},
			};
			v4 pts_b[] = {
				v4{-0.8f, -1.0f, -0.6f, 1}, v4{+0.9f, -0.7f, -0.8f, 1},
				v4{+0.1f, +1.1f, -0.9f, 1}, v4{-0.2f, -0.1f, +1.2f, 1},
			};

			auto buf_a = BuildPolytopeFromPoints(pts_a);
			auto buf_b = BuildPolytopeFromPoints(pts_b);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact sat_contact;
			Contact gjk_contact;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(0.35f, 0.05f, 0.0f);
			auto sat_hit = PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, sat_contact);
			auto gjk_hit = GjkCollide(pa, a2w, pb, b2w, gjk_contact);
			Visualise(pa, a2w, pb, b2w, sat_contact);

			PR_EXPECT(sat_hit);
			PR_EXPECT(gjk_hit);
			PR_EXPECT(CheckContact(sat_contact, Contact{
				.m_axis = v4(0.801784f, -0.267261f, 0.534523f, 0),
				.m_manifold = {
					v4(0.201429f, -0.458571f, -0.431429f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.962141f,
			}));
			PR_EXPECT(CheckContact(sat_contact, gjk_contact));
		}
	};
}
#endif
