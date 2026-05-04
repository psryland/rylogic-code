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
		enum class ESatMode
		{
			Brute,
			Pruned,
		};
		struct SatDiagnostics
		{
			int m_face_axes_tested = {};
			int m_face_axes_ignored = {};
			int m_edge_pairs_raw = {};
			int m_edge_pairs_ignored = {};
			int m_edge_pairs_parallel = {};
			int m_edge_pairs_rejected = {};
			int m_edge_axes_tested = {};
		};

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

		inline v4 FaceNormal(ShapePolytope const& shape, m4x4 const& s2w, uint32_t face_index)
		{
			return Normalise(s2w * shape.face(face_index).m_plane.direction());
		}

		inline bool AxisInEdgeNormalCone(ShapePolytope const& shape, m4x4 const& s2w, ShapePolyEdge const& edge, v4 axis)
		{
			constexpr auto tol = 1e-5f;
			auto axis_len_sq = LengthSq(axis);
			if (axis_len_sq < Sqr(math::tiny<float>))
				return false;

			axis /= Sqrt(axis_len_sq);
			auto normal0 = FaceNormal(shape, s2w, edge.m_face0);
			auto normal1 = FaceNormal(shape, s2w, edge.m_face1);
			auto cos_angle = Clamp(Dot3(normal0, normal1), -1.0f, +1.0f);
			auto det = 1.0f - Sqr(cos_angle);
			if (det < Sqr(tol))
				return false;

			auto dot0 = Dot3(axis, normal0);
			auto dot1 = Dot3(axis, normal1);
			auto weight0 = (dot0 - cos_angle * dot1) / det;
			auto weight1 = (dot1 - cos_angle * dot0) / det;
			return weight0 >= -tol && weight1 >= -tol;
		}

		inline bool EdgePairCompatible(ShapePolytope const& lhs, m4x4 const& l2w, ShapePolyEdge const& edge_a, ShapePolytope const& rhs, m4x4 const& r2w, ShapePolyEdge const& edge_b, v4 axis)
		{
			return
				(AxisInEdgeNormalCone(lhs, l2w, edge_a, +axis) && AxisInEdgeNormalCone(rhs, r2w, edge_b, -axis)) ||
				(AxisInEdgeNormalCone(lhs, l2w, edge_a, -axis) && AxisInEdgeNormalCone(rhs, r2w, edge_b, +axis));
		}

		template <typename Penetration>
		void pr_vectorcall PolytopeVsPolytope_SAT(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen, ESatMode mode, SatDiagnostics* diagnostics)
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
				if (IgnoreFaceAxis(face))
				{
					if (diagnostics) ++diagnostics->m_face_axes_ignored;
					continue;
				}

				if (diagnostics) ++diagnostics->m_face_axes_tested;
				auto axis = l2w * face.m_plane.direction();
				if (!TestAxis(lhs, l2w, rhs, r2w, axis, pen))
					return;
			}

			for (auto const& face : rhs.faces())
			{
				if (IgnoreFaceAxis(face))
				{
					if (diagnostics) ++diagnostics->m_face_axes_ignored;
					continue;
				}

				if (diagnostics) ++diagnostics->m_face_axes_tested;
				auto axis = r2w * face.m_plane.direction();
				if (!TestAxis(lhs, l2w, rhs, r2w, axis, pen))
					return;
			}

			for (auto const& edge_a : lhs.edges())
			{
				auto ignore_edge_a = IgnoreEdgeAxes(edge_a);
				auto dir_a = l2w * edge_a.m_direction;
				for (auto const& edge_b : rhs.edges())
				{
					if (diagnostics) ++diagnostics->m_edge_pairs_raw;
					if (ignore_edge_a || IgnoreEdgeAxes(edge_b))
					{
						if (diagnostics) ++diagnostics->m_edge_pairs_ignored;
						continue;
					}

					auto dir_b = r2w * edge_b.m_direction;
					auto axis = Cross(dir_a, dir_b);
					if (LengthSq(axis) < Sqr(math::tiny<float>))
					{
						if (diagnostics) ++diagnostics->m_edge_pairs_parallel;
						continue;
					}
					if (mode == ESatMode::Pruned && !EdgePairCompatible(lhs, l2w, edge_a, rhs, r2w, edge_b, axis))
					{
						if (diagnostics) ++diagnostics->m_edge_pairs_rejected;
						continue;
					}

					if (diagnostics) ++diagnostics->m_edge_axes_tested;
					if (!TestAxis(lhs, l2w, rhs, r2w, axis, pen))
						return;
				}
			}
		}
	}

	// Brute-force SAT reference for convex polytope pairs.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Penetration& pen, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		polytope::PolytopeVsPolytope_SAT(lhs, l2w, rhs, r2w, pen, polytope::ESatMode::Brute, diagnostics);
	}

	// Gauss-map-pruned SAT prototype for convex polytope pairs.
	template <typename Penetration>
	void pr_vectorcall PolytopeVsPolytope_PrunedSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Penetration& pen, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		polytope::PolytopeVsPolytope_SAT(lhs, l2w, rhs, r2w, pen, polytope::ESatMode::Pruned, diagnostics);
	}

	// Returns true if polytopes 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		TestPenetration p;
		PolytopeVsPolytope_BruteSAT(lhs, l2w, rhs, r2w, p, diagnostics);
		return p.Contact();
	}
	inline bool pr_vectorcall PolytopeVsPolytope_PrunedSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		TestPenetration p;
		PolytopeVsPolytope_PrunedSAT(lhs, l2w, rhs, r2w, p, diagnostics);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall PolytopeVsPolytope_BruteSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		ContactPenetration p;
		PolytopeVsPolytope_BruteSAT(lhs, l2w, rhs, r2w, p, diagnostics);
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
	inline bool pr_vectorcall PolytopeVsPolytope_PrunedSAT(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact, polytope::SatDiagnostics* diagnostics = nullptr)
	{
		ContactPenetration p;
		PolytopeVsPolytope_PrunedSAT(lhs, l2w, rhs, r2w, p, diagnostics);
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
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
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

		PRUnitTestMethod(FaceFaceContact)
		{
			auto buf_a = MakeCube();
			auto buf_b = MakeCube();
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			Contact pruned;
			polytope::SatDiagnostics brute_stats;
			polytope::SatDiagnostics pruned_stats;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(1.5f, 0, 0);
			PR_EXPECT(PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c, &brute_stats));
			PR_EXPECT(PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned, &pruned_stats));
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
			PR_EXPECT(CheckContact(pruned, c));
			PR_EXPECT(pruned_stats.m_edge_axes_tested < brute_stats.m_edge_axes_tested);
			PR_EXPECT(pruned_stats.m_edge_pairs_rejected != 0);
		}

		PRUnitTestMethod(EdgeFaceContact)
		{
			auto buf_a = MakeCube();
			auto buf_b = MakeCube();
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			Contact pruned;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::TransformDeg(0.0f, 0.0f, 45.0f, v4(2.2f, 0, 0, 1));
			PR_EXPECT(PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c));
			PR_EXPECT(PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned));
			Visualise(pa, a2w, pb, b2w, c);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.892893f, 0, -1.0f, 1),
					v4(0.892893f, 0, +1.0f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.214213f,
			}));
			PR_EXPECT(CheckContact(pruned, c));
		}

		PRUnitTestMethod(VertexFaceContact)
		{
			v4 tet_pts[] = {
				v4{-0.4f, 0.0f, 0.0f, 1},
				v4{+0.5f, -0.5f, -0.5f, 1},
				v4{+0.5f, +0.5f, -0.5f, 1},
				v4{+0.5f, 0.0f, +0.5f, 1},
			};

			auto buf_a = MakeCube(2.0f);
			auto buf_b = BuildPolytopeFromPoints(tet_pts);
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			Contact pruned;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(2.35f, 0, 0);
			PR_EXPECT(PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c));
			PR_EXPECT(PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned));
			Visualise(pa, a2w, pb, b2w, c);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(1.975f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.05f,
			}));
			PR_EXPECT(CheckContact(pruned, c));
		}

		PRUnitTestMethod(CubeSeparated)
		{
			auto buf_a = MakeCube();
			auto buf_b = MakeCube();
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact c;
			Contact pruned;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(3.0f, 0, 0);
			PR_EXPECT(!PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, c));
			PR_EXPECT(!PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned));
			Visualise(pa, a2w, pb, b2w, c);
			PR_EXPECT(CheckContact(c, nullptr));
			PR_EXPECT(CheckContact(pruned, nullptr));
		}

		PRUnitTestMethod(PrunedMatchesBruteForRotatedCubes)
		{
			auto buf_a = MakeCube();
			auto buf_b = MakeCube();
			auto& pa = buf_a.as<ShapePolytope>();
			auto& pb = buf_b.as<ShapePolytope>();

			Contact brute;
			Contact pruned;
			polytope::SatDiagnostics brute_stats;
			polytope::SatDiagnostics pruned_stats;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::TransformDeg(30.0f, 17.0f, -12.0f, v4(1.2f, 0.25f, -0.15f, 1));
			auto brute_hit = PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, brute, &brute_stats);
			auto pruned_hit = PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned, &pruned_stats);
			Visualise(pa, a2w, pb, b2w, brute);

			PR_EXPECT(brute_hit);
			PR_EXPECT(pruned_hit);
			PR_EXPECT(CheckContact(pruned, brute, 1e-3f));
			PR_EXPECT(pruned_stats.m_edge_axes_tested < brute_stats.m_edge_axes_tested);
			PR_EXPECT(pruned_stats.m_edge_pairs_rejected != 0);
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
			PR_EXPECT(LengthSq(pa.m_base.m_s2r.pos) > Sqr(0.1f));
			PR_EXPECT(LengthSq(pb.m_base.m_s2r.pos) > Sqr(0.1f));

			Contact sat_contact;
			Contact pruned_contact;
			Contact gjk_contact;
			auto a2w = m4x4::Identity();
			auto b2w = m4x4::Translation(0.35f, 0.05f, 0.0f);
			auto sat_hit = PolytopeVsPolytope_BruteSAT(pa, a2w, pb, b2w, sat_contact);
			auto pruned_hit = PolytopeVsPolytope_PrunedSAT(pa, a2w, pb, b2w, pruned_contact);
			auto gjk_hit = GjkCollide(pa, a2w, pb, b2w, gjk_contact);
			Visualise(pa, a2w, pb, b2w, sat_contact);

			PR_EXPECT(sat_hit);
			PR_EXPECT(pruned_hit);
			PR_EXPECT(gjk_hit);
			PR_EXPECT(CheckContact(sat_contact, Contact{
				.m_axis = v4(0.801784f, -0.267261f, 0.534523f, 0),
				.m_manifold = {
					v4(0.201429f, -0.458571f, -0.431429f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.962141f,
			}));
			PR_EXPECT(CheckContact(pruned_contact, sat_contact));
			PR_EXPECT(CheckContact(sat_contact, gjk_contact));
		}
	};
}
#endif
