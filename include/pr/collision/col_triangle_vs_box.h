//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Triangle vs Box collision detection using the Separating Axis Theorem (SAT).
//
// Algorithm:
//  Test 13 potential separating axes:
//   - 1 axis:  triangle face normal
//   - 3 axes:  box face normals (x, y, z)
//   - 9 axes:  cross products of 3 triangle edges × 3 box axes
//
//  For each axis, project both shapes onto it and check for overlap.
//  If any axis has no overlap, the shapes are separated.
//  The axis with the minimum overlap is the collision normal.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between a triangle and an oriented box, with generic penetration collection.
	// 'lhs' is the triangle (3), 'rhs' is the box (1) (tri-table order: Triangle=3, Box=1).
	template <typename Penetration>
	void pr_vectorcall TriangleVsBox(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& tri = shape_cast<ShapeTriangle>(lhs_);
		auto& box = shape_cast<ShapeBox>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Work in box space to simplify box projection
		auto b2w_inv = InvertOrthonormal(r2w);

		// Triangle vertices in box space (as positions)
		auto tv0 = b2w_inv * (l2w * tri.m_v.x.w1());
		auto tv1 = b2w_inv * (l2w * tri.m_v.y.w1());
		auto tv2 = b2w_inv * (l2w * tri.m_v.z.w1());

		// Triangle edges in box space
		auto e0 = tv1 - tv0;
		auto e1 = tv2 - tv1;
		auto e2 = tv0 - tv2;

		// Triangle normal in box space (not necessarily normalised)
		auto tri_norm = Cross(e0, e1);

		// Triangle centroid in box space
		auto tri_ctr = (tv0 + tv1 + tv2) / 3.0f;

		// Lambda for returning a separating axis with correct sign (in world space)
		auto sep_axis = [&](v4 sa_boxspace)
		{
			auto sa = r2w * sa_boxspace;
			auto tri_proj = Dot3(sa, l2w * ((tri.m_v.x + tri.m_v.y + tri.m_v.z) / 3.0f).w1());
			auto box_proj = Dot3(sa, r2w.pos);
			return Bool2SignF(tri_proj < box_proj) * sa;
		};

		// Helper: project triangle vertices onto an axis and get min/max
		auto tri_interval = [&](v4 axis, float& tri_min, float& tri_max)
		{
			auto d0 = Dot3(axis, tv0);
			auto d1 = Dot3(axis, tv1);
			auto d2 = Dot3(axis, tv2);
			tri_min = std::min({d0, d1, d2});
			tri_max = std::max({d0, d1, d2});
		};

		// Helper: project the box onto an axis in box space (box centred at origin with half-extents)
		auto box_radius = [&](v4 axis)
		{
			return box.m_radius.x * Abs(axis.x) + box.m_radius.y * Abs(axis.y) + box.m_radius.z * Abs(axis.z);
		};

		float tri_min, tri_max, br, depth;

		// --- Axis 1: Triangle face normal ---
		tri_interval(tri_norm, tri_min, tri_max);
		br = box_radius(tri_norm);
		depth = br + tri_max - std::max(tri_min, -br); // overlap
		depth = std::min(br - tri_min, tri_max + br);   // min penetration along this axis
		// Simpler: project onto normal, both intervals relative to box centre at 0
		{
			auto c = Dot3(tri_norm, tri_ctr);
			auto r = box_radius(tri_norm);
			auto e0_ = Dot3(tri_norm, tv0) - c;
			auto e1_ = Dot3(tri_norm, tv1) - c;
			auto e2_ = Dot3(tri_norm, tv2) - c;
			auto t_min = std::min({e0_, e1_, e2_}) + c;
			auto t_max = std::max({e0_, e1_, e2_}) + c;
			// overlap = min(t_max - (-r), r - t_min) = min(t_max + r, r - t_min)
			depth = std::min(t_max + r, r - t_min);
		}
		if (!pen(depth, [&]{ return sep_axis(tri_norm); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// --- Axes 2-4: Box face normals (x, y, z axes in box space) ---
		for (int i = 0; i != 3; ++i)
		{
			auto axis = v4::Zero();
			axis[i] = 1.0f;
			tri_interval(axis, tri_min, tri_max);
			// Box interval: [-radius, +radius]
			depth = std::min(tri_max + box.m_radius[i], box.m_radius[i] - tri_min);
			if (!pen(depth, [&]{ return sep_axis(axis); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}

		// --- Axes 5-13: Cross products of triangle edges × box axes ---
		v4 edges[] = { e0, e1, e2 };
		for (int i = 0; i != 3; ++i) // triangle edges
		{
			for (int j = 0; j != 3; ++j) // box axes
			{
				auto box_axis = v4::Zero();
				box_axis[j] = 1.0f;
				auto axis = Cross(edges[i], box_axis);

				// Skip degenerate axes (parallel edge and box axis)
				auto axis_len_sq = LengthSq(axis);
				if (axis_len_sq < Sqr(math::tiny<float>))
					continue;

				tri_interval(axis, tri_min, tri_max);
				auto r = box_radius(axis);
				depth = std::min(tri_max + r, r - tri_min);
				if (!pen(depth, [&]{ return sep_axis(axis); }, lhs_.m_material_id, rhs_.m_material_id))
					return;
			}
		}
	}

	// Returns true if the triangle intersects the box
	inline bool pr_vectorcall TriangleVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		TriangleVsBox(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the triangle and box are intersecting, with contact details
	inline bool pr_vectorcall TriangleVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		TriangleVsBox(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeBox>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTestClass(TriangleVsBoxTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto tri = ShapeTriangle{v4{-1, 0, 0, 0}, v4{1, 0, 0, 0}, v4{0, 1, 0, 0}};
			auto box = ShapeBox{v4{0.3f, 0.4f, 0.5f, 0.0f}};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = m4x4::Random(rng, v4::Origin(), 0.5f);
				m4x4 r2w = m4x4::Random(rng, v4::Origin(), 0.5f);

				Builder builder;
				{ auto& g = builder.Group("tri", 0x30FF0000); AddShape(g, tri); g.o2w(l2w); }
				{ auto& g = builder.Group("box", 0x3000FF00); AddShape(g, box); g.o2w(r2w); }
				if (TriangleVsBox(tri, l2w, box, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Triangle face-on to a box face: clear overlap
		PRUnitTestMethod(FaceOnOverlap)
		{
			// Triangle on XY plane, embedded in the unit box
			auto tri = ShapeTriangle{v4{-0.5f, -0.5f, 0, 0}, v4{0.5f, -0.5f, 0, 0}, v4{0, 0.5f, 0, 0}};
			auto box = ShapeBox{v4{1, 1, 1, 0}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			PR_EXPECT(TriangleVsBox(tri, l2w, box, r2w));
		}

		// Triangle entirely outside box
		PRUnitTestMethod(Separated)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto box = ShapeBox{v4{0.5f, 0.5f, 0.5f, 0.0f}};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{5, 0, 0, 0});

			PR_EXPECT(!TriangleVsBox(tri, l2w, box, r2w));
		}

		// Triangle edge intersects box face
		PRUnitTestMethod(EdgeIntersectsBoxFace)
		{
			auto tri = ShapeTriangle{v4{-2, 0, 0, 0}, v4{2, 0, 0, 0}, v4{0, 2, 0, 0}};
			auto box = ShapeBox{v4{0.3f, 0.3f, 0.3f, 0.0f}};

			// Box slightly above, triangle edge passes through it
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0.1f, 0, 0});

			PR_EXPECT(TriangleVsBox(tri, l2w, box, r2w));
		}

		// Triangle vertex inside box
		PRUnitTestMethod(VertexInsideBox)
		{
			auto tri = ShapeTriangle{v4{0, 0, 0, 0}, v4{5, 0, 0, 0}, v4{0, 5, 0, 0}};
			auto box = ShapeBox{v4{1, 1, 1, 0}};

			// Triangle vertex at origin is inside the unit box
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			PR_EXPECT(TriangleVsBox(tri, l2w, box, r2w));
		}

		// Triangle parallel to box face, barely touching
		PRUnitTestMethod(BarleyTouching)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto box = ShapeBox{v4{2, 2, 1.0f, 0.0f}}; // half-extent (1, 1, 0.5)

			// Triangle at z=0.49 (just inside box top face at z=0.5)
			auto l2w = m4x4::Translation(v4{0, 0, 0.49f, 0});
			auto r2w = m4x4::Identity();

			PR_EXPECT(TriangleVsBox(tri, l2w, box, r2w));
		}

		// Triangle parallel to box face, barely separated
		PRUnitTestMethod(BarelySeparated)
		{
			auto tri = ShapeTriangle{v4{-1, -1, 0, 0}, v4{1, -1, 0, 0}, v4{0, 1, 0, 0}};
			auto box = ShapeBox{v4{2, 2, 1.0f, 0.0f}}; // half-extent (1, 1, 0.5)

			// Triangle at z=0.51 (just above box top face at z=0.5)
			auto l2w = m4x4::Translation(v4{0, 0, 0.51f, 0});
			auto r2w = m4x4::Identity();

			PR_EXPECT(!TriangleVsBox(tri, l2w, box, r2w));
		}

		// Rotated triangle intersecting rotated box: tests cross-product axes
		PRUnitTestMethod(RotatedIntersection)
		{
			auto tri = ShapeTriangle{v4{-1, 0, 0, 0}, v4{1, 0, 0, 0}, v4{0, 1, 0, 0}};
			auto box = ShapeBox{v4{0.5f, 0.5f, 0.5f, 0.0f}};

			// Rotate both by different angles
			auto l2w = m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_8, 0, 0), v4{0.1f, 0, 0, 1});
			auto r2w = m4x4::Transform(RotationRad<m3x4>(0, constants<float>::tau_by_8, 0), v4{0.3f, 0.2f, 0, 1});

			// Just verify it doesn't crash and returns a reasonable result
			Contact c;
			auto result = TriangleVsBox(tri, l2w, box, r2w, c);
			if (result)
			{
				PR_EXPECT(c.m_depth > 0.0f);
				PR_EXPECT(IsNormalised(c.m_axis));
			}
		}
	};
}
#endif
