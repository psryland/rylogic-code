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
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

		// Work in box space to simplify box projection
		auto b2w_inv = InvertOrthonormal(r2w);

		// Triangle vertices in box space (as positions)
		auto tv0 = b2w_inv * (l2w * tri.m_v.x);
		auto tv1 = b2w_inv * (l2w * tri.m_v.y);
		auto tv2 = b2w_inv * (l2w * tri.m_v.z);

		// Triangle edges in box space
		auto e0 = tv1 - tv0;
		auto e1 = tv2 - tv1;
		auto e2 = tv0 - tv2;

		// Triangle normal in box space (not necessarily normalised)
		auto tri_norm = Cross(e0, e1);

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

		auto test_axis = [&](v4 axis)
		{
			float tri_min, tri_max;
			tri_interval(axis, tri_min, tri_max);

			auto radius = box_radius(axis);
			auto depth = std::min(tri_max + radius, radius - tri_min);
			return pen(depth, [&]{ return Bool2SignF(tri_min + tri_max <= 0.0f) * (r2w * axis); }, lhs_.m_material_id, rhs_.m_material_id);
		};

		// --- Axis 1: Triangle face normal ---
		if (!test_axis(tri_norm))
			return;

		// --- Axes 2-4: Box face normals (x, y, z axes in box space) ---
		for (int i = 0; i != 3; ++i)
		{
			auto axis = v4::Zero();
			axis[i] = 1.0f;
			if (!test_axis(axis))
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

				if (!test_axis(axis))
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

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeTriangle>(lhs), l2w, shape_cast<ShapeBox>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(TriangleVsBoxTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Triangle face-on to a box face: clear overlap
		PRUnitTestMethod(FaceOnOverlap, Quick)
		{
			auto lhs = ShapeTriangle{v4{-0.5f, -0.5f, 0, 1}, v4{0.5f, -0.5f, 0, 1}, v4{0, 0.5f, 0, 1}};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0, 0.5f, -0.25f, 1),
					v4(-0.5f, -0.5f, -0.25f, 1),
					v4(+0.5f, -0.5f, -0.25f, 1),
				},
				.m_feature = EFeature::Tri,
				.m_depth = 0.5f,
			}));
		}

		// Triangle entirely outside box
		PRUnitTestMethod(Separated, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeBox{v4{0.5f, 0.5f, 0.5f, 0.0f}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(5, 0, 0);

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Triangle edge intersects box face
		PRUnitTestMethod(EdgeIntersectsBoxFace, Quick)
		{
			auto lhs = ShapeTriangle{v4{-2, 0, 0, 1}, v4{2, 0, 0, 1}, v4{0, 2, 0, 1}};
			auto rhs = ShapeBox{v4{0.3f, 0.3f, 0.3f, 0.0f}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0.1f, 0);

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0.150000006f, 0.25f, -0.075000003f, 1),
					v4(-0.150000006f, 0.25f, -0.075000003f, 1),
					v4(-0.150000095f, 0, -0.075000003f, 1),
					v4(0.150000095f, 0, -0.075000003f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 0.150000006f,
			}));
		}

		// Triangle vertex inside box
		PRUnitTestMethod(VertexInsideBox, Quick)
		{
			auto lhs = ShapeTriangle{v4{0, 0, 0, 1}, v4{5, 0, 0, 1}, v4{0, 5, 0, 1}};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, 1, 0),
				.m_manifold = {
					v4(0.5f, 0.5f, -0.25f, 1),
					v4(0, 0.5f, -0.25f, 1),
					v4(0, 0, -0.25f, 1),
					v4(0.5f, 0, -0.25f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 0.5f,
			}));
		}

		// Triangle parallel to box face, barely touching
		PRUnitTestMethod(BarleyTouching, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeBox{v4{2, 2, 1.0f, 0.0f}};
			auto l2w = m4x4::Translation(0, 0, 0.49f);
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0, -1, 0),
				.m_manifold = {
					v4(-1, -1, 0.495000005f, 1),
					v4(0, +1, 0.495000005f, 1),
					v4(+1, -1, 0.495000005f, 1),
				},
				.m_feature = EFeature::Tri,
				.m_depth = 0.00999999046f,
			}));
		}

		// Triangle parallel to box face, barely separated
		PRUnitTestMethod(BarelySeparated, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, -1, 0, 1}, v4{1, -1, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeBox{v4{2, 2, 1.0f, 0.0f}};
			auto l2w = m4x4::Translation(0, 0, 0.51f);
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Rotated triangle intersecting rotated box: tests cross-product axes
		PRUnitTestMethod(RotatedIntersection, Quick)
		{
			auto lhs = ShapeTriangle{v4{-1, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1}};
			auto rhs = ShapeBox{v4{0.5f, 0.5f, 0.5f, 0.0f}};
			auto l2w = m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, 0, 0), v4{0.1f, 0, 0, 1});
			auto r2w = m4x4::Transform(RotationRad<m3x3>(0, constants<float>::tau_by_8, 0), v4{0.3f, 0.2f, 0, 1});

			Contact c;
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 0.707106829f, -0.707106769f, 0),
				.m_manifold = {
					v4(0.300000012f, 0.0508883521f, 0.252665043f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.285355419f,
			}));
		}

		// Triangle-vs-Box with s2r transforms 
		PRUnitTestMethod(TriangleVsBoxWithS2R, Quick)
		{ 
			auto lhs = ShapeTriangle{v4{-1, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 1, 0, 1}, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) };
			auto rhs = ShapeBox{v4{0.5f, 0.5f, 0.5f, 0.0f}, m4x4::TransformDeg(30, 10, -80, v4{0.8f, 0, 0, 1}) };
			auto l2w = m4x4::Identity(); 
			auto r2w = m4x4::Identity(); 
 
			Contact c; 
			auto r = TriangleVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c); 
 
			PR_EXPECT(r); 
			PR_EXPECT(CheckContact(c, Contact{ 
				.m_axis = v4(0.353553f,-0.707107f,0.612372f,0),
				.m_manifold = { 
					v4(0.539528f,0.21116f,0.0144039f,1),
				}, 
				.m_feature = EFeature::Vert,
				.m_depth = 0.253034f,
			})); 
		} 
	};
}
#endif
