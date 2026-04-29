//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between two oriented boxes, with generic penetration collection
	template <typename Penetration>
	void pr_vectorcall BoxVsBox(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapeBox>(lhs_);
		auto& rhs = shape_cast<ShapeBox>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Compute a transform for 'rhs' in 'lhs's frame
		auto r2l = InvertOrthonormal(l2w) * r2w;

		// Compute common sub expressions. Add in an epsilon term to counteract arithmetic
		// errors when two edges are parallel and their cross product is (near) 0
		auto r2l_abs = Abs(r2l.rot) + m3x3(math::tiny<float>);

		// Lambda for returning a separating axis with the correct sign
		auto sep_axis = [&](v4 sa) { return Sign(Dot(r2l.pos, sa)) * sa; };

		float ra, rb, sp;

		// Test axes L = lhs.x, L = lhs.y, L = lhs.z
		for (int i = 0; i != 3; ++i)
		{
			ra = lhs.m_radius[i];
			rb = rhs.m_radius.x * r2l_abs.x[i] + rhs.m_radius.y * r2l_abs.y[i] + rhs.m_radius.z * r2l_abs.z[i];
			sp = Abs(r2l.pos[i]);
			if (!pen(ra + rb - sp, [&]{ return sep_axis(l2w[i]); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}

		// Test axes L = rhs.x, L = rhs.y, L = rhs.z
		for (int i = 0; i != 3; ++i)
		{
			ra = Dot(lhs.m_radius.xyz, r2l_abs[i]);
			rb = rhs.m_radius[i];
			sp = Abs(Dot3(r2l.pos, r2l[i]));
			if (!pen(ra + rb - sp, [&]{ return sep_axis(r2w[i]); }, lhs_.m_material_id, rhs_.m_material_id))
				return;
		}

		// Test axis L = lhs.x X rhs.x
		ra = lhs.m_radius.y * r2l_abs.x.z + lhs.m_radius.z * r2l_abs.x.y;
		rb = rhs.m_radius.y * r2l_abs.z.x + rhs.m_radius.z * r2l_abs.y.x;
		sp = Abs(r2l.pos.z * r2l.x.y - r2l.pos.y * r2l.x.z);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.x, r2w.x)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.x X rhs.y
		ra = lhs.m_radius.y * r2l_abs.y.z + lhs.m_radius.z * r2l_abs.y.y;
		rb = rhs.m_radius.x * r2l_abs.z.x + rhs.m_radius.z * r2l_abs.x.x;
		sp = Abs(r2l.pos.z * r2l.y.y - r2l.pos.y * r2l.y.z);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.x, r2w.y)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.x X rhs.z
		ra = lhs.m_radius.y * r2l_abs.z.z + lhs.m_radius.z * r2l_abs.z.y;
		rb = rhs.m_radius.x * r2l_abs.y.x + rhs.m_radius.y * r2l_abs.x.x;
		sp = Abs(r2l.pos.z * r2l.z.y - r2l.pos.y * r2l.z.z);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.x, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.y X rhs.x
		ra = lhs.m_radius.x * r2l_abs.x.z + lhs.m_radius.z * r2l_abs.x.x;
		rb = rhs.m_radius.y * r2l_abs.z.y + rhs.m_radius.z * r2l_abs.y.y;
		sp = Abs(r2l.pos.x * r2l.x.z - r2l.pos.z * r2l.x.x);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.y, r2w.x)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.y X rhs.y
		ra = lhs.m_radius.x * r2l_abs.y.z + lhs.m_radius.z * r2l_abs.y.x;
		rb = rhs.m_radius.x * r2l_abs.z.y + rhs.m_radius.z * r2l_abs.x.y;
		sp = Abs(r2l.pos.x * r2l.y.z - r2l.pos.z * r2l.y.x);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.y, r2w.y)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.y X rhs.z
		ra = lhs.m_radius.x * r2l_abs.z.z + lhs.m_radius.z * r2l_abs.z.x;
		rb = rhs.m_radius.x * r2l_abs.y.y + rhs.m_radius.y * r2l_abs.x.y;
		sp = Abs(r2l.pos.x * r2l.z.z - r2l.pos.z * r2l.z.x);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.y, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.z X rhs.x
		ra = lhs.m_radius.x * r2l_abs.x.y + lhs.m_radius.y * r2l_abs.x.x;
		rb = rhs.m_radius.y * r2l_abs.z.z + rhs.m_radius.z * r2l_abs.y.z;
		sp = Abs(r2l.pos.y * r2l.x.x - r2l.pos.x * r2l.x.y);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.z, r2w.x)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.z X rhs.y
		ra = lhs.m_radius.x * r2l_abs.y.y + lhs.m_radius.y * r2l_abs.y.x;
		rb = rhs.m_radius.x * r2l_abs.z.z + rhs.m_radius.z * r2l_abs.x.z;
		sp = Abs(r2l.pos.y * r2l.y.x - r2l.pos.x * r2l.y.y);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.z, r2w.y)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		// Test axis L = lhs.z X rhs.z
		ra = lhs.m_radius.x * r2l_abs.z.y + lhs.m_radius.y * r2l_abs.z.x;
		rb = rhs.m_radius.x * r2l_abs.y.z + rhs.m_radius.y * r2l_abs.x.z;
		sp = Abs(r2l.pos.y * r2l.z.x - r2l.pos.x * r2l.z.y);
		if (!pen(ra + rb - sp, [&]{ return sep_axis(Cross(l2w.z, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;
	}

	// Returns true if orientated boxes 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall BoxVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		BoxVsBox(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall BoxVsBox(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		BoxVsBox(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		// Determine the sign of the separating axis to make it the normal from 'lhs' to 'rhs'
		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot(sep_axis, (r2w * rhs.m_s2p).pos);
		sep_axis = Bool2SignF(p0 < p1) * sep_axis;

		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeBox>(lhs), l2w, shape_cast<ShapeBox>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(BoxVsBoxTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Coincident boxes: maximum overlap
		PRUnitTestMethod(CoincidentBoxes)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-4f, 0, 0);

			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);
			
			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1,0,0,0),
				.m_manifold = {
					v4(0, -0.5f, -0.5f, 1),
					v4(0, +0.5f, -0.5f, 1),
					v4(0, +0.5f, +0.5f, 1),
					v4(0, -0.5f, +0.5f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 1.0f,
			}));
		}

		// Face-to-face overlap: boxes separated along X
		PRUnitTestMethod(FaceToFaceOverlap)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.9f, 0.3f, -0.2f);

			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1,0,0,0),
				.m_manifold = {
					v4(0.449992f, +0.5f, +0.3f, 1),
					v4(0.450007f, -0.2f, +0.3f, 1),
					v4(0.449992f, -0.2f, -0.5f, 1),
					v4(0.449992f, +0.5f, -0.5f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 0.1f,
			}));
		}

		// Face-to-face overlap, twisted
		PRUnitTestMethod(FaceToFaceTwistedOverlap)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::TransformDeg(30, 0, 0, v4{0.9f, 0.3f, -0.2f, 1});

			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1,0,0,0),
				.m_manifold = {
					v4(0.449992f, +0.500000f, +0.4535900f, 1),
					v4(0.450007f, -0.383013f, -0.0169873f, 1),
					v4(0.449992f, -0.104145f, -0.5000000f, 1),
					v4(0.449992f, +0.500000f, -0.5000000f, 1),
				},
				.m_feature = EFeature::Quad,
				.m_depth = 0.1f,
			}));
		}

		// Separated: gap between boxes
		PRUnitTestMethod(Separated)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.2f, 0, 0);

			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(!r);
			PR_EXPECT(!c.contact());
		}

		// Edge-to-Edge
		PRUnitTestMethod(EdgeEdgeContact)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::TransformDeg(45, 0, 45, v4{0.7f, 1.0f, 0.2f, 1});

			// The rotated box edge should be close to lhs corner
			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.57735f,0.816497f,0,0),
				.m_manifold = {
					v4(0.45f,0.429289f,0.265685f,1),
					v4(0,0,0,1),
					v4(0,0,0,1),
					v4(0,0,0,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.173228189f,
			}));
		}

		// Edge-to-Face: rotated box touching via edges
		PRUnitTestMethod(EdgeFaceContact)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::TransformDeg(0, 0, 45, v4{0.7f, 0.7f, 0.2f, 1});

			// The rotated box edge should be close to lhs corner
			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.707107f,0.707107f,0, 0),
				.m_manifold = {
					v4(0.423218f,0.423218f,+0.5f,1),
					v4(0.423218f,0.423218f,-0.3f,1),
					v4(0,0,0,1),
					v4(0,0,0,1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.217172399f,
			}));
		}

		// Corner-to-face: rotated box poking into face
		PRUnitTestMethod(CornerToFace)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::TransformDeg(45, 45, 0, v4{1, 0, 0, 1});

			Contact c;
			auto r = BoxVsBox(box, l2w, box, r2w, c);
			Visualise(box, l2w, box, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1,0,0,0),
				.m_manifold = {
					v4(0.323223f,0,-0.146447f,1),
					v4(0,0,0,1),
					v4(0,0,0,1),
					v4(0,0,0,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.353568316f,
			}));
		}
	};
}
#endif
