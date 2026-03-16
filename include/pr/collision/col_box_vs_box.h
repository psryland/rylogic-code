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
		auto r2l_abs = Abs(r2l.rot) + m3x4(math::tiny<float>);

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
			ra = Dot3(lhs.m_radius, r2l_abs[i]);
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
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis  = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeBox>(lhs), l2w, shape_cast<ShapeBox>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTestClass(BoxVsBoxTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto lhs = ShapeBox{ v4(0.3f, 0.4f, 0.5f, 0.0f) };
			auto rhs = ShapeBox{ v4(0.3f, 0.4f, 0.5f, 0.0f) };
			m4x4 l2w_[] =
			{
				m4x4::Identity(),
			};
			m4x4 r2w_[] =
			{
				m4x4::Transform(constants<float>::tau_by_8, 0, 0, v4(0.2f, 0.3f, 0.1f, 1.0f)),
				m4x4::Transform(0, constants<float>::tau_by_8, 0, v4(0.2f, 0.3f, 0.1f, 1.0f)),
				m4x4::Transform(0, 0, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
				m4x4::Transform(0, 0, -3 * constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
			};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4::Random(rng, v4::Origin(), 0.5f);
				m4x4 r2w = i < _countof(r2w_) ? r2w_[i] : m4x4::Random(rng, v4::Origin(), 0.5f);

				Builder builder;
				{ auto& g = builder.Group("lhs", 0x30FF0000); AddShape(g, lhs); g.o2w(l2w); }
				{ auto& g = builder.Group("rhs", 0x3000FF00); AddShape(g, rhs); g.o2w(r2w); }
				if (BoxVsBox(lhs, l2w, rhs, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Coincident boxes: maximum overlap
		PRUnitTestMethod(CoincidentBoxes)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			PR_EXPECT(BoxVsBox(box, l2w, box, r2w));
			Contact c;
			PR_EXPECT(BoxVsBox(box, l2w, box, r2w, c));
			PR_EXPECT(c.m_depth > 0.0f);
		}

		// Face-to-face overlap: boxes separated along X
		PRUnitTestMethod(FaceToFaceOverlap)
		{
			auto box = ShapeBox{v4{2, 2, 2, 0}}; // half-extent (1,1,1)
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.5f, 0, 0, 0}); // overlap of 0.5 in X

			Contact c;
			PR_EXPECT(BoxVsBox(box, l2w, box, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.5f, 0.02f));
		}

		// Separated: gap between boxes
		PRUnitTestMethod(Separated)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{3, 0, 0, 0});

			PR_EXPECT(!BoxVsBox(box, l2w, box, r2w));
		}

		// Edge-to-edge: rotated box touching via edges
		PRUnitTestMethod(EdgeContact)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();

			// Rotate rhs 45° about Z and offset so edges overlap
			auto r2w = m4x4::Transform(RotationRad<m3x4>(0, 0, constants<float>::tau_by_8), v4{1.3f, 1.3f, 0, 1});

			// The rotated box edge should be close to lhs corner
			Contact c;
			auto result = BoxVsBox(box, l2w, box, r2w, c);
			if (result)
			{
				PR_EXPECT(c.m_depth > 0.0f);
				PR_EXPECT(IsNormalised(c.m_axis));
			}
		}

		// Corner-to-face: rotated box poking into face
		PRUnitTestMethod(CornerToFace)
		{
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();

			// Rotate rhs by 45° on two axes so its corner points into lhs face
			auto r2w = m4x4::Transform(RotationRad<m3x4>(constants<float>::tau_by_8, constants<float>::tau_by_8, 0), v4{2.0f, 0, 0, 1});

			Contact c;
			auto result = BoxVsBox(box, l2w, box, r2w, c);
			if (result)
			{
				PR_EXPECT(c.m_depth > 0.0f);
			}
		}
	};
}
#endif