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
		auto r2l = InvertAffine(l2w) * r2w;

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
	PRUnitTest(CollisionBoxVsBoxTests)
	{
		using namespace pr::rdr12::ldraw;

		auto lhs = ShapeBox{v4(0.3f, 0.4f, 0.5f, 0.0f)};
		auto rhs = ShapeBox{v4(0.3f, 0.4f, 0.5f, 0.0f)};
		m4x4 l2w_[] =
		{
			m4x4::Identity(),
		};
		m4x4 r2w_[] =
		{
			m4x4::TransformRad(constants<float>::tau_by_8, 0, 0, v4(0.2f, 0.3f, 0.1f, 1.0f)),
			m4x4::TransformRad(0, constants<float>::tau_by_8, 0, v4(0.2f, 0.3f, 0.1f, 1.0f)),
			m4x4::TransformRad(0, 0, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
			m4x4::TransformRad(0, 0, -3 * constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
		};

		std::default_random_engine rng;
		for (int i = 0; i != 20; ++i)
		{
			Contact c;
			m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.5f).w1()};
			m4x4 r2w = i < _countof(r2w_) ? r2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.5f).w1()};

			Builder builder;
			builder._<LdrPhysicsShape>("lhs", 0x30FF0000).shape(lhs).o2w(l2w);
			builder._<LdrPhysicsShape>("rhs", 0x3000FF00).shape(rhs).o2w(r2w);
			//builder.Write(L"collision_unittests.ldr");
			if (BoxVsBox(lhs, l2w, rhs, r2w, c))
			{
				builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
				builder.Box("pt0", Colour32Yellow).dim(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
				builder.Box("pt1", Colour32Yellow).dim(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
			}
			//builder.Write(L"collision_unittests.ldr");
		}
	}
}
#endif