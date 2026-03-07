//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"

namespace pr::collision
{
	// Test for collision between two spheres
	template <typename Penetration>
	void pr_vectorcall SphereVsSphere(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapeSphere>(lhs_);
		auto& rhs = shape_cast<ShapeSphere>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Distance between centres
		auto r2l = r2w.pos - l2w.pos;
		auto len = Length(r2l);
		auto sep = lhs.m_radius + rhs.m_radius - len;
		
		// Use default axis if centres coincide to avoid division by zero
		pen(sep, [&]{ return len > math::tiny<float> ? r2l/len : v4{1,0,0,0}; }, lhs_.m_material_id, rhs_.m_material_id);
	}

	// Returns true if 'lhs' intersects 'rhs'
	inline bool pr_vectorcall SphereVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		SphereVsSphere(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall SphereVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		SphereVsSphere(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		// Determine the sign of the separating axis to make it the normal from 'lhs' to 'rhs'
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth   = p.Depth();
		contact.m_axis    = sign * sep_axis;
		contact.m_point   = FindContactPoint(shape_cast<ShapeSphere>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTest(CollisionSphereVsSphere)
	{
		using namespace pr::rdr12::ldraw;

		auto lhs = ShapeSphere{0.3f};
		auto rhs = ShapeSphere{0.4f};
		m4x4 l2w_[] =
		{
			m4x4::Identity(),
		};
		m4x4 r2w_[] =
		{
			m4x4::TransformRad(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
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
			if (SphereVsSphere(lhs, l2w, rhs, r2w, c))
			{
				builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
				builder.Box("pt0", Colour32Yellow).dim(0.01f).pos(c.m_point - 0.5f*c.m_depth*c.m_axis);
				builder.Box("pt1", Colour32Yellow).dim(0.01f).pos(c.m_point + 0.5f*c.m_depth*c.m_axis);
			}
			//builder.Write(L"collision_unittests.ldr");
		}
	}
}
#endif
