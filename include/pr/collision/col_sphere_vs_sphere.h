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
	PRUnitTestClass(SphereVsSphereTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto lhs = ShapeSphere{0.3f};
			auto rhs = ShapeSphere{0.4f};
			m4x4 l2w_[] =
			{
				m4x4::Identity(),
			};
			m4x4 r2w_[] =
			{
				m4x4::Transform(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
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
				if (SphereVsSphere(lhs, l2w, rhs, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.01f).pos(c.m_point - 0.5f*c.m_depth*c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.01f).pos(c.m_point + 0.5f*c.m_depth*c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Overlapping: spheres centred at the same point
		PRUnitTestMethod(CoincidentCentres)
		{
			auto sph = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			PR_EXPECT(SphereVsSphere(sph, l2w, sph, r2w));
			Contact c;
			PR_EXPECT(SphereVsSphere(sph, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 2.0f, 0.01f)); // full overlap = 2 * radius
		}

		// Barely touching: distance = sum of radii
		PRUnitTestMethod(BarelyTouching)
		{
			auto lhs = ShapeSphere{0.5f};
			auto rhs = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0.8f, 0, 0, 0}); // distance = 0.8 = 0.5+0.3

			// Exactly touching → depth = 0
			Contact c;
			auto result = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			if (result) PR_EXPECT(c.m_depth <= 0.01f);
		}

		// Clearly separated
		PRUnitTestMethod(Separated)
		{
			auto lhs = ShapeSphere{0.5f};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{3.0f, 0, 0, 0});

			PR_EXPECT(!SphereVsSphere(lhs, l2w, rhs, r2w));
		}

		// Axis direction: should point from lhs centre to rhs centre
		PRUnitTestMethod(AxisDirection)
		{
			auto lhs = ShapeSphere{1.0f};
			auto rhs = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 1.0f, 0, 0}); // rhs above lhs

			Contact c;
			PR_EXPECT(SphereVsSphere(lhs, l2w, rhs, r2w, c));
			PR_EXPECT(c.m_axis.y > 0.0f); // axis points toward rhs (+Y)
			PR_EXPECT(FEqlRelative(c.m_depth, 1.0f, 0.01f)); // depth = 2-1 = 1
		}

		// Different radii: large sphere engulfing small sphere
		PRUnitTestMethod(EngulfedSphere)
		{
			auto lhs = ShapeSphere{5.0f};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.0f, 0, 0, 0});

			// Small sphere fully inside large sphere
			PR_EXPECT(SphereVsSphere(lhs, l2w, rhs, r2w));
			Contact c;
			PR_EXPECT(SphereVsSphere(lhs, l2w, rhs, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 4.5f, 0.01f)); // 5+0.5-1 = 4.5
		}
	};
}
#endif
