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
		auto l2w = l2w_ * lhs_.m_s2r;
		auto r2w = r2w_ * rhs_.m_s2r;

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

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeSphere>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(SphereVsSphereTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Overlapping: spheres centred at the same point
		PRUnitTestMethod(CoincidentCentres, Quick)
		{
			auto lhs = ShapeSphere{1.0f};
			auto rhs = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-5f, 0, 0);

			Contact c;
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 2.0f,
			}));
		}

		// Barely touching: distance = sum of radii
		PRUnitTestMethod(BarelyTouching, Quick)
		{
			auto lhs = ShapeSphere{0.5f};
			auto rhs = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.7999f, 0, 0);

			Contact c;
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.5f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.000100016594f,
			}));
		}

		// Clearly separated
		PRUnitTestMethod(Separated, Quick)
		{
			auto lhs = ShapeSphere{0.5f};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.1f, 0, 0);

			Contact c;
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
			PR_EXPECT(!c.contact());
		}

		// Axis direction: should point from lhs centre to rhs centre
		PRUnitTestMethod(AxisDirection, Quick)
		{
			auto lhs = ShapeSphere{1.0f};
			auto rhs = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 1.0f, 0);

			Contact c;
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 1, 0, 0),
				.m_manifold = {
					v4(0, 0.5f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 1.0f,
			}));
		}

		// Different radii: large sphere engulfing small sphere
		PRUnitTestMethod(EngulfedSphere, Quick)
		{
			auto lhs = ShapeSphere{5.0f};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1, 1, 1);

			Contact c;
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.57735f,0.57735f,0.57735f,0),
				.m_manifold = {
					v4(1.79904f,1.79904f,1.79904f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 3.76794910f,
			}));
		}

		// Sphere-vs-Spherewith s2r transforms 
		PRUnitTestMethod(SphereVsSphereWithS2R, Quick)
		{ 
			auto lhs = ShapeSphere{0.6f, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) }; 
			auto rhs = ShapeSphere{0.5f, m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) }; 
			auto l2w = m4x4::Identity(); 
			auto r2w = m4x4::Identity(); 
 
			Contact c; 
			auto r = SphereVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c); 
 
			PR_EXPECT(r); 
			PR_EXPECT(CheckContact(c, Contact{ 
				.m_axis = v4(1,0,0,0),
				.m_manifold = { 
					v4(0.8f,0,0,1),
				}, 
				.m_feature = EFeature::Vert,
				.m_depth = 0.6f,
			})); 
		} 
	};
}
#endif
