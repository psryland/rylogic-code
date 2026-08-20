//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between an orientated box and a sphere, with generic penetration collection
	template <typename Penetration>
	void pr_vectorcall BoxVsSphere(Shape const& lhs, m4x4 const& l2w_, Shape const& rhs, m4x4 const& r2w_, Penetration& pen)
	{
		auto& box = shape_cast<ShapeBox   >(lhs);
		auto& sph = shape_cast<ShapeSphere>(rhs);
		auto l2w = l2w_ * lhs.m_s2r;
		auto r2w = r2w_ * rhs.m_s2r;

		// Convert into box space
		// Box centre to sphere centre vector in box space
		auto r2l = InvertOrthonormal(l2w) * r2w.pos - v4::Origin();

		// Get a vector from the sphere to the nearest point on the box
		auto closest = v4::Zero();
		auto dist_sq = 0.0f;
		for (int i = 0; i != 3; ++i)
		{
			if (r2l[i] > box.m_radius[i])
			{
				dist_sq += Sqr(r2l[i] - box.m_radius[i]);
				closest[i] = box.m_radius[i];
			}
			else if (r2l[i] < -box.m_radius[i])
			{
				dist_sq += Sqr(r2l[i] + box.m_radius[i]);
				closest[i] = -box.m_radius[i];
			}
			else
			{
				closest[i] = r2l[i];
			}
		}

		// If 'dist_sq' is zero then the centre of the sphere is inside the box.
		// The separating axis is the box face normal with the minimum penetration depth
		// (i.e., the shortest escape route for the sphere).
		if (dist_sq < math::tiny<float>)
		{
			// For each axis, the penetration is: sphere_radius + box_half_extent - |distance_from_centre|.
			// The minimum penetration axis is where (box_half_extent - |r2l|) is smallest,
			// i.e., the face the sphere centre is closest to.
			auto face_dist = v4{box.m_radius.x - Abs(r2l.x), box.m_radius.y - Abs(r2l.y), box.m_radius.z - Abs(r2l.z), FLT_MAX};
			auto i = MinElementIndex(face_dist);

			// Find the penetration depth
			auto depth = sph.m_radius + face_dist[i];
			pen(depth, [&]
			{
				// Find the separating axis
				auto norm = v4::Zero();
				norm[i] = Sign(r2l[i]);
				return l2w * norm;
			}, lhs.m_material_id, rhs.m_material_id);
		}

		// Otherwise the centre of the sphere is outside of the box
		else
		{
			// Find the penetration depth
			auto dist = Sqrt(dist_sq);
			auto depth = sph.m_radius - dist;
			pen(depth, [&]
			{
				// Find the separating axis
				return l2w * ((r2l - closest) / dist);
			}, lhs.m_material_id, rhs.m_material_id);
		}
	}

	// Returns true if the orientated box 'lhs' intersects the sphere 'rhs'
	inline bool pr_vectorcall BoxVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		BoxVsSphere(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall BoxVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		BoxVsSphere(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeBox>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(BoxVsSphereTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Sphere inside box: centre coincident
		PRUnitTestMethod(SphereInsideBox, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-5f, 0, 0);

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.35f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 1.29999995f,
			}));
		}

		// Sphere touching box face
		PRUnitTestMethod(SphereTouchingFace, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.3f, 0, 0);

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.899999976f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.200000048f,
			}));
		}

		// Sphere near box edge
		PRUnitTestMethod(SphereNearEdge, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.2f, 1.2f, 0);

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.707106829f, 0.707106829f, 0, 0),
				.m_manifold = {
					v4(0.923223317f, 0.923223317f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.21715723f,
			}));
		}

		// Sphere near box corner
		PRUnitTestMethod(SphereNearCorner, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.2f, 1.2f, 1.2f);

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.577350318f, 0.577350318f, 0.577350318f, 0),
				.m_manifold = {
					v4(0.955662429f, 0.955662429f, 0.955662429f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.153589755f,
			}));
		}

		// Separated: sphere far from box
		PRUnitTestMethod(Separated, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(5, 0, 0);

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}

		// Degenerate: zero-radius sphere inside box
		PRUnitTestMethod(ZeroRadiusSphereInside, Quick)
		{
			auto lhs = ShapeBox{v4{2, 2, 2, 0}};
			auto rhs = ShapeSphere{0.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.5f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 1.0f,
			}));
		}

		// Box-vs-Sphere with s2r transforms
		PRUnitTestMethod(BoxVsSphereWithS2R, Quick)
		{
			auto lhs = ShapeBox{ v4{1, 1, 1, 0}, m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) };
			auto rhs = ShapeSphere{ 0.5f, m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) };
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = BoxVsSphere(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.686426f,0.640856f,0.343689f,0),
				.m_manifold = {
					v4(0.882205f,-0.109975f,-0.0589793f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.656786919f,
			}));
		}
	};
}
#endif
