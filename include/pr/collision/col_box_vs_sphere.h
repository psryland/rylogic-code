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
		auto l2w = l2w_ * lhs.m_s2p;
		auto r2w = r2w_ * rhs.m_s2p;

		// Convert into box space
		// Box centre to sphere centre vector in box space
		auto r2l = InvertAffine(l2w) * r2w.pos - v4::Origin();
	
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

		// Determine the sign of the separating axis to make it the normal from 'lhs' to 'rhs'
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth   = p.Depth();
		contact.m_axis    = sign * sep_axis;
		contact.m_point   = FindContactPoint(shape_cast<ShapeBox>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTest(CollisionBoxVsSphere)
	{
		using namespace pr::rdr12::ldraw;

		auto lhs = ShapeBox{v4{0.3f, 0.4f, 0.5f, 0.0f}};
		auto rhs = ShapeSphere{0.3f};
		m4x4 l2w_[] =
		{
			m4x4::TransformRad(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
		};
		m4x4 r2w_[] =
		{
			m4x4::Identity(),
		};

		std::default_random_engine rng;
		for (int i = 0; i != 20; ++i)
		{
			Contact c;
			m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.5f).w1()};
			m4x4 r2w = i < _countof(r2w_) ? r2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.5f).w1()};

			Builder builder;
			builder._<LdrPhysicsShape>("lhs", 0x3000FF00).shape(lhs).o2w(l2w);
			builder._<LdrPhysicsShape>("rhs", 0x30FF0000).shape(rhs).o2w(r2w);
			//builder.Write("collision_unittests.ldr");
			if (BoxVsSphere(lhs, l2w, rhs, r2w, c))
			{
				builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
				builder.Box("pt0", Colour32Yellow).dim(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
				builder.Box("pt1", Colour32Yellow).dim(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
			}
			//builder.Write("collision_unittests.ldr");
		}
	}
}
#endif
