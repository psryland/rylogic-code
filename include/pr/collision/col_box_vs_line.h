//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/maths/maths.h"
#include "pr/geometry/closest_point.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"

namespace pr::collision
{
	// Test for overlap between two shapes, with generic penetration collection
	template <typename Penetration>
	void pr_vectorcall BoxVsLine(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapeBox>(lhs_);
		auto& rhs = shape_cast<ShapeLine>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Compute a transform for 'rhs' in 'lhs's frame
		auto r2l = InvertAffine(l2w) * r2w;

		// Line segment mid-point in box space
		auto mid = r2l.pos;

		// Line segment "radius" plus an epsilon term to counteract arithmetic
		// errors when segment is (near) parallel to a coordinate axis.
		auto half = rhs.m_radius * r2l.z;
		auto rad = Abs(half) + v4(maths::tinyf);

		// Try world coordinate axes as separating axes
		if (!pen(lhs.m_radius.x + rad.x - Abs(mid.x), [&]{ return Sign(mid.x) * l2w.x; }, lhs_.m_material_id, rhs_.m_material_id)) return;
		if (!pen(lhs.m_radius.y + rad.y - Abs(mid.y), [&]{ return Sign(mid.y) * l2w.y; }, lhs_.m_material_id, rhs_.m_material_id)) return;
		if (!pen(lhs.m_radius.z + rad.z - Abs(mid.z), [&]{ return Sign(mid.z) * l2w.z; }, lhs_.m_material_id, rhs_.m_material_id)) return;

		// Lambda for returning a separating axis with the correct sign
		auto sep_axis = [&](v4 sa) { return Sign(Dot(r2l.pos, sa)) * sa; };
		float ra,rb;

		// Try cross products of the segment direction with the coordinate axes.
		// Example for XAxis x LineSegment:
		//' axis = Cross(Xaxis, line) = v4(0, -line.z, line.y, 0) ('line' in box space)
		//' ra = Dot(axis, box.radius) = unsigned radius of the box in the direction of 'axis'
		//'    =  axis.y * box.radius.y + axis.z * box.radius.z;
		//'    = -line.z * box.radius.y + line.y * box.radius.z;
		//' rb = Dot(axis, mid) = distance to the line in the direction of 'axis' (note: line is perpendicular to axis)
		//'    =  axis.y * mid.y + axis.z * mid.z;
		//'    = -line.z * mid.y + line.y * mid.z;
		//'Flip 'mid' and 'axis' into the positive octant. The length of 'line'
		// doesn't matter so long as the length of the separating axis is scaled
		// by the same amount, so we can use 'rad == abs(line/2)':
		//'  ra = rad.z * box.radius.y + rad.y * box.radius.z;
		//'  rb = rad.z * abs(mid.y)    + rad.y * abs(mid.z);
		//' depth = ra - rb
		ra = rad.z * lhs.m_radius.y + rad.y * lhs.m_radius.z;
		rb = rad.z * Abs(mid.y)     + rad.y * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return rhs.m_radius * sep_axis(Cross(l2w.x, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		//' axis = Cross(Yaxis, line) = v4(line.z, 0, -line.x, 0) ('line' in box space)
		ra = rad.z * lhs.m_radius.x + rad.x * lhs.m_radius.z;
		rb = rad.z * Abs(mid.x)     + rad.x * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return rhs.m_radius * sep_axis(Cross(l2w.y, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;

		//' axis = Cross(Zaxis, line) = v4(-line.y, line.x, 0, 0) ('line' in box space)
		ra = rad.y * lhs.m_radius.x + rad.x * lhs.m_radius.y;
		rb = rad.y * Abs(mid.x)     + rad.x * Abs(mid.y);
		if (!pen(ra - rb, [&]{ return rhs.m_radius * sep_axis(Cross(l2w.z, r2w.z)); }, lhs_.m_material_id, rhs_.m_material_id))
			return;
	}

	// Returns true if the sphere  'lhs' intersects the orientated box 'rhs'
	inline bool pr_vectorcall BoxVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		BoxVsLine(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if 'lhs' and 'rhs' are intersecting.
	inline bool pr_vectorcall BoxVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		BoxVsLine(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		// Determine the sign of the separating axis to make it the normal from 'lhs' to 'rhs'
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, l2w * lhs.m_s2p.pos);
		auto p1 = Dot3(sep_axis, r2w * rhs.m_s2p.pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis  = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeBox>(lhs), l2w, shape_cast<ShapeLine>(rhs), r2w, contact.m_axis, contact.m_depth);
		contact.m_mat_idA = p.m_mat_idA;
		contact.m_mat_idB = p.m_mat_idB;
		return true;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/ldraw.h"

namespace pr::collision
{
	PRUnitTest(CollisionBoxVsLine)
	{
		using namespace pr::rdr12::ldraw;

		auto lhs = ShapeBox{v4{0.3f, 0.5f, 0.2f, 0.0f}};
		auto rhs = ShapeLine{3.0f};
		m4x4 l2w_[] =
		{
			m4x4::Identity(),
		};
		m4x4 r2w_[] =
		{
			m4x4::Transform(maths::tau_by_8f, maths::tau_by_8f, maths::tau_by_8f, v4(0.2f, 0.3f, 0.1f, 1.0f)),
		};

		std::default_random_engine rng;
		for (int i = 0; i != 20; ++i)
		{
			Contact c;
			m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);
			m4x4 r2w = i < _countof(r2w_) ? r2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);

			Builder builder;
			builder._<LdrPhysicsShape>("lhs", 0x30FF0000).shape(lhs).o2w(l2w);
			builder._<LdrPhysicsShape>("rhs", 0x3000FF00).shape(rhs).o2w(r2w);
			//bilder.Write(L"collision_unittests.ldr");
			if (!BoxVsLine(lhs, l2w, rhs, r2w, c))
				continue;

			builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point - 0.5f * c.m_depth * c.m_axis, c.m_axis);
			builder.Box("pt0", Colour32Yellow).dim(0.002f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
			builder.Box("pt1", Colour32Yellow).dim(0.002f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
			//builder.Write(L"collision_unittests.ldr");
		}
	}
}
#endif