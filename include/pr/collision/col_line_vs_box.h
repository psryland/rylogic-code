//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between two shapes, with generic penetration collection
	template <typename Penetration>
	void pr_vectorcall LineVsBox(Shape const& line_, m4x4 const& l2w_, Shape const& box_, m4x4 const& b2w_, Penetration& pen)
	{ 
		auto& box = shape_cast<ShapeBox>(box_); 
		auto& line = shape_cast<ShapeLine>(line_); 
		auto b2w = b2w_ * box_.m_s2p; 
		auto l2w = l2w_ * line_.m_s2p; 
 
		// Compute a transform for 'line' in 'box's frame 
		auto l2b = InvertAffine(b2w) * l2w; 
 
		// Line segment mid-point in box space 
		auto mid = l2b.pos; 
 
		// Line segment "radius" plus an epsilon term to counteract arithmetic 
		// errors when segment is (near) parallel to a coordinate axis. 
		auto half = line.m_radius * l2b.z; 
		auto rad = Abs(half) + v4(math::tiny<float>); 
 
		// Try world coordinate axes as separating axes 
		if (!pen(box.m_radius.x + rad.x - Abs(mid.x), [&]{ return Sign(mid.x) * b2w.x; }, box_.m_material_id, line_.m_material_id)) return; 
		if (!pen(box.m_radius.y + rad.y - Abs(mid.y), [&]{ return Sign(mid.y) * b2w.y; }, box_.m_material_id, line_.m_material_id)) return; 
		if (!pen(box.m_radius.z + rad.z - Abs(mid.z), [&]{ return Sign(mid.z) * b2w.z; }, box_.m_material_id, line_.m_material_id)) return; 
 
		// Lambda for returning a separating axis with the correct sign
		auto sep_axis = [&](v4 sa) { return Sign(Dot(l2b.pos, sa)) * sa; };
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
		ra = rad.z * box.m_radius.y + rad.y * box.m_radius.z;
		rb = rad.z * Abs(mid.y)     + rad.y * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return line.m_radius * sep_axis(Cross(b2w.x, l2w.z)); }, box_.m_material_id, line_.m_material_id))
			return;

		//' axis = Cross(Yaxis, line) = v4(line.z, 0, -line.x, 0) ('line' in box space)
		ra = rad.z * box.m_radius.x + rad.x * box.m_radius.z;
		rb = rad.z * Abs(mid.x)     + rad.x * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return line.m_radius * sep_axis(Cross(b2w.y, l2w.z)); }, box_.m_material_id, line_.m_material_id))
			return;

		//' axis = Cross(Zaxis, line) = v4(-line.y, line.x, 0, 0) ('line' in box space)
		ra = rad.y * box.m_radius.x + rad.x * box.m_radius.y;
		rb = rad.y * Abs(mid.x)     + rad.x * Abs(mid.y);
		if (!pen(ra - rb, [&]{ return line.m_radius * sep_axis(Cross(b2w.z, l2w.z)); }, box_.m_material_id, line_.m_material_id))
			return;
	}

	// Returns true if the line intersects the orientated box
	inline bool pr_vectorcall LineVsBox(Shape const& line, m4x4 const& l2w, Shape const& box, m4x4 const& b2w)
	{
		TestPenetration p;
		LineVsBox(line, l2w, box, b2w, p);
		return p.Contact();
	}

	// Returns true if 'line' and 'box' are intersecting.
	inline bool pr_vectorcall LineVsBox(Shape const& line, m4x4 const& l2w, Shape const& box, m4x4 const& b2w, Contact& contact)
	{
		ContactPenetration p;
		LineVsBox(line, l2w, box, b2w, p);
		if (!p.Contact())
			return false;

		// Determine the sign of the separating axis to make it the normal from 'line' to 'box'
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, l2w * line.m_s2p.pos);
		auto p1 = Dot3(sep_axis, b2w * box.m_s2p.pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis  = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeLine>(line), l2w, shape_cast<ShapeBox>(box), b2w, contact.m_axis, contact.m_depth);
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
	PRUnitTest(CollisionBoxVsLine)
	{
		using namespace pr::rdr12::ldraw;

		auto line = ShapeLine{3.0f};
		auto box = ShapeBox{v4{0.3f, 0.5f, 0.2f, 0.0f}};
		m4x4 l2w_[] =
		{
			m4x4::TransformRad(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
		};
		m4x4 b2w_[] =
		{
			m4x4::Identity(),
		};

		std::default_random_engine rng;
		for (int i = 0; i != 20; ++i)
		{
			Contact c;
			m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.3f).w1()};
			m4x4 b2w = i < _countof(b2w_) ? b2w_[i] : m4x4{Random<m3x4>(rng), Random<v4>(rng, v4::Origin(), 0.3f).w1()};

			Builder builder;
			builder._<LdrPhysicsShape>("line", 0x30FF0000).shape(line).o2w(l2w);
			builder._<LdrPhysicsShape>("box", 0x3000FF00).shape(box).o2w(b2w);
			//bilder.Write(L"collision_unittests.ldr");
			if (!LineVsBox(line, l2w, box, b2w, c))
				continue;

			builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point - 0.5f * c.m_depth * c.m_axis, c.m_axis);
			builder.Box("pt0", Colour32Yellow).dim(0.002f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
			builder.Box("pt1", Colour32Yellow).dim(0.002f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
			//builder.Write(L"collision_unittests.ldr");
		}
	}
}
#endif