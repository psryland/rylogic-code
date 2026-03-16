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
		auto l2b = InvertOrthonormal(b2w) * l2w; 
 
		// Line segment mid-point in box space 
		auto mid = l2b.pos; 
 
		// Line segment "radius" plus an epsilon term to counteract arithmetic 
		// errors when segment is (near) parallel to a coordinate axis. 
		auto half = line.m_radius * l2b.z; 
		auto rad = Abs(half) + v4(math::tiny<float>); 
 
		// Try box face normals as separating axes.
		// For thick lines, the collision envelope extends m_thickness perpendicular to the line axis.
		if (!pen(box.m_radius.x + rad.x + line.m_thickness - Abs(mid.x), [&]{ return Sign(mid.x) * b2w.x; }, box_.m_material_id, line_.m_material_id)) return; 
		if (!pen(box.m_radius.y + rad.y + line.m_thickness - Abs(mid.y), [&]{ return Sign(mid.y) * b2w.y; }, box_.m_material_id, line_.m_material_id)) return; 
		if (!pen(box.m_radius.z + rad.z + line.m_thickness - Abs(mid.z), [&]{ return Sign(mid.z) * b2w.z; }, box_.m_material_id, line_.m_material_id)) return; 
 
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
		// For cross-product axes, thickness contributes m_thickness * |sin(angle_between_box_axis_and_line)|.
		// In the unnormalized depth scale, this equals m_thickness * Len(relevant half-vector components).
		ra = rad.z * box.m_radius.y + rad.y * box.m_radius.z + line.m_thickness * Len(half.y, half.z);
		rb = rad.z * Abs(mid.y)     + rad.y * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return line.m_radius * sep_axis(Cross(b2w.x, l2w.z)); }, box_.m_material_id, line_.m_material_id))
			return;

		//' axis = Cross(Yaxis, line) = v4(line.z, 0, -line.x, 0) ('line' in box space)
		ra = rad.z * box.m_radius.x + rad.x * box.m_radius.z + line.m_thickness * Len(half.x, half.z);
		rb = rad.z * Abs(mid.x)     + rad.x * Abs(mid.z);
		if (!pen(ra - rb, [&]{ return line.m_radius * sep_axis(Cross(b2w.y, l2w.z)); }, box_.m_material_id, line_.m_material_id))
			return;

		//' axis = Cross(Zaxis, line) = v4(-line.y, line.x, 0, 0) ('line' in box space)
		ra = rad.y * box.m_radius.x + rad.x * box.m_radius.y + line.m_thickness * Len(half.x, half.y);
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
	PRUnitTestClass(LineVsBoxTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto line = ShapeLine{3.0f};
			auto box = ShapeBox{v4{0.3f, 0.5f, 0.2f, 0.0f}};
			m4x4 l2w_[] =
			{
				m4x4::Transform(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8, v4(0.2f, 0.3f, 0.1f, 1.0f)),
			};
			m4x4 b2w_[] =
			{
				m4x4::Identity(),
			};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);
				m4x4 b2w = i < _countof(b2w_) ? b2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);

				Builder builder;
				{ auto& g = builder.Group("line", 0x30FF0000); AddShape(g, line); g.o2w(l2w); }
				{ auto& g = builder.Group("box", 0x3000FF00); AddShape(g, box); g.o2w(b2w); }
				if (LineVsBox(line, l2w, box, b2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point - 0.5f * c.m_depth * c.m_axis, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.002f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.002f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Line through centre of box along Z
		PRUnitTestMethod(LineThroughCentre)
		{
			auto line = ShapeLine{4.0f}; // half-length = 2, along Z from -2 to +2
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto b2w = m4x4::Identity();

			PR_EXPECT(LineVsBox(line, l2w, box, b2w));
		}

		// Line parallel to box face, outside
		PRUnitTestMethod(LineParallelOutside)
		{
			auto line = ShapeLine{4.0f};
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Translation(v4{2, 0, 0, 0}); // offset in X
			auto b2w = m4x4::Identity();

			PR_EXPECT(!LineVsBox(line, l2w, box, b2w));
		}

		// Line endpoint inside box
		PRUnitTestMethod(EndpointInsideBox)
		{
			auto line = ShapeLine{2.0f}; // half-length = 1
			auto box = ShapeBox{v4{2, 2, 2, 0}};

			// Line from (0,0,-1) to (0,0,1), box extends ±2 → fully inside
			auto l2w = m4x4::Identity();
			auto b2w = m4x4::Identity();

			PR_EXPECT(LineVsBox(line, l2w, box, b2w));
		}

		// Line at 45° piercing a box face
		PRUnitTestMethod(AngledPiercing)
		{
			auto line = ShapeLine{4.0f};
			auto box = ShapeBox{v4{1, 1, 1, 0}};

			// Rotate line 45° about Y so it crosses the box diagonally
			auto l2w = m4x4::Transform(RotationRad<m3x4>(0, constants<float>::tau_by_8, 0), v4::Origin());
			auto b2w = m4x4::Identity();

			PR_EXPECT(LineVsBox(line, l2w, box, b2w));
		}

		// Separated: line well beyond box extents
		PRUnitTestMethod(Separated)
		{
			auto line = ShapeLine{2.0f};
			auto box = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Translation(v4{0, 0, 5, 0}); // line at z=[4,6], box at z=[-1,+1]
			auto b2w = m4x4::Identity();

			PR_EXPECT(!LineVsBox(line, l2w, box, b2w));
		}

		// Thick line: collision detected when thickness bridges the gap
		PRUnitTestMethod(ThickLineVsBox)
		{
			auto line = ShapeLine{2.0f, 0.6f}; // half-length=1, half-thickness=0.3
			auto box = ShapeBox{v4{2, 2, 2, 0}}; // half-extent=1
			auto b2w = m4x4::Identity();

			// Line offset 1.2 in X: zero-thickness line misses (1 < 1.2),
			// but thick line should hit (1 + 0.3 = 1.3 > 1.2)
			auto l2w = m4x4::Translation(v4{1.2f, 0, 0, 0});
			PR_EXPECT(LineVsBox(line, l2w, box, b2w));

			Contact c;
			PR_EXPECT(LineVsBox(line, l2w, box, b2w, c));
			PR_EXPECT(c.m_depth > 0.0f);
		}

		// Thick line: just outside the box + thickness envelope
		PRUnitTestMethod(ThickLineSeparated)
		{
			auto line = ShapeLine{2.0f, 0.2f}; // half-thickness=0.1
			auto box = ShapeBox{v4{2, 2, 2, 0}}; // half-extent=1
			auto b2w = m4x4::Identity();

			// Line offset 1.2 in X: box.m_radius.x + line.m_thickness = 1 + 0.1 = 1.1 < 1.2
			auto l2w = m4x4::Translation(v4{1.2f, 0, 0, 0});
			PR_EXPECT(!LineVsBox(line, l2w, box, b2w));
		}
	};
}
#endif