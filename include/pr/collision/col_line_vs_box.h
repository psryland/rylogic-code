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
	// Test for overlap between two shapes, with generic penetration collection.
	//
	// A ShapeLine is a capsule (segment + spherical end caps of radius 'm_radius'). The standard
	// analytical method for capsule-vs-OBB exploits the Minkowski sum structure of a capsule:
	//   distance(capsule, OBB) = distance(segment, OBB) - capsule_radius
	// and the contact normal is the unit vector between the closest points. This handles face,
	// edge and corner contacts uniformly and produces contact points that are always on the OBB
	// surface (i.e. inside the overlap region by construction).
	//
	// Algorithm:
	//   1. Transform line into box space; OBB becomes an axis-aligned box.
	//   2. Iteratively find the closest points on the segment and the AABB:
	//        seg_pt = segment(t),  box_pt = clamp(seg_pt, -hb, hb),
	//        t_new  = clamp((box_pt - mid) . L, -hl, hl), repeat.
	//      Converges in 1-3 iterations for any non-degenerate configuration.
	//   3. If |seg_pt - box_pt| > 0:  depth = capsule_radius - distance, axis = (seg_pt - box_pt)/|...|
	//   4. Otherwise (segment medial axis intersects the box — deep penetration), fall back
	//      to face-axis SAT to pick a usable MTV.
	template <typename Penetration>
	void pr_vectorcall LineVsBox(Shape const& line_, m4x4 const& l2w_, Shape const& box_, m4x4 const& b2w_, Penetration& pen)
	{
		auto& box = shape_cast<ShapeBox>(box_);
		auto& line = shape_cast<ShapeLine>(line_);
		auto b2w = b2w_ * box_.m_s2p;
		auto l2w = l2w_ * line_.m_s2p;

		// Transform 'line' into 'box' space (box becomes an AABB centred at the origin)
		auto l2b = InvertOrthonormal(b2w) * l2w;
		auto mid = l2b.pos;      // line segment mid-point in box space
		auto L   = l2b.z;        // line axis in box space (unit length)
		auto hl  = line.m_hlength;
		auto hb  = box.m_radius;
		auto cap_r = line.m_radius;

		// Initial guess: t that puts the segment point closest to the box centre (origin)
		auto t = Clamp(-Dot(mid, L), -hl, hl);
		auto seg_pt = mid + t * L;
		auto box_pt = Clamp(seg_pt, -hb, +hb).w1();

		// Iterate to convergence (1-3 iterations is typical)
		for (int iter = 0; iter != 8; ++iter)
		{
			auto t_new = Clamp(Dot3(box_pt - mid, L), -hl, hl);
			if (Abs(t_new - t) < math::tiny<float>) break;
			t = t_new;
			seg_pt = mid + t * L;
			box_pt = Clamp(seg_pt, -hb, hb).w1();
		}

		auto sep = seg_pt - box_pt; // from box surface toward segment medial axis (in box space)
		auto dist_sq = LengthSq(sep);
		if (dist_sq >= Sqr(math::tiny<float>))
		{
			// Standard case: medial axis is outside the box. Capsule contact iff distance < radius.
			auto dist = Sqrt(dist_sq);
			auto depth = cap_r - dist;
			pen(depth, [&]{ return b2w * (sep / dist); }, box_.m_material_id, line_.m_material_id);
			return;
		}

		// Deep penetration: segment medial axis intersects the box. Use face-axis SAT for the MTV.
		// (Choose the box face with the smallest projected penetration depth.)
		auto best_depth = limits<float>::max();
		auto best_axis_b = v4::XAxis();
		for (int i = 0; i != 3; ++i)
		{
			// Capsule extent along box axis i: |hl * L[i]| + cap_r
			auto cap_extent = Abs(hl * L[i]) + cap_r;
			auto depth_i = hb[i] + cap_extent - Abs(mid[i]);
			if (depth_i < best_depth)
			{
				best_depth = depth_i;
				best_axis_b = v4::Zero();
				best_axis_b[i] = mid[i] >= 0 ? 1.0f : -1.0f;
			}
		}
		pen(best_depth, [&]{ return b2w * best_axis_b; }, box_.m_material_id, line_.m_material_id);
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
				m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, constants<float>::tau_by_8, constants<float>::tau_by_8), v4(0.2f, 0.3f, 0.1f, 1.0f)),
			};
			m4x4 b2w_[] =
			{
				m4x4::Identity(),
			};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				auto l2w = i < _countof(l2w_) ? l2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);
				auto b2w = i < _countof(b2w_) ? b2w_[i] : m4x4::Random(rng, v4::Origin(), 0.3f);

				Builder builder;
				builder.Group("line", 0x30FF0000).o2w(l2w).Add<LdrCollisionShape>().shape(line);
				builder.Group("box", 0x3000FF00).o2w(b2w).Add<LdrCollisionShape>().shape(box);
				if (LineVsBox(line, l2w, box, b2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style("Direction").line(c.m_point - 0.5f * c.m_depth * c.m_axis, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.002f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.002f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Save(temp_dir() / L"LDraw/collision_unittests.ldr");
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
			auto l2w = m4x4::Transform(RotationRad<m3x3>(0, constants<float>::tau_by_8, 0), v4::Origin());
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

			// Line offset 1.2 in X: box.m_radius.x + line.m_radius = 1 + 0.1 = 1.1 < 1.2
			auto l2w = m4x4::Translation(1.2f, 0, 0);
			PR_EXPECT(!LineVsBox(line, l2w, box, b2w));
		}
	};
}
#endif
