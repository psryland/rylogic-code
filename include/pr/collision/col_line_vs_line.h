//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Line segment vs Line segment collision detection.
//
// Algorithm:
//  Two line segments in 3D space. The separating axis candidates are:
//   1. The cross product of the two line directions (perpendicular to both).
//   2. If the lines are parallel (cross product ≈ 0), fall back to the
//      perpendicular distance between the parallel lines.
//
//  For thick lines (m_thickness > 0), the effective collision radius is the
//  sum of both thicknesses. Depth = combined_thickness - distance.
//  For zero-thickness lines, a small tolerance is used so near-touching
//  segments register as contact.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between two line segments, with generic penetration collection.
	// Both 'lhs' and 'rhs' are ShapeLine (Line=2 vs Line=2 in the tri-table).
	template <typename Penetration>
	void pr_vectorcall LineVsLine(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& lhs = shape_cast<ShapeLine>(lhs_);
		auto& rhs = shape_cast<ShapeLine>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Line A: from (l2w.pos - lhs.m_radius * l2w.z) to (l2w.pos + lhs.m_radius * l2w.z)
		// Line B: from (r2w.pos - rhs.m_radius * r2w.z) to (r2w.pos + rhs.m_radius * r2w.z)
		auto a0 = l2w.pos - lhs.m_radius * l2w.z;
		auto a1 = l2w.pos + lhs.m_radius * l2w.z;
		auto b0 = r2w.pos - rhs.m_radius * r2w.z;
		auto b1 = r2w.pos + rhs.m_radius * r2w.z;

		// Find the closest points between the two segments
		v4 closest_a, closest_b;
		geometry::closest_point::LineToLine(a0, a1, b0, b1, closest_a, closest_b);

		// The separating "axis" is the vector between the closest points
		auto delta = closest_b - closest_a;
		auto dist_sq = LengthSq(delta);

		// For thick lines, overlap = combined_thickness - distance.
		// For zero-thickness lines, use a small tolerance for numerical near-contact.
		auto constexpr tol = 1e-4f;
		auto dist = Sqrt(dist_sq);
		auto depth = std::max(lhs.m_thickness + rhs.m_thickness, tol) - dist;

		pen(depth, [&]
		{
			// Separating axis: perpendicular to both lines (cross product)
			// Fall back to the delta between closest points, or an arbitrary axis
			if (dist_sq > Sqr(math::tiny<float>))
				return Normalise(delta);

			// Lines intersect or are coincident — use cross product of directions
			auto cross = Cross(l2w.z, r2w.z);
			auto cross_len_sq = LengthSq(cross);
			if (cross_len_sq > Sqr(math::tiny<float>))
				return cross / Sqrt(cross_len_sq);

			// Parallel and coincident — use arbitrary perpendicular to line direction
			return Perpendicular(l2w.z);
		}, lhs_.m_material_id, rhs_.m_material_id);
	}

	// Returns true if the two line segments intersect (within tolerance)
	inline bool pr_vectorcall LineVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		LineVsLine(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the two line segments are in contact, with contact details
	inline bool pr_vectorcall LineVsLine(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		LineVsLine(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeLine>(lhs), l2w, shape_cast<ShapeLine>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTestClass(LineVsLineTests)
	{
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto line_a = ShapeLine{2.0f};
			auto line_b = ShapeLine{2.0f};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = m4x4::Random(rng, v4::Origin(), 0.5f);
				m4x4 r2w = m4x4::Random(rng, v4::Origin(), 0.5f);

				Builder builder;
				{ auto& g = builder.Group("lineA", 0x30FF0000); AddShape(g, line_a); g.o2w(l2w); }
				{ auto& g = builder.Group("lineB", 0x3000FF00); AddShape(g, line_b); g.o2w(r2w); }
				if (LineVsLine(line_a, l2w, line_b, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.005f).pos(c.m_point);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Two crossing lines at the origin: should detect contact
		PRUnitTestMethod(CrossingAtOrigin)
		{
			auto line_a = ShapeLine{2.0f}; // along Z
			auto line_b = ShapeLine{2.0f}; // along Z, rotated to X

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin()); // line B along X

			// Both pass through origin, perpendicular → distance = 0 → contact
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Parallel lines separated: should not detect contact
		PRUnitTestMethod(ParallelSeparated)
		{
			auto line_a = ShapeLine{2.0f};
			auto line_b = ShapeLine{2.0f};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.0f, 0, 0, 0}); // offset in X, parallel along Z

			PR_EXPECT(!LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Skew lines: close but not touching
		PRUnitTestMethod(SkewSeparated)
		{
			auto line_a = ShapeLine{2.0f};
			auto line_b = ShapeLine{2.0f};

			auto l2w = m4x4::Identity();
			// line_b along X, offset 0.5 in Y
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0.5f, 0, 1});

			// Skew lines at distance 0.5 → no contact
			PR_EXPECT(!LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Collinear overlapping lines
		PRUnitTestMethod(CollinearOverlapping)
		{
			auto line_a = ShapeLine{2.0f}; // z: [-1, +1]
			auto line_b = ShapeLine{2.0f}; // z: [-1, +1]

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 0.5f, 0}); // shifted along Z, overlapping

			// Collinear and overlapping → distance = 0 → contact
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w));
		}

		// End-to-end touching: endpoints just meet
		PRUnitTestMethod(EndToEndTouching)
		{
			auto line_a = ShapeLine{2.0f}; // z: [-1, +1]
			auto line_b = ShapeLine{2.0f}; // z: [-1, +1]

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 2.0f, 0}); // B starts where A ends

			// Endpoints meet exactly at z=1 → distance ≈ 0 → contact
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Clearly separated: endpoints don't reach
		PRUnitTestMethod(ClearlySeparated)
		{
			auto line_a = ShapeLine{2.0f};
			auto line_b = ShapeLine{2.0f};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 5.0f, 0}); // gap of 3 units

			PR_EXPECT(!LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Thick lines: perpendicular, separated by less than combined thickness
		PRUnitTestMethod(ThickLinesCrossing)
		{
			auto line_a = ShapeLine{2.0f, 0.4f}; // half-thickness=0.2
			auto line_b = ShapeLine{2.0f, 0.4f}; // half-thickness=0.2

			// line_a along Z, line_b along X, offset 0.3 in Y
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0.3f, 0, 1});

			// Skew distance = 0.3, combined thickness = 0.2 + 0.2 = 0.4 > 0.3 → contact
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w));

			Contact c;
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.1f, 0.01f)); // 0.4 - 0.3 = 0.1
		}

		// Thick lines: separated beyond combined thickness
		PRUnitTestMethod(ThickLinesSeparated)
		{
			auto line_a = ShapeLine{2.0f, 0.2f}; // half-thickness=0.1
			auto line_b = ShapeLine{2.0f, 0.2f}; // half-thickness=0.1

			// Offset 0.3 in Y: combined thickness = 0.2 < 0.3 → no contact
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4{0, 0.3f, 0, 1});

			PR_EXPECT(!LineVsLine(line_a, l2w, line_b, r2w));
		}

		// Zero thickness: backward compatible (crossing lines still touch)
		PRUnitTestMethod(ZeroThicknessBackcompat)
		{
			auto line_a = ShapeLine{2.0f, 0.0f}; // explicit zero
			auto line_b = ShapeLine{2.0f};

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());

			// Crossing at origin → distance = 0 → contact (via tolerance)
			PR_EXPECT(LineVsLine(line_a, l2w, line_b, r2w));
		}
	};
}
#endif
