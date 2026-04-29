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
	// Test for overlap between a capsule (ShapeLine) and an OBB (ShapeBox) using SAT.
	//
	// A ShapeLine is a capsule: a segment of half-length 'line_hlen' plus spherical end caps of radius 'line_radius'.
	// For each candidate separating axis 'n' (unit, in box space):
	//   box projection radius:      rb = |n.x|*box_radii.x + |n.y|*box_radii.y + |n.z|*box_radii.z
	//   segment projection radius:  rs = |n.line_axis| * line_hlen
	//   signed centre separation:   d  = n . mid     (box centre is at origin)
	//   penetration depth:          rb + rs + line_radius - |d|
	// If any axis gives depth <= 0 the shapes are separated. Otherwise the axis with the smallest
	// depth is the MTV.
	//
	// Candidate axes (7 total):
	//   - 3 box face normals           (face contacts)
	//   - 3 box_axis x line_axis       (edge-edge contacts)
	//   - 1 closest-corner axis        (corner contacts; the other 6 can't produce this direction)
	template <typename Penetration>
	void pr_vectorcall LineVsBox(Shape const& line_, m4x4 const& l2w_, Shape const& box_, m4x4 const& b2w_, Penetration& pen)
	{
		auto& box = shape_cast<ShapeBox>(box_);
		auto& line = shape_cast<ShapeLine>(line_);
		auto b2w = b2w_ * box_.m_s2p;
		auto l2w = l2w_ * line_.m_s2p;

		// Work in box space: box is an AABB centred at the origin, line centre at 'mid' with unit direction 'line_axis'.
		auto l2b         = InvertOrthonormal(b2w) * l2w;
		auto mid         = l2b.pos.w0();
		auto line_axis   = l2b.z;
		auto box_radii   = box.m_radius;
		auto line_hlen   = line.m_hlength;
		auto line_radius = line.m_radius;

		auto best_depth = limits<float>::max();
		auto best_axis  = v4::XAxis();

		// Test one unit-length axis 'n' and track the minimum penetration depth (negative = separated).
		auto Test = [&](v4 const& n)
		{
			auto rb    = Dot3(Abs(n), box_radii);
			auto rs    = Abs(Dot3(n, line_axis)) * line_hlen;
			auto d     = Dot3(n, mid);
			auto depth = rb + rs + line_radius - Abs(d);
			if (depth < best_depth)
			{
				best_depth = depth;
				best_axis = n;// d >= 0 ? n : -n; // Choose sign so the axis points from box toward line
			}
		};

		// 3 box face normals
		Test(v4::XAxis());
		Test(v4::YAxis());
		Test(v4::ZAxis());

		// 3 cross products of line axis with box axes (edge-edge)
		for (int i = 0; i != 3; ++i)
		{
			auto e = v4(float(i == 0), float(i == 1), float(i == 2), 0.0f);
			auto n = Cross(line_axis, e);
			auto len_sq = LengthSq(n);

			// Skip if the line is parallel to this box axis — the face axis already covers this case
			if (len_sq <= Sqr(math::tiny<float>)) continue;
			Test(n / Sqrt(len_sq));
		}

		// Test an axis from the closest box corner to the line.
		{
			auto seg_pt = Clamp(-Dot3(mid, line_axis), -line_hlen, line_hlen) * line_axis + mid;
			auto box_pt = v4(Sign(seg_pt.x, false), Sign(seg_pt.y, false), Sign(seg_pt.z, false), 0.0f) * box_radii;
			seg_pt = Clamp(Dot3(box_pt - mid, line_axis), -line_hlen, line_hlen) * line_axis + mid;
			box_pt = Clamp(seg_pt, -box_radii, box_radii); // This extra refinement handles line ends near box corners
			auto sep = seg_pt - box_pt;
			if (auto len_sq = LengthSq(sep); len_sq > Sqr(math::tiny<float>))
				Test(sep/ Sqrt(len_sq));
		}

		// Report the minimum-penetration axis (depth is negative if the shapes are separated).
		pen(best_depth, [&]{ return b2w * best_axis; }, box_.m_material_id, line_.m_material_id);
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
		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, l2w * line.m_s2p.pos);
		auto p1 = Dot3(sep_axis, b2w * box.m_s2p.pos);
		sep_axis = Bool2SignF(p0 < p1) * sep_axis;

		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeLine>(line), l2w, shape_cast<ShapeBox>(box), b2w, sep_axis, depth);

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
#include "pr/collision/ldraw.h"

namespace pr::collision::tests
{
	PRUnitTestClass(LineVsBoxTests)
	{
		inline static constexpr bool CreateVisualizations = false;

		PRUnitTestMethod(Visualise)
		{
			#if PR_UNITTESTS_VISUALISE
			if constexpr (CreateVisualizations)
			{
				using namespace pr::ldraw;
				auto line = ShapeLine{ 3.0f };
				auto box = ShapeBox{ v4{0.3f, 0.5f, 0.2f, 0.0f} };
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
						builder.Add<LdrCollisionContact>().contact(c);

					builder.Save(temp_dir() / L"LDraw/collision_unittests.ldr");
				}
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
			auto line = ShapeLine{2.0f, 0.1f};
			auto box = ShapeBox{v4{2, 2, 2, 0}};
			auto b2w = m4x4::Identity();

			// Line offset 1.2 in X: box.m_radius.x + line.m_radius = 1 + 0.1 = 1.1 < 1.2
			auto l2w = m4x4::Translation(1.2f, 0, 0);
			PR_EXPECT(!LineVsBox(line, l2w, box, b2w));
		}
	};
}
#endif
