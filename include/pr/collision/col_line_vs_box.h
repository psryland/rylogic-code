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
		auto b2w = b2w_ * box_.m_s2r;
		auto l2w = l2w_ * line_.m_s2r;

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
				best_axis = d <= 0.0f ? n : -n; // Choose sign so the axis points from line toward box
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
		pen(best_depth, [&]{ return b2w * best_axis; }, line_.m_material_id, box_.m_material_id);
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

		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
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
#include "pr/collision/unittest_helpers.h"

namespace pr::collision::tests
{
	PRUnitTestClass(LineVsBoxTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Line through centre of box along Z
		PRUnitTestMethod(LineThroughCentre, Quick)
		{
			auto lhs = ShapeLine{4.0f, 0.0f};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto b2w = m4x4::Translation(1e-5f, 0, 0);

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, b2w, c);
			Visualise(lhs, l2w, rhs, b2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(-0.25f, 0, -0.5f, 1),
					v4(-0.25f, 0, +0.5f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.5f,
			}));
		}

		// Line parallel to box face, outside
		PRUnitTestMethod(LineParallelOutside, Quick)
		{
			auto lhs = ShapeLine{4.0f, 0.0f};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Translation(2, 0, 0);
			auto b2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, b2w, c);
			Visualise(lhs, l2w, rhs, b2w, c);

			PR_EXPECT(!r);
		}

		// Line endpoint inside box
		PRUnitTestMethod(EndpointInsideBox, Quick)
		{
			auto lhs = ShapeLine{1.0f, 0.0f};
			auto rhs = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Identity();
			auto b2w = m4x4::Translation(1e-5f, 0, 0);

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, b2w, c);
			Visualise(lhs, l2w, rhs, b2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(-0.5f, 0, -0.5f, 1),
					v4(-0.5f, 0, +0.5f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 1.0f,
			}));
		}

		// Line at 45° piercing a box face
		PRUnitTestMethod(AngledPiercing, Quick)
		{
			auto lhs = ShapeLine{4.0f, 0.0f};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::TransformDeg(0, 45, 0, v4{0, 0, -1e-5f, 1});
			auto b2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, b2w, c);
			Visualise(lhs, l2w, rhs, b2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 1, 0, 0),
				.m_manifold = {
					v4(-0.5f, -0.25f, -0.5f, 1),
					v4(+0.5f, -0.25f, +0.5f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.5f,
			}));
		}

		// Separated: line well beyond box extents
		PRUnitTestMethod(Separated, Quick)
		{
			auto lhs = ShapeLine{2.0f, 0.0f};
			auto rhs = ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Translation(0, 0, 5); // line at z=[4,6], box at z=[-1,+1]
			auto b2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, b2w, c);
			Visualise(lhs, l2w, rhs, b2w, c);

			PR_EXPECT(!r);
		}

		// Thick line: collision detected when thickness bridges the gap
		PRUnitTestMethod(ThickLineVsBox, Quick)
		{
			auto lhs = ShapeLine{2.0f, 0.6f};
			auto rhs = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Translation(1.2f, 0, 0);
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(-1, 0, 0, 0),
				.m_manifold = {
					v4(0.8f, 0, -1.0f, 1),
					v4(0.8f, 0, +1.0f, 1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.4f,
			}));
		}

		// Thick line: end vs box corner
		PRUnitTestMethod(ThickLineVsBoxCorner, Quick)
		{
			auto lhs = ShapeLine{2.0f, 0.6f};
			auto rhs = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::TransformDeg(0, 45, 0, v4(1, 1.2f, 1.1f, 1));
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, -1, 0, 0),
				.m_manifold = {
					v4(0.292893f,0.8f,0.392893f,1),
					v4(0.9f,0.8f,1,1),
				},
				.m_feature = EFeature::Edge,
				.m_depth = 0.4f,
			}));
		}

		// Thick line: end vs box corner
		PRUnitTestMethod(ThickLineVsBoxCornerAngled, Quick)
		{
			auto lhs = ShapeLine{2.0f, 0.6f};
			auto rhs = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::TransformDeg(45, 45, 45, v4(1, 1.2f, 1.1f, 1));
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(-0.224002f,-0.663298f,-0.714044f,0),
				.m_manifold = {
					v4(0.955655f,0.868688f,0.858642f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.395936102f,
			}));
		}

		// Thick line: just outside the box + thickness envelope
		PRUnitTestMethod(ThickLineSeparated, Quick)
		{
			auto lhs = ShapeLine{2.0f, 0.1f};
			auto rhs = ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Translation(1.2f, 0, 0);
			auto r2w = m4x4::Identity();

			Contact c;
			auto r = LineVsBox(lhs, l2w, rhs, r2w, c);
			Visualise(lhs, l2w, rhs, r2w, c);

			PR_EXPECT(!r);
		}
 
		// Box-vs-Box with s2r transforms 
		PRUnitTestMethod(LineVsBoxWithS2R, Quick)
		{ 
			auto lhs = ShapeLine{2.0f, 0.6f,  m4x4::TransformDeg(45, 30, -25, v4{0.5f, 0, 0, 1}) };
			auto rhs = ShapeBox{v4{2, 2, 2, 0},  m4x4::TransformDeg(30, 10, -80, v4{1.0f, 0, 0, 1}) };
			auto l2w = m4x4::Identity(); 
			auto r2w = m4x4::Identity(); 
 
			Contact c; 
			auto r = LineVsBox(lhs, l2w, rhs, r2w, c); 
			Visualise(lhs, l2w, rhs, r2w, c); 
 
			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.984923f,0.150384f,-0.0855051f,0),
				.m_manifold = {
					v4(0.805754f,-0.714405f,0.616522f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 1.29706311f,
			})); 
		} 
	};
}
#endif
