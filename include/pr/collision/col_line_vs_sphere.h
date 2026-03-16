//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Line segment vs Sphere collision detection.
//
// Algorithm:
//  Find the closest point on the line segment to the sphere centre.
//  The separating axis is the vector from that closest point to the sphere centre.
//  Penetration depth = (line.m_thickness + sphere_radius) - distance.
//
// When the line has non-zero thickness, it behaves as a capsule for collision
// depth calculation (the cylindrical envelope extends m_thickness from the axis).
// The support function uses hemispherical end-caps for accurate contact points.
//
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/penetration.h"
#include "pr/collision/support.h"

namespace pr::collision
{
	// Test for overlap between a line segment and a sphere, with generic penetration collection.
	// 'lhs' is the line, 'rhs' is the sphere (matching tri-table order: Line=2, Sphere=0).
	template <typename Penetration>
	void pr_vectorcall LineVsSphere(Shape const& lhs_, m4x4 const& l2w_, Shape const& rhs_, m4x4 const& r2w_, Penetration& pen)
	{
		auto& line = shape_cast<ShapeLine>(lhs_);
		auto& sph = shape_cast<ShapeSphere>(rhs_);
		auto l2w = l2w_ * lhs_.m_s2p;
		auto r2w = r2w_ * rhs_.m_s2p;

		// Work in line space: the line segment runs from (0,0,-R) to (0,0,+R) along Z.
		auto s2l = InvertOrthonormal(l2w) * r2w.pos - v4::Origin();

		// Clamp the sphere centre's Z-coordinate to the line segment extent.
		// This gives the closest point on the segment to the sphere centre.
		auto t = Clamp(s2l.z, -line.m_radius, +line.m_radius);
		auto closest_on_line = v4(0, 0, t, 1);

		// Vector from closest point on line to sphere centre (in line space)
		auto delta = s2l - v4(0, 0, t, 0);
		auto dist_sq = LengthSq(delta);

		// Penetration depth: positive means overlap.
		// For thick lines, the collision envelope extends m_thickness from the line axis.
		auto dist = Sqrt(dist_sq + math::tiny<float>);
		auto depth = (line.m_thickness + sph.m_radius) - dist;

		pen(depth, [&]
		{
			// Separating axis: from line toward sphere centre (in world space).
			// If the sphere centre lies exactly on the line, use an arbitrary perpendicular.
			if (dist_sq > Sqr(math::tiny<float>))
				return l2w * (delta / dist);
			else
				return l2w.x; // arbitrary perpendicular to line's Z-axis
		}, lhs_.m_material_id, rhs_.m_material_id);
	}

	// Returns true if the line segment intersects the sphere
	inline bool pr_vectorcall LineVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w)
	{
		TestPenetration p;
		LineVsSphere(lhs, l2w, rhs, r2w, p);
		return p.Contact();
	}

	// Returns true if the line and sphere are intersecting, with contact details
	inline bool pr_vectorcall LineVsSphere(Shape const& lhs, m4x4 const& l2w, Shape const& rhs, m4x4 const& r2w, Contact& contact)
	{
		ContactPenetration p;
		LineVsSphere(lhs, l2w, rhs, r2w, p);
		if (!p.Contact())
			return false;

		// Determine the sign of the separating axis to make it the normal from 'lhs' to 'rhs'
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		auto sign = Bool2SignF(p0 < p1);

		contact.m_depth = p.Depth();
		contact.m_axis = sign * sep_axis;
		contact.m_point = FindContactPoint(shape_cast<ShapeLine>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, contact.m_axis, contact.m_depth);
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
	PRUnitTestClass(LineVsSphereTests)
	{
		// Visual debugging: run with PR_UNITTESTS_VISUALISE defined to write LDraw output
		PRUnitTestMethod(Visualise)
		{
			using namespace pr::ldraw;

			#if PR_UNITTESTS_VISUALISE
			auto line = ShapeLine{2.0f};    // length=2, half-length=1
			auto sph = ShapeSphere{0.5f};
			m4x4 l2w_[] =
			{
				m4x4::Identity(),
				m4x4::Transform(0, constants<float>::tau_by_8, 0, v4::Origin()),
			};
			m4x4 r2w_[] =
			{
				m4x4::Translation(v4{0.3f, 0.0f, 0.0f, 0}),
				m4x4::Translation(v4{0.5f, 0.3f, 0.5f, 0}),
			};

			std::default_random_engine rng;
			for (int i = 0; i != 20; ++i)
			{
				Contact c;
				m4x4 l2w = i < _countof(l2w_) ? l2w_[i] : m4x4::Random(rng, v4::Origin(), 1.0f);
				m4x4 r2w = i < _countof(r2w_) ? r2w_[i] : m4x4::Random(rng, v4::Origin(), 1.0f);

				Builder builder;
				{ auto& g = builder.Group("line", 0x30FF0000); AddShape(g, line); g.o2w(l2w); }
				{ auto& g = builder.Group("sph", 0x3000FF00); AddShape(g, sph); g.o2w(r2w); }
				if (LineVsSphere(line, l2w, sph, r2w, c))
				{
					builder.Line("sep_axis", Colour32Yellow).style(ELineStyle::Direction).line(c.m_point, c.m_axis);
					builder.Box("pt0", Colour32Yellow).box(0.01f).pos(c.m_point - 0.5f * c.m_depth * c.m_axis);
					builder.Box("pt1", Colour32Yellow).box(0.01f).pos(c.m_point + 0.5f * c.m_depth * c.m_axis);
				}
				builder.Write(L"collision_unittests.ldr");
			}
			#endif
		}

		// Sphere centred on the line midpoint: maximum penetration
		PRUnitTestMethod(SphereCentredOnLine)
		{
			auto line = ShapeLine{2.0f}; // half-length = 1
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity(); // sphere at origin = line midpoint

			// Sphere centre is on the line, so distance = 0, depth = radius = 0.5
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w));

			Contact c;
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w, c));
			PR_EXPECT(c.m_depth > 0.0f);
		}

		// Sphere near the end of the line segment
		PRUnitTestMethod(SphereNearEndpoint)
		{
			auto line = ShapeLine{2.0f}; // half-length = 1, runs from z=-1 to z=+1
			auto sph = ShapeSphere{0.3f};

			// Place sphere just beyond the +Z end of the line
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0.2f, 0, 1.2f, 0});

			// Distance from endpoint (0,0,1) to sphere centre (0.2,0,1.2) = sqrt(0.04+0.04) ≈ 0.283
			// Depth = 0.3 - 0.283 ≈ 0.017 → should be touching
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w));
		}

		// Sphere clearly separated from line
		PRUnitTestMethod(Separated)
		{
			auto line = ShapeLine{2.0f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{2.0f, 0, 0, 0}); // far away laterally

			PR_EXPECT(!LineVsSphere(line, l2w, sph, r2w));
		}

		// Sphere beyond the line endpoint (closest point is the endpoint)
		PRUnitTestMethod(SphereBeyondEndpoint)
		{
			auto line = ShapeLine{2.0f}; // half-length = 1
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0, 0, 3.0f, 0}); // well past +Z end

			// Distance from endpoint (0,0,1) to (0,0,3) = 2.0, depth = 0.5 - 2.0 = -1.5
			PR_EXPECT(!LineVsSphere(line, l2w, sph, r2w));
		}

		// Degenerate: zero-length line (point vs sphere)
		PRUnitTestMethod(ZeroLengthLine)
		{
			auto line = ShapeLine{0.0f}; // degenerate point
			auto sph = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0.5f, 0, 0, 0});

			// Distance = 0.5, depth = 1.0 - 0.5 = 0.5
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w));

			Contact c;
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.5f, 0.01f));
		}

		// Rotated line: line along X-axis via rotation
		PRUnitTestMethod(RotatedLine)
		{
			auto line = ShapeLine{4.0f}; // half-length = 2
			auto sph = ShapeSphere{0.5f};

			// Rotate line so its Z-axis maps to the X-axis
			auto l2w = m4x4::Transform(v4::XAxis(), v4::ZAxis(), v4::Origin());
			auto r2w = m4x4::Translation(v4{1.0f, 0.3f, 0, 0});

			// Line now runs along X from -2 to +2. Sphere at (1, 0.3, 0).
			// Closest point on line = (1, 0, 0). Distance = 0.3. Depth = 0.5-0.3 = 0.2.
			Contact c;
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.2f, 0.01f));
		}

		// Contact axis direction: should point from line toward sphere
		PRUnitTestMethod(ContactAxisDirection)
		{
			auto line = ShapeLine{2.0f};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0.3f, 0, 0, 0}); // sphere offset in +X

			Contact c;
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w, c));

			// Axis should point from line to sphere, i.e., in +X direction
			PR_EXPECT(c.m_axis.x > 0.0f);
		}

		// Thick line: sphere within thickness envelope but beyond zero-thickness range
		PRUnitTestMethod(ThickLineVsSphere)
		{
			auto line = ShapeLine{2.0f, 0.4f}; // half-length=1, half-thickness=0.2
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();

			// Place sphere 0.4 away laterally: zero-thickness line misses (0.3 < 0.4),
			// but thick line should hit (0.2 + 0.3 = 0.5 > 0.4)
			auto r2w = m4x4::Translation(v4{0.4f, 0, 0, 0});
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w));

			Contact c;
			PR_EXPECT(LineVsSphere(line, l2w, sph, r2w, c));
			PR_EXPECT(FEqlRelative(c.m_depth, 0.1f, 0.01f)); // (0.2 + 0.3) - 0.4 = 0.1
		}

		// Thick line: sphere just outside thickness envelope
		PRUnitTestMethod(ThickLineSeparated)
		{
			auto line = ShapeLine{2.0f, 0.2f}; // half-thickness=0.1
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();

			// Distance 0.5 laterally: 0.1 + 0.3 = 0.4 < 0.5, should not collide
			auto r2w = m4x4::Translation(v4{0.5f, 0, 0, 0});
			PR_EXPECT(!LineVsSphere(line, l2w, sph, r2w));
		}

		// Zero thickness: backwards compatible with original behaviour
		PRUnitTestMethod(ZeroThicknessBackcompat)
		{
			auto line_thick = ShapeLine{2.0f, 0.0f}; // explicit zero thickness
			auto line_default = ShapeLine{2.0f};       // default (no thickness)
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{0.3f, 0, 0, 0});

			Contact c1, c2;
			PR_EXPECT(LineVsSphere(line_thick, l2w, sph, r2w, c1));
			PR_EXPECT(LineVsSphere(line_default, l2w, sph, r2w, c2));
			PR_EXPECT(FEqlRelative(c1.m_depth, c2.m_depth, 0.001f));
		}
	};
}
#endif
