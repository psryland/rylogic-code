//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Line segment vs Sphere collision detection.
//
// Algorithm:
//  Find the closest point on the line segment to the sphere centre.
//  The separating axis is the vector from that closest point to the sphere centre.
//  Penetration depth = (line.m_radius + sphere_radius) - distance.
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
		auto t = Clamp(s2l.z, -line.m_hlength, +line.m_hlength);
		auto closest_on_line = v4(0, 0, t, 1);

		// Vector from closest point on line to sphere centre (in line space)
		auto delta = s2l - v4(0, 0, t, 0);
		auto dist_sq = LengthSq(delta);

		// Penetration depth: positive means overlap.
		// For thick lines, the collision envelope extends m_thickness from the line axis.
		auto dist = Sqrt(dist_sq);
		auto depth = (line.m_radius + sph.m_radius) - dist;

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
		auto depth = p.Depth();
		auto sep_axis = p.SeparatingAxis();
		auto p0 = Dot3(sep_axis, (l2w * lhs.m_s2p).pos);
		auto p1 = Dot3(sep_axis, (r2w * rhs.m_s2p).pos);
		sep_axis = Bool2SignF(p0 < p1) * sep_axis;

		auto [manifold, feature] = FindContactManifold(shape_cast<ShapeLine>(lhs), l2w, shape_cast<ShapeSphere>(rhs), r2w, sep_axis, depth);

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
	PRUnitTestClass(LineVsSphereTests)
	{
		inline static constexpr bool CreateVisuals = false;

		// Draw the scene
		void Visualise(collision::Shape const& a, m4x4 a2w, collision::Shape const& b, m4x4 b2w, collision::Contact const& c)
		{
			if constexpr (CreateVisuals)
				VisualiseCollision(temp_dir() / L"LDraw/collision.ldr", a, a2w, b, b2w, c);
		}

		// Sphere centred on the line midpoint: maximum penetration
		PRUnitTestMethod(SphereCentredOnLine)
		{
			auto line = ShapeLine{2.0f};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1e-5f, 0, 0); // sphere at origin = line midpoint

			// Sphere centre is on the line, so distance = 0, depth = radius = 0.5
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(-0.25f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.5f,
			}));
		}

		// Sphere near the end of the line segment
		PRUnitTestMethod(SphereVsLineEndpoint)
		{
			auto line = ShapeLine{2.0f, 0.0f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.2f, 0, 1.2f);

			// Distance from endpoint (0,0,1) to sphere centre (0.2,0,1.2) = sqrt(0.04+0.04) ≈ 0.283
			// Depth = 0.3 - 0.283 ≈ 0.017 → should be touching
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.707106709f, 0, 0.707106888f, 0),
				.m_manifold = {
					v4(-0.0060660094f, 0, 0.993933976f, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.0171572566f,
			}));
		}

		// Sphere clearly separated from line
		PRUnitTestMethod(Separated)
		{
			auto line = ShapeLine{1.0f, 0.3f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.6f, 0.4f, 0.5f); // far away laterally

			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(!r);
		}

		// Sphere beyond the line endpoint (closest point is the endpoint)
		PRUnitTestMethod(SphereBeyondEndpoint)
		{
			auto line = ShapeLine{2.0f, 0.0f};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, 1.55f); // well past +Z end

			// Distance from endpoint (0,0,1) to (0,0,3) = 2.0, depth = 0.5 - 2.0 = -1.5
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(!r);
		}

		// Degenerate: zero-length line (point vs sphere)
		PRUnitTestMethod(ZeroLengthLine)
		{
			auto line = ShapeLine{0.0f, 0.0f}; // degenerate point
			auto sph = ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.5f, 0, 0);

			// Distance = 0.5, depth = 1.0 - 0.5 = 0.5
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(-0.25f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.5f,
			}));
		}

		// Rotated line: line along X-axis via rotation
		PRUnitTestMethod(RotatedLine)
		{
			auto line = ShapeLine{4.0f}; // half-length = 2
			auto sph = ShapeSphere{0.5f};

			// Rotate line so its Z-axis maps to the X-axis
			auto l2w = m4x4::TransformDeg(0, 90, 0, v4::Origin());
			auto r2w = m4x4::Translation(1.0f, 0.3f, 0);

			// Line now runs along X from -2 to +2. Sphere at (1, 0.3, 0).
			// Closest point on line = (1, 0, 0). Distance = 0.3. Depth = 0.5-0.3 = 0.2.
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0, 1, 0, 0),
				.m_manifold = {
					v4(1, -0.1f, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.2f,
			}));
		}

		// Contact axis direction: should point from line toward sphere
		PRUnitTestMethod(ContactAxisDirection)
		{
			auto line = ShapeLine{2.0f};
			auto sph = ShapeSphere{0.5f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.3f, 0, 0); // sphere offset in +X

			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(-0.1f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.2f,
			}));
		}

		// Thick line: sphere within thickness envelope but beyond zero-thickness range
		PRUnitTestMethod(ThickLineVsSphere)
		{
			auto line = ShapeLine{2.0f, 0.2f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto s2w = m4x4::Translation(0.4f, 0, 0);

			// Place sphere 0.4 away laterally: zero-thickness line misses (0.3 < 0.4),
			// but thick line should hit (0.2 + 0.3 = 0.5 > 0.4)
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, s2w, c);
			Visualise(line, l2w, sph, s2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(1, 0, 0, 0),
				.m_manifold = {
					v4(0.15f, 0, 0, 1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.1f,
			}));
		}

		// Thick line: sphere just outside thickness envelope
		PRUnitTestMethod(ThickLineSeparated)
		{
			auto line = ShapeLine{2.0f, 0.2f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.5001f, 0, 0);

			// Distance 0.5 laterally: 0.1 + 0.3 = 0.4 < 0.5, should not collide
			Contact c;
			auto r = LineVsSphere(line, l2w, sph, r2w, c);
			Visualise(line, l2w, sph, r2w, c);

			PR_EXPECT(!r);
		}

		// Thick line: sphere contacting end
		PRUnitTestMethod(ThickLineEndVsSphere)
		{
			auto line = ShapeLine{1.0f, 0.2f};
			auto sph = ShapeSphere{0.3f};
			auto l2w = m4x4::Identity();
			auto s2w = m4x4::Translation(0.05f, 0.05f, 0.7f);

			Contact c;
			auto r = LineVsSphere(line, l2w, sph, s2w, c);
			Visualise(line, l2w, sph, s2w, c);

			PR_EXPECT(r);
			PR_EXPECT(CheckContact(c, Contact{
				.m_axis = v4(0.235702f,0.235702f,0.942809f,0),
				.m_manifold = {
					v4(0.0132149f,0.0132149f,0.55286f,1),
				},
				.m_feature = EFeature::Vert,
				.m_depth = 0.287867963f,
			}));
		}
	};
}
#endif
