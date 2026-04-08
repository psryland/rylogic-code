//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// Unit tests that compare GPU GJK collision results against CPU GJK collision results.
// Each test creates a pair of shapes, runs both the CPU path (GjkCollide) and the
// GPU path (GpuDetectCollisions), and verifies the results match within tolerance.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collide_gpu.h"
#include "src/collision/shape_cache.h"

namespace pr::physics::tests
{
	void ForceLink_GpuCollision() {}
	PRUnitTestClass(GpuCollisionTests)
	{
		// Tolerance thresholds for comparing CPU vs GPU results
		static constexpr float DepthRelTol = 0.05f;   // 5% relative depth tolerance
		static constexpr float AxisAngleTol = 0.1f;    // radians
		static constexpr float PointTol = 0.1f;        // world units

		Gpu m_gpu;
		EngineConfig m_config;
		GpuCollisionDetector m_detector;
		TestClass_GpuCollisionTests()
			: m_gpu()
			, m_config()
			, m_detector(m_gpu, m_config)
		{}

		// Run both CPU GjkCollide and GPU GpuDetectCollisions for a shape pair.
		// Returns true if both agree on collision (or both agree on no collision).
		// When both detect collision, validates that axis/depth/point match within tolerance.
		void CompareGpuVsCpu(
			collision::Shape const& sa, m4x4 const& l2w,
			collision::Shape const& sb, m4x4 const& r2w,
			bool expect_collision)
		{
			// --- CPU path ---
			auto cpu_contact = collision::Contact{};
			auto cpu_hit = collision::GjkCollide(sa, l2w, sb, r2w, cpu_contact);

			// --- GPU path ---
			// Pack shapes into GPU buffers
			ShapeCache shape_cache;
			shape_cache.GetOrAdd(sa);
			shape_cache.GetOrAdd(sb);

			// Build collision pair. GJK runs with A at identity, B at b2a.
			auto pair = GpuCollisionPair{};
			pair.body_idx_a = 0;
			pair.body_idx_b = 1;
			pair.shape_idx_a = 0;
			pair.shape_idx_b = 1;
			pair.b2a = InvertOrthonormal(l2w) * r2w;

			auto pairs = std::vector<GpuCollisionPair>{ pair };
			auto out_contacts = std::vector<GpuResolveContact>{1};

			// Create a GpuIntegrator (which owns the D3D12 device and command queue),
			// then create the collision detector sharing the same Gpu instance.
			auto [gpu_contacts, diag] = m_detector.DetectCollisions(m_gpu.m_job, pairs, shape_cache, out_contacts, {});
			auto gpu_hit = gpu_contacts.size() > 0;
			(void)diag; // diagnostics not used in this test

			// --- Compare collision/no-collision agreement ---
			PR_EXPECT(cpu_hit == expect_collision);
			PR_EXPECT(gpu_hit == expect_collision);

			if (!expect_collision)
				return;

			// --- Compare contact details ---
			PR_EXPECT(gpu_contacts.size() == 1u);
			if (gpu_contacts.empty())
				return;

			auto const& gc = gpu_contacts[0];

			// GPU contact is in objA's local space. Transform CPU contact to objA space for comparison.
			auto w2a = InvertOrthonormal(l2w);
			auto cpu_axis_local = (w2a * cpu_contact.m_axis.w0()).w0(); // transform direction (w=0)
			auto cpu_point_local = w2a * cpu_contact.m_point;

			// Depth comparison (relative tolerance, with absolute floor for near-zero depths)
			auto depth_err = Abs(gc.depth - cpu_contact.m_depth);
			auto depth_ref = Max(Abs(cpu_contact.m_depth), 0.01f);
			PR_EXPECT(depth_err / depth_ref < DepthRelTol);

			// Axis direction comparison (angle between normals, allowing sign flip)
			auto dot = Abs(Dot3(Normalise(gc.axis), Normalise(cpu_axis_local)));
			dot = Clamp(dot, 0.0f, 1.0f);
			auto angle = acos(dot);
			PR_EXPECT(angle < AxisAngleTol);

			// Contact point comparison
			auto pt_err = Length(gc.contact_point - cpu_point_local);
			PR_EXPECT(pt_err < PointTol);
		}

		// 1. Overlapping spheres (same radius)
		PRUnitTestMethod(SphereVsSphere_Overlap)
		{
			auto sa = collision::ShapeSphere{1.0f};
			auto sb = collision::ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.0f, 0, 0, 0}); // overlap by 1.0 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 2. Overlapping aligned boxes
		PRUnitTestMethod(BoxVsBox_Overlap)
		{
			auto sa = collision::ShapeBox{v4{2, 2, 2, 0}};  // half-extents = 1,1,1
			auto sb = collision::ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.5f, 0, 0, 0}); // overlap by 0.5 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 3. Box vs sphere overlap
		PRUnitTestMethod(BoxVsSphere_Overlap)
		{
			auto sa = collision::ShapeBox{v4{2, 2, 2, 0}};  // half-extents = 1,1,1
			auto sb = collision::ShapeSphere{0.8f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.5f, 0, 0, 0}); // sphere centre 1.5 from box centre

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 4. Separated shapes (should both return no collision)
		PRUnitTestMethod(SphereVsSphere_Separated)
		{
			auto sa = collision::ShapeSphere{1.0f};
			auto sb = collision::ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{3.0f, 0, 0, 0}); // gap of 1.0 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, false);
		}

		// 5. Rotated box vs sphere
		PRUnitTestMethod(RotatedBoxVsSphere)
		{
			auto sa = collision::ShapeBox{v4{2, 4, 2, 0}};  // half-extents = 1,2,1
			auto sb = collision::ShapeSphere{0.5f};

			// Rotate the box 45 degrees about the Z axis
			auto l2w = m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin());
			auto r2w = m4x4::Translation(v4{1.8f, 0, 0, 0});

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 6. Polytope (tetrahedron) vs box
		PRUnitTestMethod(PolytopeVsBox)
		{
			// Build a tetrahedron from 4 points
			v4 tet_pts[] = {
				v4{0, 0, 0, 1},
				v4{2, 0, 0, 1},
				v4{1, 2, 0, 1},
				v4{1, 1, 2, 1},
			};
			auto poly_buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto const& poly = poly_buf.as<collision::ShapePolytope>();

			auto sb = collision::ShapeBox{v4{2, 2, 2, 0}}; // half-extents = 1,1,1
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(v4{1.0f, 0.5f, 0.5f, 0}); // overlapping the tetrahedron

			CompareGpuVsCpu(poly, l2w, sb, r2w, true);
		}
	};
}
#endif
