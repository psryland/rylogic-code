//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// Unit tests that compare GPU collision results against CPU collision results.
// Each test creates a pair of shapes, runs both the CPU path and the GPU path,
// and verifies the results match within tolerance.
// Notes:
//  - The GPU detection actually runs on the GPU so decrepencies could indicate a difference
//    in behaviour between the hlsl interop versions and true GPU versions.
//  - This tests are *not* the same as test_gpu_collision.cpp because the actual GPU is used here.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collide_gpu.h"
#include "src/collision/shape_cache.h"

namespace pr::physics::tests
{
	void ForceLink_GpuCollisionDetector() {}
	PRUnitTestClass(GpuCollisionDetectorTests)
	{
		// Tolerance thresholds for comparing CPU vs GPU results
		static constexpr float DepthRelTol = 0.001f;  // depth tolerance
		static constexpr float AxisAngleTol = 0.001f; // radians
		static constexpr float PointTol = 0.001f;     // world units

		Gpu m_gpu;
		EngineConfig m_config;
		GpuCollisionDetector m_detector;
		TestClass_GpuCollisionDetectorTests()
			: m_gpu()
			, m_config()
			, m_detector(m_gpu, m_config)
		{}

		// Run both CPU Collide() and the GPU GpuCollisionDetector for a shape pair.
		void CompareGpuVsCpu(
			collision::Shape const& sa, m4x4 const& l2w,
			collision::Shape const& sb, m4x4 const& r2w,
			bool expect_collision)
		{
			// --- CPU path ---
			auto cpu_contact = collision::Contact{};
			auto cpu_hit = false;
			{
				// This runs the collision dispatch to collide using specilised functions when appropriate
				cpu_hit = collision::Collide(sa, l2w, sb, r2w, cpu_contact);
			}

			// --- GPU path ---
			auto gpu_contact = GpuResolveContact{};
			auto gpu_hit = false;
			{
				// Pack shapes into GPU buffers
				ShapeCache shape_cache;
				shape_cache.GetOrAdd(sa);
				shape_cache.GetOrAdd(sb);

				// Build collision pair.
				auto pair = GpuCollisionPair{};
				pair.body_idx_a = 0;
				pair.body_idx_b = 1;
				pair.shape_idx_a = 0;
				pair.shape_idx_b = 1;
				pair.b2a = InvertOrthonormal(l2w) * r2w;

				// Setup the GPU collision detector buffers
				auto pairs = std::vector<GpuCollisionPair>{ pair };
				auto out_contacts = std::vector<GpuResolveContact>{ 1 };
				auto [gpu_contacts, diag] = m_detector.DetectCollisions(m_gpu.m_job, pairs, shape_cache, out_contacts, {});
				gpu_hit = gpu_contacts.size() > 0;
				if (gpu_hit) gpu_contact = gpu_contacts[0];
			}

			// --- Compare collision/no-collision agreement ---
			PR_EXPECT(cpu_hit == expect_collision);
			PR_EXPECT(gpu_hit == expect_collision);
			if (!expect_collision)
				return;

			// GPU contact is in objA's local space. Transform CPU contact to objA space for comparison.
			auto w2a = InvertOrthonormal(l2w);
			auto cpu_axis_local = (w2a * cpu_contact.m_axis.w0()).w0(); // transform direction (w=0)
			auto cpu_point_local = w2a * cpu_contact.m_point;

			// --- Compare contact details ---
			// Depth comparison (relative tolerance, with absolute floor for near-zero depths)
			PR_EXPECT(FEqlRelative(cpu_contact.m_depth, gpu_contact.depth, DepthRelTol));

			// Axis direction comparison (angle between normals, allowing sign flip)
			auto angle = Angle(Normalise(cpu_axis_local), Normalise(gpu_contact.axis));
			PR_EXPECT(angle < AxisAngleTol);

			// Contact point comparison
			auto pt_err = Length(gpu_contact.contact_point - cpu_point_local);
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
			auto r2w = m4x4::Translation(v4{1.0f, 0.7f, 0.3f, 0}); // overlapping the tetrahedron (asymmetric to avoid axis ambiguity)

			CompareGpuVsCpu(poly, l2w, sb, r2w, true);
		}

		// 7. Tumbling tetrahedron deeply penetrating a large ground box.
		// Captured from the StressDropTests scenario to guard the deep polytope-vs-ground
		// case that exposed resolver issues after the GPU and CPU GJK paths already agreed.
		PRUnitTestMethod(PolytopeVsGround_DeepTumbling)
		{
			// Tetrahedron from StressDropTests
			v4 tet_pts[] = {
				v4{0, 0.3f, 0, 1}, v4{0.25f, -0.15f, 0, 1},
				v4{-0.125f, -0.15f, 0.22f, 1}, v4{-0.125f, -0.15f, -0.22f, 1},
			};
			auto poly_buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto const& poly = poly_buf.as<collision::ShapePolytope>();

			// Ground box: 100x100x10, centred at z=-5 (top surface at z=0)
			auto ground = collision::ShapeBox{v4{100, 100, 10, 0}};
			auto ground_l2w = m4x4::Translation(v4{0, 0, -5, 0});

			// Polytope captured orientation/position from the failing log
			auto poly_o2w = m4x4{
				v4{-0.197323f,  0.977689f, -0.072023f, 0},
				v4{ 0.907228f,  0.154274f, -0.391326f, 0},
				v4{-0.371484f, -0.142558f, -0.917429f, 0},
				v4{-1.505407f,  1.204721f, -2.204973f, 1},
			};

			// Confirm CPU detects this collision (sanity check)
			auto cpu_contact = collision::Contact{};
			auto cpu_hit = collision::Collide(ground, ground_l2w, poly, poly_o2w, cpu_contact);
			PR_EXPECT(cpu_hit);
			PR_EXPECT(cpu_contact.m_depth > 2.0f); // expected deep penetration

			CompareGpuVsCpu(ground, ground_l2w, poly, poly_o2w, true);
		}
	};
}
#endif
