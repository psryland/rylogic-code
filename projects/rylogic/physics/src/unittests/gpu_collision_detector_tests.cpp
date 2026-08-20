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
//  - These tests are *not* the same as gpu_collision_tests.cpp because the actual GPU is used here.

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/collision/unittest_helpers.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
#include "src/compute/collide_gpu.h"
#include "src/collision/shape_cache.h"

namespace pr::physics::tests
{
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

		static bool PointInManifold(collision::Contact const& manifold_contact, v4 point, float tol)
		{
			auto count = manifold_contact.Count();
			if (count == 0)
				return false;
			if (count == 1)
				return FEqlRelative(point, manifold_contact.m_manifold[0], tol);
			if (count == 2)
			{
				auto const& p0 = manifold_contact.m_manifold[0];
				auto const& p1 = manifold_contact.m_manifold[1];
				auto edge = p1 - p0;
				auto len_sq = LengthSq(edge);
				if (len_sq <= Sqr(tol))
					return FEqlRelative(point, p0, tol);

				auto t = Clamp(Dot3(point - p0, edge) / len_sq, 0.0f, 1.0f);
				return LengthSq(point - (p0 + t * edge)) <= Sqr(tol);
			}

			if (Abs(Dot3(point - manifold_contact.Point(), manifold_contact.m_axis)) > tol)
				return false;

			for (int i = 0; i != count; ++i)
			{
				auto const& p0 = manifold_contact.m_manifold[i];
				auto const& p1 = manifold_contact.m_manifold[(i + 1) % count];
				if (Dot3(Cross(p1 - p0, point - p0), manifold_contact.m_axis) < -tol)
					return false;
			}
			return true;
		}
		static bool CheckReducedGpuContact(collision::Contact const& gpu_contact, collision::Contact const& cpu_contact, float tol)
		{
			if (gpu_contact.m_feature != collision::EFeature::Vert)
				return false;
			if (cpu_contact.Count() < 2)
				return false;

			return PointInManifold(cpu_contact, gpu_contact.m_manifold[0], tol);
		}

		// Run both CPU Collide() and the GPU GpuCollisionDetector for a shape pair.
		void CompareGpuVsCpu(
			collision::Shape const& sa, m4x4 const& l2w,
			collision::Shape const& sb, m4x4 const& r2w,
			bool expect_collision,
			bool allow_reduced_gpu_contact = false)
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
				auto gpu_contacts = m_detector.DetectCollisions(m_gpu.m_job, pairs, shape_cache, out_contacts);
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
			auto cpu_contact_local = cpu_contact;
			cpu_contact_local.m_axis = w2a * cpu_contact.m_axis;
			for (auto& point : std::span{cpu_contact_local.m_manifold}.subspan(0, cpu_contact_local.Count()))
				point = w2a * point;

			auto gpu_contact_local = collision::Contact{};
			gpu_contact_local.m_axis = gpu_contact.axis;
			gpu_contact_local.m_feature = static_cast<collision::EFeature>(gpu_contact.feature);
			gpu_contact_local.m_depth = gpu_contact.depth;
			for (int i = 0, iend = gpu_contact_local.Count(); i != iend; ++i)
				gpu_contact_local.m_manifold[i] = gpu_contact.manifold[i];

			// --- Compare contact details ---
			// Depth comparison (relative tolerance, with absolute floor for near-zero depths)
			PR_EXPECT(FEqlRelative(cpu_contact.m_depth, gpu_contact.depth, DepthRelTol));

			// Axis direction comparison. Contact normals must point from object A toward object B.
			auto cpu_axis = Normalise(cpu_contact_local.m_axis);
			auto gpu_axis = Normalise(gpu_contact.axis);
			PR_EXPECT(dot(cpu_axis, gpu_axis) > 0.0f);
			auto angle = Angle(cpu_axis, gpu_axis);
			PR_EXPECT(angle < AxisAngleTol);

			if (allow_reduced_gpu_contact && CheckReducedGpuContact(gpu_contact_local, cpu_contact_local, PointTol))
				return;

			PR_EXPECT(collision::tests::CheckContact(gpu_contact_local, cpu_contact_local, PointTol));
		}

		// 1. Overlapping spheres (same radius)
		PRUnitTestMethod(SphereVsSphere_Overlap)
		{
			auto sa = collision::ShapeSphere{1.0f};
			auto sb = collision::ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.0f, 0, 0); // overlap by 1.0 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 2. Overlapping aligned boxes
		PRUnitTestMethod(BoxVsBox_Overlap)
		{
			auto sa = collision::ShapeBox{v4{2, 2, 2, 0}};  // half-extents = 1,1,1
			auto sb = collision::ShapeBox{v4{2, 2, 2, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.5f, 0, 0); // overlap by 0.5 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 3. Box vs sphere overlap
		PRUnitTestMethod(BoxVsSphere_Overlap)
		{
			auto sa = collision::ShapeBox{v4{2, 2, 2, 0}};  // half-extents = 1,1,1
			auto sb = collision::ShapeSphere{0.8f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.5f, 0, 0); // sphere centre 1.5 from box centre

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 4. Line vs box overlap
		PRUnitTestMethod(LineVsBox_Overlap)
		{
			auto sa = collision::ShapeLine{1.0f, 0.2f};
			auto sb = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Translation(0.65f, 0, 0);
			auto r2w = m4x4::Identity();

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 5. Box vs line overlap
		PRUnitTestMethod(BoxVsLine_Overlap)
		{
			auto sa = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto sb = collision::ShapeLine{1.0f, 0.2f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0.65f, 0, 0);

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 6. Triangle vs box overlap
		PRUnitTestMethod(TriangleVsBox)
		{
			auto sa = collision::ShapeTriangle{
				v4{-0.5f, -0.5f, 0, 1},
				v4{+0.5f, -0.5f, 0, 1},
				v4{0, +0.5f, 0, 1}};
			auto sb = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Identity();

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 7. Separated shapes (should both return no collision)
		PRUnitTestMethod(SphereVsSphere_Separated)
		{
			auto sa = collision::ShapeSphere{1.0f};
			auto sb = collision::ShapeSphere{1.0f};
			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(3.0f, 0, 0); // gap of 1.0 unit

			CompareGpuVsCpu(sa, l2w, sb, r2w, false);
		}

		// 8. Rotated box vs sphere
		PRUnitTestMethod(RotatedBoxVsSphere)
		{
			auto sa = collision::ShapeBox{v4{2, 4, 2, 0}};  // half-extents = 1,2,1
			auto sb = collision::ShapeSphere{0.5f};

			// Rotate the box 45 degrees about the Z axis
			auto l2w = m4x4::Transform(RotationRad<m3x3>(0, 0, constants<float>::tau_by_8), v4::Origin());
			auto r2w = m4x4::Translation(1.8f, 0, 0);

			CompareGpuVsCpu(sa, l2w, sb, r2w, true);
		}

		// 9. Polytope (tetrahedron) vs box
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
			auto r2w = m4x4::Translation(1.0f, 0.7f, 0.3f); // overlapping the tetrahedron (asymmetric to avoid axis ambiguity)

			CompareGpuVsCpu(poly, l2w, sb, r2w, true, true);
		}

		// 10. Polytope vs polytope face contact
		PRUnitTestMethod(PolytopeVsTriangle)
		{
			v4 tet_pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{+0.8f, -0.8f, -0.5f, 1},
				v4{+0.0f, +0.8f, -0.5f, 1},
				v4{+0.0f, +0.0f, +0.8f, 1},
			};
			auto poly_buf = collision::BuildPolytopeFromPoints(tet_pts);
			auto const& poly = poly_buf.as<collision::ShapePolytope>();

			auto triangle = collision::ShapeTriangle(
				v4{-0.5f, -0.5f, 0, 1},
				v4{+0.5f, -0.5f, 0, 1},
				v4{+0.0f, +0.5f, 0, 1});

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(0, 0, -0.1f);

			CompareGpuVsCpu(poly, l2w, triangle, r2w, true);
		}

		// 11. Polytope vs polytope face contact
		PRUnitTestMethod(PolytopeVsPolytope)
		{
			v4 cube_pts[] = {
				v4{-1, -1, -1, 1}, v4{+1, -1, -1, 1},
				v4{-1, +1, -1, 1}, v4{+1, +1, -1, 1},
				v4{-1, -1, +1, 1}, v4{+1, -1, +1, 1},
				v4{-1, +1, +1, 1}, v4{+1, +1, +1, 1},
			};
			auto buf_a = collision::BuildPolytopeFromPoints(cube_pts);
			auto buf_b = collision::BuildPolytopeFromPoints(cube_pts);
			auto const& poly_a = buf_a.as<collision::ShapePolytope>();
			auto const& poly_b = buf_b.as<collision::ShapePolytope>();

			auto l2w = m4x4::Identity();
			auto r2w = m4x4::Translation(1.5f, 0, 0);

			CompareGpuVsCpu(poly_a, l2w, poly_b, r2w, true);
		}

		// 11. Tumbling tetrahedron deeply penetrating a large ground box.
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
			auto ground_l2w = m4x4::Translation(0, 0, -5);

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

		// 12. Shallow polytope/ground contact captured from stress_test_1000.
		// The CPU path detects contact before the body gets deeply embedded; the GPU
		// path must not miss this or the eventual deep correction can launch the body.
		PRUnitTestMethod(PolytopeVsGround_ShallowFastTumble)
		{
			v4 poly_pts[] = {
				v4{-1.0296705f,  0.2203998f, -0.1274743f, 1},
				v4{ 0.2744289f, -0.4075151f, -0.0194631f, 1},
				v4{ 0.2335063f, -0.0944359f, -0.4734315f, 1},
				v4{-0.1430905f,  0.3362911f,  0.0826810f, 1},
				v4{ 0.0412595f, -0.4096290f, -0.3432735f, 1},
				v4{ 0.3788538f,  0.3895851f,  0.5945120f, 1},
				v4{-0.1899699f, -0.1738422f, -0.2568449f, 1},
				v4{ 0.5233417f,  0.1291325f, -0.6528223f, 1},
				v4{-0.2173630f, -0.1193020f,  0.2580981f, 1},
			};
			auto poly_buf = collision::BuildPolytopeFromPoints(poly_pts);
			auto const& poly = poly_buf.as<collision::ShapePolytope>();

			auto ground = collision::ShapeBox{v4{790.38454f, 790.38454f, 10.0f, 0}};
			auto ground_l2w = m4x4::Translation(0, 0, -5);
			auto poly_o2w = m4x4{
				v4{-0.48897f, -0.03973f, -0.87140f, 0},
				v4{ 0.39218f,  0.88229f, -0.26030f, 0},
				v4{ 0.77917f, -0.46902f, -0.41583f, 0},
				v4{18.36216f,-60.80049f,  0.39107f, 1},
			};

			auto cpu_contact = collision::Contact{};
			auto cpu_hit = collision::Collide(poly, poly_o2w, ground, ground_l2w, cpu_contact);
			PR_EXPECT(cpu_hit);
			PR_EXPECT(cpu_contact.m_depth > 0.2f);

			auto shape_cache = ShapeCache{};
			shape_cache.GetOrAdd(poly);
			shape_cache.GetOrAdd(ground);

			auto pair = GpuCollisionPair{};
			pair.body_idx_a = 0;
			pair.body_idx_b = 1;
			pair.shape_idx_a = 0;
			pair.shape_idx_b = 1;
			pair.b2a = InvertOrthonormal(poly_o2w) * ground_l2w;

			auto pairs = std::vector<GpuCollisionPair>{ pair };
			auto out_contacts = std::vector<GpuResolveContact>{ 1 };
			auto gpu_contacts = m_detector.DetectCollisions(m_gpu.m_job, pairs, shape_cache, out_contacts);
			PR_EXPECT(!gpu_contacts.empty());
		}
	};
}
#endif
