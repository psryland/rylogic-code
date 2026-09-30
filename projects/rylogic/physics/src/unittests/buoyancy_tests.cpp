//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/articulation/articulation.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/buoyancy/gpu_buoyancy.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "pr/physics/shape/shape_builder.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_array.h"
#include "src/buoyancy/buoyancy_analytical.h"
#include "src/unittests/shared_gpu.h"
#include <chrono>
#include <complex>
#include <format>
#include <algorithm>

namespace pr::physics::tests
{
	PRUnitTestClass(BuoyancyAnalyticTests)
	{
		// Verify the CPU analytic clipped-box result for simple flat-water depths.
		PRUnitTestMethod(FlatWaterBoxVolume, Extended)
		{
			auto const half_extents = v4{1.0f, 1.0f, 0.5f, 0.0f};
			auto check = [half_extents](float z, bool valid, float volume, v4 centroid)
			{
				auto const result = SubmergedBoxVolumeCentroid(m4x4::Translation(0.0f, 0.0f, z), half_extents, 0.0f);
				if (result.m_valid != valid)
				{
					PR_EXPECT(false);
					return;
				}
				if (!valid)
					return;

				if (!FEqlAbsolute(result.m_volume_m3, volume, 1e-5f) || !FEqlAbsolute(result.m_centroid_ws, centroid, 1e-5f))
				{
					PR_EXPECT(false);
				}
			};

			check(+1.0f, false, 0.0f, v4::Zero());
			check(+0.25f, true, 1.0f, v4{0.0f, 0.0f, -0.125f, 1.0f});
			check(0.0f, true, 2.0f, v4{0.0f, 0.0f, -0.25f, 1.0f});
			check(-1.0f, true, 4.0f, v4{0.0f, 0.0f, -1.0f, 1.0f});
		}
	};

	// Volume sample spacing used by the GPU tests. The analytic tolerances below were calibrated at about 8192 samples on a 2 x 2 x 1 m box,
	// so the tests pin this spacing rather than following the (cheaper) shipped default.
	static constexpr float HarnessVolumeSpacing = 0.08f;

	// Retain the heavyweight GPU queue across test methods while keeping the body resolver bound to stable storage.
	struct HarnessStorage
	{
		std::vector<RigidBody> m_bodies;
		Engine m_engine;
		GpuBuoyancy m_buoyancy;

		explicit HarnessStorage(bool enable_diagnostics)
			: m_bodies()
			, m_engine(
				EngineConfig{},
				nullptr,
				SharedTestGpu().m_gpu.device(),
				SharedTestGpu().m_gpu.queue())
			, m_buoyancy(
				m_engine.Device(),
				m_engine,
				GpuBuoyancy::Config{
					.m_volume_spacing = HarnessVolumeSpacing,
					.m_enable_diagnostics = enable_diagnostics,
				},
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[this](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(m_bodies))
						return body_state;

					body_state.m_o2w = m_bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = m_bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = m_bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				})
		{}
	};

	// Return independent retained fixtures for diagnostic and production configurations.
	static HarnessStorage& SharedHarnessStorage(bool enable_diagnostics)
	{
		if (enable_diagnostics)
		{
			static auto storage = HarnessStorage(true);
			return storage;
		}

		static auto storage = HarnessStorage(false);
		return storage;
	}

	// Present isolated per-method body state while reusing the configuration's long-lived GPU resources.
	struct Harness
	{
		HarnessStorage& m_storage;
		std::vector<RigidBody>& m_bodies;
		Engine& m_engine;
		GpuBuoyancy& m_buoyancy;

		explicit Harness(bool enable_diagnostics = true)
			: m_storage(SharedHarnessStorage(enable_diagnostics))
			, m_bodies(m_storage.m_bodies)
			, m_engine(m_storage.m_engine)
			, m_buoyancy(m_storage.m_buoyancy)
		{
			// Restore all mutable fixture state so retained GPU resources cannot couple otherwise-independent test methods.
			m_bodies.clear();
			m_engine.ResetCaches();
			m_buoyancy.SetWaterField(terrain::water::WaterField{});
			m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = enable_diagnostics,
			});
		}
	};

	// Coverage for sampled-composite hull flattening, registration lifetime, and GPU integration.
	// GPU-vs-oracle cases validate deterministic sampling as well as force and diagnostic readback.
	PRUnitTestClass(BuoyancyCompositeHostTests)
	{
		// Keep the GPU pass and CPU oracle on the shared buoyancy sample densities.
		PRUnitTestMethod(SharedSampleDensityDefaults, Extended)
		{
			PR_EXPECT(GpuBuoyancy::Config{}.m_surface_spacing == buoyancy::DefaultSurfaceSpacing);
			PR_EXPECT(GpuBuoyancy::Config{}.m_volume_spacing == buoyancy::DefaultVolumeSpacing);
			PR_EXPECT(buoyancy::SamplerConfig{}.m_surface_spacing == buoyancy::DefaultSurfaceSpacing);
			PR_EXPECT(buoyancy::SamplerConfig{}.m_volume_spacing == buoyancy::DefaultVolumeSpacing);
		}

		// A single box flattens to one analytic Box primitive carrying its half-extents and no geometry.
		PRUnitTestMethod(FlattenBoxSinglePrimitive, Extended)
		{
			auto const half = v4{1.5f, 0.5f, 0.25f, 0.0f};
			auto box = collision::ShapeBox(half * 2.0f);
			auto const hull = buoyancy::FlattenShape(collision::shape_cast(box));

			PR_EXPECT(!hull.Empty());
			PR_EXPECT(hull.m_primitives.size() == 1);

			auto const& p = hull.m_primitives[0];
			PR_EXPECT(p.m_type == static_cast<int>(buoyancy::EPrimitiveType::Box));
			PR_EXPECT(p.m_sibling_index == 0);
			PR_EXPECT(FEqlAbsolute(p.m_params, v4{half.x, half.y, half.z, 0.0f}, 1e-6f));

			// Boxes are analytic: no concatenated geometry on the hull.
			PR_EXPECT(p.m_vert_count == 0 && p.m_volume_vert_count == 0 && p.m_tet_count == 0 && p.m_face_count == 0);
			PR_EXPECT(hull.m_verts.empty() && hull.m_volume_verts.empty() && hull.m_tets.empty() && hull.m_tet_cdf.empty() && hull.m_face_planes.empty());
		}

		// The face fan for the stress-scene octahedron has one shared centre, one copy of each surface
		// vertex, and one tetrahedron per face while preserving volume and first moment.
		PRUnitTestMethod(FlattenOctahedronFaceFan, Extended)
		{
			auto const points = std::array{
				v4{+0.7f, 0.0f, 0.0f, 1.0f},
				v4{-0.7f, 0.0f, 0.0f, 1.0f},
				v4{0.0f, +0.7f, 0.0f, 1.0f},
				v4{0.0f, -0.7f, 0.0f, 1.0f},
				v4{0.0f, 0.0f, +0.6f, 1.0f},
				v4{0.0f, 0.0f, -0.6f, 1.0f},
			};
			auto shape = collision::BuildPolytopeFromPoints(points);
			auto const& poly = shape.as<collision::ShapePolytope>();
			auto const hull = buoyancy::FlattenShape(poly, -1);
			auto const logical_bytes =
				hull.m_primitives.size() * sizeof(hull.m_primitives[0]) +
				hull.m_verts.size() * sizeof(hull.m_verts[0]) +
				hull.m_volume_verts.size() * sizeof(hull.m_volume_verts[0]) +
				hull.m_tets.size() * sizeof(hull.m_tets[0]) +
				hull.m_tet_cdf.size() * sizeof(hull.m_tet_cdf[0]) +
				hull.m_face_planes.size() * sizeof(hull.m_face_planes[0]) +
				hull.m_face_verts.size() * sizeof(hull.m_face_verts[0]);
			PR_EXPECT(hull.m_primitives.size() == 1);
			PR_EXPECT(hull.m_verts.size() == 6);
			PR_EXPECT(hull.m_volume_verts.size() == 7);
			PR_EXPECT(hull.m_tets.size() == 8);
			PR_EXPECT(hull.m_tet_cdf.size() == 8);
			PR_EXPECT(hull.m_face_planes.size() == 8);
			PR_EXPECT(logical_bytes == 752);

			auto volume = 0.0f;
			auto first_moment = v4::Zero();
			for (auto const& tet : hull.m_tets)
			{
				auto const a = hull.m_volume_verts[tet.x];
				auto const b = hull.m_volume_verts[tet.y];
				auto const c = hull.m_volume_verts[tet.z];
				auto const d = hull.m_volume_verts[tet.w];
				auto const tet_volume = tetramesh::Volume(a, b, c, d);
				volume += tet_volume;
				first_moment += (tet_volume * (a + b + c + d) / 4.0f).w0();
			}
			PR_EXPECT(FEqlRelative(volume, buoyancy::PrimitiveVolume(poly), 1e-5f));
			PR_EXPECT(FEqlAbsolute(first_moment, v4::Zero(), 1e-6f));
		}

		// A ShapeArray of two boxes flattens to two Box primitives in child order with distinct transforms.
		PRUnitTestMethod(FlattenArrayTwoBoxes, Extended)
		{
			ShapeBuilder sb;
			sb.AddShape(collision::ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(+0.5f, 0.0f, 0.0f)));
			sb.AddShape(collision::ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(-0.5f, 0.0f, 0.0f)));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* arr = sb.BuildShape(data, mp, model_to_com);
			PR_EXPECT(arr != nullptr && arr->m_type == collision::EShape::Array);

			auto const hull = buoyancy::FlattenShape(*arr);
			PR_EXPECT(hull.m_primitives.size() == 2);
			PR_EXPECT(hull.m_primitives[0].m_type == static_cast<int>(buoyancy::EPrimitiveType::Box));
			PR_EXPECT(hull.m_primitives[1].m_type == static_cast<int>(buoyancy::EPrimitiveType::Box));
			PR_EXPECT(hull.m_primitives[0].m_sibling_index == 0);
			PR_EXPECT(hull.m_primitives[1].m_sibling_index == 1);

			// Each child keeps its own half-extents (1, 0.5, 0.5)...
			PR_EXPECT(FEqlAbsolute(hull.m_primitives[0].m_params, v4{1.0f, 0.5f, 0.5f, 0.0f}, 1e-6f));
			PR_EXPECT(FEqlAbsolute(hull.m_primitives[1].m_params, v4{1.0f, 0.5f, 0.5f, 0.0f}, 1e-6f));

			// ...and the two transforms differ (the boxes sit either side of the shared centre).
			PR_EXPECT(!FEqlAbsolute(hull.m_primitives[0].m_s2r.pos, hull.m_primitives[1].m_s2r.pos, 1e-4f));
		}

		// An empty ShapeArray flattens to an empty hull (no primitives). The array is constructed
		// directly rather than via ShapeBuilder::BuildShape, which asserts that at least one shape
		// has been added; a zero-child array is a degenerate input FlattenShape must still tolerate.
		PRUnitTestMethod(FlattenEmptyArray, Extended)
		{
			auto arr = collision::ShapeArray{};
			arr.Complete(0);
			PR_EXPECT(arr.m_base.m_type == collision::EShape::Array);

			auto const hull = buoyancy::FlattenShape(arr);
			PR_EXPECT(hull.Empty());
		}

		// A sphere flattens to one Sphere primitive carrying its radius in m_params.x and no geometry.
		PRUnitTestMethod(FlattenSphere, Extended)
		{
			auto sphere = collision::ShapeSphere(2.5f);
			auto const hull = buoyancy::FlattenShape(collision::shape_cast(sphere));

			PR_EXPECT(hull.m_primitives.size() == 1);
			auto const& p = hull.m_primitives[0];
			PR_EXPECT(p.m_type == static_cast<int>(buoyancy::EPrimitiveType::Sphere));
			PR_EXPECT(FEqlAbsolute(p.m_params.x, 2.5f, 1e-6f));
			PR_EXPECT(hull.m_verts.empty() && hull.m_volume_verts.empty());
		}

		// A triangle flattens to one Triangle primitive contributing its three surface corners.
		PRUnitTestMethod(FlattenTriangle, Extended)
		{
			auto tri = collision::ShapeTriangle(v4{0.0f, 0.0f, 0.0f, 1.0f}, v4{1.0f, 0.0f, 0.0f, 1.0f}, v4{0.0f, 1.0f, 0.0f, 1.0f});
			auto const hull = buoyancy::FlattenShape(collision::shape_cast(tri));

			PR_EXPECT(hull.m_primitives.size() == 1);
			auto const& p = hull.m_primitives[0];
			PR_EXPECT(p.m_type == static_cast<int>(buoyancy::EPrimitiveType::Triangle));
			PR_EXPECT(p.m_vert_count == 3);
			PR_EXPECT(p.m_vert_ofs == 0);
			PR_EXPECT(hull.m_verts.size() == 3);
		}

		// A tessellated polytope flattens to one Polytope primitive whose concatenated tet geometry
		// conserves the polytope's volume.
		PRUnitTestMethod(FlattenPolytopeTessellated, Extended)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};
			auto buf = collision::BuildPolytopeFromPoints(pts, m4x4::Identity(), 0, collision::Shape::EFlags::None, 4);
			auto& poly = buf.as<collision::ShapePolytope>();

			auto const hull = buoyancy::FlattenShape(collision::shape_cast(poly));
			PR_EXPECT(hull.m_primitives.size() == 1);

			auto const& p = hull.m_primitives[0];
			PR_EXPECT(p.m_type == static_cast<int>(buoyancy::EPrimitiveType::Polytope));
			PR_EXPECT(p.m_vert_count == poly.m_vert_count);
			PR_EXPECT(p.m_face_count == poly.m_face_count);
			PR_EXPECT(p.m_tet_count == poly.m_tet_count && p.m_tet_count > 0);
			PR_EXPECT(p.m_volume_vert_count == poly.m_volume_vert_count && p.m_volume_vert_count > 0);

			// The descriptor counts must match the lengths of the concatenated geometry arrays.
			PR_EXPECT(isize(hull.m_verts) == p.m_vert_count);
			PR_EXPECT(isize(hull.m_volume_verts) == p.m_volume_vert_count);
			PR_EXPECT(isize(hull.m_tets) == p.m_tet_count);
			PR_EXPECT(hull.m_tet_cdf.size() == hull.m_tets.size());
			PR_EXPECT(isize(hull.m_face_planes) == p.m_face_count);

			// Volume conservation: the cube has volume 8, and each CDF entry must equal the running
			// volume in tet order. Tet indices are relative to this single primitive and absolute here.
			auto sum = 0.0f;
			for (auto i = 0; i != isize(hull.m_tets); ++i)
			{
				auto const& t = hull.m_tets[i];
				auto a = hull.m_volume_verts[t.x];
				auto b = hull.m_volume_verts[t.y];
				auto c = hull.m_volume_verts[t.z];
				auto d = hull.m_volume_verts[t.w];
				sum += tetramesh::Volume(a, b, c, d);
				PR_EXPECT(FEqlRelative(hull.m_tet_cdf[i], sum, 1e-6f));
			}
			PR_EXPECT(FEqlRelative(sum, 8.0f, 1e-4f));
		}

		// A polytope without an interior tessellation cannot supply volume samples, so flattening throws.
		PRUnitTestMethod(FlattenPolytopeMissingTetsThrows, Extended)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};
			auto buf = collision::BuildPolytopeFromPoints(pts); // tess_resolution defaults to 0 => no tets
			auto& poly = buf.as<collision::ShapePolytope>();
			PR_EXPECT(poly.m_tet_count == 0);

			auto threw = false;
			try { (void)buoyancy::FlattenShape(collision::shape_cast(poly)); }
			catch (std::exception const&) { threw = true; }
			PR_EXPECT(threw);
		}

		// A collision-only polytope can be converted to compact buoyancy geometry without changing the
		// source shape. The face fan contributes one centre vertex and one tetrahedron per surface face.
		PRUnitTestMethod(FlattenPolytopeDerivesMissingTets, Extended)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};
			auto buf = collision::BuildPolytopeFromPoints(pts);
			auto const& poly = buf.as<collision::ShapePolytope>();
			auto const hull = buoyancy::FlattenShape(collision::shape_cast(poly), -1);

			PR_EXPECT(poly.m_tet_count == 0);
			PR_EXPECT(hull.m_primitives.size() == 1);
			PR_EXPECT(hull.m_primitives[0].m_tet_count == poly.m_face_count);
			PR_EXPECT(hull.m_primitives[0].m_volume_vert_count == poly.m_vert_count + 1);
			PR_EXPECT(hull.m_tet_cdf.size() == hull.m_tets.size());
			PR_EXPECT(FEqlRelative(buoyancy::PrimitiveVolume(collision::shape_cast(poly)), 8.0f, 1e-4f));
		}

		// A thin line has no volume or outward normal, so the composite model rejects it.
		PRUnitTestMethod(FlattenUnsupportedTypeThrows, Extended)
		{
			auto line = collision::ShapeLine(2.0f);
			auto threw = false;
			try { (void)buoyancy::FlattenShape(collision::shape_cast(line)); }
			catch (std::exception const&) { threw = true; }
			PR_EXPECT(threw);
		}

		// A capsule flattens to one analytic Capsule primitive, and its volume samples fill the capsule uniformly.
		PRUnitTestMethod(FlattenCapsule, Extended)
		{
			auto capsule = collision::ShapeLine(3.0f, 0.5f);
			auto const hull = buoyancy::FlattenShape(collision::shape_cast(capsule));
			PR_EXPECT(hull.m_primitives.size() == 1);

			auto const& p = hull.m_primitives[0];
			PR_EXPECT(p.m_type == static_cast<int>(buoyancy::EPrimitiveType::Capsule));
			PR_EXPECT(FEqlAbsolute(p.m_params.x, 0.5f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(p.m_params.y, 1.5f, 1e-6f));

			// Volume is a cylinder plus one sphere.
			auto const pi = static_cast<float>(math::constants<double>::tau_by_2);
			auto const cylinder = pi * 0.25f * 3.0f;
			auto const caps = (4.0f / 3.0f) * pi * 0.125f;
			auto const volume = buoyancy::PrimitiveVolume(collision::shape_cast(capsule));
			PR_EXPECT(FEqlRelative(volume, cylinder + caps, 1e-5f));

			// Every sample lies inside, the samples are centred, and the cap fraction matches the cap volume fraction.
			auto const table = buoyancy::BuildVolumeSampleTable(collision::shape_cast(capsule));
			auto const count = 8192;
			auto in_caps = 0;
			auto centre = v4::Zero();
			for (int i = 0; i != count; ++i)
			{
				auto const s = buoyancy::EmitVolumeSample(collision::shape_cast(capsule), buoyancy::SampleIndex(7u, 0, i), 1.0f, table);
				PR_EXPECT(buoyancy::ContainsLocal(collision::shape_cast(capsule), s.m_pos_local, 1e-5f));
				in_caps += std::abs(s.m_pos_local.z) > 1.5f ? 1 : 0;
				centre += s.m_pos_local.w0();
			}
			PR_EXPECT(FEqlAbsolute(static_cast<float>(in_caps) / count, caps / (cylinder + caps), 0.02f));
			PR_EXPECT(Length(centre / static_cast<float>(count)) < 0.02f);
		}

		// Registering a composite hull twice for the same body is an error.
		PRUnitTestMethod(CompositeDoubleRegisterThrows, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			PR_EXPECT(static_cast<bool>(reg));

			auto threw = false;
			try { auto reg2 = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0); }
			catch (std::exception const&) { threw = true; }
			PR_EXPECT(threw);
		}

		// Registration derives an untessellated collision polytope directly from the rigid body. A
		// fully submerged cube reports its exact total volume because every generated sample is wet.
		PRUnitTestMethod(GpuCompositeDerivesBodyPolytope, Extended)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};
			auto poly_buffer = collision::BuildPolytopeFromPoints(pts);
			auto const& poly = poly_buffer.as<collision::ShapePolytope>();

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&poly), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(poly.m_tet_count == 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(FEqlRelative(diag.m_volume_m3, 8.0f, 1e-4f));
		}

		// A live registration follows RigidBody::ShapeChange. Replacing a submerged box with a sphere
		// updates the cached geometry before the next step rather than applying forces from stale data.
		PRUnitTestMethod(GpuCompositeRefreshesChangedBodyShape, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 2.0f, 0.0f});
			auto sphere = collision::ShapeSphere(1.0f);

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_bodies[0].Shape(collision::shape_cast(&sphere));
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const expected_volume = (4.0f / 3.0f) * constants<float>::tau_by_2;
			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(FEqlRelative(diag.m_volume_m3, expected_volume, 1e-4f));
		}

		// Registration marks the body NeverSleep, and releasing the handle restores the prior flag.
		PRUnitTestMethod(CompositeUnregisterRestoresNeverSleep, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());
			h.m_bodies[0].NeverSleep(false);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			PR_EXPECT(h.m_bodies[0].NeverSleep() == true);

			reg.Reset();
			PR_EXPECT(h.m_bodies[0].NeverSleep() == false);
		}

		// Phase-11 GATE: a single box registered through the sampled-composite path must reproduce
		// the closed-form analytic box result (volume / force / centre-of-buoyancy / torque) to within
		// Monte-Carlo sampling error. This is the first end-to-end exercise of DispatchComposite plus
		// both volume kernels; it supersedes the old "Apply() throws" note (the force kernels have
		// landed, so Apply no longer throws for the composite path and a full Engine::Step is safe).
		//
		// Setup mirrors BuoyancyAnalyticTests::GpuDiagnosticMatchesAnalyticBox: a 2x2x1 box of mass
		// 500 kg at identity, flat water at z = 0, per-body gravity = AnalyticGravityWS. Half the box
		// (z in [-0.5, 0]) is submerged, so the expected readback is volume 2 m^3, buoyancy force
		// (0, 0, rho*|g|*V) = (0, 0, 19620) N, COB (0, 0, -0.25), torque ~ 0.
		//
		// The composite path is checked directly against the known analytic expectations.
		PRUnitTestMethod(GpuCompositeBoxMatchesAnalyticBox, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Low-discrepancy volume sampling of a symmetric half-submerged box leaves a small residual
			// in the symmetric-cancellation quantities (lateral force, COB x/y, torque). Tolerances are
			// the measured residual plus margin; the dominant quantities (wet volume, vertical force)
			// converge tightly because they are sums of equal-weight wet samples.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 2.0f, 0.005f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, v4{0.0f, 0.0f, 19620.0f, 0.0f}, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_ws, v4{0.0f, 0.0f, -0.25f, 1.0f}, 0.002f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 10.0f));
		}

		// Production stepping applies buoyancy without publishing validation readback.
		PRUnitTestMethod(DiagnosticsAreOptIn, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h(false);
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			PR_EXPECT(!h.m_buoyancy.LatestDiagnostics(0, 0).m_valid);

			// Prove the buoyancy force is actually applied to the rigid body, not merely reported in the
			// diagnostic record. The body also receives m*g during integration (it carries its own
			// gravity vector), so the net Z force is buoyancy + m*g = 19620 + 500*(-9.81) = 14715 N.
			auto const dt = 1.0f / 60.0f;
			auto const expected_velocity = (19620.0f / 500.0f + AnalyticGravityWS.z) * dt;
			PR_EXPECT(FEqlAbsolute(h.m_bodies[0].VelocityWS().lin, v4{0.0f, 0.0f, expected_velocity, 0.0f}, 1e-2f));
		}

		// Fully-dry fast path. A box positioned entirely above the z=0 water surface must
		// contribute zero buoyancy. The GPU per-sample fully-dry early-out
		// (BuoySupportAlongUp + lo >= water_max_height) suppresses all samples for the primitive.
		// The result is identical to the per-sample wet test (every sample is dry anyway), so this test
		// is a regression guard for the support-interval math driving the fast path, not a behaviour
		// change. The body still receives m*g, so only buoyancy must be zero, verified via the
		// diagnostic record.
		PRUnitTestMethod(GpuCompositeBoxFullyDryContributesZero, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);

			// Lift the box well clear of the water: half-height 0.5, so the lowest point sits at z=4.5.
			h.m_bodies[0].O2W(m4x4::Translation(v4{0.0f, 0.0f, 5.0f, 1.0f}));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// A fully-dry primitive displaces no fluid: zero volume, force, COB-moment, and torque.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 0.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, v4::Zero(), 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 1e-3f));
		}

		// Host-side dry broadphase cull must NOT cull a body that straddles the water line. A box
		// centred at z=0 (half-height 0.5, so it spans z=[-0.5,+0.5]) is half submerged in flat water,
		// so its registration-time AABB dips below the surface and the cull is rejected. The body must
		// be dispatched and report the normal half-submerged buoyancy (V=1 m^3, force=ρgV up), proving
		// the cull's conservative lowest-extent test does not over-aggressively drop wetted bodies.
		PRUnitTestMethod(GpuCompositeBoxStraddlingNotCulled, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Not culled: the lower half (z=[-0.5,0]) is submerged, displacing 2*2*0.5 = 2 m^3.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 2.0f, 0.005f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, v4{0.0f, 0.0f, 19620.0f, 0.0f}, 25.0f));
		}

		// The dry cull uses the water field's maximum height, so it stays exact under waves. A stationary long wave (amplitude 0.5)
		// has its crest at x = wavelength/4. A box above every crest is culled with zero buoyancy, while a box above the still-water
		// level but below the local crest must still be dispatched and report the wetted crest volume.
		PRUnitTestMethod(GpuCompositeWaveDryCullUsesMaxHeight, Extended)
		{
			// A stationary wave keeps the crest fixed whatever simulation time the engine passes to the dispatch.
			auto const wave = terrain::water::SineWave(v2{1.0f, 0.0f}, 0.5f, 1000.0f, 0.0f);
			auto const crest_x = 250.0f;
			auto run = [&](float centre_z)
			{
				// Each run uses a fresh harness so the diagnostics come from this placement only.
				auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
				Harness h;
				h.m_buoyancy.SetWaterField(terrain::water::WaterField(0.0, std::span{&wave, 1}));
				h.m_bodies.emplace_back();
				h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
				h.m_bodies[0].O2W(m4x4::Translation(v4{crest_x, 0.0f, centre_z, 1.0f}));
				h.m_bodies[0].NeverSleep(true);
				h.m_bodies[0].GravityWS(AnalyticGravityWS);

				auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
				h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
				h.m_buoyancy.CompleteStep();
				return h.m_buoyancy.LatestDiagnostics(0, 0);
			};

			// Lowest point at z=0.6 is above the 0.5 crest bound: culled, zero buoyancy.
			auto const dry = run(1.1f);
			PR_EXPECT(dry.m_valid);
			PR_EXPECT(FEqlAbsolute(dry.m_volume_m3, 0.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(dry.m_force_ws, v4::Zero(), 1e-3f));

			// Lowest point at z=0.25 is above the still-water level but 0.25 m below the crest: 2*2*0.25 = 1 m^3 wet.
			auto const wet = run(0.75f);
			PR_EXPECT(wet.m_valid);
			PR_EXPECT(FEqlAbsolute(wet.m_volume_m3, 1.0f, 0.01f));
		}

		// A flat gravity-frame water field for feeding the CPU oracle in GPU-vs-oracle parity tests:
		// height is zero everywhere along 'up', no slope, no fluid velocity. Combined with the default
		// WaterFrame (up=+Z, ref=origin) this exactly mirrors the GPU's flat z=0 water surface.
		struct FlatField
		{
			float Height(v2) const { return 0.0f; }
			v2 PressureGradient(v2, float) const { return v2::Zero(); }
			v4 Velocity(v4) const { return v4::Zero(); }
		};

		// A partially submerged resolution-5 polytope compares the GPU tet-CDF binary search with the
		// CPU oracle. Partial submersion makes the result depend on the selected tet and sample position,
		// unlike the fully submerged volume check where every selection contributes the same weight.
		PRUnitTestMethod(GpuCompositePolytopeCdfMatchesOracle, Extended)
		{
			v4 pts[] = {
				v4{+0.7f, 0.0f, 0.0f, 1.0f},
				v4{-0.7f, 0.0f, 0.0f, 1.0f},
				v4{0.0f, +0.7f, 0.0f, 1.0f},
				v4{0.0f, -0.7f, 0.0f, 1.0f},
				v4{0.0f, 0.0f, +0.6f, 1.0f},
				v4{0.0f, 0.0f, -0.6f, 1.0f},
			};
			auto poly_buffer = collision::BuildPolytopeFromPoints(pts, m4x4::Identity(), 0, collision::Shape::EFlags::None, 5);
			auto const& poly = poly_buffer.as<collision::ShapePolytope>();
			auto const o2w = m4x4::Translation(0.0f, 0.0f, 0.1f);

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&poly), 500.0f);
			h.m_bodies[0].O2W(o2w);
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			auto const config = h.m_buoyancy.GetConfig();
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			// Feed the same tessellated shape, transform, stable hull id, and sample spacings to the CPU
			// oracle so any CDF offset or binary-search boundary error changes the sampled wet volume.
			auto const oracle_body = buoyancy::BodyState{
				.m_o2w = o2w,
				.m_gravity_ws = AnalyticGravityWS,
			};
			auto const oracle_cfg = buoyancy::SamplerConfig{
				.m_fluid_density = config.m_fluid_density,
				.m_linear_drag_time_constant_s = config.m_linear_drag_time_constant_s,
				.m_angular_drag_time_constant_s = config.m_angular_drag_time_constant_s,
				.m_quadratic_drag_coefficient = config.m_quadratic_drag_coefficient,
				.m_tangential_drag_coefficient = config.m_tangential_drag_coefficient,
				.m_surface_spacing = config.m_surface_spacing,
				.m_volume_spacing = config.m_volume_spacing,
			};
			auto const oracle = buoyancy::SampleHull(
				collision::shape_cast(poly),
				0,
				oracle_body,
				buoyancy::WaterFrame{},
				FlatField{},
				oracle_cfg);
			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);

			PR_EXPECT(oracle.m_valid && diag.m_valid);
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, oracle.m_volume_m3, std::max(oracle.m_volume_m3 * 0.002f, 1e-4f)));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, oracle.m_buoyancy_force_ws, std::max(Length(oracle.m_buoyancy_force_ws) * 0.002f, 0.5f)));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, oracle.m_buoyancy_torque_ws, std::max(Length(oracle.m_buoyancy_torque_ws) * 0.02f, 0.5f)));
		}

		// Composite union volume: two concentric boxes (a big 2x2x1 and a small 1x1x0.5 fully inside it)
		// must report the union volume, NOT the sum. The lower-index volume sibling-cull means every
		// sample of the inner box lands inside the outer box (a lower-index sibling) and is discarded, so
		// the submerged union is exactly the outer box's half (2 m^3) rather than 2 + 0.25 = 2.25 m^3.
		// This is the key composite-dedup test: without the cull the volume (and buoyancy force) would be
		// inflated by the embedded inner primitive.
		PRUnitTestMethod(GpuCompositeOverlappingBoxesUnionVolume, Extended)
		{
			// Outer box is sibling 0, inner box is sibling 1; both concentric at the origin so the inner
			// box is entirely contained within the outer one.
			ShapeBuilder sb;
			sb.AddShape(collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f}));
			sb.AddShape(collision::ShapeBox(v4{1.0f, 1.0f, 0.5f, 0.0f}));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* arr = sb.BuildShape(data, mp, model_to_com);
			PR_EXPECT(arr != nullptr && arr->m_type == collision::EShape::Array);

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(arr, 500.0f);
			h.m_bodies[0].O2W(m4x4::Identity());
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Union submerged volume is the outer box half (2 m^3), proving the inner box's samples were
			// deduplicated. The buoyancy force and COB therefore match the single-box half-submerged case.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 2.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, v4{0.0f, 0.0f, 19620.0f, 0.0f}, 50.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_ws, v4{0.0f, 0.0f, -0.25f, 1.0f}, 0.005f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 15.0f));
		}

		// A fully-submerged sphere registered through the composite backend must reproduce the closed-form
		// sphere buoyancy: V = 4/3*pi*R^3, force = (0, 0, rho*|g|*V) straight up, centre of buoyancy at the
		// sphere centre, zero torque. Tolerances are looser than the box case because the sphere volume
		// sampler maps a Halton radius through pow(u, 1/3) (the oracle uses std::cbrt) so individual sample
		// positions differ slightly; with the sphere fully submerged every sample is wet, so the net volume
		// and vertical force are insensitive to that difference and only the symmetric-cancellation
		// quantities (lateral force, COB drift, torque) carry the residual noise.
		PRUnitTestMethod(GpuCompositeSphereMatchesAnalytic, Extended)
		{
			auto const radius = 1.0f;
			auto sphere = collision::ShapeSphere(radius);

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&sphere), 500.0f);
			// Submerge the whole sphere well below the flat water surface (z = 0).
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			auto const volume = (4.0f / 3.0f) * (constants<float>::tau / 2.0f) * radius * radius * radius;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * volume;

			// Volume and vertical force converge tightly (sums of equal-weight wet samples).
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, volume, volume * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
			// Lateral force cancels by symmetry; COB sits at the sphere centre; torque vanishes.
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, 0.0f, std::abs(rho_g_v) * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, std::abs(rho_g_v) * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_ws, v4{0.0f, 0.0f, -5.0f, 1.0f}, 0.02f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), std::abs(rho_g_v) * 0.02f));
		}

		// A tilted capsule through the GPU composite backend displaces its closed-form volume when fully submerged, and
		// half of it when its centre sits on the water surface, because a capsule is symmetric through its centre.
		PRUnitTestMethod(GpuCompositeCapsuleMatchesAnalytic, Extended)
		{
			auto const radius = 0.5f;
			auto const length = 3.0f;
			auto capsule = collision::ShapeLine(length, radius);
			auto const pi = constants<float>::tau / 2.0f;
			auto const volume = pi * radius * radius * length + (4.0f / 3.0f) * pi * radius * radius * radius;
			auto const axis = Normalise(v4{1.0f, 1.0f, 0.0f, 0.0f});

			Harness h;
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_volume_spacing = 0.07f,
				.m_enable_diagnostics = true,
			});
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&capsule), 500.0f);
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			// Fully submerged: every sample is wet, so volume, lift and centre of buoyancy are exact up to sampling noise.
			{
				h.m_bodies[0].O2W(m4x4::Transform(axis, 0.6f, v4{0.0f, 0.0f, -5.0f, 1.0f}));
				h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
				h.m_buoyancy.CompleteStep();

				auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
				auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * volume;
				PR_EXPECT(diag.m_valid);
				PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, volume, volume * 0.01f));
				PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
				PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_ws, v4{0.0f, 0.0f, -5.0f, 1.0f}, 0.03f));
			}

			// Centred on the surface: the point symmetry of the capsule puts exactly half its volume below the water.
			{
				h.m_bodies[0].O2W(m4x4::Transform(axis, 0.6f, v4::Origin()));
				h.m_bodies[0].VelocityWS(v4::Zero(), v4::Zero());
				h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
				h.m_buoyancy.CompleteStep();

				auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
				PR_EXPECT(diag.m_valid);
				PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 0.5f * volume, volume * 0.02f));
				PR_EXPECT(diag.m_centre_buoyancy_ws.z < 0.0f);
			}
		}

		// GPU-vs-oracle parity with drag active. A fully-submerged box translating and yawing exercises
		// volume-based linear damping plus normal and tangential quadratic surface drag. The CPU sampler
		// is the deterministic reference oracle: fed the same stable hull id (0), the same volume and surface
		// spacing, the same flat water frame and the same body state, it walks the identical hash and cull,
		// so the GPU combined force/torque must match the oracle's buoyancy+drag sum to within single-precision
		// sampling noise. This validates both sampled integration passes, not just buoyancy. The GPU diagnostic
		// force/torque are the combined buoyancy and drag values, which we compare against the oracle sum.
		PRUnitTestMethod(GpuCompositeMatchesOracleWithDrag, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto const o2w = m4x4::Translation(0.0f, 0.0f, -5.0f);
			auto const vel_lin = v4{1.5f, 0.0f, 0.0f, 0.0f};
			auto const omega = v4{0.0f, 0.0f, 0.8f, 0.0f};

			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(o2w);
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			// Start-of-step velocity drives the drag pass; capture it for the oracle before stepping.
			h.m_bodies[0].VelocityWS(omega, vel_lin);

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			// Use the runtime configuration rather than duplicating its defaults in the oracle.
			auto const config = h.m_buoyancy.GetConfig();

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Match volume spacing/hash, surface spacing, flat water, and the dispatch-time body state.
			auto oracle_body = buoyancy::BodyState{
				.m_o2w = o2w,
				.m_gravity_ws = AnalyticGravityWS,
				.m_vel_lin_ws = vel_lin,
				.m_omega_ws = omega,
			};
			auto oracle_cfg = buoyancy::SamplerConfig{
				.m_fluid_density = config.m_fluid_density,
				.m_linear_drag_time_constant_s = config.m_linear_drag_time_constant_s,
				.m_angular_drag_time_constant_s = config.m_angular_drag_time_constant_s,
				.m_quadratic_drag_coefficient = config.m_quadratic_drag_coefficient,
				.m_tangential_drag_coefficient = config.m_tangential_drag_coefficient,
				.m_surface_spacing = config.m_surface_spacing,
				.m_volume_spacing = config.m_volume_spacing,
			};
			auto const frame = buoyancy::WaterFrame{};
			auto const field = FlatField{};
			auto const oracle = buoyancy::SampleHull(collision::shape_cast(box), 0, oracle_body, frame, field, oracle_cfg);
			PR_EXPECT(oracle.m_valid);

			auto const expected_force = oracle.m_buoyancy_force_ws + oracle.m_drag_force_ws;
			auto const expected_torque = oracle.m_buoyancy_torque_ws + oracle.m_drag_torque_ws;

			// Volume matches tightly (fully submerged, equal-weight samples). Force/torque carry sampling
			// noise from the sampled drag integrals, so use a small tolerance about the oracle magnitude.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, oracle.m_volume_m3, oracle.m_volume_m3 * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, expected_force, std::max(Length(expected_force) * 0.02f, 5.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, expected_torque, std::max(Length(expected_torque) * 0.05f, 5.0f)));
		}

		// Exercise shared emission above one reduction block's capacity, including transformed, partially wet compounds.
		PRUnitTestMethod(GpuSurfaceStreamingAndCompoundParity, Extended)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.0f);
			v4 points[] = {v4{1,0,0,1}, v4{-1,0,0,1}, v4{0,1,0,1}, v4{0,-1,0,1}, v4{0,0,1,1}, v4{0,0,-1,1}};
			auto poly_buffer = collision::BuildPolytopeFromPoints(points, m4x4::Identity(), 0, collision::Shape::EFlags::None, 5);
			auto const& poly = poly_buffer.as<collision::ShapePolytope>();
			auto builder = ShapeBuilder{};
			builder.AddShape(collision::ShapeBox(v4{2,1,1,0}, m4x4::Translation(-0.5f, 0, 0)));
			builder.AddShape(collision::ShapeSphere(0.7f, m4x4::Translation(0.5f, 0, 0)));
			builder.AddShape(collision::ShapeTriangle(v4{-1,0,-0.7f,1}, v4{1,0,-0.7f,1}, v4{0,1,-0.7f,1}));
			auto compound_data = byte_data<16>{};
			auto mass_properties = MassProperties{};
			auto model_to_com = v4::Zero();
			auto const* compound = builder.BuildShape(compound_data, mass_properties, model_to_com);
			collision::Shape const* shapes[] = {&box.m_base, &sphere.m_base, &poly.m_base, compound};
			PR_EXPECT(surface::BuildPlan(box.m_base, 0.035f).m_count > 32768);

			// Run one dynamic target at a time so contact forces cannot contaminate surface diagnostics.
			for (auto const* shape : shapes)
			{
				for (auto level_offset : {-3.0f, 0.13f})
				{
					Harness h;
					auto config = h.m_buoyancy.GetConfig();
					config.m_surface_spacing = 0.035f;
					h.m_buoyancy.SetConfig(config);
					auto const o2w = m4x4::Transform(v4::YAxis(), 0.23f, v4{0.3f, -0.2f, level_offset, 1});
					auto const velocity = v4{1.1f, -0.4f, 0.3f, 0};
					auto const omega = v4{0.2f, -0.3f, 0.4f, 0};
					h.m_bodies.emplace_back();
					auto& body = h.m_bodies[0];
					body.Shape(shape, 500.0f);
					body.O2W(o2w);
					body.GravityWS(AnalyticGravityWS);
					body.VelocityWS(omega, velocity);
					auto registration = h.m_buoyancy.RegisterCompositeHull(body, 0, 0);
					auto const oracle_body = buoyancy::BodyState{
						.m_o2w = o2w,
						.m_centre_of_mass_os = body.CentreOfMassOS(),
						.m_gravity_ws = AnalyticGravityWS,
						.m_vel_lin_ws = velocity,
						.m_omega_ws = omega,
					};
					auto const oracle_cfg = buoyancy::SamplerConfig{
						.m_fluid_density = config.m_fluid_density,
						.m_linear_drag_time_constant_s = config.m_linear_drag_time_constant_s,
						.m_angular_drag_time_constant_s = config.m_angular_drag_time_constant_s,
						.m_quadratic_drag_coefficient = config.m_quadratic_drag_coefficient,
						.m_tangential_drag_coefficient = config.m_tangential_drag_coefficient,
						.m_surface_spacing = config.m_surface_spacing,
						.m_volume_spacing = config.m_volume_spacing,
					};
					auto const oracle = buoyancy::SampleHull(*shape, 0, oracle_body, buoyancy::WaterFrame{}, FlatField{}, oracle_cfg);
					h.m_engine.Step(0.0001f, std::span{h.m_bodies});
					h.m_buoyancy.CompleteStep();
					auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
					auto const force = oracle.m_buoyancy_force_ws + oracle.m_drag_force_ws;
					auto const torque = oracle.m_buoyancy_torque_ws + oracle.m_drag_torque_ws;
					PR_EXPECT(diag.m_valid);
					PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, oracle.m_volume_m3, 0.002f));
					PR_EXPECT(FEqlAbsolute(diag.m_force_ws, force, std::max(Length(force) * 0.001f, 0.1f)));
					PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, torque, std::max(Length(torque) * 0.003f, 0.2f)));
				}
			}
		}

		// Wet-volume linear damping should produce the same acceleration for equal-density geometrically
		// similar bodies because their mass and damping force both scale with volume.
		PRUnitTestMethod(LinearDragTimeConstantIsScaleIndependent, Extended)
		{
			auto const frame = buoyancy::WaterFrame{};
			auto const field = FlatField{};
			auto const body = buoyancy::BodyState{
				.m_o2w = m4x4::Translation(0.0f, 0.0f, -10.0f),
				.m_gravity_ws = AnalyticGravityWS,
				.m_vel_lin_ws = v4{3.0f, -1.0f, 0.5f, 0.0f},
			};
			auto const config = buoyancy::SamplerConfig{
				.m_fluid_density = 1000.0f,
				.m_linear_drag_time_constant_s = 2.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_volume_spacing = 0.0625f, // exact in binary, so each box gets 4096 samples with an exactly representable weight
			};

			auto const drag_per_volume = [&](float scale)
			{
				// Scale the sample spacing with the body so every size sees the same, geometrically similar sample pattern.
				auto scaled = config;
				scaled.m_volume_spacing *= scale;
				auto const box = collision::ShapeBox(v4{scale, scale, scale, 0.0f});
				auto const result = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, scaled);
				return result.m_drag_force_ws / result.m_volume_m3;
			};

			auto const small_drag = drag_per_volume(0.5f);
			auto const medium_drag = drag_per_volume(1.0f);
			auto const large_drag = drag_per_volume(2.0f);
			auto const expected = v4{-1500.0f, 500.0f, -250.0f, 0.0f};
			PR_EXPECT(FEqlRelative(small_drag, medium_drag, 1.0e-5f));
			PR_EXPECT(FEqlRelative(large_drag, medium_drag, 1.0e-5f));
			PR_EXPECT(FEqlRelative(medium_drag, expected, 1.0e-5f));
		}

		// Rotational volume drag integrates the sampled lever arms independently of translational damping.
		// A symmetric fully submerged box therefore receives negligible net force and the closed-form
		// opposing torque -rho/tau * V*(dx^2+dy^2)/12 about its Z axis.
		PRUnitTestMethod(AngularDragTimeConstantUsesWetGeometry, Extended)
		{
			auto const dimensions = v4{2.0f, 2.0f, 1.0f, 0.0f};
			auto const box = collision::ShapeBox(dimensions);
			auto const body = buoyancy::BodyState{
				.m_o2w = m4x4::Translation(0.0f, 0.0f, -10.0f),
				.m_gravity_ws = AnalyticGravityWS,
				.m_omega_ws = v4{0.0f, 0.0f, 1.0f, 0.0f},
			};
			auto const frame = buoyancy::WaterFrame{};
			auto const field = FlatField{};
			auto config = buoyancy::SamplerConfig{
				.m_fluid_density = 1000.0f,
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 1.0f,
				.m_volume_spacing = 0.08f,
			};

			auto const result = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, config);
			auto const volume = dimensions.x * dimensions.y * dimensions.z;
			auto const polar_volume_moment = volume * (dimensions.x * dimensions.x + dimensions.y * dimensions.y) / 12.0f;
			auto const expected_torque_z = -(config.m_fluid_density / config.m_angular_drag_time_constant_s) * polar_volume_moment;
			PR_EXPECT(FEqlAbsolute(result.m_drag_force_ws, v4::Zero(), 25.0f));
			PR_EXPECT(FEqlAbsolute(result.m_drag_torque_ws.x, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(result.m_drag_torque_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlRelative(result.m_drag_torque_ws.z, expected_torque_z, 0.01f));

			// With angular damping disabled, pure rotation must not leak into the independent linear term.
			config.m_linear_drag_time_constant_s = 0.1f;
			config.m_angular_drag_time_constant_s = 0.0f;
			auto const linear_only = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, config);
			PR_EXPECT(FEqlAbsolute(linear_only.m_drag_force_ws, v4::Zero(), 1.0e-5f));
			PR_EXPECT(FEqlAbsolute(linear_only.m_drag_torque_ws, v4::Zero(), 1.0e-5f));
		}

		// A fully-submerged box under a long wave receives the configured orbital acceleration
		// integrated over displaced volume: F_x = -rho*V*A*omega^2 at phase zero.
		PRUnitTestMethod(GpuCompositeWavePressureHorizontalForce, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			// Sink the body so the entire volume is well below the wavy surface (water_level = 0).
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			// Disable all drag terms so the lateral force is purely from the wave pressure field.
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			// A long wavelength keeps cos(k*x) effectively uniform across the two-metre box.
			auto const wavelength = 1000.0f;
			auto const amplitude = 0.5f;
			auto const omega = 0.4f;
			auto const wave = terrain::water::SineWave(v2{1.0f, 0.0f}, amplitude, wavelength, omega);
			h.m_buoyancy.SetWaterField(terrain::water::WaterField(0.0, std::span{&wave, 1}));

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// The box is fully submerged, so the sampled volume is the full box (4 m^3) up to sampling noise.
			auto const volume = diag.m_volume_m3;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * volume;

			auto const expected_force_x = -AnalyticFluidDensity * volume * amplitude * omega * omega;
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_force_x, std::max(std::abs(expected_force_x) * 0.02f, 25.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
		}

		// Composite port of the legacy GpuLinearDragForce check, asserting the wet-volume linear-drag closed form.
		// A fully-submerged box translating at v = (1,0,0) in flat water sees a uniform relative flow at every
		// wet volume sample, so the linear drag integral collapses to F = -c_lin * V * v, where c_lin is
		// fluid_density / tau. Quadratic drag is disabled so the horizontal force is purely the linear term.
		PRUnitTestMethod(GpuCompositeLinearDragForce, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			// Uniform translation drives the damping term; capture before stepping.
			h.m_bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			// Isolate linear drag by disabling both surface-drag terms.
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			auto const config = h.m_buoyancy.GetConfig();
			auto const c_lin = config.m_fluid_density / config.m_linear_drag_time_constant_s;
			auto const expected_force_x = -c_lin * diag.m_volume_m3 * 1.0f;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_volume_m3;

			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 4.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_force_x, std::max(std::abs(expected_force_x) * 0.02f, 25.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
			// Symmetric submerged geometry + uniform translation -> zero net torque.
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 25.0f));
		}

		// The GPU volume pass must apply the angular coefficient only to rotational point velocity.
		// Symmetric fully submerged geometry cancels the force while preserving its opposing drag torque.
		PRUnitTestMethod(GpuCompositeAngularDragTorque, Extended)
		{
			auto const dimensions = v4{2.0f, 2.0f, 1.0f, 0.0f};
			auto box = collision::ShapeBox(dimensions);
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			h.m_bodies[0].VelocityWS(v4{0.0f, 0.0f, 1.0f, 0.0f}, v4::Zero());
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 1.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			auto const config = h.m_buoyancy.GetConfig();
			auto const volume = dimensions.x * dimensions.y * dimensions.z;
			auto const polar_volume_moment = volume * (dimensions.x * dimensions.x + dimensions.y * dimensions.y) / 12.0f;
			auto const expected_torque_z = -(config.m_fluid_density / config.m_angular_drag_time_constant_s) * polar_volume_moment;
			auto const rho_g_v = config.m_fluid_density * Length(AnalyticGravityWS) * volume;

			PR_EXPECT(diag.m_valid);
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, volume, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, rho_g_v * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws.x, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlRelative(diag.m_torque_ws.z, expected_torque_z, 0.02f));
		}

		// Composite port of the legacy GpuQuadraticDragLinearMotion check. A fully-submerged box moving at
		// v = (1,0,0) only sees outward-normal flow on its +X face (the -X face is leeward, the +/-Y and +/-Z
		// faces have v_n = 0). With linear drag disabled, the lateral force is purely the quadratic form drag
		// integrated over the +X face: F_x = -0.5*rho*Cd*A_front*v_n^2. A_front = (2*hy)*(2*hz) = 2 m^2 and
		// v_n = 1 m/s, giving F_x = -0.5*1000*1.05*2*1 = -1050 N. This closed form is independent of the
		// surface sampler's distribution because every +X sample's dA sums exactly to the face area.
		PRUnitTestMethod(GpuCompositeQuadraticDragLinearMotion, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			h.m_bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			// Other drag terms are disabled so the lateral force is purely normal quadratic form drag.
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			auto const config = h.m_buoyancy.GetConfig();
			auto const hy = 1.0f;
			auto const hz = 0.5f;
			auto const front_face_area = (2.0f * hy) * (2.0f * hz);
			auto const v_n = 1.0f;
			auto const expected_drag_x = -0.5f * config.m_fluid_density * config.m_quadratic_drag_coefficient * front_face_area * v_n * v_n;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_volume_m3;

			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 4.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_drag_x, std::max(std::abs(expected_drag_x) * 0.02f, 25.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
			// Symmetric submerged geometry + uniform translation -> zero net torque.
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 25.0f));
		}

		// Tangential surface drag acts on faces parallel to the motion. For this 2x2x1 box moving along
		// +X, the +/-Y and +/-Z faces have 12 m^2 total area and unit tangential speed. With normal and
		// linear drag disabled, F_x = -0.5*rho*Ct*A_tangent*|v_t|*v_t = -300 N for Ct=0.05.
		PRUnitTestMethod(GpuCompositeTangentialDragLinearMotion, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			h.m_bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.05f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			auto const tangent_area = 12.0f;
			auto const expected_drag_x =
				-0.5f *
				h.m_buoyancy.GetConfig().m_fluid_density *
				h.m_buoyancy.GetConfig().m_tangential_drag_coefficient *
				tangent_area;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_volume_m3;

			PR_EXPECT(diag.m_valid);
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 4.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_drag_x, std::max(std::abs(expected_drag_x) * 0.02f, 25.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, 25.0f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 25.0f));
		}

		// A quadratic drag impulse is limited at the minimum-relative-energy point so an explicit step
		// cannot reverse the body's velocity and turn nominal damping into an energy source.
		PRUnitTestMethod(GpuCompositeTangentialDragDoesNotOvershoot, Extended)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			h.m_bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			h.m_bodies[0].NeverSleep(true);
			h.m_bodies[0].GravityWS(AnalyticGravityWS);
			h.m_bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 10.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto reg = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);
			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const velocity = h.m_bodies[0].VelocityWS().lin;
			PR_EXPECT(velocity.x >= -1e-4f);
			PR_EXPECT(velocity.x <= 0.05f);
		}

		// Rigid and articulation registrations share one dispatch while resolving the ordinary-body prefix and hidden-proxy suffix independently.
		PRUnitTestMethod(GpuCompositeMixedRigidAndArticulationLinks, Extended)
		{
			auto const dimensions = v4{1.0f, 1.0f, 1.0f, 0.0f};
			auto box = collision::ShapeBox(dimensions);
			Harness h;
			h.m_bodies.emplace_back();
			h.m_bodies[0].Shape(collision::shape_cast(&box), 100.0f);
			h.m_bodies[0].O2W(m4x4::Translation(2.0f, 0.0f, -0.25f));
			h.m_bodies[0].GravityWS(AnalyticGravityWS);

			auto link_desc = ArticulationLinkDesc{
				.m_inertia = Inertia::Box(dimensions, 100.0f),
				.m_shape = collision::shape_cast(&box),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Translation(0.0f, 0.0f, -0.25f));
			auto articulation = builder.Build();
			articulation.GravityWS(root, AnalyticGravityWS);
			auto forest = std::array{&articulation};
			h.m_buoyancy.SetConfig(GpuBuoyancy::Config{
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_volume_spacing = HarnessVolumeSpacing,
				.m_enable_diagnostics = true,
			});

			auto rigid_registration = h.m_buoyancy.RegisterCompositeHull(h.m_bodies[0], 0, 0);

			// Releasing one of several link registrations must not restore the shared tree policy early.
			{
				auto first_sleep_owner = h.m_buoyancy.RegisterCompositeHull(articulation, root, 2, 0);
				auto second_sleep_owner = h.m_buoyancy.RegisterCompositeHull(articulation, root, 3, 0);
				first_sleep_owner.Reset();
				PR_EXPECT(articulation.NeverSleep());
				second_sleep_owner.Reset();
				PR_EXPECT(!articulation.NeverSleep());
			}

			auto link_registration = h.m_buoyancy.RegisterCompositeHull(articulation, root, 1, 0);
			PR_EXPECT(articulation.NeverSleep());
			auto bodies = std::array<RigidBody*, 1>{&h.m_bodies[0]};

			// Both mostly submerged low-density targets receive more upward buoyancy than downward gravity.
			h.m_engine.Step(Engine::StepInput{
				.m_bodies = std::span{bodies},
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			h.m_buoyancy.CompleteStep();

			auto const rigid_diagnostic = h.m_buoyancy.LatestDiagnostics(0, 0);
			auto const link_diagnostic = h.m_buoyancy.LatestDiagnostics(1, 0);
			PR_EXPECT(rigid_diagnostic.m_valid);
			PR_EXPECT(link_diagnostic.m_valid);
			PR_EXPECT(rigid_diagnostic.m_volume_m3 > 0.70f);
			PR_EXPECT(link_diagnostic.m_volume_m3 > 0.70f);
			PR_EXPECT(rigid_diagnostic.m_force_ws.z > 0.0f);
			PR_EXPECT(link_diagnostic.m_force_ws.z > 0.0f);
			PR_EXPECT(h.m_bodies[0].VelocityWS().lin.z > 0.0f);
			PR_EXPECT(articulation.RootVelocity().lin.z > 0.0f);
			link_registration.Reset();
			PR_EXPECT(!articulation.NeverSleep());
		}

		// Performance benchmark for the SampledComposite dispatch. This drives 1, 10, and 100 identical
		// submerged boxes plus one heterogeneous (box + sphere) scene through a full Engine::Step +
		// CompleteStep loop and logs the median per-step wall-clock cost to the unit-test output stream.
		// There are NO hard timing assertions: GPU throughput is machine dependent, so the numbers are
		// captured purely for manual perf tracking across changes. The single PR_EXPECT only guards that
		// the dispatch actually ran (a valid diagnostic came back), so the benchmark still fails loudly if
		// the composite path regresses to producing nothing.
		PRUnitTestMethod(GpuCompositeDispatchBenchmark, Extended)
		{
			using clock = std::chrono::steady_clock;
			auto& out = pr::unittests::TestFramework::out();

			// Steps the harness 'warmup' frames (to prime GPU resources / driver state), then times
			// 'measure' frames of Step + CompleteStep individually and returns the median frame time in
			// milliseconds. CompleteStep blocks on the GPU readback fence, so each timed interval includes
			// the full GPU dispatch, not just the CPU-side enqueue.
			auto time_steps = [](Harness& h, int warmup, int measure) -> double
			{
				for (auto i = 0; i != warmup; ++i)
				{
					h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
					h.m_buoyancy.CompleteStep();
				}

				std::vector<double> samples;
				samples.reserve(measure);
				for (auto i = 0; i != measure; ++i)
				{
					auto const t0 = clock::now();
					h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
					h.m_buoyancy.CompleteStep();
					auto const t1 = clock::now();
					samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
				}

				std::sort(samples.begin(), samples.end());
				return samples.empty() ? 0.0 : samples[samples.size() / 2];
			};

			// Builds a harness of 'count' identical submerged boxes spread along X (so each one straddles the
			// flat water plane and is not removed by the dry broadphase cull), registers them all through the
			// composite backend, benchmarks the dispatch, and asserts the last body produced a diagnostic.
			auto bench_boxes = [&](int count, int warmup, int measure)
			{
				auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
				Harness h;
				std::vector<GpuBuoyancy::Registration> regs;
				regs.reserve(count);

				for (auto i = 0; i != count; ++i)
				{
					h.m_bodies.emplace_back();
					h.m_bodies[i].Shape(collision::shape_cast(&box), 500.0f);
					h.m_bodies[i].O2W(m4x4::Translation(4.0f * i, 0.0f, 0.0f));
					h.m_bodies[i].NeverSleep(true);
					h.m_bodies[i].GravityWS(AnalyticGravityWS);
				}
				for (auto i = 0; i != count; ++i)
					regs.push_back(h.m_buoyancy.RegisterCompositeHull(h.m_bodies[i], i, 0));

				auto const median_ms = time_steps(h, warmup, measure);
				out << "  [benchmark] " << count << " identical box bodies: median step " << median_ms << " ms\n";

				auto const diag = h.m_buoyancy.LatestDiagnostics(count - 1, 0);
				PR_EXPECT(diag.m_valid);
			};

			// Benchmarks a mixed scene of alternating box and sphere bodies to exercise both primitive
			// samplers in the same dispatch. Boxes and spheres carry different per-primitive sample counts,
			// so the heterogeneous case checks the dispatch handles varied sample densities together.
			auto bench_heterogeneous = [&](int count, int warmup, int measure)
			{
				auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
				auto sphere = collision::ShapeSphere(1.0f);
				Harness h;
				std::vector<GpuBuoyancy::Registration> regs;
				regs.reserve(count);

				for (auto i = 0; i != count; ++i)
				{
					h.m_bodies.emplace_back();
					if ((i & 1) == 0)
						h.m_bodies[i].Shape(collision::shape_cast(&box), 500.0f);
					else
						h.m_bodies[i].Shape(collision::shape_cast(&sphere), 400.0f);
					h.m_bodies[i].O2W(m4x4::Translation(4.0f * i, 0.0f, 0.0f));
					h.m_bodies[i].NeverSleep(true);
					h.m_bodies[i].GravityWS(AnalyticGravityWS);
				}
				for (auto i = 0; i != count; ++i)
				{
					regs.push_back(h.m_buoyancy.RegisterCompositeHull(h.m_bodies[i], i, 0));
				}

				auto const median_ms = time_steps(h, warmup, measure);
				out << "  [benchmark] " << count << " heterogeneous box/sphere bodies: median step " << median_ms << " ms\n";

				auto const diag = h.m_buoyancy.LatestDiagnostics(count - 1, 0);
				PR_EXPECT(diag.m_valid);
			};

			// Warmup and measurement counts are kept small so the benchmark stays well under a second even at
			// 100 bodies; the median over a handful of frames is stable enough for manual tracking.
			bench_boxes(1, 2, 5);
			bench_boxes(10, 2, 5);
			bench_boxes(100, 2, 5);
			bench_heterogeneous(10, 2, 5);
		}
	};

	// Measures how the buoyancy sample densities change the motion of floating bodies, relative to a dense reference.
	// Every (surface spacing, volume spacing) grid point runs the same scenarios through the real Engine and GpuBuoyancy.
	// Each scenario reduces its trajectory to behavioural metrics, and a grid point passes when every metric is within
	// 5% of the reference. The test prints the whole grid and the cheapest robust pair, and requires the defaults to pass.
	PRUnitTestClass(BuoyancySampleDensityTests)
	{
		// One behavioural measurement of a trajectory. Errors are measured relative to max(|reference|, m_floor) so a
		// near-zero reference does not demand exact agreement. Phase metrics compare the wrapped angle difference
		// against pi, and only when the reference response is large enough for its phase to be meaningful.
		struct Metric
		{
			std::string m_name;
			double m_value;
			double m_floor;
			bool m_is_phase;
			bool m_valid;
		};

		// Body state recorded after each step.
		struct Sample
		{
			double m_time;
			v4 m_pos;
			v4 m_up;
			v4 m_vel;
		};

		// Simulation settings shared by every grid point.
		static constexpr float StepSize = 1.0f / 60.0f;
		static constexpr double FlatDuration = 10.0;
		static constexpr double WaveDuration = 15.0;
		static constexpr double WaveWarmup = 5.0;
		static constexpr float WaveLength = 8.0f;
		static constexpr float BodyDensity = 500.0f;
		static constexpr double Tolerance = 0.05;
		static constexpr double LengthFloor = 0.05;
		static constexpr double AngleFloor = 5.0 * constants<double>::tau / 360.0;
		static constexpr double TimeFloor = 0.1;
		static constexpr double SpeedFloor = 0.1;

		// Return the signed angle between two phases, wrapped into [-pi, pi].
		static double WrapPhase(double a)
		{
			// Normalise via atan2 so any number of whole turns is removed.
			return std::atan2(std::sin(a), std::cos(a));
		}

		// Return the complex amplitude of 'signal' at angular frequency 'omega' over the samples in [t0, t1].
		template <typename Fn>
		static std::complex<double> Fourier(std::vector<Sample> const& samples, double t0, double t1, double omega, Fn signal)
		{
			// Project onto exp(-i*omega*t); the window holds a whole number of wave periods so the projection does not leak.
			auto sum = std::complex<double>{};
			auto mean = 0.0;
			auto count = 0;
			for (auto const& s : samples)
			{
				if (s.m_time < t0 || s.m_time > t1)
					continue;

				mean += signal(s);
				++count;
			}
			mean /= std::max(count, 1);
			for (auto const& s : samples)
			{
				if (s.m_time < t0 || s.m_time > t1)
					continue;

				sum += (signal(s) - mean) * std::polar(1.0, -omega * s.m_time);
			}
			return sum * (2.0 / std::max(count, 1));
		}

		// Return the mean of 'signal' over the samples in [t0, t1].
		template <typename Fn>
		static double Mean(std::vector<Sample> const& samples, double t0, double t1, Fn signal)
		{
			auto sum = 0.0;
			auto count = 0;
			for (auto const& s : samples)
			{
				if (s.m_time < t0 || s.m_time > t1)
					continue;

				sum += signal(s);
				++count;
			}
			return sum / std::max(count, 1);
		}

		// Return the sample nearest to time 't'.
		static Sample const& At(std::vector<Sample> const& samples, double t)
		{
			// Samples are uniformly spaced from the first step.
			auto const i = std::clamp(static_cast<int>(std::lround(t / StepSize)) - 1, 0, isize(samples) - 1);
			return samples[i];
		}

		// Signed rotation about world X, and about world Y, of a body's local up axis.
		static double RollX(Sample const& s)
		{
			return std::atan2(-s.m_up.y, s.m_up.z);
		}
		static double PitchY(Sample const& s)
		{
			return std::atan2(s.m_up.x, s.m_up.z);
		}

		// Integrated absolute deviation from 'final', normalised by the initial deviation. This is a continuous
		// settling-time measure (seconds) that does not jump when an oscillation peak crosses a threshold.
		template <typename Fn>
		static double SettleTime(std::vector<Sample> const& samples, double initial, double final, double floor, Fn signal)
		{
			auto iae = 0.0;
			for (auto const& s : samples)
				iae += std::abs(signal(s) - final) * StepSize;

			return iae / std::max(std::abs(initial - final), floor);
		}

		// Reduce a drop test (released level, 1 m above the water) to heave metrics.
		static void DropMetrics(std::vector<Metric>& out, std::string const& prefix, std::vector<Sample> const& samples, double z0)
		{
			// Equilibrium height is the mean over the final second, after the heave oscillation has decayed.
			auto const z = [](Sample const& s) { return double(s.m_pos.z); };
			auto const z_final = Mean(samples, FlatDuration - 1.0, FlatDuration, z);

			// The deepest point after water entry, and the highest rebound after that.
			auto trough = std::ranges::min_element(samples, {}, z);
			auto rebound = std::ranges::max_element(trough, samples.end(), {}, z);
			out.push_back({prefix + "drop.trough", z_final - z(*trough), LengthFloor, false, true});
			out.push_back({prefix + "drop.rebound", z(*rebound) - z_final, LengthFloor, false, true});
			out.push_back({prefix + "drop.settle", SettleTime(samples, z0, z_final, LengthFloor, z), TimeFloor, false, true});
			out.push_back({prefix + "drop.final_z", z_final, LengthFloor, false, true});
		}

		// Reduce a tilt test (released at rest, rolled 30 degrees about X) to roll metrics.
		static void TiltMetrics(std::vector<Metric>& out, std::string const& prefix, std::vector<Sample> const& samples, double roll0)
		{
			// The resting attitude is the mean over the final second.
			auto const roll_final = Mean(samples, FlatDuration - 1.0, FlatDuration, RollX);
			auto const tilt_final = Mean(samples, FlatDuration - 1.0, FlatDuration, [](Sample const& s) { return std::acos(std::clamp(double(s.m_up.z), -1.0, 1.0)); });

			// The release swings the body through its resting attitude to a peak on the opposite side.
			auto const roll_min = RollX(*std::ranges::min_element(samples, {}, RollX));
			out.push_back({prefix + "tilt.overshoot", roll_final - roll_min, AngleFloor, false, true});
			out.push_back({prefix + "tilt.settle", SettleTime(samples, roll0, roll_final, AngleFloor, RollX), TimeFloor, false, true});
			out.push_back({prefix + "tilt.final", tilt_final, AngleFloor, false, true});
		}

		// Reduce a push test (floating level with an initial 2 m/s along X) to drift and drag metrics.
		static void PushMetrics(std::vector<Metric>& out, std::string const& prefix, std::vector<Sample> const& samples)
		{
			// Early speed measures the drag impulse; final position measures the total drift.
			auto const& s1 = At(samples, 1.0);
			out.push_back({prefix + "push.x1", s1.m_pos.x, LengthFloor, false, true});
			out.push_back({prefix + "push.v1", s1.m_vel.x, SpeedFloor, false, true});
			out.push_back({prefix + "push.x_final", samples.back().m_pos.x, LengthFloor, false, true});
		}

		// Reduce a wave test to steady-state response metrics, measured over whole wave periods after a warm-up. Pitch metrics
		// are only recorded when 'has_righting' is set, because a body with no righting moment has no preferred attitude.
		static void WaveMetrics(std::vector<Metric>& out, std::string const& prefix, std::vector<Sample> const& samples, float amplitude, float omega, bool has_righting)
		{
			// Choose the latest window holding a whole number of periods.
			auto const period = constants<double>::tau / omega;
			auto const periods = std::floor((WaveDuration - WaveWarmup) / period);
			auto const t1 = WaveDuration;
			auto const t0 = t1 - periods * period;

			// Responses are compared with the water height at the body's current position so drift does not appear as phase.
			auto const k = constants<double>::tau / WaveLength;
			auto const water = Fourier(samples, t0, t1, omega, [&](Sample const& s) { return amplitude * std::sin(k * s.m_pos.x + omega * s.m_time); });
			auto const heave = Fourier(samples, t0, t1, omega, [](Sample const& s) { return double(s.m_pos.z); });
			auto const pitch = Fourier(samples, t0, t1, omega, PitchY);
			out.push_back({prefix + "heave_amp", std::abs(heave), LengthFloor, false, true});
			out.push_back({prefix + "heave_phase", WrapPhase(std::arg(heave) - std::arg(water)), constants<double>::tau_by_2, true, std::abs(heave) > LengthFloor});
			if (has_righting)
			{
				// Attitude response to the passing wave slope.
				out.push_back({prefix + "pitch_amp", std::abs(pitch), AngleFloor, false, true});
				out.push_back({prefix + "pitch_phase", WrapPhase(std::arg(pitch) - std::arg(water)), constants<double>::tau_by_2, true, std::abs(pitch) > AngleFloor});
			}

			// Mean drift velocity and mean height over the same window.
			auto const& a = At(samples, t0);
			auto const& b = At(samples, t1);
			out.push_back({prefix + "drift", (b.m_pos.x - a.m_pos.x) / (b.m_time - a.m_time), SpeedFloor, false, true});
			out.push_back({prefix + "mean_z", Mean(samples, t0, t1, [](Sample const& s) { return double(s.m_pos.z); }), LengthFloor, false, true});
		}

		// Return the largest relative error of 'metrics' against 'reference', and a list of every metric outside the tolerance.
		static std::pair<double, std::string> Compare(std::vector<Metric> const& metrics, std::vector<Metric> const& reference)
		{
			auto worst = 0.0;
			auto failures = std::string{};
			for (int i = 0; i != isize(metrics); ++i)
			{
				// Phase is only meaningful when the reference response is above its amplitude floor.
				auto const& m = metrics[i];
				auto const& r = reference[i];
				if (m.m_is_phase && !r.m_valid)
					continue;

				auto const err = m.m_is_phase
					? std::abs(WrapPhase(m.m_value - r.m_value)) / m.m_floor
					: std::abs(m.m_value - r.m_value) / std::max(std::abs(r.m_value), m.m_floor);
				if (err > Tolerance)
					failures += std::format(" {}={:.1f}%", m.m_name, 100.0 * err);

				worst = std::max(worst, err);
			}
			return {worst, failures};
		}

		PRUnitTestMethod(SampleDensitySweep, Extended | Stress)
		{
			using clock = std::chrono::steady_clock;
			auto& out = pr::unittests::TestFramework::out();

			// Floating shapes with a stable upright attitude at half the fluid density: a 1 x 1 x 0.5 m slab, a 0.5 m radius
			// sphere, a hexagonal prism polytope and an asymmetric slab + sphere compound.
			auto slab = collision::ShapeBox(v4{1.0f, 1.0f, 0.5f, 0.0f});
			auto sphere = collision::ShapeSphere(0.5f);
			auto prism_points = std::vector<v4>{};
			for (int i = 0; i != 6; ++i)
			{
				// Two hexagons of radius 0.6 m, 0.5 m apart.
				auto const a = static_cast<float>(i * constants<double>::tau / 6);
				prism_points.push_back(v4{0.6f * std::cos(a), 0.6f * std::sin(a), +0.25f, 1.0f});
				prism_points.push_back(v4{0.6f * std::cos(a), 0.6f * std::sin(a), -0.25f, 1.0f});
			}
			auto prism_data = collision::BuildPolytopeFromPoints(prism_points);
			auto builder = ShapeBuilder{};
			builder.AddShape(collision::ShapeBox(v4{1.0f, 1.0f, 0.5f, 0.0f}, m4x4::Translation(-0.5f, 0.0f, 0.0f)));
			builder.AddShape(collision::ShapeSphere(0.35f, m4x4::Translation(0.35f, 0.0f, 0.0f)));
			auto compound_data = byte_data<16>{};
			auto mass_properties = MassProperties{};
			auto model_to_com = v4::Zero();
			auto const* compound = builder.BuildShape(compound_data, mass_properties, model_to_com);
			collision::Shape const* shapes[] = {&slab.m_base, &sphere.m_base, &prism_data.as<collision::ShapePolytope>().m_base, compound};
			char const* shape_names[] = {"slab", "sphere", "prism", "compound"};

			// A sphere's buoyancy does not depend on its attitude, so it has no righting moment and its orientation drifts freely
			// under tiny torque differences. Orientation metrics are only meaningful for the other shapes.
			bool const has_righting[] = {true, false, true, true};
			auto const shape_count = isize(shapes);

			// Grid axes, densest first. The first entry of each axis is the reference. Wave amplitudes stay below the breaking
			// steepness (height/length ~ 1/7) because a sine field beyond that makes floating bodies tumble chaotically.
			static constexpr float spacings[] = {0.02f, 0.04f, 0.08f, 0.16f, 0.25f, 0.35f, 0.5f, 0.75f, 1.0f};
			static constexpr float volume_spacings[] = {0.02f, 0.025f, 0.03f, 0.04f, 0.05f, 0.06f, 0.08f, 0.1f, 0.125f, 0.16f, 0.2f, 0.25f, 0.35f, 0.5f};
			static constexpr float wave_amplitudes[] = {0.1f, 0.25f, 0.4f};
			auto const wave_omega = std::sqrt(Length(AnalyticGravityWS) * float(constants<double>::tau) / WaveLength);
			auto const roll0 = 30.0 * constants<double>::tau / 360.0;

			// Create a body per shape in a row along Y, far enough apart that bodies never touch.
			auto add_body = [](Harness& h, collision::Shape const* shape, m4x4 const& o2w, v4 vel)
			{
				auto& body = h.m_bodies.emplace_back();
				body.Shape(shape, BodyDensity, true);
				body.O2W(o2w);
				body.GravityWS(AnalyticGravityWS);
				body.VelocityWS(v4::Zero(), vel);
			};

			// Step the harness bodies for 'duration' seconds, recording every body after each step.
			auto run = [](Harness& h, double duration, std::vector<std::vector<Sample>>& traj, double& step_ms)
			{
				// Registration keeps each body awake and binds it to the current sample densities.
				auto regs = std::vector<GpuBuoyancy::Registration>{};
				for (int b = 0; b != isize(h.m_bodies); ++b)
					regs.push_back(h.m_buoyancy.RegisterCompositeHull(h.m_bodies[b], b, 0));

				traj.assign(h.m_bodies.size(), {});
				auto const steps = static_cast<int>(std::lround(duration / StepSize));
				auto const t0 = clock::now();
				for (int i = 0; i != steps; ++i)
				{
					// Gravity is a per-frame force and must be reapplied before every step.
					for (auto& body : h.m_bodies)
						body.GravityWS(AnalyticGravityWS);

					h.m_engine.Step(StepSize, std::span{h.m_bodies}, i * double(StepSize));
					h.m_buoyancy.CompleteStep();
					for (int b = 0; b != isize(h.m_bodies); ++b)
					{
						auto const& body = h.m_bodies[b];
						traj[b].push_back({(i + 1) * double(StepSize), body.O2W().pos, body.O2W().z, body.VelocityWS().lin});
					}
				}
				step_ms += std::chrono::duration<double, std::milli>(clock::now() - t0).count() / steps;
			};

			// Run every scenario at one grid point and reduce the trajectories to metrics in a fixed order.
			auto measure = [&](float spacing, float volume_spacing, double& step_ms)
			{
				auto metrics = std::vector<Metric>{};
				auto traj = std::vector<std::vector<Sample>>{};
				auto const config = GpuBuoyancy::Config{
					.m_surface_spacing = spacing,
					.m_volume_spacing = volume_spacing,
				};

				// Flat water: drop, tilt and push scenarios for every shape in one engine.
				{
					Harness h(false);
					h.m_buoyancy.SetConfig(config);
					for (int s = 0; s != shape_count; ++s)
					{
						auto const y = 10.0f * (3 * s);
						auto const half_height = collision::CalcBBox(*shapes[s]).m_radius.z;
						add_body(h, shapes[s], m4x4::Translation(0.0f, y + 0.0f, 1.0f + half_height), v4::Zero());
						add_body(h, shapes[s], m4x4::Transform(v4::XAxis(), float(roll0), v4{0.0f, y + 10.0f, 0.0f, 1.0f}), v4::Zero());
						add_body(h, shapes[s], m4x4::Translation(0.0f, y + 20.0f, 0.0f), v4{2.0f, 0.0f, 0.0f, 0.0f});
					}
					run(h, FlatDuration, traj, step_ms);
					for (int s = 0; s != shape_count; ++s)
					{
						auto const prefix = std::format("{}.", shape_names[s]);
						DropMetrics(metrics, prefix, traj[3 * s + 0], traj[3 * s + 0].front().m_pos.z);
						if (has_righting[s])
							TiltMetrics(metrics, prefix, traj[3 * s + 1], roll0);

						PushMetrics(metrics, prefix, traj[3 * s + 2]);
					}
				}

				// Travelling waves along X. Bodies share an X position so they all see the same wave phase.
				for (auto amplitude : wave_amplitudes)
				{
					Harness h(false);
					h.m_buoyancy.SetConfig(config);
					auto const wave = terrain::water::SineWave(v2{1.0f, 0.0f}, amplitude, WaveLength, wave_omega);
					h.m_buoyancy.SetWaterField(terrain::water::WaterField(0.0, std::span{&wave, 1}));
					for (int s = 0; s != shape_count; ++s)
						add_body(h, shapes[s], m4x4::Translation(0.0f, 10.0f * s, 0.0f), v4::Zero());

					run(h, WaveDuration, traj, step_ms);
					for (int s = 0; s != shape_count; ++s)
						WaveMetrics(metrics, std::format("{}.wave{:.2f}.", shape_names[s], amplitude), traj[s], amplitude, wave_omega, has_righting[s]);
				}

				step_ms /= 1 + std::size(wave_amplitudes);
				return metrics;
			};

			// Per-body sample cost of a grid point, summed over the test shapes.
			auto cost = [&](float spacing, float volume_spacing)
			{
				auto total = 0.0;
				for (auto const* shape : shapes)
				{
					auto volumes = std::vector<float>{};
					for (auto const* prim : buoyancy::CollectPrimitives(*shape))
					{
						// Each primitive contributes its own volume samples and surface cells.
						volumes.push_back(buoyancy::PrimitiveVolume(*prim));
						total += surface::BuildPlan(*prim, spacing).m_count;
					}
					for (auto n : buoyancy::VolumeSampleCounts(volumes, volume_spacing))
						total += n;
				}
				return total / shape_count;
			};

			// Measure the reference, then every grid point against it.
			auto ref_ms = 0.0;
			auto const reference = measure(spacings[0], volume_spacings[0], ref_ms);
			out << "\n  [density sweep] reference: spacing " << spacings[0] << " m, " << volume_spacings[0] << " m volume spacing, " << ref_ms << " ms/step\n";
			for (auto const& m : reference)
				out << std::format("    {:<32} {:+.5f}\n", m.m_name, m.m_value);

			constexpr int ns = int(std::size(spacings));
			constexpr int nv = int(std::size(volume_spacings));
			double error[ns][nv] = {};
			double step_ms[ns][nv] = {};
			std::string failures[ns][nv];
			for (int i = 0; i != ns; ++i)
			{
				for (int j = 0; j != nv; ++j)
				{
					auto const metrics = measure(spacings[i], volume_spacings[j], step_ms[i][j]);
					std::tie(error[i][j], failures[i][j]) = Compare(metrics, reference);
					out << std::format("    spacing {:.2f}, volume {:.3f}: max error {:6.2f}%, {:.2f} ms/step, {:.0f} samples/body; failing:{}\n",
						spacings[i], volume_spacings[j], 100.0 * error[i][j], step_ms[i][j], cost(spacings[i], volume_spacings[j]), failures[i][j]
					);
				}
			}

			// A pair is robust when it and every denser pair pass, so the choice does not rely on a lucky cancellation.
			auto robust = [&](int i, int j)
			{
				for (int a = 0; a <= i; ++a)
				{
					for (int b = 0; b <= j; ++b)
					{
						if (error[a][b] > Tolerance)
							return false;
					}
				}
				return true;
			};

			// Print the grid of maximum errors; '*' marks robust passes.
			out << "\n  [density sweep] max metric error (%) by surface spacing (rows) and volume spacing (columns); * = robust pass\n        ";
			for (auto v : volume_spacings)
				out << std::format("{:>9.3f}", v);

			out << "\n";
			auto best = std::pair<int, int>{-1, -1};
			for (int i = 0; i != ns; ++i)
			{
				out << std::format("    {:4.2f}", spacings[i]);
				for (int j = 0; j != nv; ++j)
				{
					// Track the cheapest robust pair.
					auto const ok = robust(i, j);
					out << std::format("{:>8.1f}{}", 100.0 * error[i][j], ok ? "*" : " ");
					if (ok && (best.first == -1 || cost(spacings[i], volume_spacings[j]) < cost(spacings[best.first], volume_spacings[best.second])))
						best = {i, j};
				}
				out << "\n";
			}
			PR_EXPECT(best.first != -1);
			if (best.first != -1)
			{
				out << std::format("  [density sweep] cheapest robust pair: spacing {:.2f} m, volume spacing {:.3f} m ({:.0f} samples/body vs {:.0f} reference)\n",
					spacings[best.first], volume_spacings[best.second], cost(spacings[best.first], volume_spacings[best.second]), cost(spacings[0], volume_spacings[0])
				);
			}

			// Report where the shipped defaults sit on the grid. The defaults are a cost/accuracy choice, so this is informational.
			auto const di = std::ranges::find(spacings, buoyancy::DefaultSurfaceSpacing) - std::begin(spacings);
			auto const dj = std::ranges::find(volume_spacings, buoyancy::DefaultVolumeSpacing) - std::begin(volume_spacings);
			PR_EXPECT(di != ns && dj != nv);
			if (di != ns && dj != nv)
			{
				out << std::format("  [density sweep] defaults: spacing {:.2f} m, volume spacing {:.3f} m: max error {:.2f}%, {}robust; failing:{}\n",
					spacings[di], volume_spacings[dj], 100.0 * error[di][dj], robust(int(di), int(dj)) ? "" : "not ", failures[di][dj]
				);
			}
		}
	};
}
#endif
