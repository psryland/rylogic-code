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

		// Verify water-surface wave normalisation, flat detection, and CPU height evaluation.
		PRUnitTestMethod(WaterSurfaceEvaluation, Extended)
		{
			auto flat = GpuBuoyancy::WaterSurface{};
			flat.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 4.0f,
				.m_amplitude = 0.0f,
			});
			PR_EXPECT(flat.IsFlat());

			auto water = GpuBuoyancy::WaterSurface{};
			water.m_level = 3.0f;
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{2.0f, 0.0f},
				.m_wavelength = 4.0f,
				.m_amplitude = 0.25f,
			});

			auto normalised = water.Normalised();
			PR_EXPECT(!normalised.IsFlat());
			PR_EXPECT(FEqlAbsolute(Length(normalised.m_waves.front().m_direction), 1.0f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(normalised.EvaluateHeight(v2{1.0f, 0.0f}, 0.0f), 3.25f, 1e-6f));
		}

		// Verify the analytic XY gradient agrees with a central finite difference of the
		// height field used by rendering and wet/dry classification.
		PRUnitTestMethod(WaterSurfaceGradient, Extended)
		{
			auto water = GpuBuoyancy::WaterSurface{};
			water.m_level = 0.0f;
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 4.0f,
				.m_amplitude = 0.2f,
				.m_phase_speed = 1.5f,
			});
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{0.0f, 1.0f},
				.m_wavelength = 6.0f,
				.m_amplitude = 0.1f,
				.m_phase_speed = -0.75f,
			});
			water = water.Normalised();

			// Flat water gradient is exactly zero.
			auto flat = GpuBuoyancy::WaterSurface{};
			flat.m_level = 2.5f;
			PR_EXPECT(FEqlAbsolute(flat.EvaluateGradient(v2{1.0f, -3.0f}, 0.5f), v2::Zero(), 1e-8f));

			// Analytic gradient agrees with a central finite difference at a handful of sample points.
			auto const samples = std::array<std::pair<v2, float>, 4>{
				std::pair{v2{0.0f, 0.0f}, 0.0f},
				std::pair{v2{0.7f, 0.4f}, 0.0f},
				std::pair{v2{-1.3f, 2.1f}, 1.2f},
				std::pair{v2{0.25f, -0.75f}, 3.4f},
			};
			auto const eps = 1e-3f;
			for (auto const& [xy, t] : samples)
			{
				auto const analytic = water.EvaluateGradient(xy, t);
				auto const dh_dx = (water.EvaluateHeight(xy + v2{eps, 0.0f}, t) - water.EvaluateHeight(xy - v2{eps, 0.0f}, t)) / (2.0f * eps);
				auto const dh_dy = (water.EvaluateHeight(xy + v2{0.0f, eps}, t) - water.EvaluateHeight(xy - v2{0.0f, eps}, t)) / (2.0f * eps);
				PR_EXPECT(FEqlAbsolute(analytic, v2{dh_dx, dh_dy}, 1e-3f));
			}
		}

		// The lateral pressure gradient follows configured orbital acceleration. It equals
		// geometric slope only when the wave obeys deep-water gravity dispersion.
		PRUnitTestMethod(WaterSurfacePressureGradient, Extended)
		{
			auto const gravity = 9.8f;
			auto water = GpuBuoyancy::WaterSurface{};
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 20.0f,
				.m_amplitude = 0.6f,
				.m_phase_speed = 0.5f,
			});

			auto const expected = v2{0.6f * 0.5f * 0.5f / gravity, 0.0f};
			PR_EXPECT(FEqlAbsolute(water.EvaluatePressureGradient(v2::Zero(), 0.0f, gravity), expected, 1e-6f));
			PR_EXPECT(FEqlAbsolute(water.EvaluatePressureGradient(v2::Zero(), 0.0f, 0.0f), v2::Zero(), 1e-8f));

			auto const k = constants<float>::tau / water.m_waves.front().m_wavelength;
			water.m_waves.front().m_phase_speed = std::sqrt(gravity * k);
			PR_EXPECT(FEqlAbsolute(
				water.EvaluatePressureGradient(v2::Zero(), 0.0f, gravity),
				water.EvaluateGradient(v2::Zero(), 0.0f),
				1e-6f));
		}

		// Verify the orbital water velocity is consistent with the height field: at the still-water
		// level the vertical component equals dh/dt (the linear kinematic free-surface condition),
		// the orbital speed decays exponentially with depth, a single wave traces a circular orbit
		// of radius A*omega, and flat water produces no flow.
		PRUnitTestMethod(WaterSurfaceVelocity, Extended)
		{
			// Flat water has no orbital flow anywhere.
			auto flat = GpuBuoyancy::WaterSurface{};
			flat.m_level = 1.5f;
			PR_EXPECT(FEqlAbsolute(flat.EvaluateVelocity(v4{0.5f, -2.0f, 0.25f, 1.0f}, 0.7f), v4::Zero(), 1e-8f));

			auto water = GpuBuoyancy::WaterSurface{};
			water.m_level = 0.0f;
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 4.0f,
				.m_amplitude = 0.2f,
				.m_phase_speed = 1.5f,
			});
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{0.0f, 1.0f},
				.m_wavelength = 6.0f,
				.m_amplitude = 0.1f,
				.m_phase_speed = -0.75f,
			});
			water = water.Normalised();

			// At the still-water level the vertical velocity equals the time derivative of the
			// height field. Compare the analytic value against a central finite difference.
			auto const samples = std::array<std::pair<v2, float>, 4>{
				std::pair{v2{0.0f, 0.0f}, 0.0f},
				std::pair{v2{0.7f, 0.4f}, 0.3f},
				std::pair{v2{-1.3f, 2.1f}, 1.2f},
				std::pair{v2{0.25f, -0.75f}, 3.4f},
			};
			auto const dt = 1e-3f;
			for (auto const& [xy, t] : samples)
			{
				auto const vel = water.EvaluateVelocity(v4{xy.x, xy.y, water.m_level, 1.0f}, t);
				auto const dh_dt = (water.EvaluateHeight(xy, t + dt) - water.EvaluateHeight(xy, t - dt)) / (2.0f * dt);
				PR_EXPECT(FEqlAbsolute(vel.z, dh_dt, 1e-3f));
				PR_EXPECT(vel.w == 0.0f);
			}

			// Use a single-wave surface so the depth attenuation factor e^(k*z) is unambiguous.
			auto single = GpuBuoyancy::WaterSurface{};
			single.m_level = 0.0f;
			single.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 4.0f,
				.m_amplitude = 0.15f,
				.m_phase_speed = 2.0f,
			});
			single = single.Normalised();

			auto const k = constants<float>::tau / single.m_waves.front().m_wavelength;
			auto const omega = single.m_waves.front().m_phase_speed;
			auto const amp = single.m_waves.front().m_amplitude;
			auto const xy = v2{0.3f, 0.0f};
			auto const t = 0.4f;
			auto const surface_vel = single.EvaluateVelocity(v4{xy.x, xy.y, 0.0f, 1.0f}, t);

			// A single deep-water wave traces a circular orbit: horizontal and vertical components
			// are in quadrature with equal envelope, so the speed is A*omega independent of phase.
			PR_EXPECT(FEqlAbsolute(Length(surface_vel), amp * omega, 1e-5f));

			// Orbital speed decays exponentially with depth below the still-water level.
			auto const depth = -0.5f;
			auto const deep_vel = single.EvaluateVelocity(v4{xy.x, xy.y, depth, 1.0f}, t);
			PR_EXPECT(FEqlAbsolute(Length(deep_vel), Length(surface_vel) * std::exp(k * depth), 1e-5f));

			// Points above the surface use the unattenuated (z = 0) velocity rather than amplifying.
			auto const above_vel = single.EvaluateVelocity(v4{xy.x, xy.y, 0.75f, 1.0f}, t);
			PR_EXPECT(FEqlAbsolute(above_vel, surface_vel, 1e-6f));
		}

	};

	// Coverage for sampled-composite hull flattening, registration lifetime, and GPU integration.
	// GPU-vs-oracle cases validate deterministic sampling as well as force and diagnostic readback.
	PRUnitTestClass(BuoyancyCompositeHostTests)
	{
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
				m_buoyancy.SetWaterSurface({});
				m_buoyancy.SetConfig(GpuBuoyancy::Config{
					.m_enable_diagnostics = enable_diagnostics,
				});
			}
		};

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

		// A collision shape type the composite model does not understand (e.g. a line) is rejected.
		PRUnitTestMethod(FlattenUnsupportedTypeThrows, Extended)
		{
			auto line = collision::ShapeLine(2.0f);
			auto threw = false;
			try { (void)buoyancy::FlattenShape(collision::shape_cast(line)); }
			catch (std::exception const&) { threw = true; }
			PR_EXPECT(threw);
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

		// Flat-water fully-dry fast path. A box positioned entirely above the z=0 water surface must
		// contribute zero buoyancy. With flat water (wave_count==0) the GPU per-sample fully-dry
		// early-out (BuoySupportAlongUp + lo >= water_level) suppresses all samples for the primitive.
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

			// Feed the same tessellated shape, transform, stable hull id, and sample budgets to the CPU
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
			};
			auto const oracle = buoyancy::SampleHull(
				collision::shape_cast(poly),
				0,
				oracle_body,
				buoyancy::WaterFrame{},
				FlatField{},
				oracle_cfg,
				8192,
				8192);
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

		// GPU-vs-oracle parity with drag active. A fully-submerged box translating and yawing exercises
		// volume-based linear damping plus normal and tangential quadratic surface drag. The CPU sampler
		// is the deterministic reference oracle: fed the same stable hull id (0), the same 8192/8192 sample
		// totals, the same flat water frame and the same body state, it walks the identical hash and cull,
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

			// The harness uses the default drag configuration: linear tau=3 s, normal Cd=1.05, tangential Ct=0.20.
			auto const config = h.m_buoyancy.GetConfig();

			h.m_engine.Step(1.0f / 60.0f, std::span{h.m_bodies});
			h.m_buoyancy.CompleteStep();

			auto const diag = h.m_buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Run the CPU oracle with the SAME hull id (0), sample totals (8192/8192), flat water frame and
			// body state. SampleHull internally distributes the totals across primitives exactly as the GPU
			// does (shared DistributeCounts), so per-primitive counts and per-sample hashes coincide.
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
			};
			auto const frame = buoyancy::WaterFrame{};
			auto const field = FlatField{};
			auto const oracle = buoyancy::SampleHull(collision::shape_cast(box), 0, oracle_body, frame, field, oracle_cfg, 8192, 8192);
			PR_EXPECT(oracle.m_valid);

			auto const expected_force = oracle.m_buoyancy_force_ws + oracle.m_drag_force_ws;
			auto const expected_torque = oracle.m_buoyancy_torque_ws + oracle.m_drag_torque_ws;

			// Volume matches tightly (fully submerged, equal-weight samples). Force/torque carry sampling
			// noise from the sampled drag integrals, so use a small tolerance about the oracle magnitude.
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, oracle.m_volume_m3, oracle.m_volume_m3 * 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, expected_force, std::max(Length(expected_force) * 0.02f, 5.0f)));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, expected_torque, std::max(Length(expected_torque) * 0.05f, 5.0f)));
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
			};

			auto const drag_per_volume = [&](float scale)
			{
				auto const box = collision::ShapeBox(v4{scale, scale, scale, 0.0f});
				auto const result = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, config, 8192, 0);
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
			};

			auto const result = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, config, 8192, 0);
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
			auto const linear_only = buoyancy::SampleHull(collision::shape_cast(box), 0, body, frame, field, config, 8192, 0);
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
				.m_enable_diagnostics = true,
			});

			// A long wavelength keeps cos(k*x) effectively uniform across the two-metre box.
			auto const wavelength = 1000.0f;
			auto const amplitude = 0.5f;
			auto const omega = 0.4f;
			auto water = GpuBuoyancy::WaterSurface{};
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = wavelength,
				.m_amplitude = amplitude,
				.m_phase_speed = omega,
			});
			h.m_buoyancy.SetWaterSurface(water);

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
}
#endif
