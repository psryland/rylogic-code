//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/buoyancy/gpu_buoyancy.h"
#include "src/buoyancy/buoyancy_analytical.h"

namespace pr::physics::tests
{
	void ForceLink_Buoyancy() {}

	PRUnitTestClass(BuoyancyAnalyticTests)
	{
		// Verify the CPU analytic clipped-box result for simple flat-water depths.
		PRUnitTestMethod(FlatWaterBoxVolume)
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
		PRUnitTestMethod(WaterSurfaceEvaluation)
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
		// height field. The buoyancy pass uses this gradient to compute wave-slope forces,
		// so any drift between height and gradient would manifest as a spurious horizontal
		// force on flat-water-equivalent setups.
		PRUnitTestMethod(WaterSurfaceGradient)
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

		// Verify the GPU diagnostic readback against the expected half-submerged box result.
		PRUnitTestMethod(GpuDiagnosticMatchesAnalyticBox)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[0].NeverSleep(true);
			// Give the body the same world-space gravity vector the analytic expectations assume.
			// Buoyancy now uses each body's own gravity sample, so a body with no gravity assigned
			// would produce zero hydrostatic force.
			bodies[0].GravityWS(AnalyticGravityWS);

			auto engine = Engine{};
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{},
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});
			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(diag.m_analytic_valid);
			PR_EXPECT(FEqlAbsolute(diag.m_volume_m3, 2.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_analytic_volume_m3, 2.0f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(diag.m_volume_error_m3, 0.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws, v4{0.0f, 0.0f, 19620.0f, 0.0f}, 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_error_ws, v4::Zero(), 0.01f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_ws, v4{0.0f, 0.0f, -0.25f, 1.0f}, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_error_ws, v4::Zero(), 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_error_ws, v4::Zero(), 1e-3f));

			// The body now carries its own gravity vector (so buoyancy can use a per-body local
			// gravity sample), which means the engine ALSO applies the m*g gravity force during
			// integration. Net Z force is buoyancy + m*g = 19620 + 500*(-9.81) = 14715 N.
			auto const dt = 1.0f / 60.0f;
			auto const expected_velocity = (19620.0f / 500.0f + AnalyticGravityWS.z) * dt;
			PR_EXPECT(FEqlAbsolute(bodies[0].VelocityWS().lin, v4{0.0f, 0.0f, expected_velocity, 0.0f}, 1e-4f));
		}

		// Verify non-flat water keeps GPU diagnostics valid while disabling flat-water analytic error reporting.
		PRUnitTestMethod(GpuDiagnosticDisablesAnalyticForWaves)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[0].NeverSleep(true);
			bodies[0].GravityWS(AnalyticGravityWS);

			auto engine = Engine{};
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{},
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});

			auto water = GpuBuoyancy::WaterSurface{};
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = 8.0f,
				.m_amplitude = 0.1f,
			});
			buoyancy.SetWaterSurface(water);
			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(!diag.m_analytic_valid);
			PR_EXPECT(FEqlAbsolute(diag.m_volume_error_m3, 0.0f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_error_ws, v4::Zero(), 1e-6f));
			PR_EXPECT(FEqlAbsolute(diag.m_centre_buoyancy_error_ws, v4::Zero(), 1e-6f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_error_ws, v4::Zero(), 1e-6f));
		}

		// Verify that a fully-submerged box on a uniformly-sloped water surface produces a
		// lateral hydrostatic force equal to rho*g*V*dh/dx in the down-slope direction. Uses
		// a single long-wavelength sine wave so the slope across the body is effectively
		// constant. Drag is disabled so only the new wave-slope term contributes laterally.
		PRUnitTestMethod(GpuSlopedSurfaceHorizontalForce)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			// Sink the body so the entire volume is well below the wavy surface (water_level=0).
			bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			bodies[0].NeverSleep(true);
			bodies[0].GravityWS(AnalyticGravityWS);

			auto engine = Engine{};
			// Drag disabled at construction so the body's lateral force is purely from wave slope.
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{ .m_drag_time_constant_s = 0.0f },
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});

			// Long wavelength (1000 m) means cos(k*x) is essentially 1 across the 2 m box, so
			// the surface slope is uniform and equal to A*k at the origin. Choose A so the
			// slope works out to 0.1 (10% rise per unit X) -- well within linearisation.
			auto const wavelength = 1000.0f;
			auto const slope_target = 0.1f;
			auto const k = constants<float>::tau / wavelength;
			auto const amplitude = slope_target / k;
			auto water = GpuBuoyancy::WaterSurface{};
			water.m_waves.push_back(GpuBuoyancy::SineWave{
				.m_direction = v2{1.0f, 0.0f},
				.m_wavelength = wavelength,
				.m_amplitude = amplitude,
				.m_phase_speed = 0.0f,
			});
			buoyancy.SetWaterSurface(water);

			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);

			// Fully submerged volume is the full box (4 m^3) plus the water above the top face;
			// the top face sits at z = -4.5, so an extra slab of thickness ~|water_height| is
			// added. At x=0 the slope wave goes through zero, so the average extra is ~0.
			auto const volume = diag.m_volume_m3;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * volume;

			// Expected lateral force: -rho*g*V*dh/dx, where dh/dx at the body's XY centre = slope_target.
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, -rho_g_v * slope_target, std::abs(rho_g_v) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, std::abs(rho_g_v) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 1e-3f));
		}

		// Verify the per-column linear drag scales with submerged volume and body velocity.
		// Setup: a fully-submerged box with zero angular velocity and a known linear velocity.
		// All columns share the same velocity vector at their centroid, so the total drag
		// force equals -c_drag*V*v with zero net torque (symmetric submerged geometry).
		PRUnitTestMethod(GpuLinearDragForce)		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			bodies[0].NeverSleep(true);
			bodies[0].GravityWS(AnalyticGravityWS);
			// Set v_lin = (1, 0, 0); the per-column drag should reproduce that direction.
			bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			auto engine = Engine{};
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{},
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});

			// Flat water so only drag drives the horizontal force.
			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(diag.m_analytic_valid);

			// Default drag time-constant is 3 s and default fluid density is 1000 kg/m^3, so
			// c_drag = 333.33 kg/m^3/s. Volume is the analytic full-box result (4 m^3).
			auto const config = buoyancy.GetConfig();
			auto const c_drag = config.m_fluid_density / config.m_drag_time_constant_s;
			auto const expected_force_x = -c_drag * diag.m_analytic_volume_m3 * 1.0f;

			PR_EXPECT(FEqlAbsolute(diag.m_analytic_volume_m3, 4.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_force_x, std::abs(expected_force_x) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, std::abs(expected_force_x) * 1e-3f));
			// z component is full-volume buoyancy, unchanged by drag.
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_analytic_volume_m3;
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 1e-3f));
			// Symmetric submerged geometry + linear velocity -> zero net torque.
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), std::abs(expected_force_x) * 1e-2f));
		}

		// Verify per-face quadratic (form) drag along a straight-line translation. A fully-submerged
		// box moving with v_lin = (1,0,0) only sees outward-normal velocity on its +X face: the
		// other five faces have v_n <= 0 (leeward) or v_n == 0 (top/bottom). With linear drag
		// disabled (drag_time_constant_s = 0), the lateral force is purely from the +X face's
		// 2x2 sub-sample grid, all four of which are fully submerged and share v_n = 1 m/s.
		// Each sub-sample integrates F = -0.5*rho*Cd*A_sub*v_n^2 * n_ws, summing across the four
		// sub-tiles to the same closed-form result as a single full-face evaluation. The +X face
		// area is (2*hy)*(2*hz) = 2*1*2*0.5 = 2 m^2, giving F_x = -0.5*1000*1.05*2*1 = -1050 N.
		PRUnitTestMethod(GpuQuadraticDragLinearMotion)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			bodies[0].NeverSleep(true);
			bodies[0].GravityWS(AnalyticGravityWS);
			bodies[0].VelocityWS(v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0.0f});

			auto engine = Engine{};
			// Linear drag disabled so the lateral force is purely the new quadratic form drag.
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{ .m_drag_time_constant_s = 0.0f },
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});

			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(diag.m_analytic_valid);

			auto const config = buoyancy.GetConfig();

			// Half-extents derived from RegisterBoxHull(size) which divides by 2.
			auto const hy = 1.0f;
			auto const hz = 0.5f;
			auto const front_face_area = (2.0f * hy) * (2.0f * hz);
			auto const v_n = 1.0f;
			auto const expected_drag_x = -0.5f * config.m_fluid_density * config.m_quadratic_drag_coefficient * front_face_area * v_n * v_n;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_analytic_volume_m3;

			PR_EXPECT(FEqlAbsolute(diag.m_analytic_volume_m3, 4.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, expected_drag_x, std::abs(expected_drag_x) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, std::abs(expected_drag_x) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 1e-3f));
			// Symmetric submerged geometry + linear velocity -> zero net torque.
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws, v4::Zero(), std::abs(expected_drag_x) * 1e-2f));
		}

		// Verify per-face quadratic drag generates the expected damping torque for pure yaw rotation.
		// Setup: fully-submerged box with omega = (0,0,1) rad/s about world Z. The 2x2 sub-sample
		// grid (rather than a single centroid sample per face) is what makes this test meaningful:
		// for pure yaw, the velocity omega x r at the centre of any side face is purely tangential
		// to that face, so a centroid-only model would produce zero rotational drag. With sub-samples,
		// each side face has 2 of 4 sub-samples with v_n > 0 (the two on the "leading" tangent edge),
		// and each contributes -coef*A_sub*v_n^2 of normal force.
		//
		// Per side-face contribution to torque about Z:
		//   v_n = omega_z * (sample tangent offset) = 1 * 0.5 = 0.5 m/s
		//   F_sub on +X face at y=-0.5: -coef * A_sub * v_n^2 in +x-direction
		//   Torque_z = -arm.y * F.x = -(-0.5) * F.x = 0.5 * F.x (negative because F.x < 0)
		//   Per sample: -0.5 * coef * A_sub * v_n^2
		//   Summed over 2 contributing samples per face: -coef * A_sub * v_n^2
		//   Summed over 4 side faces: -4 * coef * A_sub * v_n^2
		// where A_sub = hy*hz = 0.5 m^2, v_n = 0.5, coef = 0.5*rho*Cd, so:
		//   expected_torque_z = -4 * 0.5 * 1000 * 1.05 * 0.5 * 0.25 = -262.5 N.m
		// The X/Y torque components and the linear force cancel by symmetry between opposite faces.
		PRUnitTestMethod(GpuQuadraticDragYawMotion)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Translation(0.0f, 0.0f, -5.0f));
			bodies[0].NeverSleep(true);
			bodies[0].GravityWS(AnalyticGravityWS);
			// Pure yaw about world Z, no translation.
			bodies[0].VelocityWS(v4{0.0f, 0.0f, 1.0f, 0.0f}, v4::Zero());

			auto engine = Engine{};
			// Linear drag disabled to isolate the per-face quadratic contribution.
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
				GpuBuoyancy::Config{ .m_drag_time_constant_s = 0.0f },
				[](int stable_body_index)
				{
					return stable_body_index;
				},
				[&bodies](int stable_body_index)
				{
					auto body_state = GpuBuoyancy::BodyState{};
					if (stable_body_index < 0 || stable_body_index >= isize(bodies))
						return body_state;

					body_state.m_o2w = bodies[stable_body_index].O2W();
					body_state.m_centre_of_mass_os = bodies[stable_body_index].CentreOfMassOS();
					body_state.m_ws_gravity = bodies[stable_body_index].GravityWS();
					body_state.m_valid = true;
					return body_state;
				});

			auto registration = buoyancy.RegisterBoxHull(bodies[0], 0, 0, v4{2.0f, 2.0f, 1.0f, 0.0f});

			engine.Step(1.0f / 60.0f, std::span{bodies});
			buoyancy.CompleteStep();

			auto const diag = buoyancy.LatestDiagnostics(0, 0);
			PR_EXPECT(diag.m_valid);
			PR_EXPECT(diag.m_analytic_valid);

			auto const config = buoyancy.GetConfig();
			auto const coef = 0.5f * config.m_fluid_density * config.m_quadratic_drag_coefficient;
			// Side face sub-sample area: hu*hv for the +/-X and +/-Y faces happens to be the same
			// (hy*hz on +/-X, hx*hz on +/-Y) because hx == hy for this 2x2x1 box.
			auto const sub_sample_area = 1.0f * 0.5f;
			auto const v_n = 0.5f;
			auto const expected_torque_z = -4.0f * coef * sub_sample_area * v_n * v_n;
			auto const rho_g_v = AnalyticFluidDensity * Length(AnalyticGravityWS) * diag.m_analytic_volume_m3;

			PR_EXPECT(FEqlAbsolute(diag.m_analytic_volume_m3, 4.0f, 1e-4f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.x, 0.0f, std::abs(expected_torque_z) * 1e-2f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.y, 0.0f, std::abs(expected_torque_z) * 1e-2f));
			PR_EXPECT(FEqlAbsolute(diag.m_force_ws.z, rho_g_v, std::abs(rho_g_v) * 1e-3f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws.x, 0.0f, std::abs(expected_torque_z) * 1e-2f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws.y, 0.0f, std::abs(expected_torque_z) * 1e-2f));
			PR_EXPECT(FEqlAbsolute(diag.m_torque_ws.z, expected_torque_z, std::abs(expected_torque_z) * 1e-3f));
		}
	};
}
#endif
