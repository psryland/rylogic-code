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

		// Verify the GPU diagnostic readback against the expected half-submerged box result.
		PRUnitTestMethod(GpuDiagnosticMatchesAnalyticBox)
		{
			auto box = collision::ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			bodies[0].Shape(collision::shape_cast(&box), 500.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[0].NeverSleep(true);

			auto engine = Engine{};
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
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

			auto const expected_velocity = 19620.0f * (1.0f / 60.0f) / 500.0f;
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

			auto engine = Engine{};
			auto buoyancy = GpuBuoyancy(
				engine.Device(),
				engine,
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
	};
}
#endif
