//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/terrain/water/water_field.h"
#include <array>

namespace pr::physics::terrain::water::tests
{
	PRUnitTestClass(TerrainWaterFieldTests)
	{
		// A still-water field is flat, its bounds equal its level, and every sample returns the level with zero slope and flow.
		PRUnitTestMethod(FlatFieldIsLevelEverywhere, Quick)
		{
			auto const field = WaterField(2.5);
			PR_EXPECT(field.IsFlat());
			PR_EXPECT(field.MaxHeight() == 2.5 && field.MinHeight() == 2.5);
			PR_EXPECT(field.Height(v2d{1e6, -3e5}, 12.0) == 2.5);

			auto const sample = field.Sample(v2d{1.0, -3.0}, 0.5);
			PR_EXPECT(sample.m_height == 2.5);
			PR_EXPECT(All(sample.m_gradient_xy == v2d::Zero()));
			PR_EXPECT(All(field.PressureGradient(v2{1.0f, 2.0f}, 0.5f, 9.8f) == v2::Zero()));
			PR_EXPECT(All(field.Velocity(v4{0.5f, -2.0f, 0.25f, 1.0f}, 0.7f) == v4::Zero()));

			// Zero-amplitude elements remain flat.
			auto const calm = SineWave(v2{1.0f, 0.0f}, 0.0f, 4.0f, 1.0f);
			PR_EXPECT(WaterField(0.0, std::span{&calm, 1}).IsFlat());
		}

		// Element factories normalise directions, the sampled height matches the documented formula, and the bounds enclose every sample.
		PRUnitTestMethod(ElementsEvaluateDocumentedHeights, Quick)
		{
			auto const elements = std::array{
				SineWave(v2{2.0f, 0.0f}, 0.25f, 4.0f, 0.0f),
				GerstnerWave(v2{0.0f, 3.0f}, 0.1f, 8.0f, 2.0f, 0.5f),
			};
			auto const field = WaterField(3.0, elements);
			PR_EXPECT(!field.IsFlat());
			PR_EXPECT(FEqlAbsolute(Length(field.Elements()[0].position.xy), 1.0f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(field.MaxHeight(), 3.35, 1e-7));
			PR_EXPECT(FEqlAbsolute(field.MinHeight(), 2.65, 1e-7));

			// The sine crest is at x = wavelength/4; the Gerstner wave is A*sin(k*y - k*c*t).
			PR_EXPECT(FEqlAbsolute(field.Height(v2d{1.0, 0.0}, 0.0), 3.25, 1e-6));
			auto const k = constants<double>::tau / 8.0;
			auto const expected = 3.0 + 0.25 * std::sin(constants<double>::tau / 4.0 * 0.3) + 0.1 * std::sin(k * 1.7 - k * 2.0 * 0.9);
			PR_EXPECT(FEqlAbsolute(field.Height(v2d{0.3, 1.7}, 0.9), expected, 1e-5));

			// Samples over a range of positions and times never leave the conservative bounds.
			for (auto i = 0; i != 200; ++i)
			{
				auto const h = field.Height(v2d{0.37 * i, -0.11 * i}, 0.05 * i);
				PR_EXPECT(h <= field.MaxHeight() && h >= field.MinHeight());
			}
		}

		// The analytic XY gradient agrees with a central finite difference for every element type.
		PRUnitTestMethod(GradientMatchesFiniteDifference, Quick)
		{
			auto const elements = std::array{
				SineWave(v2{1.0f, 0.0f}, 0.2f, 4.0f, 1.5f),
				GerstnerWave(v2{0.0f, 1.0f}, 0.1f, 6.0f, -0.75f, 0.3f),
				RadialPacket(v2{0.5f, -0.5f}, 0.3f, 3.0f, 4.0f, 1.2f, 1.0f, 6.0f, 0.2f, 5.0f),
			};
			auto const field = WaterField(0.0, elements);
			auto const samples = std::array<std::pair<v2d, double>, 4>{
				std::pair{v2d{0.0, 0.0}, 0.0},
				std::pair{v2d{0.7, 0.4}, 0.0},
				std::pair{v2d{-1.3, 2.1}, 1.2},
				std::pair{v2d{0.25, -0.75}, 3.4},
			};
			auto const eps = 1e-3;
			for (auto const& [xy, t] : samples)
			{
				auto const analytic = field.Sample(xy, t).m_gradient_xy;
				auto const dh_dx = (field.Height(xy + v2d{eps, 0.0}, t) - field.Height(xy - v2d{eps, 0.0}, t)) / (2.0 * eps);
				auto const dh_dy = (field.Height(xy + v2d{0.0, eps}, t) - field.Height(xy - v2d{0.0, eps}, t)) / (2.0 * eps);
				PR_EXPECT(FEqlAbsolute(analytic, v2d{dh_dx, dh_dy}, 2e-3));
			}
		}

		// The lateral pressure gradient follows the configured orbital acceleration and equals the slope under deep-water dispersion.
		PRUnitTestMethod(PressureGradientFollowsOrbitalAcceleration, Quick)
		{
			auto const gravity = 9.8f;
			auto const wave = SineWave(v2{1.0f, 0.0f}, 0.6f, 20.0f, 0.5f);
			auto const field = WaterField(0.0, std::span{&wave, 1});
			auto const expected = v2{0.6f * 0.5f * 0.5f / gravity, 0.0f};
			PR_EXPECT(FEqlAbsolute(field.PressureGradient(v2::Zero(), 0.0f, gravity), expected, 1e-6f));
			PR_EXPECT(All(field.PressureGradient(v2::Zero(), 0.0f, 0.0f) == v2::Zero()));

			// With omega^2 = g*k the orbital acceleration equals the geometric slope.
			auto const k = constants<float>::tau / 20.0f;
			auto const dispersive = SineWave(v2{1.0f, 0.0f}, 0.6f, 20.0f, std::sqrt(gravity * k));
			auto const dispersive_field = WaterField(0.0, std::span{&dispersive, 1});
			auto const slope = dispersive_field.Sample(v2d::Zero(), 0.0).m_gradient_xy;
			PR_EXPECT(FEqlAbsolute(dispersive_field.PressureGradient(v2::Zero(), 0.0f, gravity), v2{static_cast<float>(slope.x), static_cast<float>(slope.y)}, 1e-6f));
		}

		// The orbital velocity is consistent with the height: at the still-water level the vertical component equals dh/dt,
		// a single wave traces a circular orbit of radius A*omega, speed decays as e^(k*z) below the level, and it does not grow above it.
		PRUnitTestMethod(VelocityMatchesHeightKinematics, Quick)
		{
			auto const elements = std::array{
				SineWave(v2{1.0f, 0.0f}, 0.2f, 4.0f, 1.5f),
				GerstnerWave(v2{0.0f, 1.0f}, 0.1f, 6.0f, 0.75f, 0.0f),
			};
			auto const field = WaterField(0.0, elements);
			auto const samples = std::array<std::pair<v2, float>, 3>{
				std::pair{v2{0.7f, 0.4f}, 0.3f},
				std::pair{v2{-1.3f, 2.1f}, 1.2f},
				std::pair{v2{0.25f, -0.75f}, 3.4f},
			};
			auto const dt = 1e-3;
			for (auto const& [xy, t] : samples)
			{
				auto const vel = field.Velocity(v4{xy.x, xy.y, 0.0f, 1.0f}, t);
				auto const xy_d = v2d{xy.x, xy.y};
				auto const dh_dt = (field.Height(xy_d, t + dt) - field.Height(xy_d, t - dt)) / (2.0 * dt);
				PR_EXPECT(FEqlAbsolute(static_cast<double>(vel.z), dh_dt, 2e-3));
				PR_EXPECT(vel.w == 0.0f);
			}

			// A single wave isolates the orbit radius and the depth attenuation.
			auto const single = SineWave(v2{1.0f, 0.0f}, 0.15f, 4.0f, 2.0f);
			auto const single_field = WaterField(0.0, std::span{&single, 1});
			auto const k = constants<float>::tau / 4.0f;
			auto const surface_vel = single_field.Velocity(v4{0.3f, 0.0f, 0.0f, 1.0f}, 0.4f);
			PR_EXPECT(FEqlAbsolute(Length(surface_vel), 0.15f * 2.0f, 1e-5f));
			auto const deep_vel = single_field.Velocity(v4{0.3f, 0.0f, -0.5f, 1.0f}, 0.4f);
			PR_EXPECT(FEqlAbsolute(Length(deep_vel), Length(surface_vel) * std::exp(k * -0.5f), 1e-5f));
			auto const above_vel = single_field.Velocity(v4{0.3f, 0.0f, 0.75f, 1.0f}, 0.4f);
			PR_EXPECT(FEqlAbsolute(above_vel, surface_vel, 1e-6f));
		}

		// Invalid inputs are rejected and a failed update leaves the field unchanged.
		PRUnitTestMethod(RejectsInvalidInput, Quick)
		{
			auto field = WaterField(1.0);
			PR_THROWS(field.Level(std::numeric_limits<double>::infinity()), std::invalid_argument);
			PR_THROWS(SineWave(v2::Zero(), 1.0f, 4.0f, 1.0f), std::invalid_argument);

			// Bad wavelength, unknown type, and too many elements.
			auto bad = SineWave(v2{1.0f, 0.0f}, 1.0f, 4.0f, 1.0f);
			bad.wave.y = 0.0f;
			PR_THROWS(field.Elements(std::span{&bad, 1}), std::invalid_argument);
			auto unknown = WaterFieldElement{};
			unknown.info.x = 99;
			PR_THROWS(field.Elements(std::span{&unknown, 1}), std::invalid_argument);
			auto const many = std::vector<WaterFieldElement>(MaxElementCount + 1, SineWave(v2{1.0f, 0.0f}, 0.1f, 4.0f, 1.0f));
			PR_THROWS(field.Elements(many), std::invalid_argument);

			PR_EXPECT(field.Level() == 1.0 && field.Elements().empty());
		}
	};
}
#endif
