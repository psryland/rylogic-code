//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/terrain/water/water_field.h"
#include "pr/physics/terrain/water/water_field.hlsli"
#include "pr/physics/terrain/water/water_depth.hlsli"
#include "pr/physics/terrain/water/wave_spectrum.h"
#include "pr/physics/terrain/water/wind_sea.h"
#include <algorithm>
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
				GerstnerWave(v2{0.6f, 0.8f}, 0.15f, 5.0f, 1.25f, 0.0f, 0.7f),
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
				GerstnerWave(v2{0.6f, 0.8f}, 0.15f, 5.0f, 1.25f, 0.0f, 0.5f),
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

		// The crest profile is a sine at zero sharpness. Sharper profiles keep the crest-to-trough height, raise the crest, flatten the trough,
		// keep a zero mean, and have an exact phase derivative.
		PRUnitTestMethod(CrestProfileSharpensCrests, Quick)
		{
			auto const pi = constants<float>::tau / 2.0f;
			for (auto i = 0; i != 16; ++i)
			{
				auto const phase = constants<float>::tau * i / 16.0f;
				auto const sine = shared::WaterFieldWaveProfile(phase, 0.0f);
				PR_EXPECT(FEqlAbsolute(sine.x, std::sin(phase), 1e-6f));
				PR_EXPECT(FEqlAbsolute(sine.y, std::cos(phase), 1e-6f));
			}
			for (auto r : {0.2f, 0.5f, 0.9f})
			{
				// Crest at phase pi/2 and trough at 3pi/2.
				PR_EXPECT(FEqlAbsolute(shared::WaterFieldWaveProfile(pi / 2.0f, r).x, 1.0f + r, 1e-5f));
				PR_EXPECT(FEqlAbsolute(shared::WaterFieldWaveProfile(3.0f * pi / 2.0f, r).x, -(1.0f - r), 1e-5f));

				// Zero mean, and the derivative matches a central difference.
				auto mean = 0.0;
				auto const n = 4096;
				for (auto j = 0; j != n; ++j)
				{
					auto const phase = constants<float>::tau * (j + 0.5f) / n;
					mean += shared::WaterFieldWaveProfile(phase, r).x / n;
					if (j % 64 == 0)
					{
						auto const eps = 1e-3f;
						auto const fd = (shared::WaterFieldWaveProfile(phase + eps, r).x - shared::WaterFieldWaveProfile(phase - eps, r).x) / (2.0f * eps);
						PR_EXPECT(FEqlAbsolute(shared::WaterFieldWaveProfile(phase, r).y, fd, 5e-3f * (1.0f + std::abs(fd))));
					}
				}
				PR_EXPECT(FEqlAbsolute(mean, 0.0, 1e-5));
			}

			// The field's height bound includes the raised crest.
			auto const wave = GerstnerWave(v2{1.0f, 0.0f}, 0.2f, 8.0f, 2.0f, 0.0f, 0.5f);
			auto const field = WaterField(0.0, std::span{&wave, 1});
			PR_EXPECT(FEqlAbsolute(field.MaxHeight(), 0.3, 1e-6) && FEqlAbsolute(field.MinHeight(), -0.3, 1e-6));
			PR_EXPECT(FEqlAbsolute(field.Height(v2d{2.0, 0.0}, 0.0), 0.3, 1e-5));
		}

		// Rendered Gerstner displacement pulls points towards crests, and the analytic normal matches the displaced surface's geometry.
		PRUnitTestMethod(GerstnerSurfaceSharpensCrests, Quick)
		{
			// One wave along +x has its crest at x = wavelength/4 at time zero. The steepness gives a crest slope of 0.8 of the looping limit.
			auto const amplitude = 0.1f;
			auto const wavelength = 6.0f;
			auto const k = constants<float>::tau / wavelength;
			auto const wave = GerstnerWave(v2{1.0f, 0.0f}, amplitude, wavelength, 3.0f, 0.8f / (k * amplitude));
			auto const surface = [&](float x)
			{
				// Displaced position (x, z) and analytic normal (nx, nz) of the flat-surface point 'x'.
				auto sample = shared::WaterFieldSurfaceSampleZero();
				shared::WaterFieldAccumulateSurface(wave, v2{x, 0.0f}, 0.0f, sample);
				return v4{x + sample.displacement_foam.x, sample.displacement_foam.z, sample.normal_delta.x, 1.0f + sample.normal_delta.z};
			};

			// Points either side of the crest move towards it; points either side of the trough move away from it.
			auto const crest = wavelength / 4.0f;
			auto const trough = 3.0f * wavelength / 4.0f;
			PR_EXPECT(std::abs(surface(crest - 0.3f).x - crest) < 0.3f && std::abs(surface(crest + 0.3f).x - crest) < 0.3f);
			PR_EXPECT(std::abs(surface(trough - 0.3f).x - trough) > 0.3f && std::abs(surface(trough + 0.3f).x - trough) > 0.3f);

			// The analytic normal is perpendicular to the displaced surface's tangent everywhere along the wave.
			for (auto i = 0; i != 24; ++i)
			{
				auto const x = wavelength * i / 24.0f;
				auto const eps = 1e-3f;
				auto const p = surface(x);
				auto const tangent = surface(x + eps) - surface(x - eps);
				PR_EXPECT(FEqlAbsolute(tangent.x * p.z + tangent.y * p.w, 0.0f, 1e-5f));
			}
		}

		// Gerstner steepness is unchanged for gentle waves, never folds the surface, reaches the fold limit at the breaking height,
		// and keeps the horizontal displacement within the depth so it vanishes at the shoreline.
		PRUnitTestMethod(GerstnerSteepnessRespectsFoldLimit, Quick)
		{
			PR_EXPECT(shared::WaterFieldGerstnerSteepness(0.5f, 0.0f, 0.1f, 0.1f, 100.0f) == 0.5f);
			PR_EXPECT(shared::WaterFieldGerstnerSteepness(0.5f, 0.0f, 0.0f, 0.0f, 100.0f) == 0.5f);
			PR_EXPECT(FEqlAbsolute(shared::WaterFieldGerstnerSteepness(0.5f, 0.0f, 4.0f, 0.1f, 100.0f), 0.25f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(shared::WaterFieldGerstnerSteepness(0.5f, 1.0f, 0.2f, 0.1f, 100.0f), 5.0f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(shared::WaterFieldGerstnerSteepness(0.5f, 0.5f, 0.2f, 0.1f, 100.0f), 2.75f, 1e-5f));
			PR_EXPECT(FEqlAbsolute(shared::WaterFieldGerstnerSteepness(0.5f, 1.0f, 0.2f, 0.1f, 0.3f), 3.0f, 1e-5f));
			PR_EXPECT(shared::WaterFieldGerstnerSteepness(0.5f, 1.0f, 0.2f, 0.1f, 0.0f) == 0.0f);
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

			// Crest sharpness outside its range.
			auto const blunt = GerstnerWave(v2{1.0f, 0.0f}, 0.1f, 4.0f, 1.0f, 0.0f, -0.1f);
			auto const too_sharp = GerstnerWave(v2{1.0f, 0.0f}, 0.1f, 4.0f, 1.0f, 0.0f, shared::WaterFieldMaxCrestSharpness + 0.01f);
			PR_THROWS(field.Elements(std::span{&blunt, 1}), std::invalid_argument);
			PR_THROWS(field.Elements(std::span{&too_sharp, 1}), std::invalid_argument);

			PR_EXPECT(field.Level() == 1.0 && field.Elements().empty());
		}

		// Bathymetry heights interpolate bilinearly, clamp outside the grid, and the rectangle minimum bounds every interpolated height.
		PRUnitTestMethod(BathymetryInterpolatesAndBounds, Quick)
		{
			auto const heights = std::array{
				0.0f, 2.0f, 4.0f,
				-6.0f, 8.0f, 1.0f,
			};
			auto const grid = Bathymetry(v2d{10.0, 20.0}, 5.0, 3, 2, heights);
			PR_EXPECT(grid.HeightAt(v2d{15.0, 20.0}) == 2.0);
			PR_EXPECT(grid.HeightAt(v2d{12.5, 22.5}) == 1.0);
			PR_EXPECT(grid.HeightAt(v2d{-100.0, 100.0}) == -6.0);
			PR_EXPECT(grid.MinHeight() == -6.0 && grid.MaxHeight() == 8.0);
			PR_EXPECT(grid.MinHeight(v2d{16.0, 20.0}, v2d{24.0, 21.0}) <= 1.0);
			PR_EXPECT(grid.MinHeight(v2d{16.0, 20.0}, v2d{24.0, 21.0}) >= -6.0);

			for (auto i = 0; i != 50; ++i)
			{
				auto const xy = v2d{11.0 + 0.17 * i, 20.2 + 0.09 * i};
				PR_EXPECT(grid.HeightAt(xy) >= grid.MinHeight(v2d{11.0, 20.2}, v2d{19.5, 24.8}));
			}

			PR_THROWS(Bathymetry(v2d{}, 1.0, 1, 2, std::span{heights.data(), 2}), std::invalid_argument);
			PR_THROWS(Bathymetry(v2d{}, 0.0, 3, 2, heights), std::invalid_argument);
		}

		// Waves are unchanged in deep water, limited to the breaking height in shallow water, run up the shore, and are absent over dry land.
		PRUnitTestMethod(DepthCorrectionShoalsAndBreaks, Quick)
		{
			auto const wave = GerstnerWave(v2{1.0f, 0.0f}, 0.5f, 20.0f, 5.0f, 0.0f);
			auto const flat_grid = [](float height)
			{
				// A uniform terrain height over a large area.
				auto const heights = std::array{height, height, height, height};
				return std::make_shared<Bathymetry const>(v2d{-1000.0, -1000.0}, 2000.0, 2, 2, heights);
			};

			// Deep water matches the uncorrected field.
			auto deep = WaterField(0.0, std::span{&wave, 1});
			auto const reference = deep;
			deep.TerrainHeights(flat_grid(-500.0f));
			auto const swash = static_cast<double>(shared::WaterFieldSwashRatio) * std::sqrt(2.0) * 0.5;
			PR_EXPECT(FEqlAbsolute(deep.WaveDepth(v2d{3.0, 4.0}), static_cast<float>(500.0 + swash), 1e-3f));
			for (auto i = 0; i != 20; ++i)
				PR_EXPECT(FEqlAbsolute(deep.Height(v2d{0.9 * i, 0.0}, 0.3 * i), reference.Height(v2d{0.9 * i, 0.0}, 0.3 * i), 1e-6));

			// A long wave shoals strongly in half a metre of water, so the breaking limit on the water depth plus its own swash allowance caps it.
			auto const long_wave = GerstnerWave(v2{1.0f, 0.0f}, 2.0f, 2000.0f, 56.0f, 0.0f);
			auto const long_swash = static_cast<double>(shared::WaterFieldSwashRatio) * std::sqrt(2.0) * 2.0;
			auto shallow = WaterField(0.0, std::span{&long_wave, 1});
			shallow.TerrainHeights(flat_grid(-0.5f));
			auto const limit = 0.5 * WaterField::DefaultBreakingRatio * (0.5 + long_swash);
			auto peak = 0.0;
			for (auto i = 0; i != 200; ++i)
				peak = std::max(peak, std::abs(shallow.Height(v2d{5.0 * i, 0.0}, 0.0)));

			PR_EXPECT(peak <= limit + 1e-5 && peak >= 0.9 * limit);
			PR_EXPECT(shallow.MaxHeight(v2d{-5.0, -5.0}, v2d{1000.0, 5.0}) >= peak - 1e-5);

			// Ground just above the still-water level is within the swash, so waves still reach it.
			auto shore = WaterField(0.0, std::span{&wave, 1});
			shore.TerrainHeights(flat_grid(0.5f * static_cast<float>(swash)));
			auto shore_peak = 0.0;
			for (auto i = 0; i != 200; ++i)
				shore_peak = std::max(shore_peak, shore.Height(v2d{0.1 * i, 0.0}, 0.0));

			PR_EXPECT(shore_peak > 0.0 && shore_peak <= 0.5 * WaterField::DefaultBreakingRatio * 0.5 * swash + 1e-5);
			PR_EXPECT(shore.MaxHeight(v2d{-5.0, -5.0}, v2d{5.0, 5.0}) >= shore_peak);

			// Dry land beyond the swash has no waves, and its local bound is the still-water level.
			auto dry = WaterField(0.0, std::span{&wave, 1});
			dry.TerrainHeights(flat_grid(5.0f));
			PR_EXPECT(dry.Height(v2d{5.0, 0.0}, 1.0) == 0.0);
			PR_EXPECT(dry.MaxHeight(v2d{-5.0, -5.0}, v2d{5.0, 5.0}) == 0.0);
			PR_EXPECT(dry.MinHeight(v2d{-5.0, -5.0}, v2d{5.0, 5.0}) == 0.0);
		}

		// Crests near the breaking height sharpen towards the shore sharpness, but the total still respects the breaking limit and height bounds.
		PRUnitTestMethod(ShoreSharpnessPeaksBreakingWaves, Quick)
		{
			// A long wave in half a metre of water is at its breaking height.
			auto const long_wave = GerstnerWave(v2{1.0f, 0.0f}, 2.0f, 2000.0f, 56.0f, 0.0f);
			auto const heights = std::array{-0.5f, -0.5f, -0.5f, -0.5f};
			auto const terrain = std::make_shared<Bathymetry const>(v2d{-1000.0, -1000.0}, 2000.0, 2, 2, heights);
			auto const long_swash = static_cast<double>(shared::WaterFieldSwashRatio) * std::sqrt(2.0) * 2.0;
			auto const limit = 0.5 * WaterField::DefaultBreakingRatio * (0.5 + long_swash);
			auto const extremes = [](WaterField const& field)
			{
				// Highest crest and deepest trough over one wavelength.
				auto hi = -1e30, lo = 1e30;
				for (auto i = 0; i != 2000; ++i)
				{
					auto const h = field.Height(v2d{1.0 * i, 0.0}, 0.0);
					hi = std::max(hi, h);
					lo = std::min(lo, h);
				}
				return std::pair{hi, lo};
			};

			// Without shore sharpness the sine crest and trough are symmetric.
			auto smooth = WaterField(0.0, std::span{&long_wave, 1});
			smooth.TerrainHeights(terrain);
			auto const [smooth_hi, smooth_lo] = extremes(smooth);
			PR_EXPECT(FEqlAbsolute(smooth_hi, -smooth_lo, 1e-3 * smooth_hi));

			// With shore sharpness the crest is higher than the trough is deep, and the total crest stays within the breaking limit and bounds.
			auto peaked = WaterField(0.0, std::span{&long_wave, 1});
			peaked.TerrainHeights(terrain);
			peaked.ShoreSharpness(0.8f);
			auto const [peaked_hi, peaked_lo] = extremes(peaked);
			PR_EXPECT(peaked_hi > 4.0 * -peaked_lo);
			PR_EXPECT(peaked_hi <= limit + 1e-5);
			PR_EXPECT(peaked.MaxHeight(v2d{-5.0, -5.0}, v2d{2000.0, 5.0}) >= peaked_hi - 1e-5);

			PR_THROWS(peaked.ShoreSharpness(-0.1f), std::invalid_argument);
			PR_THROWS(peaked.ShoreSharpness(shared::WaterFieldMaxCrestSharpness + 0.01f), std::invalid_argument);
		}

		// The wind-sea table follows the wind, and the spectrum produces band-tagged elements that bound the table and repeat after the repeat period.
		PRUnitTestMethod(WaveSpectrumFollowsWeather, Quick)
		{
			auto const spectrum = WaveSpectrum{};
			auto const bands = spectrum.Layout().m_bands;
			auto const count = static_cast<size_t>(spectrum.ComponentCount());
			PR_EXPECT(spectrum.ComponentCount() == 96);

			// Build tables of 81 rows per band, from calm to 40 m/s.
			constexpr int speed_count = 81;
			constexpr float speed_step = 0.5f;
			auto const sea_table = [&](float fetch)
			{
				auto table = std::vector<WindSeaTableRow>(static_cast<size_t>(bands) * speed_count);
				spectrum.SeaTable(fetch, speed_step, table);
				return table;
			};
			auto const row = [&](std::span<WindSeaTableRow const> table, int b, float wind_speed) -> WindSeaTableRow const&
			{
				return table[static_cast<size_t>(b) * speed_count + static_cast<int>(wind_speed / speed_step)];
			};
			auto const table = sea_table(10000.0f);

			// A 10 m/s wind over 10 km gives a significant wave height of about 0.6 m. Each band's variance is half the downwind amplitude squared
			// times the sum of its alignment weights.
			auto variance = 0.0;
			for (int b = 0; b != bands; ++b)
			{
				auto const& r = row(table, b, 10.0f);
				variance += 0.5 * r.x * r.x * r.w;
			}
			auto const hs = 4.0 * std::sqrt(variance);
			PR_EXPECT(hs > 0.45 && hs < 0.85);

			// Calm air makes no waves, and the crest sharpness follows CrestSharpness.
			for (int b = 0; b != bands; ++b)
			{
				PR_EXPECT(row(table, b, 0.0f).x == 0.0f);
				PR_EXPECT(row(table, b, 10.0f).z == WaveSpectrum::CrestSharpness(10.0f));
			}

			// A gale's spectrum peak is longer than every band, yet its long waves are still more closely aligned with the wind than a fresh breeze's.
			auto const ocean = sea_table(500000.0f);
			PR_EXPECT(row(ocean, bands - 1, 30.0f).y > row(ocean, bands - 1, 8.0f).y);

			// Every component has its own wavelength and direction.
			for (auto i = 0; i != spectrum.ComponentCount(); ++i)
			{
				for (auto j = i + 1; j != spectrum.ComponentCount(); ++j)
				{
					auto const& a = spectrum.Components()[i];
					auto const& b = spectrum.Components()[j];
					PR_EXPECT(a.m_wavelength != b.m_wavelength);
					PR_EXPECT(Dot(a.m_direction, b.m_direction) < 0.99999f);
				}
			}

			// Layouts must describe a non-empty range and at most MaxElementCount components, and tables need at least two rows per band.
			PR_THROWS(WaveSpectrum(WaveSpectrumLayout{.m_min_wavelength = 4.0f, .m_max_wavelength = 2.0f}), std::invalid_argument);
			PR_THROWS(WaveSpectrum(WaveSpectrumLayout{.m_bands = 13, .m_directions = 8}), std::invalid_argument);
			PR_THROWS(WaveSpectrum(WaveSpectrumLayout{.m_directions = 0}), std::invalid_argument);
			auto short_table = std::vector<WindSeaTableRow>(static_cast<size_t>(bands));
			PR_THROWS(spectrum.SeaTable(10000.0f, speed_step, short_table), std::invalid_argument);

			// Relaxing moves part way towards the targets, then reaches them.
			auto values = std::vector<float>{0.0f, 0.0f};
			auto const targets = std::vector<float>{1.0f, 2.0f};
			WaveSpectrum::Relax(values, targets, 20.0f, 20.0f);
			PR_EXPECT(FEqlAbsolute(values[1], 2.0f * (1.0f - std::exp(-1.0f)), 1e-4f));
			WaveSpectrum::Relax(values, targets, 1e4f, 20.0f);
			PR_EXPECT(FEqlAbsolute(values[1], 2.0f, 1e-6f));

			// A minimum wavelength removes the short components.
			auto all = std::vector<WaterFieldElement>(count);
			auto some = std::vector<WaterFieldElement>(count);
			auto const n_all = spectrum.Elements(table, 0.0f, all);
			auto const n_some = spectrum.Elements(table, 4.0f, some);
			PR_EXPECT(n_all > n_some && n_some > 0);
			for (auto i = 0; i != n_some; ++i)
				PR_EXPECT(some[i].wave.y >= 4.0f);

			// Each element is tagged with its band and holds the band's largest table amplitude and sharpness.
			for (auto i = 0; i != n_all; ++i)
			{
				auto const b = shared::WaterFieldWindSeaBand(all[i]);
				PR_EXPECT(b >= 0 && b < bands);
				auto max_amplitude = 0.0f;
				auto max_sharpness = 0.0f;
				for (int r = 0; r != speed_count; ++r)
				{
					max_amplitude = std::max(max_amplitude, table[static_cast<size_t>(b) * speed_count + r].x);
					max_sharpness = std::max(max_sharpness, table[static_cast<size_t>(b) * speed_count + r].z);
				}
				PR_EXPECT(all[i].wave.x == max_amplitude && all[i].timing.x == max_sharpness);
			}

			// Light winds give sine-shaped crests; stronger winds sharpen them, within the limit.
			PR_EXPECT(WaveSpectrum::CrestSharpness(0.0f) == 0.0f && WaveSpectrum::CrestSharpness(4.0f) == 0.0f);
			auto previous = 0.0f;
			for (auto wind_speed : {5.0f, 10.0f, 20.0f, 40.0f, 1000.0f})
			{
				auto const sharpness = WaveSpectrum::CrestSharpness(wind_speed);
				PR_EXPECT(sharpness > previous && sharpness <= WaveSpectrum::MaxCrestSharpness);
				previous = sharpness;
			}
			PR_THROWS(WaveSpectrum::CrestSharpness(-1.0f), std::invalid_argument);
			auto sharp_table = table;
			sharp_table[1].z = 0.95f;
			PR_THROWS(spectrum.Elements(sharp_table, 0.0f, all), std::invalid_argument);
			// The surface repeats after the repeat period, so wrapping the clock does not move it.
			auto field = WaterField(0.0, std::span{all.data(), static_cast<size_t>(n_all)});
			field.RepeatPeriod(1024.0);
			PR_EXPECT(field.LocalTime(5 * 1024.0 + 1.5) == 1.5f);
			for (auto i = 0; i != 10; ++i)
			{
				auto const xy = v2d{7.3 * i, -3.1 * i};
				auto const t = 0.7 * i;
				auto const unwrapped = WaterField(0.0, std::span{all.data(), static_cast<size_t>(n_all)});
				PR_EXPECT(FEqlAbsolute(field.Height(xy, 1024.0 * 3 + t), unwrapped.Height(xy, t), 1e-3));
			}
		}

		// A wind sea scales each element by the local wind, varies smoothly as the wind turns, and stays within the field's height bounds.
		PRUnitTestMethod(WindSeaFollowsLocalWind, Quick)
		{
			auto const spectrum = WaveSpectrum{};
			auto const bands = spectrum.Layout().m_bands;
			constexpr int speed_count = 81;
			auto table = std::vector<WindSeaTableRow>(static_cast<size_t>(bands) * speed_count);
			spectrum.SeaTable(10000.0f, 0.5f, table);
			auto elements = std::vector<WaterFieldElement>(static_cast<size_t>(spectrum.ComponentCount()));
			elements.resize(static_cast<size_t>(spectrum.Elements(table, 0.0f, elements)));

			// A 2x2 grid of 100 m cells, with every band's wind at 'wind' except node (1, 0), which has 'wind10'.
			auto const grid = WindSeaGrid{.origin = {0.0f, 0.0f}, .cell_size = 100.0f, .speed_step = 0.5f, .dims = {2, 2}, .band_count = bands, .speed_count = speed_count, .level_count = bands};
			auto const make_sea = [&](v2 wind, v2 wind10)
			{
				auto winds = std::vector<v2>(static_cast<size_t>(4 * bands), wind);
				for (int b = 0; b != bands; ++b)
					winds[static_cast<size_t>(1 * bands + b)] = wind10;

				return std::make_shared<WindSea const>(grid, table, winds);
			};

			// Downwind elements are larger than upwind elements, and the band state is the table row for the wind speed.
			auto const east = make_sea(v2{10.0f, 0.0f}, v2{10.0f, 0.0f});
			auto band_states = std::vector<WindSeaBand>(static_cast<size_t>(bands));
			east->Bands(v2{30.0f, 70.0f}, band_states);
			auto const peak = static_cast<int>(std::ranges::max_element(band_states, {}, &WindSeaBand::amplitude) - band_states.begin());
			PR_EXPECT(FEqlAbsolute(band_states[peak].amplitude, table[static_cast<size_t>(peak) * speed_count + 20].x, 1e-6f));
			PR_EXPECT(FEqlAbsolute(band_states[peak].direction.x, 1.0f, 1e-6f));
			PR_EXPECT(shared::WindSeaAmplitude(v2{1.0f, 0.0f}, band_states[peak]) > 10.0f * shared::WindSeaAmplitude(v2{-1.0f, 0.0f}, band_states[peak]));

			// Winds are blended between nodes, so the midpoint between an east and a north wind blows north-east.
			auto const turning = make_sea(v2{10.0f, 0.0f}, v2{0.0f, 10.0f});
			turning->Bands(v2{50.0f, 0.0f}, band_states);
			PR_EXPECT(FEqlAbsolute(band_states[peak].direction.x, band_states[peak].direction.y, 1e-5f));

			// The field takes local amplitudes from the wind sea; elements without a band keep their own.
			auto field = WaterField(0.0, elements);
			auto const fixed_wave = SineWave(v2{0.0f, 1.0f}, 0.05f, 30.0f, 0.2f);
			auto mixed = elements;
			mixed.push_back(fixed_wave);
			field.Elements(mixed);
			field.WindSea(east);
			auto local = std::vector<v2>(mixed.size());
			field.LocalWaves(v2d{30.0, 70.0}, local);
			PR_EXPECT(local.back().x == 0.0f && local.back().y == 0.0f);
			for (size_t i = 0; i != elements.size(); ++i)
			{
				auto const b = shared::WaterFieldWindSeaBand(elements[i]);
				east->Bands(v2{30.0f, 70.0f}, band_states);
				PR_EXPECT(FEqlAbsolute(local[i].x, shared::WindSeaAmplitude(elements[i].position.xy, band_states[b]), 1e-6f));
				PR_EXPECT(local[i].x <= elements[i].wave.x);
				PR_EXPECT(FEqlAbsolute(local[i].y, shared::WindSeaSharpness(elements[i].position.xy, band_states[b]), 1e-6f));
				PR_EXPECT(local[i].y <= band_states[b].sharpness);
			}

			// Crest sharpness is full downwind, decays quickly away from it, and is zero across the wind and upwind.
			{
				auto band = shared::WindSeaBand{.direction = v2{1.0f, 0.0f}, .amplitude = 1.0f, .exponent = 1.0f, .sharpness = 0.8f};
				auto const at = [&](float degrees)
				{
					// Sharpness of an element travelling 'degrees' from downwind.
					auto const r = degrees * 3.14159265f / 180.0f;
					return shared::WindSeaSharpness(v2{std::cos(r), std::sin(r)}, band);
				};
				PR_EXPECT(FEqlAbsolute(at(0.0f), 0.8f, 1e-6f));
				PR_EXPECT(FEqlAbsolute(at(45.0f), 0.2f, 1e-5f));
				PR_EXPECT(FEqlAbsolute(at(90.0f), 0.0f, 1e-6f));
				PR_EXPECT(at(180.0f) == 0.0f);
			}

			// Turning the wind slightly moves the surface slightly, so a smoothly turning wind never makes the surface jump.
			auto const angle = 1.0e-3f;
			auto const turned = make_sea(v2{10.0f * std::cos(angle), 10.0f * std::sin(angle)}, v2{10.0f * std::cos(angle), 10.0f * std::sin(angle)});
			auto turned_field = field;
			turned_field.WindSea(turned);
			for (int i = 0; i != 10; ++i)
			{
				auto const xy = v2d{11.0 * i, 9.0 * i};
				PR_EXPECT(std::abs(field.Height(xy, 3.0 * i) - turned_field.Height(xy, 3.0 * i)) < 1e-2);
			}

			// The height bounds enclose sampled heights.
			for (int i = 0; i != 50; ++i)
			{
				auto const xy = v2d{17.0 * i - 300.0, 13.0 * i - 200.0};
				auto const h = field.Height(xy, 1.7 * i);
				PR_EXPECT(h <= field.MaxHeight() && h >= field.MinHeight());
			}

			// With two levels, the first band takes the first level's wind, the last band the second level's, and middle bands a blend of both.
			auto const level_grid = WindSeaGrid{.origin = {0.0f, 0.0f}, .cell_size = 100.0f, .speed_step = 0.5f, .dims = {2, 2}, .band_count = bands, .speed_count = speed_count, .level_count = 2};
			auto level_winds = std::vector<v2>(8);
			for (int n = 0; n != 4; ++n)
			{
				level_winds[static_cast<size_t>(2 * n + 0)] = v2{10.0f, 0.0f};
				level_winds[static_cast<size_t>(2 * n + 1)] = v2{0.0f, 10.0f};
			}
			auto const levels = WindSea(level_grid, table, level_winds);
			levels.Bands(v2{30.0f, 70.0f}, band_states);
			PR_EXPECT(FEqlAbsolute(band_states.front().direction.x, 1.0f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(band_states.back().direction.y, 1.0f, 1e-6f));
			PR_EXPECT(band_states[bands / 2].direction.x > 0.1f && band_states[bands / 2].direction.y > 0.1f);
			PR_THROWS(WindSea(WindSeaGrid{.origin = {0.0f, 0.0f}, .cell_size = 100.0f, .speed_step = 0.5f, .dims = {2, 2}, .band_count = bands, .speed_count = speed_count, .level_count = 0}, table, std::span<v2 const>{}), std::invalid_argument);

			// A wind sea must describe every band the elements use.
			auto const narrow_grid = WindSeaGrid{.origin = {0.0f, 0.0f}, .cell_size = 100.0f, .speed_step = 0.5f, .dims = {2, 2}, .band_count = 1, .speed_count = speed_count, .level_count = 1};
			auto const narrow = std::make_shared<WindSea const>(narrow_grid, std::span{table}.first(speed_count), std::vector<v2>(4, v2{10.0f, 0.0f}));
			PR_THROWS(field.WindSea(narrow), std::invalid_argument);
			PR_THROWS(WindSea(grid, std::span{table}.first(speed_count), std::vector<v2>(static_cast<size_t>(4 * bands))), std::invalid_argument);
		}
	};
}
#endif
