//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include <future>
#include <thread>
#include "pr/common/unittests.h"
#include "pr/algorithm/perlin_noise.h"
#include "pr/physics/materials/material.h"
#include "pr/physics/terrain/landscape/landscape.h"

namespace pr::physics::terrain::landscape::tests
{
	namespace
	{
		// Return one default baseline surface used by the terrain regression tests.
		BaselineSurface MakeSurface()
		{
			return BaselineSurface(BaselineSurfaceConfig{
				.m_seed = 0x1234ABCDu,
				.m_material_id = Material::DefaultID,
			});
		}

		// Return one centered finite-difference derivative for the baseline surface height.
		v2d FiniteDifferenceGradient(BaselineSurface const& surface, v2d position_xy, double step)
		{
			auto const dx = (surface.Sample(v2d{position_xy.x + step, position_xy.y}).m_height - surface.Sample(v2d{position_xy.x - step, position_xy.y}).m_height) / (2.0 * step);
			auto const dy = (surface.Sample(v2d{position_xy.x, position_xy.y + step}).m_height - surface.Sample(v2d{position_xy.x, position_xy.y - step}).m_height) / (2.0 * step);
			return v2d{dx, dy};
		}

		// Require two terrain samples to agree within mixed absolute and relative tolerances.
		void ExpectNear(SurfaceSample const& lhs, SurfaceSample const& rhs, double tolerance)
		{
			PR_EXPECT(std::abs(lhs.m_height - rhs.m_height) <= tolerance * std::max({1.0, std::abs(lhs.m_height), std::abs(rhs.m_height)}));
			PR_EXPECT(Length(lhs.m_gradient_xy - rhs.m_gradient_xy) <= tolerance * std::max({1.0, Length(lhs.m_gradient_xy), Length(rhs.m_gradient_xy)}));
			PR_EXPECT(lhs.m_material_id == rhs.m_material_id);
		}
	}

	PRUnitTestClass(TerrainLandscapeTests)
	{
		PRUnitTestMethod(RejectsInvalidConfigAndCoordinates, Quick)
		{
			auto config = BaselineSurfaceConfig{};
			config.m_supported_coordinate_abs_m = 0.0;
			PR_THROWS(BaselineSurface{config}, std::invalid_argument);

			config = BaselineSurfaceConfig{};
			config.m_material_id = Material::MaxMaterialId;
			PR_THROWS(BaselineSurface{config}, std::invalid_argument);

			config = BaselineSurfaceConfig{};
			config.m_plains.m_wavelength_m = -1.0;
			PR_THROWS(BaselineSurface{config}, std::invalid_argument);

			config = BaselineSurfaceConfig{};
			config.m_hills.m_octave_count = BaselineSurface::MaxOctaveCount + 1;
			PR_THROWS(BaselineSurface{config}, std::invalid_argument);

			auto const surface = MakeSurface();
			PR_THROWS(surface.Sample(v2d{std::numeric_limits<double>::quiet_NaN(), 0.0}), std::invalid_argument);
			PR_THROWS(surface.Sample(v2d{surface.Config().m_supported_coordinate_abs_m + 1.0, 0.0}), std::out_of_range);
		}

		PRUnitTestMethod(DeterminismGradientsAndNormalsRemainStable, Quick)
		{
			auto const surface = MakeSurface();
			auto const positions = std::array<v2d, 6>
			{
				v2d{-4000.0, 0.0},
				v2d{-1732.125, 845.75},
				v2d{-17.0, -11.0},
				v2d{0.0, 0.0},
				v2d{1137.5, -2842.25},
				v2d{3999.5, 3999.25},
			};
			auto baseline = std::array<SurfaceSample, positions.size()>{};
			auto reordered = std::array<SurfaceSample, positions.size()>{};
			for (auto index = size_t{}; index != positions.size(); ++index)
				baseline[index] = surface.Sample(positions[index]);
			for (auto index = positions.size(); index-- != 0;)
				reordered[index] = surface.Sample(positions[index]);

			for (auto index = size_t{}; index != positions.size(); ++index)
			{
				ExpectNear(baseline[index], reordered[index], 1.0e-12);
				PR_EXPECT(IsFinite(baseline[index].m_height));
				PR_EXPECT(IsFinite(baseline[index].m_gradient_xy));
				PR_EXPECT(std::abs(Length(baseline[index].Normal()) - 1.0) < 1.0e-10);
			}
		}

		PRUnitTestMethod(BatchSamplingMatchesScalarAcrossBoundaryCases, Quick)
		{
			auto const surface = MakeSurface();
			auto const positions = std::vector<v2d>
			{
				v2d{-2048.0, -2048.0},
				v2d{-0.25, -0.25},
				v2d{-0.25, -0.25},
				v2d{0.25, -0.25},
				v2d{0.25, 0.25},
				v2d{-0.25, 0.25},
				v2d{3999.875, -3999.875},
			};
			auto batch = std::vector<SurfaceSample>(positions.size());
			surface.Sample(positions, batch);
			for (auto index = size_t{}; index != positions.size(); ++index)
				ExpectNear(batch[index], surface.Sample(positions[index]), 1.0e-12);

			auto empty_positions = std::span<v2d const>{};
			auto empty_samples = std::span<SurfaceSample>{};
			surface.Sample(empty_positions, empty_samples);

			auto wrong = std::vector<SurfaceSample>(positions.size() - 1);
			PR_THROWS(surface.Sample(positions, wrong), std::invalid_argument);
		}

		PRUnitTestMethod(PerlinAndLandscapeDerivativesMatchFiniteDifferences, Quick)
		{
			auto const noise = algorithm::PerlinNoiseGenerator(98765);
			auto const noise_sample = noise.NoiseWithDerivatives(0.73, -1.17, 2.41);
			auto const noise_step = 1.0e-5;
			auto const noise_dx = (noise.NoiseWithDerivatives(0.73 + noise_step, -1.17, 2.41).m_value - noise.NoiseWithDerivatives(0.73 - noise_step, -1.17, 2.41).m_value) / (2.0 * noise_step);
			auto const noise_dy = (noise.NoiseWithDerivatives(0.73, -1.17 + noise_step, 2.41).m_value - noise.NoiseWithDerivatives(0.73, -1.17 - noise_step, 2.41).m_value) / (2.0 * noise_step);
			PR_EXPECT(std::abs(noise_sample.m_dx - noise_dx) < 5.0e-5);
			PR_EXPECT(std::abs(noise_sample.m_dy - noise_dy) < 5.0e-5);

			auto const surface = MakeSurface();
			auto const position = v2d{137.25, -281.875};
			auto const sample = surface.Sample(position);
			auto const fd = FiniteDifferenceGradient(surface, position, 1.0e-3);
			PR_EXPECT(Length(sample.m_gradient_xy - fd) < 2.0e-2);
		}

		PRUnitTestMethod(PrecisionAndConcurrentReadsRemainValidAcrossEightKilometres, Quick)
		{
			auto const surface = MakeSurface();
			auto found_sensitive_case = false;
			for (auto const base : std::array<v2d, 4>{v2d{3999.99, -1275.4321}, v2d{-3999.98, 2111.25}, v2d{3210.123, -2710.456}, v2d{-2875.875, 3888.5}})
			{
				auto const lhs = surface.Sample(base);
				auto const rhs = surface.Sample(base + v2d{0.01, -0.01});
				if (std::abs(lhs.m_height - rhs.m_height) > 1.0e-8 || Length(lhs.m_gradient_xy - rhs.m_gradient_xy) > 1.0e-8)
				{
					found_sensitive_case = true;
					break;
				}
			}
			PR_EXPECT(found_sensitive_case);

			auto const worker_surface = std::async(std::launch::async, []{ return MakeSurface(); }).get();
			ExpectNear(surface.Sample(v2d{512.0, -768.0}), worker_surface.Sample(v2d{512.0, -768.0}), 1.0e-12);

			auto const positions = std::array<v2d, 4>{v2d{-133.0, 278.0}, v2d{0.0, 0.0}, v2d{1820.25, -442.5}, v2d{3900.0, 125.0}};
			auto const expected = std::array<SurfaceSample, 4>
			{
				surface.Sample(positions[0]),
				surface.Sample(positions[1]),
				surface.Sample(positions[2]),
				surface.Sample(positions[3]),
			};
			auto task = [&](int index)
			{
				return surface.Sample(positions[index]);
			};
			auto f0 = std::async(std::launch::async, task, 0);
			auto f1 = std::async(std::launch::async, task, 1);
			auto f2 = std::async(std::launch::async, task, 2);
			auto f3 = std::async(std::launch::async, task, 3);
			ExpectNear(f0.get(), expected[0], 1.0e-12);
			ExpectNear(f1.get(), expected[1], 1.0e-12);
			ExpectNear(f2.get(), expected[2], 1.0e-12);
			ExpectNear(f3.get(), expected[3], 1.0e-12);
		}
	};
}
#endif
