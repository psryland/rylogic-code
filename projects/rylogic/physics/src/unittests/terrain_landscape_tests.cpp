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
		// Packed gradient selection must preserve every corner direction, including the four repeated entries.
		PRUnitTestMethod(OriginalNoiseGradientTable, Quick)
		{
			auto expected = std::array<pr::algorithm::shared::NoiseReal3, 16>{{
				{1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0},
				{1, 0, 1}, {-1, 0, 1}, {1, 0, -1}, {-1, 0, -1},
				{0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1},
				{1, 1, 0}, {0, -1, 1}, {-1, 1, 0}, {0, -1, -1},
			}};
			for (uint32_t index = 0; index != 256; ++index)
				PR_EXPECT(All(pr::algorithm::shared::NoiseGradient(index) == expected[index & 15]));
		}

		// Pin untouched pre-interop CPU samples so both shared paths cannot silently redefine the seeded landscape.
		PRUnitTestMethod(OriginalDoublePrecisionGoldenSamples, Quick)
		{
			// Captured with VS2026 v145 Debug before extracting the evaluator; columns are seed, XY, height, d/dx, d/dy.
			struct Golden
			{
				uint32_t m_seed;
				double m_x, m_y, m_height, m_dx, m_dy;
			};
			auto const golden = std::array{
				Golden{0, 0, 0, -40.444455061792326, -0.052994628330581536, -0.18048986208123949},
				Golden{0, -1732.125, 845.75, -13.561110606069061, -0.085916236914072658, 0.14793085877910553},
				Golden{0, 4000, 0, 131.41609925570515, -0.12613237852244147, 0.098962602780610198},
				Golden{0, -1000000, 1000000, 71.495591198924842, 0.54590277077801264, -0.14456107777661231},
				Golden{42, 0, 0, -34.275864262002301, -0.023753021077289126, 0.099738091632972689},
				Golden{42, -1732.125, 845.75, -16.363474506446043, 0.028061813621000804, -0.045633719683946639},
				Golden{42, 4000, 0, 35.149761034473947, -0.12633481819337139, -0.056576791906619323},
				Golden{42, -1000000, 1000000, -16.505697581530004, 0.0022780842564344931, -0.10585267032534719},
				Golden{12648430, 0, 0, 23.157891633246322, -0.19622573987896538, -0.47081938468783108},
				Golden{12648430, -1732.125, 845.75, -3.3973069162501517, 0.08725645829141894, 0.097865920489424552},
				Golden{12648430, 4000, 0, -46.682805970167308, 0.056043354947194438, -0.1628626783636466},
				Golden{12648430, -1000000, 1000000, 189.28827921356748, -0.28064343494875588, 0.52585219023282159},
				Golden{4294967295u, 0, 0, 34.300620693872247, -0.0010110151776767384, 0.48090187529939521},
				Golden{4294967295u, -1732.125, 845.75, 1.1495370704610046, 0.0644987469096579, -0.068850701577611584},
				Golden{4294967295u, 4000, 0, 312.04196672604985, -0.90697967362907794, -0.3459226102797977},
				Golden{4294967295u, -1000000, 1000000, 230.02876254207285, -0.090663634585462782, -0.30760454344553056},
			};
			double max_height_error = 0, max_gradient_error = 0;
			for (auto const& item : golden)
			{
				auto const surface = BaselineSurface(BaselineSurfaceConfig{.m_seed = item.m_seed});
				auto const actual = surface.Sample(v2d{item.m_x, item.m_y});
				ExpectNear(actual, {.m_height = item.m_height, .m_gradient_xy = v2d{item.m_dx, item.m_dy}}, 1.0e-12);
				max_height_error = std::max(max_height_error, std::abs(actual.m_height - item.m_height));
				max_gradient_error = std::max({max_gradient_error, std::abs(actual.m_gradient_xy.x - item.m_dx), std::abs(actual.m_gradient_xy.y - item.m_dy)});
			}
			std::printf("Terrain original CPU goldens: height_error=%.17g gradient_error=%.17g\n", max_height_error, max_gradient_error);
		}

		PRUnitTestMethod(BoundedRayCastRefinesCrossingsAndReportsLimits, Quick)
		{
			// Planes provide exact reference intersections for vertical, oblique, horizontal and inside-origin rays.
			struct Plane
			{
				v2d m_gradient = v2d::Zero();
				SurfaceSample Sample(v2d xy) const
				{
					return SurfaceSample{.m_height = Dot(m_gradient, xy), .m_gradient_xy = m_gradient};
				}
			};
			auto const flat = Plane{};
			auto const origin = v4d{0, 0, 10, 1};
			auto const down = RayCast(flat, origin, v4d{0, 0, -2, 0});
			PR_EXPECT(down.m_hit && std::abs(down.m_distance - 10) <= 0.01);
			auto const oblique = RayCast(flat, origin, v4d{1, 0, -0.1, 0});
			PR_EXPECT(oblique.m_hit && std::abs(oblique.m_distance - std::sqrt(10100.0)) <= 0.01);
			auto const horizontal = RayCast(Plane{v2d{0.5, 0}}, origin, v4d::XAxis());
			PR_EXPECT(horizontal.m_hit && std::abs(horizontal.m_distance - 20) <= 0.01);
			PR_EXPECT(RayCast(flat, v4d::Origin(), v4d::ZAxis()).m_distance == 0);
			PR_EXPECT(RayCast(flat, v4d{0, 0, -1, 1}, v4d::ZAxis()).m_hit);
			PR_EXPECT(!RayCast(flat, origin, v4d::ZAxis(), {.m_max_distance = 20}).m_hit);
			PR_EXPECT(!RayCast(flat, origin, -v4d::ZAxis(), {.m_max_distance = 9}).m_hit);
			PR_EXPECT(RayCast(flat, origin, -v4d::ZAxis(), {.m_max_distance = 10}).m_hit);
			PR_THROWS(RayCast(flat, origin, v4d::Zero()), std::invalid_argument);
			PR_THROWS(RayCast(flat, origin, v4d::ZAxis(), {.m_max_step = 0}), std::invalid_argument);
			PR_THROWS(RayCast(flat, origin, v4d::ZAxis(), {.m_max_samples = 2}), std::runtime_error);

			// A narrow ridge demonstrates why sampling resolution is not a conservative intersection guarantee.
			struct Ridge
			{
				SurfaceSample Sample(v2d xy) const
				{
					auto const height = 2 * std::exp(-400 * (xy.x - 0.5) * (xy.x - 0.5));
					return SurfaceSample{.m_height = height, .m_gradient_xy = v2d{-800 * (xy.x - 0.5) * height, 0}};
				}
			};
			auto const grazing_origin = v4d{0, 0, 1, 1};
			PR_EXPECT(!RayCast(Ridge{}, grazing_origin, v4d::XAxis(), {.m_max_distance = 1, .m_max_step = 1}).m_hit);
			auto const ridge_hit = RayCast(Ridge{}, grazing_origin, v4d::XAxis(), {.m_max_distance = 1, .m_max_step = 0.01, .m_tolerance = 0.0001});
			PR_EXPECT(ridge_hit.m_hit);
			PR_EXPECT(std::abs(ridge_hit.m_distance - (0.5 - std::sqrt(std::log(2.0) / 400))) <= 0.0001);

			// Check the real procedural evaluator without replacing it with a triangulated rendering proxy.
			auto const surface = MakeSurface();
			auto const height = surface.Sample(v2d{60, 80}).m_height;
			auto const hit = RayCast(surface, v4d{60, 80, height + 50, 1}, -v4d::ZAxis());
			PR_EXPECT(hit.m_hit && std::abs(hit.m_position.z - height) <= 0.01);
			PR_EXPECT(hit.m_samples <= 32768);
		}

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

			// Finite configuration scalars can still exceed the representable noise lattice during evaluation.
			config = BaselineSurfaceConfig{};
			config.m_domain_warp.m_wavelength_m = 1e-300;
			auto const unrepresentable = BaselineSurface(config);
			PR_THROWS(unrepresentable.Sample(v2d{1, 1}), std::runtime_error);
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
