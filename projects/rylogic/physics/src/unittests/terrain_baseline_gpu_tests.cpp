// Evaluator-only CPU/GPU precision proof; no sphere or Engine dependency.
#if PR_UNITTESTS
#include "src/unittests/terrain_gpu_probe.h"

namespace pr::physics::tests
{
	using namespace terrain_probe;

	namespace
	{
		// Representative conditioning cases, not a claim about every configurable finite recipe.
		enum class ERecipe { Default, Lattice, Cancellation, HighDatum, ChangedBands };

		// Keep the accepted gates absolute, including high-altitude and near-zero references.
		struct Errors
		{
			double m_height = 0;
			double m_dx = 0;
			double m_dy = 0;
			double m_normal = 0;

			// Each normal is a vector; a componentwise threshold is not this contract.
			bool Accepted() const
			{
				return m_height <= 0.001 && m_dx <= 1e-4 && m_dy <= 1e-4 && m_normal <= 1e-4;
			}
		};

		// Measure against the CPU source and the GPU-produced normal, rejecting nonfinite and failed results explicitly.
		Errors MeasureErrors(terrain::SurfaceSample const& reference, BaselineResult const& field, BaselineResult const& normal)
		{
			if (field.m_status != 0 || normal.m_status != 0 || field.m_material_id != reference.m_material_id || normal.m_material_id != reference.m_material_id ||
				!std::isfinite(field.m_height) || !std::isfinite(field.m_dx) || !std::isfinite(field.m_dy) ||
				!std::isfinite(normal.m_height) || !std::isfinite(normal.m_dx) || !std::isfinite(normal.m_dy))
				return {INFINITY, INFINITY, INFINITY, INFINITY};

			auto expected_normal = reference.Normal();
			auto difference = terrain::v4d{normal.m_height - expected_normal.x, normal.m_dx - expected_normal.y, normal.m_dy - expected_normal.z, 0};
			return {std::abs(field.m_height - reference.m_height), std::abs(field.m_dx - reference.m_gradient_xy.x), std::abs(field.m_dy - reference.m_gradient_xy.y), Length(difference)};
		}

		// Keep config mutations deterministic and independent of observed GPU errors.
		BaselineSurfaceConfig ProbeConfig(uint32_t seed, ERecipe variant)
		{
			auto config = BaselineSurfaceConfig{.m_seed = seed, .m_material_id = 7};
			switch (variant)
			{
				case ERecipe::Default: { break; }
				case ERecipe::Lattice:
				{
					config.m_domain_warp.m_amplitude_m = 0;
					config.m_domain_warp.m_octave_count = BaselineSurface::MaxOctaveCount;
					config.m_regional_base.m_octave_count = config.m_region_selector.m_octave_count = config.m_region_uplift.m_octave_count = BaselineSurface::MaxOctaveCount;
					config.m_plains.m_octave_count = config.m_hills.m_octave_count = config.m_mountains.m_octave_count = BaselineSurface::MaxOctaveCount;
					config.m_hills.m_persistence = -0.55;
					config.m_mountains.m_roundness = 0.001;
					break;
				}
				case ERecipe::Cancellation:
				{
					config.m_sea_level_bias_m -= BaselineSurface(config).Sample({731234.5, -642198.25}).m_height;
					break;
				}
				case ERecipe::HighDatum: { config.m_sea_level_bias_m += 7000; break; }
				case ERecipe::ChangedBands:
				{
					config.m_domain_warp.m_amplitude_m = 240;
					config.m_plains.m_wavelength_m = 730;
					config.m_hills.m_wavelength_m = 375;
					config.m_hills.m_lacunarity = 1.9;
					config.m_mountains.m_roundness = 0.03;
					break;
				}
				default: { throw std::invalid_argument("Unknown terrain proof recipe"); }
			}
			return config;
		}

		// Cover every stratum of the default square, plus fine-scale and lattice-adjacent inputs at widely separated coordinates.
		std::vector<v2d> DomainPositions(BaselineRecipe const& recipe)
		{
			auto positions = std::vector<v2d>{};
			positions.reserve(2600);
			for (int y = 0; y != 33; ++y)
			{
				for (int x = 0; x != 33; ++x)
					positions.push_back({-1e6 + x * 62500.0, -1e6 + y * 62500.0});
			}

			// One deterministic jittered point in each 62.5km square avoids relying only on aligned grid phases.
			auto state = uint32_t{0x12345678};
			for (int y = 0; y != 32; ++y)
			{
				for (int x = 0; x != 32; ++x)
				{
					state = 1664525u * state + 1013904223u;
					auto u = (state + 0.5) / 4294967296.0;
					state = 1664525u * state + 1013904223u;
					auto v = (state + 0.5) / 4294967296.0;
					positions.push_back({-1e6 + (x + u) * 62500, -1e6 + (y + v) * 62500});
				}
			}

			// With zero warp these are actual detail-field boundaries, including the highest configured octave.
			for (auto const& band : recipe.m_fields)
			{
				for (int octave : {0, band.m_octave_count - 1})
				{
					auto frequency = 1 / band.m_wavelength_m;
					for (int i = 0; i != octave; ++i)
						frequency *= band.m_lacunarity;

					for (double anchor : {-900000, -250000, 0, 250000, 900000})
					{
						auto boundary = std::round(anchor * frequency) / frequency;
						for (double offset : {-1e-8, 0.0, 1e-8})
							positions.push_back({boundary + offset, -boundary - offset});
					}
				}
			}

			// Resolve cancellation and centimetre/submillimetre variation without narrowing the world-domain matrix.
			for (auto centre : {v2d{0, 0}, v2d{4000, -4000}, v2d{731234.5, -642198.25}, v2d{-999999, 999999}})
			{
				for (int i = -16; i != 17; ++i)
					positions.push_back(centre + v2d{i * 0.001, -i * 0.0005});
			}
			return positions;
		}
	}

	// Validate the canonical evaluator on hardware, without contact or throughput fixtures.
	PRUnitTestClass(TerrainBaselineGpuTests)
	{
		// A float seed is acceptable only after refinement recovers double accuracy, including subnormal and extreme inputs.
		PRUnitTestMethod(DoubleRootRefinementAcrossExponentRange, Quick)
		{
			auto& gpu = SharedTestGpu();
			auto positions = std::vector<v2d>{{std::numeric_limits<double>::denorm_min(), 0}, {std::numeric_limits<double>::min(), 0}, {std::numeric_limits<double>::max(), 0}};
			for (int exponent = -1074; exponent <= 1023; exponent += 17)
			{
				auto value = std::ldexp(1.0, exponent);
				positions.push_back({value, 0});
				positions.push_back({value * 1.987654321, 0});
			}
			auto code = CompileBaselineProbe(L"CSRootPrecisionProbe", L"cs_6_0", false, true);
			auto step = ComputeStep{};
			step.m_sig = RootSig(ERootSigFlags::ComputeOnly).U32<uint32_t>(ECBufReg::b0).SRV(ESRVReg::t0).SRV(ESRVReg::t1).UAV(EUAVReg::u0).Create(gpu, "Terrain:RootSig");
			step.m_pso = ComputePSO(step.m_sig.get(), ByteCode(code)).Create(gpu, "Terrain:RootPSO");
			auto results = SampleBaselineGpu(gpu, step, BaselineSurface().Recipe(), positions);
			double maximum_relative = 0;
			for (size_t index = 0; index != positions.size(); ++index)
			{
				auto expected = std::sqrt(positions[index].x);
				maximum_relative = std::max({maximum_relative, std::abs(results[index].m_height / expected - 1), std::abs(results[index].m_dx * expected - 1)});
				PR_EXPECT(results[index].m_status == 0 && maximum_relative <= 8 * std::numeric_limits<double>::epsilon());
			}
			auto limits = std::array<v2d, 4>{{{0, 0}, {-0.0, 0}, {-1, 0}, {std::numeric_limits<double>::infinity(), 0}}};
			auto special = SampleBaselineGpu(gpu, step, BaselineSurface().Recipe(), limits);
			PR_EXPECT(special[0].m_height == 0 && special[0].m_dx == std::numeric_limits<double>::infinity());
			PR_EXPECT(std::signbit(special[1].m_height) && special[1].m_dx == -std::numeric_limits<double>::infinity());
			PR_EXPECT(std::isnan(special[2].m_height) && std::isnan(special[2].m_dx));
			PR_EXPECT(special[3].m_height == std::numeric_limits<double>::infinity() && special[3].m_dx == 0);
			std::printf("Terrain double roots: %zu finite values, maximum relative error %.17g\n", positions.size(), maximum_relative);
		}

		// Analytic easing identities catch cancellation even when CPU/GPU terrain algorithms otherwise agree.
		PRUnitTestMethod(FloatEasingNearLatticeBoundaries, Quick)
		{
			auto& gpu = SharedTestGpu();
			auto positions = std::vector<v2d>{{0, 0}, {0.5, 0}, {1, 0}};
			for (int exponent = 1; exponent != 25; ++exponent)
			{
				auto t = std::ldexp(1.0f, -exponent);
				positions.push_back({t, 0});
				positions.push_back({1.0f - t, 0});
			}
			auto cs = CompileBaselineProbe(L"CSNoisePrecisionProbe", L"cs_6_0", false, true);
			auto step = ComputeStep{};
			step.m_sig = RootSig(ERootSigFlags::ComputeOnly).U32<uint32_t>(ECBufReg::b0).SRV(ESRVReg::t0).SRV(ESRVReg::t1).UAV(EUAVReg::u0).Create(gpu, "Terrain:EasingSig");
			step.m_pso = ComputePSO(step.m_sig.get(), ByteCode(cs)).Create(gpu, "Terrain:EasingPSO");
			auto results = SampleBaselineGpu(gpu, step, BaselineSurface().Recipe(), positions);
			for (size_t index = 0; index != positions.size(); ++index)
			{
				auto t = positions[index].x;
				auto expected = t * t * t * (10 - 15 * t + 6 * t * t);
				auto derivative = 30 * t * t * (1 - t) * (1 - t);
				PR_EXPECT(std::abs(results[index].m_height - expected) <= 2 * std::numeric_limits<float>::epsilon());
				PR_EXPECT(results[index].m_dx >= 0 && std::abs(results[index].m_dx - derivative) <= 8 * std::numeric_limits<float>::epsilon() * derivative);
			}
		}

		// Prove every absolute gate rejects known wrong data, including a normal whose individual components all pass.
		PRUnitTestMethod(AbsoluteGateNegativeControls, Quick)
		{
			auto reference = terrain::SurfaceSample{.m_height = 7000, .m_material_id = 7};
			auto field = BaselineResult{.m_height = 7000, .m_material_id = 7};
			auto normal = BaselineResult{.m_height = 0, .m_dx = 0, .m_dy = 1, .m_material_id = 7};
			PR_EXPECT(MeasureErrors(reference, field, normal).Accepted());
			PR_EXPECT((Errors{0.001, 1e-4, 1e-4, 1e-4}).Accepted());
			field.m_height += 0.00101;
			auto height_error = MeasureErrors(reference, field, normal);
			PR_EXPECT(height_error.m_height > 0.001 && !height_error.Accepted());
			field.m_height = reference.m_height;
			field.m_dx = 0.000101;
			PR_EXPECT(MeasureErrors(reference, field, normal).m_dx > 1e-4 && !MeasureErrors(reference, field, normal).Accepted());
			field.m_dx = 0;
			field.m_dy = -0.000101;
			PR_EXPECT(MeasureErrors(reference, field, normal).m_dy > 1e-4 && !MeasureErrors(reference, field, normal).Accepted());
			field.m_dy = 0;

			// A unit vector with 80-microradian X/Y components defeats a mistaken per-component normal gate.
			normal.m_height = normal.m_dx = 0.00008;
			normal.m_dy = std::sqrt(1 - 2 * 0.00008 * 0.00008);
			auto normal_error = MeasureErrors(reference, field, normal);
			PR_EXPECT(std::abs(normal.m_height) < 1e-4 && std::abs(normal.m_dx) < 1e-4 && std::abs(normal.m_dy - 1) < 1e-4);
			PR_EXPECT(normal_error.m_normal > 1e-4 && !normal_error.Accepted());
			field.m_height = std::numeric_limits<double>::quiet_NaN();
			PR_EXPECT(!MeasureErrors(reference, field, normal).Accepted());
			std::printf("Terrain negative controls: height=%.9g gradient_x/y=0.000101 normal_length=%.9g; all rejected\n", height_error.m_height, normal_error.m_normal);
		}

		// Dispatch FP32 and FP64 fields/normals under both compiler modes over an invariant bounded representative matrix.
		PRUnitTestMethod(DomainWideAbsolutePrecisionAndStages, Extended)
		{
			auto& gpu = SharedTestGpu();
			RequireBaselineDevice(gpu);
			for (bool optimise : {false, true})
			{
				for (bool fp32 : {false, true})
				{
					auto field_code = CompileBaselineProbe(L"CSBaselineProbe", L"cs_6_0", optimise, fp32);
					auto normal_code = CompileBaselineProbe(L"CSBaselineNormalProbe", L"cs_6_0", optimise, fp32);
					auto vertex_code = CompileBaselineProbe(L"VSBaselineProbe", L"vs_6_0", optimise, fp32);
					PR_EXPECT(!vertex_code.empty());
					auto field_step = ComputeStep{};
					field_step.m_sig = RootSig(ERootSigFlags::ComputeOnly).U32<uint32_t>(ECBufReg::b0).SRV(ESRVReg::t0).SRV(ESRVReg::t1).UAV(EUAVReg::u0).Create(gpu, "Terrain:FieldSig");
					field_step.m_pso = ComputePSO(field_step.m_sig.get(), ByteCode(field_code)).Create(gpu, "Terrain:FieldPSO");
					auto normal_step = ComputeStep{};
					normal_step.m_sig = field_step.m_sig;
					normal_step.m_pso = ComputePSO(normal_step.m_sig.get(), ByteCode(normal_code)).Create(gpu, "Terrain:NormalPSO");
					auto total = size_t{};
					auto failed = 0;
					auto maximum = Errors{};
					std::printf("Terrain compiler: HLSL2021 -Gis %s PR_TERRAIN_FP32=%d CS=%zu normal_CS=%zu VS=%zu bytes; VS compile-only\n",
						optimise ? "-O3" : "-Od", fp32, field_code.size(), normal_code.size(), vertex_code.size());

					// CPU references are computed once per recipe and never derived from a GPU result.
					for (uint32_t seed : {0u, 42u, 12648430u, 0xffffffffu, 0xa53e7a21u})
					{
						for (auto variant : {ERecipe::Default, ERecipe::Lattice, ERecipe::Cancellation, ERecipe::HighDatum, ERecipe::ChangedBands})
						{
							auto surface = BaselineSurface(ProbeConfig(seed, variant));
							auto positions = DomainPositions(surface.Recipe());
							PR_EXPECT(positions.size() == 2485);
							auto reference = std::vector<terrain::SurfaceSample>(positions.size());
							surface.Sample(positions, reference);
							auto fields = SampleBaselineGpu(gpu, field_step, surface.Recipe(), positions);
							auto normals = SampleBaselineGpu(gpu, normal_step, surface.Recipe(), positions);
							auto errors = Errors{};
							auto recipe_failed = 0;
							for (size_t index = 0; index != positions.size(); ++index)
							{
								auto error = MeasureErrors(reference[index], fields[index], normals[index]);
								errors.m_height = std::max(errors.m_height, error.m_height);
								errors.m_dx = std::max(errors.m_dx, error.m_dx);
								errors.m_dy = std::max(errors.m_dy, error.m_dy);
								errors.m_normal = std::max(errors.m_normal, error.m_normal);
								auto accepted = error.Accepted();
								if (!fp32)
									accepted = accepted && error.m_height < 1e-8 && error.m_dx < 1e-10 && error.m_dy < 1e-10 && error.m_normal < 1e-10;

								if (!accepted && failed + recipe_failed < 8)
									std::printf("Terrain FAILED seed=%u variant=%d xy=(%.17g,%.17g) height=%.9g dx=%.9g dy=%.9g normal_length=%.9g\n",
										seed, static_cast<int>(variant), positions[index].x, positions[index].y, error.m_height, error.m_dx, error.m_dy, error.m_normal);

								recipe_failed += !accepted;
							}
							failed += recipe_failed;
							total += positions.size();
							maximum.m_height = std::max(maximum.m_height, errors.m_height);
							maximum.m_dx = std::max(maximum.m_dx, errors.m_dx);
							maximum.m_dy = std::max(maximum.m_dy, errors.m_dy);
							maximum.m_normal = std::max(maximum.m_normal, errors.m_normal);
							std::printf("Terrain parity optimise=%d fp32=%d seed=%u variant=%d samples=%zu failed=%d height=%.9g dx=%.9g dy=%.9g normal_length=%.9g\n",
								optimise, fp32, seed, static_cast<int>(variant), positions.size(), recipe_failed, errors.m_height, errors.m_dx, errors.m_dy, errors.m_normal);
							std::fflush(stdout);
						}
					}
					std::printf("Terrain TOTAL optimise=%d fp32=%d samples=%zu failed=%d height=%.17g dx=%.17g dy=%.17g normal_length=%.17g\n",
						optimise, fp32, total, failed, maximum.m_height, maximum.m_dx, maximum.m_dy, maximum.m_normal);
					PR_EXPECT(failed == 0);

					// Invalid coordinates/arithmetic remain failures regardless of selected shader precision and optimization.
					auto source = BaselineSurface{};
					auto invalid = std::array<v2d, 4>{{{std::numeric_limits<double>::quiet_NaN(), 0}, {0, std::numeric_limits<double>::infinity()}, {1e6 + 1, 0}, {0, -1e6 - 1}}};
					for (auto const& result : SampleBaselineGpu(gpu, field_step, source.Recipe(), invalid))
						PR_EXPECT(result.m_status == 1 && result.m_material_id == -1);

					auto point = std::array<v2d, 1>{{{1, 1}}};
					auto broken = source.Recipe();
					broken.m_fields[0].m_octave_count = 9;
					PR_EXPECT(SampleBaselineGpu(gpu, field_step, broken, point)[0].m_status == 2);
					broken = source.Recipe();
					broken.m_fields[0].m_wavelength_m = 1e-300;
					PR_EXPECT(SampleBaselineGpu(gpu, field_step, broken, point)[0].m_status == 2);
					if (fp32)
					{
						broken = source.Recipe();
						broken.m_fields[0].m_amplitude = 1e300;
						PR_EXPECT(SampleBaselineGpu(gpu, field_step, broken, point)[0].m_status == 2);
					}
					PR_EXPECT(SampleBaselineGpu(gpu, field_step, source.Recipe(), {}).empty());
					auto oversized = std::vector<v2d>(4097);
					PR_THROWS(SampleBaselineGpu(gpu, field_step, source.Recipe(), oversized), std::length_error);
				}
			}
		}
	};
}
#endif
