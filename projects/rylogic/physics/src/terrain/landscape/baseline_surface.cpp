//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include <format>
#include "pr/physics/terrain/landscape/landscape.h"
#include "pr/algorithm/perlin_noise.h"
#include "pr/physics/materials/material.h"

namespace pr::physics::terrain::landscape
{
	namespace
	{
		using NoiseSample = algorithm::PerlinNoiseSample<double>;

		enum class EFieldId
		{
			RegionalBase,
			RegionSelector,
			RegionUplift,
			Plains,
			Hills,
			Mountains,
			WarpX,
			WarpY,
			Count,
		};

		// One scalar value and its world-space XY derivatives.
		struct ScalarSample
		{
			double m_value = 0.0;
			v2d m_gradient = v2d::Zero();
		};

		// One 2D vector value and the Jacobian of its XY derivatives.
		struct WarpSample
		{
			v2d m_value = v2d::Zero();
			v2d m_dqdx = v2d::XAxis();
			v2d m_dqdy = v2d::YAxis();
		};

		// Return a deterministic decorrelated field seed derived from the user-visible seed.
		uint32_t MixSeed(uint32_t seed, uint32_t salt)
		{
			auto hash = uint64_t{seed} + 0x9E3779B97F4A7C15ull + (uint64_t{salt} << 1);
			hash ^= hash >> 30;
			hash *= 0xBF58476D1CE4E5B9ull;
			hash ^= hash >> 27;
			hash *= 0x94D049BB133111EBull;
			hash ^= hash >> 31;
			return static_cast<uint32_t>(hash);
		}

		// Return true when a scalar is finite.
		bool IsFinite(double value)
		{
			return std::isfinite(value) != 0;
		}

		// Require one scalar configuration value to be finite.
		void RequireFinite(double value, char const* name)
		{
			if (!IsFinite(value))
				throw std::invalid_argument(std::format("Terrain config '{}' must be finite", name));
		}

		// Require one positive scalar configuration value to be finite and strictly positive.
		void RequirePositive(double value, char const* name)
		{
			RequireFinite(value, name);
			if (value <= 0.0)
				throw std::invalid_argument(std::format("Terrain config '{}' must be greater than zero", name));
		}

		// Require one bounded octave count to stay within the supported contract.
		void RequireOctaves(int octave_count, char const* name)
		{
			if (octave_count < 1 || octave_count > BaselineSurface::MaxOctaveCount)
				throw std::invalid_argument(std::format("Terrain config '{}' must be in the range [1, {}]", name, BaselineSurface::MaxOctaveCount));
		}

		// Validate one ordinary fractal band.
		void Validate(FractalConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude, std::format("{}.amplitude", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
		}

		// Validate one domain-warp band.
		void Validate(DomainWarpConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude_m, std::format("{}.amplitude_m", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
		}

		// Validate one rounded-ridge fractal band.
		void Validate(RidgedFractalConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude, std::format("{}.amplitude", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
			RequirePositive(config.m_roundness, std::format("{}.roundness", prefix).c_str());
			RequirePositive(config.m_weight_gain, std::format("{}.weight_gain", prefix).c_str());
		}

		// Validate the supported public baseline-surface contract before any sampling occurs.
		void Validate(BaselineSurfaceConfig const& config)
		{
			if (config.m_material_id < 0 || config.m_material_id >= Material::MaxMaterialId)
				throw std::invalid_argument(std::format("Terrain config 'material_id' must be in the range [0, {})", Material::MaxMaterialId));

			RequirePositive(config.m_supported_coordinate_abs_m, "supported_coordinate_abs_m");
			RequireFinite(config.m_sea_level_bias_m, "sea_level_bias_m");
			RequireFinite(config.m_uplift_height_m, "uplift_height_m");
			RequireFinite(config.m_mountain_base_height_m, "mountain_base_height_m");

			Validate(config.m_regional_base, "regional_base");
			Validate(config.m_region_selector, "region_selector");
			Validate(config.m_region_uplift, "region_uplift");
			Validate(config.m_domain_warp, "domain_warp");
			Validate(config.m_plains, "plains");
			Validate(config.m_hills, "hills");
			Validate(config.m_mountains, "mountains");
		}

		// Require one query position to stay finite and within the documented supported range.
		void Validate(BaselineSurfaceConfig const& config, BaselineSurface::Position position_xy)
		{
			if (!IsFinite(position_xy.x) || !IsFinite(position_xy.y))
				throw std::invalid_argument("Terrain query coordinates must be finite");
			if (std::abs(position_xy.x) > config.m_supported_coordinate_abs_m || std::abs(position_xy.y) > config.m_supported_coordinate_abs_m)
				throw std::out_of_range("Terrain query coordinates exceed the supported absolute range");
		}

		// Evaluate one scalar field and propagate its derivatives from warped space back to world XY.
		ScalarSample ApplyWarp(ScalarSample sample, WarpSample const& warp)
		{
			auto const grad_q = sample.m_gradient;
			sample.m_gradient = v2d{
				Dot(warp.m_dqdx, grad_q),
				Dot(warp.m_dqdy, grad_q),
			};
			return sample;
		}

		// Scale one scalar sample.
		ScalarSample operator *(double lhs, ScalarSample rhs)
		{
			rhs.m_value *= lhs;
			rhs.m_gradient *= lhs;
			return rhs;
		}

		// Add two scalar samples.
		ScalarSample operator +(ScalarSample lhs, ScalarSample const& rhs)
		{
			lhs.m_value += rhs.m_value;
			lhs.m_gradient += rhs.m_gradient;
			return lhs;
		}

		// Subtract two scalar samples.
		ScalarSample operator -(ScalarSample lhs, ScalarSample const& rhs)
		{
			lhs.m_value -= rhs.m_value;
			lhs.m_gradient -= rhs.m_gradient;
			return lhs;
		}

		// Multiply two scalar samples using the product rule.
		ScalarSample operator *(ScalarSample lhs, ScalarSample const& rhs)
		{
			return ScalarSample{
				.m_value = lhs.m_value * rhs.m_value,
				.m_gradient = lhs.m_gradient * rhs.m_value + rhs.m_gradient * lhs.m_value,
			};
		}

		// Divide one scalar sample by a finite scalar.
		ScalarSample operator /(ScalarSample lhs, double rhs)
		{
			lhs.m_value /= rhs;
			lhs.m_gradient /= rhs;
			return lhs;
		}

		// Return one constant-valued scalar sample.
		ScalarSample Constant(double value)
		{
			return ScalarSample{ .m_value = value, .m_gradient = v2d::Zero() };
		}

		// Clamp one scalar sample into [0, 1] and zero its derivative outside the active interval.
		ScalarSample Clamp01(ScalarSample sample)
		{
			if (sample.m_value <= 0.0)
				return Constant(0.0);
			if (sample.m_value >= 1.0)
				return Constant(1.0);
			return sample;
		}

		// Remap one scalar sample from [-1, +1] into [0, 1] and clamp the result.
		ScalarSample ToUnitInterval(ScalarSample sample)
		{
			return Clamp01(Constant(0.5) + 0.5 * sample);
		}

		// Evaluate one smoothstep transition and propagate its derivative.
		ScalarSample SmoothStep(double edge0, double edge1, ScalarSample sample)
		{
			if (!(edge1 > edge0))
				throw std::invalid_argument("SmoothStep requires edge1 > edge0");

			auto const t = (sample.m_value - edge0) / (edge1 - edge0);
			if (t <= 0.0)
				return Constant(0.0);
			if (t >= 1.0)
				return Constant(1.0);

			auto const value = t * t * (3.0 - 2.0 * t);
			auto const deriv = (6.0 * t * (1.0 - t)) / (edge1 - edge0);
			return ScalarSample{
				.m_value = value,
				.m_gradient = deriv * sample.m_gradient,
			};
		}

		// Evaluate one bell-shaped smooth pulse by subtracting two smoothstep ramps.
		ScalarSample SmoothPulse(double edge0, double edge1, double edge2, double edge3, ScalarSample sample)
		{
			return SmoothStep(edge0, edge1, sample) - SmoothStep(edge2, edge3, sample);
		}

		// Normalize one positive weight by the shared weight sum.
		ScalarSample NormalizeWeight(ScalarSample weight, ScalarSample sum)
		{
			auto const inv_sum = 1.0 / sum.m_value;
			return ScalarSample{
				.m_value = weight.m_value * inv_sum,
				.m_gradient = (weight.m_gradient * sum.m_value - sum.m_gradient * weight.m_value) / (sum.m_value * sum.m_value),
			};
		}

		// Evaluate one rounded ridge profile from a signed coherent-noise value.
		ScalarSample RoundedRidge(ScalarSample sample, double roundness)
		{
			auto const magnitude = std::sqrt(sample.m_value * sample.m_value + roundness * roundness);
			auto const magnitude_deriv = magnitude > 0.0
				? sample.m_value / magnitude
				: 0.0;
			auto const unit = Clamp01(ScalarSample{
				.m_value = 1.0 - magnitude,
				.m_gradient = -magnitude_deriv * sample.m_gradient,
			});
			return unit * unit;
		}

		// Convert one normalized scalar into plains, hills, and mountain weights whose sum remains one.
		std::array<ScalarSample, 3> TerrainWeights(ScalarSample selector)
		{
			auto weights = std::array
			{
				Constant(1.0) - SmoothStep(0.28, 0.58, selector),
				SmoothPulse(0.18, 0.45, 0.55, 0.82, selector),
				SmoothStep(0.52, 0.82, selector),
			};

			auto const sum = weights[0] + weights[1] + weights[2];
			if (sum.m_value <= std::numeric_limits<double>::epsilon())
				throw std::runtime_error("Terrain region weights collapsed to zero");

			weights[0] = NormalizeWeight(weights[0], sum);
			weights[1] = NormalizeWeight(weights[1], sum);
			weights[2] = NormalizeWeight(weights[2], sum);
			return weights;
		}

		// Sample one bounded fractal field in 2D by taking a non-integer slice through 3D Perlin noise.
		ScalarSample SampleFractal(BaselineSurface::Position position_xy, uint32_t seed, FractalConfig const& config)
		{
			auto generator = algorithm::PerlinNoiseGenerator(seed);
			auto sample = ScalarSample{};
			auto amplitude = config.m_amplitude;
			auto frequency = 1.0 / config.m_wavelength_m;
			auto const slice = 0.17320508075688773 + static_cast<double>(seed) * (1.0 / 65537.0);
			for (auto octave = 0; octave != config.m_octave_count; ++octave)
			{
				auto const noise = generator.NoiseWithDerivatives(position_xy.x * frequency, position_xy.y * frequency, slice + octave * 0.6180339887498949);
				sample.m_value += amplitude * noise.m_value;
				sample.m_gradient += amplitude * frequency * v2d{noise.m_dx, noise.m_dy};
				frequency *= config.m_lacunarity;
				amplitude *= config.m_persistence;
			}
			return sample;
		}

		// Sample one bounded rounded-ridge multifractal field and propagate every octave's weight derivative.
		ScalarSample SampleRidgedFractal(BaselineSurface::Position position_xy, uint32_t seed, RidgedFractalConfig const& config)
		{
			auto generator = algorithm::PerlinNoiseGenerator(seed);
			auto sample = ScalarSample{};
			auto amplitude = config.m_amplitude;
			auto frequency = 1.0 / config.m_wavelength_m;
			auto weight = Constant(1.0);
			auto const slice = 0.38196601125010515 + static_cast<double>(seed) * (1.0 / 65539.0);
			for (auto octave = 0; octave != config.m_octave_count; ++octave)
			{
				auto const noise = generator.NoiseWithDerivatives(position_xy.x * frequency, position_xy.y * frequency, slice + octave * 0.4142135623730950);
				auto const ridge = RoundedRidge(ScalarSample{
					.m_value = noise.m_value,
					.m_gradient = frequency * v2d{noise.m_dx, noise.m_dy},
				}, config.m_roundness);
				sample = sample + amplitude * (weight * ridge);
				weight = Clamp01(config.m_weight_gain * ridge);
				frequency *= config.m_lacunarity;
				amplitude *= config.m_persistence;
			}
			return sample;
		}

		// Sample the two warp channels and build the Jacobian for the warped query position.
		WarpSample SampleDomainWarp(BaselineSurface::Position position_xy, std::array<uint32_t, static_cast<size_t>(EFieldId::Count)> const& field_seeds, BaselineSurfaceConfig const& config)
		{
			auto const warp_x = SampleFractal(position_xy, field_seeds[static_cast<size_t>(EFieldId::WarpX)], FractalConfig{
				.m_amplitude = config.m_domain_warp.m_amplitude_m,
				.m_wavelength_m = config.m_domain_warp.m_wavelength_m,
				.m_octave_count = config.m_domain_warp.m_octave_count,
				.m_lacunarity = config.m_domain_warp.m_lacunarity,
				.m_persistence = config.m_domain_warp.m_persistence,
			});
			auto const warp_y = SampleFractal(position_xy, field_seeds[static_cast<size_t>(EFieldId::WarpY)], FractalConfig{
				.m_amplitude = config.m_domain_warp.m_amplitude_m,
				.m_wavelength_m = config.m_domain_warp.m_wavelength_m,
				.m_octave_count = config.m_domain_warp.m_octave_count,
				.m_lacunarity = config.m_domain_warp.m_lacunarity,
				.m_persistence = config.m_domain_warp.m_persistence,
			});
			return WarpSample{
				.m_value = v2d{warp_x.m_value, warp_y.m_value},
				.m_dqdx = v2d{1.0 + warp_x.m_gradient.x, warp_y.m_gradient.x},
				.m_dqdy = v2d{warp_x.m_gradient.y, 1.0 + warp_y.m_gradient.y},
			};
		}

		// Evaluate the baseline landscape composition at one position.
		SurfaceSample Evaluate(BaselineSurfaceConfig const& config, std::array<uint32_t, static_cast<size_t>(EFieldId::Count)> const& field_seeds, BaselineSurface::Position position_xy)
		{
			// Apply the same smooth domain warp to every downstream regional and detail field.
			auto const warp = SampleDomainWarp(position_xy, field_seeds, config);
			auto const warped_xy = position_xy + warp.m_value;

			// Sample the broad regional structure and map every gradient back to world-space XY.
			auto const regional_base = ApplyWarp(SampleFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::RegionalBase)], config.m_regional_base), warp);
			auto const uplift_mask = ToUnitInterval(ApplyWarp(SampleFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::RegionUplift)], config.m_region_uplift), warp));
			auto selector = ToUnitInterval(ApplyWarp(SampleFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::RegionSelector)], config.m_region_selector), warp));
			selector = Clamp01(selector + 0.35 * (uplift_mask - Constant(0.5)));

			// Blend distinct terrain families with smooth normalized weights so no seams appear between regions.
			auto const weights = TerrainWeights(selector);
			auto const plains = regional_base + config.m_uplift_height_m * (uplift_mask - Constant(0.35)) + ApplyWarp(SampleFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::Plains)], config.m_plains), warp);
			auto const hills = regional_base + Constant(35.0) + config.m_uplift_height_m * (uplift_mask - Constant(0.20)) + ApplyWarp(SampleFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::Hills)], config.m_hills), warp);
			auto const mountains = regional_base + Constant(config.m_mountain_base_height_m) + config.m_uplift_height_m * uplift_mask + ApplyWarp(SampleRidgedFractal(warped_xy, field_seeds[static_cast<size_t>(EFieldId::Mountains)], config.m_mountains), warp);
			auto const height = Constant(config.m_sea_level_bias_m) + weights[0] * plains + weights[1] * hills + weights[2] * mountains;
			return SurfaceSample{
				.m_height = height.m_value,
				.m_gradient_xy = height.m_gradient,
				.m_material_id = config.m_material_id,
			};
		}
	}

	// Construct one immutable surface after validating its bounded configuration.
	BaselineSurface::BaselineSurface(BaselineSurfaceConfig config)
		: m_config(std::move(config))
		, m_field_seeds()
	{
		Validate(m_config);

		// Expand the user-visible seed into one deterministic field seed per independent noise source.
		for (auto index = uint32_t{}; index != m_field_seeds.size(); ++index)
			m_field_seeds[index] = MixSeed(m_config.m_seed, index + 1);
	}

	// Return the validated immutable configuration that defines this surface.
	BaselineSurfaceConfig const& BaselineSurface::Config() const noexcept
	{
		return m_config;
	}

	// Sample one world-space XY position and return its double-precision terrain result.
	SurfaceSample BaselineSurface::Sample(Position position_xy) const
	{
		Validate(m_config, position_xy);
		return Evaluate(m_config, m_field_seeds, position_xy);
	}

	// Sample a caller-owned batch in stable order without retaining caller buffers.
	void BaselineSurface::Sample(std::span<Position const> positions_xy, std::span<SurfaceSample> samples) const
	{
		if (positions_xy.size() != samples.size())
			throw std::invalid_argument("Terrain batch sampling requires equal input and output sizes");

		// Reuse the scalar evaluator so the batch boundary preserves exact scalar behaviour.
		for (auto index = size_t{}; index != positions_xy.size(); ++index)
			samples[index] = Sample(positions_xy[index]);
	}
}
