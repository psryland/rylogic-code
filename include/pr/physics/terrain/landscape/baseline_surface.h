//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/surface_sample.h"
#include "pr/physics/terrain/landscape/baseline_types.hlsli"

namespace pr::physics::terrain::landscape
{
	// One bounded fractal band whose amplitude is expressed in the caller's output units.
	struct FractalConfig
	{
		double m_amplitude = 1.0;
		double m_wavelength_m = 1.0;
		int m_octave_count = 1;
		double m_lacunarity = 2.0;
		double m_persistence = 0.5;
	};

	// One bounded domain-warp band whose output offsets are expressed in metres.
	struct DomainWarpConfig
	{
		double m_amplitude_m = 0.0;
		double m_wavelength_m = 1.0;
		int m_octave_count = 1;
		double m_lacunarity = 2.0;
		double m_persistence = 0.5;
	};

	// One bounded rounded-ridge band for mountain-like multifractal detail.
	struct RidgedFractalConfig
	{
		double m_amplitude = 1.0;
		double m_wavelength_m = 1.0;
		int m_octave_count = 1;
		double m_lacunarity = 2.0;
		double m_persistence = 0.5;
		double m_roundness = 0.2;
		double m_weight_gain = 1.0;
	};

	// Immutable configuration for the baseline procedural landscape surface.
	struct BaselineSurfaceConfig
	{
		uint32_t m_seed = 0xA53E7A21u;
		MaterialId m_material_id = 0;
		double m_supported_coordinate_abs_m = 1.0e6;
		double m_sea_level_bias_m = -55.0;
		double m_uplift_height_m = 130.0;
		double m_mountain_base_height_m = 110.0;
		FractalConfig m_regional_base = {140.0, 6000.0, 3, 2.1, 0.5};
		FractalConfig m_region_selector = {1.0, 3600.0, 3, 2.0, 0.55};
		FractalConfig m_region_uplift = {1.0, 3200.0, 4, 2.0, 0.5};
		DomainWarpConfig m_domain_warp = {170.0, 2600.0, 3, 2.0, 0.5};
		FractalConfig m_plains = {16.0, 900.0, 4, 2.1, 0.5};
		FractalConfig m_hills = {70.0, 650.0, 5, 2.05, 0.55};
		RidgedFractalConfig m_mountains = {260.0, 520.0, 5, 2.0, 0.55, 0.18, 1.15};
	};

	// Immutable baseline recipe with ordinary CPU queries and a shared stage-neutral shader evaluator.
	class BaselineSurface
	{
	public:
		using Position = v2d;
		inline static int constexpr MaxOctaveCount = shared::BaselineMaxOctaveCount;

		// Construct one immutable surface after validating its bounded configuration.
		explicit BaselineSurface(BaselineSurfaceConfig config = {});

		// Return the validated immutable configuration that defines this surface.
		BaselineSurfaceConfig const& Config() const noexcept;

		// Return the immutable 488-byte StructuredBuffer recipe used by baseline_surface.hlsli in any shader stage.
		shared::BaselineRecipe const& Recipe() const noexcept;

		// Sample one world-space XY position in metres, returning double height and dimensionless XY derivatives.
		// Nonfinite coordinates throw invalid_argument; coordinates outside Config()'s square bound throw out_of_range.
		// Unsupported intermediate arithmetic (including lattice coordinates outside [-2^63, 2^63)) throws runtime_error.
		SurfaceSample Sample(Position position_xy) const;

		// Sample a caller-owned batch in stable order without retaining caller buffers.
		void Sample(std::span<Position const> positions_xy, std::span<SurfaceSample> samples) const;

	private:
		BaselineSurfaceConfig m_config;
		shared::BaselineRecipe m_recipe;
	};
}
