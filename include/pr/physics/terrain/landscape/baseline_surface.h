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
		double m_basin_depth_m = 120.0;
		double m_basin_threshold = 0.62;
		FractalConfig m_regional_base = {140.0, 6000.0, 3, 2.1, 0.5};
		FractalConfig m_region_selector = {1.0, 3600.0, 3, 2.0, 0.55};
		FractalConfig m_region_uplift = {1.0, 3200.0, 4, 2.0, 0.5};
		DomainWarpConfig m_domain_warp = {170.0, 2600.0, 3, 2.0, 0.5};
		FractalConfig m_plains = {16.0, 900.0, 4, 2.1, 0.5};
		FractalConfig m_hills = {70.0, 650.0, 5, 2.05, 0.55};
		RidgedFractalConfig m_mountains = {260.0, 520.0, 5, 2.0, 0.55, 0.18, 1.15};

		// Slow depression field in unit-noise terms. Basins form where it exceeds m_basin_threshold, and a band of 0.1 either side forms the shore.
		FractalConfig m_basin_selector = {1.0, 4800.0, 3, 2.0, 0.5};
	};

	// One circular height override on a baseline surface; see terrain_decals.hlsli for how decals combine with the surface and each other.
	// Distances are metres. The surface is exactly 'm_height' within 'm_pad_radius' of 'm_centre' (where no other decal reaches) and is unchanged
	// from 'm_blend_radius' outward. 'm_user' is a caller value that the local decal blob carries unchanged, for example to identify a decal in a shader.
	struct TerrainDecal
	{
		v2d m_centre;
		double m_pad_radius;
		double m_blend_radius;
		double m_height;
		uint32_t m_user;
	};

	// Immutable baseline recipe with ordinary CPU queries and a shared stage-neutral shader evaluator.
	// A surface may carry a set of terrain decals. Sample() includes them; Recipe() and LocalRecipe() describe only the undecorated surface,
	// and shaders apply LocalDecals() on top (see BaselineApplyDecalsLocal in baseline_surface_local.hlsli).
	class BaselineSurface
	{
	public:
		using Position = v2d;
		inline static int constexpr MaxOctaveCount = shared::BaselineMaxOctaveCount;
		inline static int constexpr MaxDecalCount = 1 << 20;

		// Construct one immutable surface after validating its bounded configuration.
		explicit BaselineSurface(BaselineSurfaceConfig config = {});

		// Return the validated immutable configuration that defines this surface.
		BaselineSurfaceConfig const& Config() const noexcept;

		// Return the immutable 560-byte StructuredBuffer recipe used by baseline_surface.hlsli in any shader stage.
		shared::BaselineRecipe const& Recipe() const noexcept;

		// Prepare a single-precision recipe for baseline_surface_local.hlsli, which samples positions relative to 'origin_xy'.
		// The shader may sample any local position with |x| and |y| at most 'radius_m'. That square must lie within Config()'s supported range.
		// A non-finite origin or a non-positive radius throws invalid_argument; an unsupported square throws out_of_range.
		shared::BaselineLocalRecipe LocalRecipe(Position origin_xy, double radius_m) const;

		// Return a copy of this surface whose decals are 'decals', replacing any existing decals. The copy shares the recipe.
		// Every decal must be finite, with 0 <= pad radius < blend radius, and its disc must lie within the supported range; otherwise this throws
		// invalid_argument. Decal heights should lie within the range of the undecorated surface near each decal, because consumers that bound the
		// surface's height from its recipe (such as the physics engine) assume this.
		BaselineSurface WithDecals(std::span<TerrainDecal const> decals) const;

		// Return this surface's decals in the order given to WithDecals.
		std::span<TerrainDecal const> Decals() const noexcept;

		// Prepare the single-precision decal blob for BaselineApplyDecalsLocal, with positions relative to 'origin_xy'. The blob is a whole number of
		// 32-bit words; see BaselineDecalLocalHeader for its layout. A surface without decals gives a header-only blob that changes nothing.
		std::vector<std::byte> LocalDecals(Position origin_xy) const;

		// Sample one world-space XY position in metres, returning double height and dimensionless XY derivatives. Decals are included.
		// Nonfinite coordinates throw invalid_argument; coordinates outside Config()'s square bound throw out_of_range.
		// Unsupported intermediate arithmetic (including lattice coordinates outside [-2^63, 2^63)) throws runtime_error.
		SurfaceSample Sample(Position position_xy) const;

		// Sample a caller-owned batch in stable order without retaining caller buffers.
		void Sample(std::span<Position const> positions_xy, std::span<SurfaceSample> samples) const;

	private:

		// Decals indexed by a uniform grid, so each sample visits only nearby decals.
		struct DecalGrid;

		BaselineSurfaceConfig m_config;
		shared::BaselineRecipe m_recipe;
		std::shared_ptr<DecalGrid const> m_decals;
	};
}
