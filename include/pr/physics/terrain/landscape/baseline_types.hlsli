//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_TERRAIN_BASELINE_TYPES_HLSLI
#define PR_TERRAIN_BASELINE_TYPES_HLSLI
#ifdef __cplusplus
#include "pr/physics/terrain/forward.h"
namespace pr::physics::terrain::landscape::shared
{
	using uint = uint32_t;
#endif

// Bound all per-field iteration in both language front ends.
static const int BaselineMaxOctaveCount = 8;

// One field in a prepared recipe; scalar layout is identical in C++ and StructuredBuffer HLSL.
struct BaselineBand
{
	double m_amplitude;
	double m_wavelength_m;
	double m_lacunarity;
	double m_persistence;
	double m_roundness;
	double m_weight_gain;
	uint m_seed;
	int m_octave_count;
};

// Immutable CPU-prepared recipe. Field order: regional base, selector, uplift, plains, hills, mountains, warp X, warp Y.
// Upload as one StructuredBuffer element, NOT as a legacy cbuffer (whose array packing differs).
struct BaselineRecipe
{
	BaselineBand m_fields[8];
	double m_supported_coordinate_abs_m;
	double m_sea_level_bias_m;
	double m_uplift_height_m;
	double m_mountain_base_height_m;
	int m_material_id;
	uint m_reserved;
};

// Status is 0 on success, 1 for invalid/out-of-range coordinates, 2 for unsupported arithmetic.
// Failure returns zero height/derivatives and material -1, which must not be treated as a terrain plane.
struct BaselineResult
{
	double m_height;
	double m_dx;
	double m_dy;
	int m_material_id;
	uint m_status;
};

#ifdef __cplusplus
	static_assert(sizeof(BaselineBand) == 56 && alignof(BaselineBand) == 8);
	static_assert(offsetof(BaselineBand, m_amplitude) == 0 && offsetof(BaselineBand, m_wavelength_m) == 8);
	static_assert(offsetof(BaselineBand, m_lacunarity) == 16 && offsetof(BaselineBand, m_persistence) == 24);
	static_assert(offsetof(BaselineBand, m_roundness) == 32 && offsetof(BaselineBand, m_weight_gain) == 40);
	static_assert(offsetof(BaselineBand, m_seed) == 48 && offsetof(BaselineBand, m_octave_count) == 52);
	static_assert(sizeof(BaselineRecipe) == 488 && alignof(BaselineRecipe) == 8);
	static_assert(offsetof(BaselineRecipe, m_supported_coordinate_abs_m) == 448);
	static_assert(offsetof(BaselineRecipe, m_sea_level_bias_m) == 456 && offsetof(BaselineRecipe, m_uplift_height_m) == 464);
	static_assert(offsetof(BaselineRecipe, m_mountain_base_height_m) == 472 && offsetof(BaselineRecipe, m_reserved) == 484);
	static_assert(offsetof(BaselineRecipe, m_material_id) == 480);
	static_assert(sizeof(BaselineResult) == 32 && offsetof(BaselineResult, m_status) == 28);
	static_assert(offsetof(BaselineResult, m_height) == 0 && offsetof(BaselineResult, m_dx) == 8 && offsetof(BaselineResult, m_dy) == 16);
	static_assert(offsetof(BaselineResult, m_material_id) == 24 && sizeof(v2d) == 16);
	static_assert(std::is_trivially_copyable_v<BaselineRecipe> && std::is_standard_layout_v<BaselineRecipe>);
}
#endif
#endif
