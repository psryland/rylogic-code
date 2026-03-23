//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Shared types for terrain vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_TERRAIN_CBUF_HLSLI
#define LAS_TERRAIN_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

// Terrain constant buffer. Bound to b3 (reusing the CBufScreenSpace slot since
// the terrain shader does not use screen-space geometry).
// Shared parameters set once per frame in SetupFrame.
// Per-patch morph range updated per instance in SetupElement.
cbuffer CBufTerrain :reg(b3,0)
{
	// Camera world-space position (xyz), w = unused
	float4 m_camera_pos;

	// Per-patch config: x=morph_start, y=morph_end, z=grid_n (32), w=unused
	float4 m_patch_config;

	// Noise parameters: x=octaves(int), y=base_frequency, z=persistence, w=amplitude
	float4 m_noise_params;

	// x=sea_level_bias, yzw=unused
	float4 m_noise_bias;

	// Sun direction (world space, normalised, points toward sun)
	float4 m_sun_direction;

	// Sun colour (RGB intensity)
	float4 m_sun_colour;

	// Weather params: x=warp_freq, y=warp_strength, z=ridge_threshold, w=macro_freq
	float4 m_weather_params;

	// Beach params: x=beach_height, y=macro_scale_min, z=macro_scale_max, w=underwater_smooth_depth
	float4 m_beach_params;
};

#endif
