//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for distant ocean vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_DISTANT_OCEAN_CBUF_HLSLI
#define LAS_DISTANT_OCEAN_CBUF_HLSLI

#include "pr/hlsl/interop.hlsli"

// Distant ocean constant buffer. Bound to b3 (same slot as other overlays).
cbuffer CBufDistantOcean :reg(b3)
{
	// Camera world-space position (xyz), w = unused
	float4 m_camera_pos;

	// Fog parameters: x=fog_start, y=fog_end, zw=unused
	float4 m_fog_params;

	// Ocean colours
	float4 m_colour_shallow;
	float4 m_colour_deep;
	float4 m_fog_colour;

	// Sun direction (world space, normalised, points toward sun)
	float4 m_sun_direction;

	// Sun colour (RGB intensity)
	float4 m_sun_colour;

	// Environment map: non-zero if a cubemap is bound at t1
	int m_has_env_map;
	int3 distant_ocean_pad0;
};

#endif
