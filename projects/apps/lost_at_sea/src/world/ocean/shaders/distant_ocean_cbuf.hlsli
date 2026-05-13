//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for distant ocean vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_DISTANT_OCEAN_CBUF_HLSLI
#define LAS_DISTANT_OCEAN_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
#endif

// Distant ocean constant buffer. Bound to b3 (same slot as other overlays).
struct CBufDistantOcean //:reg(b3)
{
	// Camera world-space position (xyz), w = unused
	float4 camera_pos;

	// Fog parameters: x=fog_start, y=fog_end, zw=unused
	float4 fog_params;

	// Ocean colours
	float4 colour_shallow;
	float4 colour_deep;
	float4 fog_colour;

	// Sun direction (world space, normalised, points toward sun)
	float4 sun_direction;

	// Sun colour (RGB intensity)
	float4 sun_colour;

	// Environment map: non-zero if a cubemap is bound at t1
	int has_env_map;
	int3 pad0;
};

#ifdef __cplusplus
}
#endif
#endif
