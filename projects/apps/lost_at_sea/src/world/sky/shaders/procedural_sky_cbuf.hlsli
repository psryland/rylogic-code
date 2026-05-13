//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for procedural sky vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_PROCEDURAL_SKY_CBUF_HLSLI
#define LAS_PROCEDURAL_SKY_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
#endif

// Procedural sky constant buffer. Bound to b3.
struct CBufProceduralSky //:reg(b3)
{
	// Sun direction (world space, normalised, points toward sun)
	float4 sun_direction;

	// Sun colour (RGB intensity)
	float4 sun_colour;

	// x = sun intensity (0=night, 1=noon), yzw = unused
	float sun_intensity;
	float3 pad0;
};

#ifdef __cplusplus
}
#endif
#endif
