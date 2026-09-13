//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for procedural sky vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef PR_VIEW3D_PROCEDURAL_SKY_CBUF_HLSLI
#define PR_VIEW3D_PROCEDURAL_SKY_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::sky
{
	using namespace pr::hlsl;
#endif

// Procedural sky constant buffer. Bound to b3.
struct CBufProceduralSky //:reg(b3)
{
	// Sun direction in the atmosphere frame (normalised, points toward sun).
	float4 sun_direction;

	// Sun colour (RGB intensity)
	float4 sun_colour;

	// Sun intensity (0=night, 1=noon) and the atmosphere's share of the background colour.
	float sun_intensity;
	float blend_weight;
	float2 pad0;

	// Direction transforms from the current scene frame into the atmosphere and source cube frames.
	row_major float4x4 world_to_sky;
	row_major float4x4 world_to_cube;
};

#ifdef __cplusplus
}
#endif
#endif
