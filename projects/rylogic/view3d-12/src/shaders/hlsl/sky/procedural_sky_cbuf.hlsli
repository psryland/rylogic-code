//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for procedural sky vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef PR_VIEW3D_PROCEDURAL_SKY_CBUF_HLSLI
#define PR_VIEW3D_PROCEDURAL_SKY_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

// The cloud constants belong to the shared shader namespace, so the forward constants and the sky constants use the same type.
#ifdef __cplusplus
namespace pr::rdr12::shaders
{
	using namespace pr::hlsl;
#endif
#include "view3d-12/src/shaders/hlsl/sky/cloud_cbuf.hlsli"
#ifdef __cplusplus
}
namespace pr::rdr12::sky
{
	using namespace pr::hlsl;
	using shaders::CloudConstants;
#endif

// Procedural sky constant buffer. Bound to b3.
struct CBufProceduralSky //:reg(b3)
{
	// Sun colour (RGB intensity)
	float4 sun_colour;

	// Sun intensity (0=night, 1=noon), the atmosphere's share of the background colour, time in seconds, wrapped at PR_SKY_TIME_PERIOD,
	// for star twinkle and sun rays, and the moon's brightness (0 = no moon, 1 = a full moon on a clear night).
	float sun_intensity;
	float blend_weight;
	float time;
	float moon_intensity;

	// Lightning flashes inside the cloud: xy = position in the atmosphere frame, z = radius, w = brightness (0 = no flash).
	float4 lightning[PR_SKY_LIGHTNING_MAX];

	// Direction transform from the current scene frame into the source cube frame.
	row_major float4x4 world_to_cube;

	// Unit direction toward the moon, and the unit direction from the moon toward the light that lights it, both in the atmosphere frame.
	float4 moon_direction;
	float4 moon_light_direction;

	// Rotation from the atmosphere frame into the star frame (Z = north celestial pole, X = right ascension 0h).
	row_major float4x4 sky_to_stars;

	// The cloud field, shared with the scene lighting for cloud shadows.
	CloudConstants clouds;
};

#ifdef __cplusplus
}
#endif
#endif