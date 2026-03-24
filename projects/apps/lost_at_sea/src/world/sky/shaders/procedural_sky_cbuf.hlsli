//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Shared types for procedural sky vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_PROCEDURAL_SKY_CBUF_HLSLI
#define LAS_PROCEDURAL_SKY_CBUF_HLSLI

#include "pr/hlsl/interop.hlsli"

// Procedural sky constant buffer. Bound to b3.
cbuffer CBufProceduralSky :reg(b3)
{
	// Sun direction (world space, normalised, points toward sun)
	float4 m_sun_direction;

	// Sun colour (RGB intensity)
	float4 m_sun_colour;

	// x = sun intensity (0=night, 1=noon), yzw = unused
	float m_sun_intensity;
	float3 sky_pad0;
};

#endif
