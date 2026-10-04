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

// Cloud layer parameters shared by the CPU and the shader: (altitude, noise feature size x, noise feature size y, wind speed multiplier).
// Altitudes and feature sizes are in world units (assumed metres). Lower layers move faster to give parallax.
#define PR_SKY_CLOUD_LAYER0 1500.0f, 2400.0f, 2400.0f, 1.00f
#define PR_SKY_CLOUD_LAYER1 4000.0f, 1100.0f, 1100.0f, 0.60f
#define PR_SKY_CLOUD_LAYER2 9000.0f, 9000.0f, 1800.0f, 0.35f

// Cloud noise repeats every PR_SKY_CLOUD_PERIOD feature sizes, so the CPU wraps layer offsets at this period without visible jumps.
#define PR_SKY_CLOUD_PERIOD 128

// Cloud shapes change along a third noise axis. That axis repeats every PR_SKY_CLOUD_EVOLVE_PERIOD units, so the CPU wraps the
// evolution phase at this period. PR_SKY_CLOUD_EVOLVE_RATE is the change rate (units per second) in still air; wind adds to it.
#define PR_SKY_CLOUD_EVOLVE_PERIOD 32
#define PR_SKY_CLOUD_EVOLVE_RATE 0.01f

// The CPU wraps the time it passes to the shader at this period (seconds). Time-based effects must repeat exactly over this period.
#define PR_SKY_TIME_PERIOD 60.0f

// Planet radius in world units (assumed metres). Cloud layers are spherical shells, so they curve down to meet the horizon.
#define PR_SKY_PLANET_RADIUS 6371000.0f

// The fraction of a weather map's area, at each edge, over which its cover fades to the default cover. Shared with WeatherMap::CoverAt.
#define PR_SKY_WEATHER_EDGE_FADE 0.1f

// Procedural sky constant buffer. Bound to b3.
struct CBufProceduralSky //:reg(b3)
{
	// Sun direction in the atmosphere frame (normalised, points toward sun).
	float4 sun_direction;

	// Sun colour (RGB intensity)
	float4 sun_colour;

	// Sun intensity (0=night, 1=noon), the atmosphere's share of the background colour, the default cloud cover in [0,1],
	// and time in seconds, wrapped at PR_SKY_TIME_PERIOD, for star twinkle.
	float sun_intensity;
	float blend_weight;
	float cloud_cover;
	float time;

	// Cloud layer offsets in noise space (feature sizes), in [0, PR_SKY_CLOUD_PERIOD). Layer 0 = xy, layer 1 = zw.
	float4 cloud_offset01;

	// Layer 2 offset in noise space, and nonzero when a weather map is bound.
	float2 cloud_offset2;
	float has_weather;
	float pad0;

	// Cloud evolution phase per layer (xyz) in [0, PR_SKY_CLOUD_EVOLVE_PERIOD).
	float4 cloud_evolve;

	// Weather map area in the atmosphere frame: xy = area minimum, zw = 1 / area size.
	float4 weather_area;

	// Direction transforms from the current scene frame into the atmosphere and source cube frames.
	row_major float4x4 world_to_sky;
	row_major float4x4 world_to_cube;
};

#ifdef __cplusplus
}
#endif
#endif
