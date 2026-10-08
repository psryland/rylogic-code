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

// Cloud layer parameters shared by the CPU and the shader: (altitude, noise tile size along the wind, noise tile size across the wind, wind speed multiplier).
// Altitudes and tile sizes are in world units (assumed metres). One tile of the cloud noise texture spans about eight cloud masses.
// Longer tiles along the wind stretch the clouds into streaks that follow it: strongly for cirrus, moderately for mid-level cloud, slightly for cumulus.
// Lower layers move faster to give parallax.
#define PR_SKY_CLOUD_LAYER0 1500.0f, 22000.0f, 18000.0f, 1.00f
#define PR_SKY_CLOUD_LAYER1 4000.0f, 14000.0f, 6000.0f, 0.60f
#define PR_SKY_CLOUD_LAYER2 9000.0f, 72000.0f, 14400.0f, 0.35f

// Storm cloud hangs lower: as the cover above the camera rises from PR_SKY_CLOUD_LOWER_START to 1, the low and mid layers drop to
// PR_SKY_CLOUD_LOWER_SCALE times their altitude. Cirrus keeps its altitude.
#define PR_SKY_CLOUD_LOWER_START 0.9f
#define PR_SKY_CLOUD_LOWER_SCALE (1.0f / 3.0f)

// The maximum number of lightning flashes the sky shows at once.
#define PR_SKY_LIGHTNING_MAX 4

// The width and height, in texels, of the tileable cloud noise texture.
#define PR_SKY_CLOUD_NOISE_SIZE 512

// Layer offsets are in noise tiles. The shader samples the noise at whole-number multiples and quarter fractions of the offset, so all samples
// repeat after PR_SKY_CLOUD_PERIOD tiles, and the CPU wraps the offsets at this period without visible jumps.
#define PR_SKY_CLOUD_PERIOD 4

// Cloud shapes change as a second noise sample drifts across the first. The drift repeats every PR_SKY_CLOUD_EVOLVE_PERIOD units, so the CPU
// wraps the evolution phase at this period. PR_SKY_CLOUD_EVOLVE_RATE is the change rate (units per second) in still air; wind adds to it.
#define PR_SKY_CLOUD_EVOLVE_PERIOD 2
#define PR_SKY_CLOUD_EVOLVE_RATE 0.0003f

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
	// and time in seconds, wrapped at PR_SKY_TIME_PERIOD, for star twinkle and sun rays.
	float sun_intensity;
	float blend_weight;
	float cloud_cover;
	float time;

	// Cloud layer offsets in noise tiles, in [0, PR_SKY_CLOUD_PERIOD). Layer 0 = xy, layer 1 = zw.
	float4 cloud_offset01;

	// Layer 2 offset in noise tiles, nonzero when a weather map is bound, and the direction the wind blows toward
	// (radians, counter-clockwise from the atmosphere frame's +X axis).
	float2 cloud_offset2;
	float has_weather;
	float wind_direction;

	// Cloud evolution phase per layer (xyz) in [0, PR_SKY_CLOUD_EVOLVE_PERIOD).
	float4 cloud_evolve;

	// Weather map area in the atmosphere frame: xy = area minimum, zw = 1 / area size.
	float4 weather_area;

	// Bit i hides cloud layer i. The padding keeps the following matrices 16-byte aligned.
	uint hidden_cloud_layers;
	uint pad0;
	uint pad1;
	uint pad2;

	// Lightning flashes inside the cloud: xy = position in the atmosphere frame, z = radius, w = brightness (0 = no flash).
	float4 lightning[PR_SKY_LIGHTNING_MAX];

	// Direction transforms from the current scene frame into the atmosphere and source cube frames.
	row_major float4x4 world_to_sky;
	row_major float4x4 world_to_cube;
};

#ifdef __cplusplus
}
#endif
#endif
