//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Cloud field constants shared by the procedural sky and the scene lighting, so cloud shadows match the visible clouds.
// This file is included from both HLSL and C++ source. C++ includers provide the enclosing namespace (see common.h).
#ifndef PR_VIEW3D_CLOUD_CBUF_HLSLI
#define PR_VIEW3D_CLOUD_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

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

// The state of the cloud field for one frame. Positions and directions are in the atmosphere frame (Z up) unless stated otherwise.
struct CloudConstants
{
	// Sun direction in the atmosphere frame (normalised, points toward the sun).
	float4 sun_direction;

	// The default cloud cover in [0,1], nonzero when a weather map is bound, the direction the wind blows toward (radians, counter-clockwise
	// from the atmosphere frame's +X axis), and the strength of cloud shadows on the scene in [0,1] (0 = no cloud shadows).
	float cover;
	float has_weather;
	float wind_direction;
	float shadow_strength;

	// Cloud layer offsets in noise tiles, in [0, PR_SKY_CLOUD_PERIOD). Layer 0 = xy, layer 1 = zw.
	float4 offset01;

	// Layer 2 offset in noise tiles, and a bit mask where bit i hides cloud layer i.
	float2 offset2;
	uint hidden_layers;
	uint pad0;

	// Cloud evolution phase per layer (xyz) in [0, PR_SKY_CLOUD_EVOLVE_PERIOD).
	float4 evolve;

	// Weather map area in the atmosphere frame: xy = area minimum, zw = 1 / area size.
	float4 weather_area;

	// Direction transform from the scene frame into the atmosphere frame. It is a rotation, so it also maps positions about the scene origin.
	row_major float4x4 world_to_sky;
};

#endif
