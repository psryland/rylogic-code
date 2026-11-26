//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared types for the rain shaders.
// This file is included from both HLSL and C++ source.
#ifndef PR_GAME_SYSTEMS_RAIN_CBUF_HLSLI
#define PR_GAME_SYSTEMS_RAIN_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

// The number of nested tile layers and drop speed classes. See pr/game-systems/sky/rain.h.
#define PR_RAIN_LAYERS 4
#define PR_RAIN_SPEED_CLASSES 4

#ifdef __cplusplus
namespace pr::game_systems::sky
{
	using namespace pr::hlsl;
#endif

// Rain constant buffer. Bound to b3.
// Each drop vertex carries the drop's position in its tile as a fraction of the tile size (vert.xyz), and (intensity threshold, corner along
// the streak, corner across the streak, slot) in its colour. The slot is 'layer * PR_RAIN_SPEED_CLASSES + speed class'.
struct CBufRain //:reg(b3)
{
	// Rotation from world to camera space, and the camera's projection.
	// Drops are placed relative to the camera, so only the rotation part of 'w2c' is used.
	row_major float4x4 w2c;
	row_major float4x4 c2s;

	// The camera's world position (xyz). Drops are placed relative to it, and it gives their world positions for depth sorting.
	float4 cam_pos;

	// The render target size in pixels (xy), and the number of pixels per world unit at clip w = 1 (z). A length at clip w covers z / w pixels.
	float4 screen_dim;

	// The colour of a lit drop (rgb), and the most a single streak covers what is behind it (a).
	float4 colour;

	// A world plane (xyz = normal, w = offset). Drops at points p where dot(xyz, p) + w < 0 are not drawn. A zero plane hides no drops.
	float4 clip_plane;

	// Per layer: the tile size (x), the number of smallest-tile drops that one drop stands for (y), and the distances over which the layer fades in (zw).
	float4 layers[PR_RAIN_LAYERS];

	// Per slot: the drops' offset within their tile, as a fraction of the tile size. It moves the tile with the drops' drift, relative to the camera.
	float4 offset[PR_RAIN_LAYERS * PR_RAIN_SPEED_CLASSES];

	// Per speed class: the drops' velocity relative to the camera, in world units per second.
	float4 velocity[PR_RAIN_SPEED_CLASSES];

	// The rain intensity in [0,1], the streak exposure time in seconds, the drop's half width in world units, and the smallest drawn width in pixels.
	float intensity;
	float streak_time;
	float half_width;
	float min_pixels;

	// The longest drawn streak, in world units, and the length of a streak for a drop falling at the base fall speed seen by a still camera.
	float max_length;
	float ref_length;
	float pad1;
	float pad2;
};

#ifdef __cplusplus
}
#endif
#endif
