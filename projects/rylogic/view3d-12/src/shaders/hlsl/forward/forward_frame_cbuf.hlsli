//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Per-frame constant buffer definition for forward shaders. Separate from forward_cbuf.hlsli so that procedural shaders
// for other stages can use the frame constants without the forward per-element definitions.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_FORWARD_FRAME_CBUF_HLSL
#define PR_VIEW3D_SHADER_FORWARD_FRAME_CBUF_HLSL
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/cloud_cbuf.hlsli"

// The low cloud accumulated over frames by the procedural sky. See AccumulateLowCloud in procedural_sky.hlsl.
struct SkyHistory
{
	// The previous frame's world to screen transform, used to find where a cloud point was on screen in that frame.
	row_major float4x4 prev_w2s;

	// The viewport of both frames, in pixels: (x, y, width, height).
	float4 viewport;

	// How far the low cloud moved with the wind since the previous frame, in world units in the atmosphere frame, the offset along each march step
	// for this frame in [0,1), and 1 when the history holds the previous frame, otherwise 0.
	float2 cloud_shift;
	float frame_jitter;
	float valid;

	// 1 when the low cloud is accumulated over frames, otherwise 0.
	int enabled;
	int pad0, pad1, pad2;
};

// The plane whose reflection is projected from the rendered scene. See reflection_plane.hlsl.
struct ReflectionPlane
{
	// xyz = the plane's unit normal, w = its distance from the origin along the normal.
	float4 plane;

	// The viewport the reflection was projected into, in pixels: (x, y, width, height).
	float4 viewport;

	// How far beyond the viewport the reflection sources reach, in normalised device units: x = to the left and right, y = above the top.
	// Sources in this border were rendered at lower resolution, so the reflection is still found for them. zw are unused.
	float4 guard;

	// 1 when the projected reflection is available, otherwise 0.
	int enabled;
	int pad0, pad1, pad2;
};

// Constants per frame.
struct CBufFrame// :reg(b0)
{
	// Camera transform
	Camera cam;
	
	// Scene lighting. The lights themselves are in the 'g_lights' structured buffer.
	float4 ambient;    // .rgb = scene-wide ambient light colour
	int4 light_info;   // x = light count

	// Output settings
	float4 output;     // x = dither amount applied before 8-bit colour output (see DitherSrgb8); 0 disables dithering

	// EnvMap
	EnvMap env_map;

	// Projected textures
	ProjTexture proj_tex;

	// The procedural sky's cloud field, for cloud shadows on the scene. 'clouds.shadow_strength' is 0 when there are no cloud shadows.
	CloudConstants clouds;

	// The procedural sky's low cloud history. 'sky_history.enabled' is 0 when frames are not accumulated.
	SkyHistory sky_history;

	// The projected planar reflection. 'reflection_plane.enabled' is 0 when there is none.
	ReflectionPlane reflection_plane;

	// Applied after the projection to clip space, to move the output into another render target region. 'cam.c2s' and 'cam.w2s' already include it.
	// It is the identity for normal rendering.
	row_major float4x4 clip_remap;
};

#endif
