//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#define PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Return the world-space direction from 'centre' to the environment seen along the ray from 'ws_pos' in direction 'ws_dir'.
// The environment is treated as the inside of a sphere of 'radius' around 'centre', the position the map was captured from. Without this,
// a cube map lookup by direction alone treats the environment as infinitely distant, so nearby objects appear shifted in reflections.
// The ray's first hit on the sphere is used: the far intersection when 'ws_pos' is inside the sphere, or the near one when it is outside.
// 'ws_dir' must be normalised. A radius of 0, or a ray that misses the sphere ahead of 'ws_pos', uses 'ws_dir' unchanged.
float3 EnvMapProxyDirection(float3 ws_pos, float3 ws_dir, float3 centre, float radius)
{
	// Solve |o + t*dir| = radius, where 'o' is the ray start relative to the centre
	float3 o = ws_pos - centre;
	float b = dot(o, ws_dir);
	float c = dot(o, o) - radius * radius;
	float disc = b * b - c;
	float t = c > 0.0f ? -b - sqrt(max(disc, 0.0f)) : -b + sqrt(max(disc, 0.0f));
	if (radius <= 0.0f || disc < 0.0f || t <= 0.0f)
		return ws_dir;

	return o + t * ws_dir;
}

// Alpha values at or above this are treated as infinitely distant, because 8-bit distances this close to 1 are too coarse to be useful
static const float EnvMapMaxDistanceAlpha = 0.998f;

// Return the env-map space direction to sample in 'tex' for the world-space ray from 'ws_pos' in normalised direction 'ws_dir'.
// 'centre' is the map's capture centre (xyz) and distance scale (w), and 'radius' is the proxy sphere radius (see 'EnvMapProxyDirection').
// When the map stores distances in alpha, the proxy hit point gives a first guess of the reflected texel. The distance stored there replaces
// the proxy radius for a second intersection, which moves the lookup close to the true reflected point for both near and far geometry.
float3 EnvMapLookupDirection(TextureCube<float4> tex, float3 ws_pos, float3 ws_dir, float4 centre, float radius)
{
	// First guess: the environment lies on the proxy sphere
	float3 dir = EnvMapProxyDirection(ws_pos, ws_dir, centre.xyz, radius);

	// Refine with the distance stored at the first guess. The branch is uniform across the frame.
	if (radius > 0.0f && centre.w > 0.0f)
	{
		// Decode 'a = d / (d + S)'. Alpha near 1 is the sky or the far plane, which is treated as infinitely distant.
		float a = tex.SampleLevel(g_envmap_sampler, mul(float4(dir, 0.0f), g_frame.env_map.w2env).xyz, 0).a;
		dir = a < EnvMapMaxDistanceAlpha ? EnvMapProxyDirection(ws_pos, ws_dir, centre.xyz, centre.w * a / (1.0f - a)) : ws_dir;
	}
	return mul(float4(dir, 0.0f), g_frame.env_map.w2env).xyz;
}

// Return the env-map space directions to sample in the current and previous maps for the world-space ray from 'ws_pos' in direction 'ws_dir'.
// Each map is corrected for parallax about its own capture centre, using its own stored distances (see 'EnvMapLookupDirection').
void EnvMapDirections(float3 ws_pos, float3 ws_dir, out float3 dir, out float3 dir_prev)
{
	// The previous map is only sampled while it is fading out, so its lookup is skipped otherwise
	float3 n = normalize(ws_dir);
	float radius = g_frame.env_map.blend.y;
	dir = EnvMapLookupDirection(g_envmap_texture, ws_pos, n, g_frame.env_map.centre, radius);
	dir_prev = g_frame.env_map.blend.x < 1.0f ? EnvMapLookupDirection(g_envmap_prev_texture, ws_pos, n, g_frame.env_map.centre_prev, radius) : dir;
}

// Sample the environment map in env-map space directions 'dir' (current map) and 'dir_prev' (previous map). While a new map fades in, the
// result blends from the previous map. The blend weight is the same for every pixel in the frame, so the branch does not diverge.
// Maps that store distances in alpha return an alpha of 1.
float4 SampleEnvMap(float3 dir, float3 dir_prev)
{
	float4 col = g_envmap_texture.Sample(g_envmap_sampler, dir);
	col.a = g_frame.env_map.centre.w > 0.0f ? 1.0f : col.a;

	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
	{
		// Blend from the previous map, which may or may not store distances
		float4 prev = g_envmap_prev_texture.Sample(g_envmap_sampler, dir_prev);
		prev.a = g_frame.env_map.centre_prev.w > 0.0f ? 1.0f : prev.a;
		col = lerp(prev, col, blend);
	}
	return col;
}

// Sample mip level 'lod' of the environment maps, blending from the previous map like 'SampleEnvMap'
float4 SampleEnvMapLevel(float3 dir, float3 dir_prev, float lod)
{
	float4 col = g_envmap_texture.SampleLevel(g_envmap_sampler, dir, lod);
	col.a = g_frame.env_map.centre.w > 0.0f ? 1.0f : col.a;

	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
	{
		// Blend from the previous map, which may or may not store distances
		float4 prev = g_envmap_prev_texture.SampleLevel(g_envmap_sampler, dir_prev, lod);
		prev.a = g_frame.env_map.centre_prev.w > 0.0f ? 1.0f : prev.a;
		col = lerp(prev, col, blend);
	}
	return col;
}

// Return the colour due to lighting. Returns unlit_diff if ws_norm is zero
float4 EnvironmentMap(float4 ws_pos, float4 ws_norm, float4 ws_cam, float4 initial_diff)
{
	float3 dir, dir_prev;
	EnvMapDirections(ws_pos.xyz, reflect(ws_pos - ws_cam, ws_norm).xyz, dir, dir_prev);
	float4 col = SampleEnvMap(dir, dir_prev);
	return lerp(initial_diff, col, g_nugget.env_reflectivity);
}

#endif