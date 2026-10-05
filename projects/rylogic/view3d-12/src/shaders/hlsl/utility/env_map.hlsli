//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#define PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Sample the environment map in env-map space direction 'dir'. While a new map fades in, the result blends from the previous map.
// The blend weight is the same for every pixel in the frame, so the branch does not diverge.
float4 SampleEnvMap(float3 dir)
{
	float4 col = g_envmap_texture.Sample(g_envmap_sampler, dir);
	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
		col = lerp(g_envmap_prev_texture.Sample(g_envmap_sampler, dir), col, blend);

	return col;
}

// Sample mip level 'lod' of the environment map in env-map space direction 'dir', blending from the previous map like 'SampleEnvMap'
float4 SampleEnvMapLevel(float3 dir, float lod)
{
	float4 col = g_envmap_texture.SampleLevel(g_envmap_sampler, dir, lod);
	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
		col = lerp(g_envmap_prev_texture.SampleLevel(g_envmap_sampler, dir, lod), col, blend);

	return col;
}

// Return the colour due to lighting. Returns unlit_diff if ws_norm is zero
float4 EnvironmentMap(in uniform EnvMap envmap, float4 ws_pos, float4 ws_norm, float4 ws_cam, float4 initial_diff)
{
	float4 r = mul(reflect(ws_pos - ws_cam, ws_norm), envmap.w2env);
	float4 col = SampleEnvMap(r.xyz);
	return lerp(initial_diff, col, g_nugget.env_reflectivity);
}

#endif