//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#define PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Return the colour due to lighting. Returns unlit_diff if ws_norm is zero
float4 EnvironmentMap(in uniform EnvMap envmap, float4 ws_pos, float4 ws_norm, float4 ws_cam, float4 initial_diff)
{
	float4 r = mul(reflect(ws_pos - ws_cam, ws_norm), envmap.w2env);
	float4 col = g_envmap_texture.Sample(g_envmap_sampler, r.xyz);
	return lerp(initial_diff, col, g_nugget.env_reflectivity);
}

#endif