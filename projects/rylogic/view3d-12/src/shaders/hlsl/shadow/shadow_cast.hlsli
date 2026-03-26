//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
#ifndef PR_VIEW3D_SHADER_SHADOW_CAST_HLSL
#define PR_VIEW3D_SHADER_SHADOW_CAST_HLSL

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/camera.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Returns a value between [0,1] where 0 means fully in shadow, 1 means not in shadow
float LightVisibility(uniform Shadow shadow, float4 ws_pos)
{
	float visibility = 1.0f;
	for (int i = 0; i != ShadowMapCount(shadow); ++i)
	{
		float2 nf = ClipPlanes(shadow.l2s[i]);

		// Get the distance from the light, normalised within the projection volume
		float4 ls_pos = mul(ws_pos, shadow.w2l[i]);
		float z = Frac(nf.y, -ls_pos.z, nf.x);
		z = saturate(z);

		// Get the distance from the light, from the shadow map
		float4 ss_pos = mul(ls_pos, shadow.l2s[i]);
		float2 uv = 0.5*float2(1.0 + ss_pos.x, 1.0 - ss_pos.y);

		// Sample the SMAP in a patch
		const float Eps = TINY * 40;
		const int NumSmapSamples = 9;
		const int2 Ofs[NumSmapSamples] =
		{
			{-1,-1}, {+0,-1}, {+1,-1},
			{-1,+0}, {+0,+0}, {+1,+0},
			{-1,+1}, {+0,+1}, {+1,+1},
		};

		float lit = 0.0;
		[unroll] for (int s = 0; s != NumSmapSamples; ++s)
		{
			lit += g_smap_texture[i].SampleCmpLevelZero(g_smap_sampler, uv, z + Eps, Ofs[s]);
		}
		lit /= NumSmapSamples;

		// 1.0 = near plane, 0.0 = far plane, so the
		// largest value is closest to the light source
		visibility = lit;
	}
	return visibility;
}

#endif
