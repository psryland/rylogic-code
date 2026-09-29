//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// Shadow atlas sampling for lights with shadow views
#ifndef PR_VIEW3D_SHADER_SHADOW_CAST_HLSL
#define PR_VIEW3D_SHADER_SHADOW_CAST_HLSL

#include "pr/hlsl/core.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"

// Return the point light cube face index (+X,-X,+Y,-Y,+Z,-Z) that contains the direction 'light_to_point'.
// Must match 'rdr12::ShadowCubeFace' in shadow_view.h.
int ShadowCubeFace(float3 light_to_point)
{
	// The face is the dominant axis and its sign
	float3 a = abs(light_to_point);
	if (a.x >= a.y && a.x >= a.z) return light_to_point.x >= 0 ? 0 : 1;
	if (a.y >= a.z) return light_to_point.y >= 0 ? 2 : 3;
	return light_to_point.z >= 0 ? 4 : 5;
}

// Returns a value in [0,1] where 0 means 'ws_pos' is fully in the shadow of 'light', and 1 means not in shadow.
// 'light' must have shadow views. 'ws_norm' is the surface normal at 'ws_pos' (it does not need to be normalised).
float ShadowVisibility(Texture2D<float> atlas, SamplerComparisonState cmp_sampler, StructuredBuffer<ShadowView> views, Light light, float4 ws_pos, float4 ws_norm)
{
	// Choose the view that covers 'ws_pos'. Point lights have one view per cube face.
	float4 light_to_pos = ws_pos - light.ws_position;
	int view_index = light.info.y;
	if (PointLight(light))
		view_index += ShadowCubeFace(light_to_pos.xyz);

	ShadowView view = views[view_index];

	// Move the receiver along its normal, toward the light, by about a shadow texel so that surfaces do not shadow themselves.
	// Perspective views have texels that grow with distance from the light, so their offset is scaled by that distance.
	float4 light_dir = DirectionalLight(light) ? light.ws_direction : light_to_pos;
	float3 norm = normalize(ws_norm.xyz + float3(0, 0, TINY));
	norm = dot(norm, light_dir.xyz) > 0 ? -norm : norm;
	float offset = DirectionalLight(light) ? view.bias.x : view.bias.x * length(light_to_pos.xyz);
	float4 pos = float4(ws_pos.xyz + offset * norm, 1.0f);

	// Project into the view. Points outside the view are not covered by any caster, so they are lit.
	float4 ss_pos = mul(pos, view.w2s);
	ss_pos.xyz /= ss_pos.w;
	float2 uv = float2(0.5f + 0.5f * ss_pos.x, 0.5f - 0.5f * ss_pos.y);
	if (ss_pos.w <= 0 || any(uv < 0.0f) || any(uv > 1.0f) || ss_pos.z > 1.0f)
		return 1.0f;

	// Map to the view's region of the atlas. Keep the filter footprint inside the region so neighbouring views are never sampled.
	float2 atlas_dim;
	atlas.GetDimensions(atlas_dim.x, atlas_dim.y);
	float2 texel = 1.0f / atlas_dim;
	float2 atlas_uv = uv * view.atlas_rect.xy + view.atlas_rect.zw;
	atlas_uv = clamp(atlas_uv, view.atlas_rect.zw + 1.5f * texel, view.atlas_rect.zw + view.atlas_rect.xy - 1.5f * texel);

	// Average a 3x3 patch of depth comparisons to soften the shadow edge
	float z = saturate(ss_pos.z);
	float lit = 0.0f;
	[unroll] for (int y = -1; y <= 1; ++y)
	{
		[unroll] for (int x = -1; x <= 1; ++x)
		{
			lit += atlas.SampleCmpLevelZero(cmp_sampler, atlas_uv, z, int2(x, y));
		}
	}
	return lit / 9.0f;
}

#endif
