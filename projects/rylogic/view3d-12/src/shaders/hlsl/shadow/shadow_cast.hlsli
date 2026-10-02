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

// Return the fraction of a 'filter_size' x 'filter_size' texel area around 'atlas_uv' that is closer to the light than depth 'z'.
// Bilinear comparison samples each cover 2x2 texels, so the area is covered with 9 samples (5x5) or 16 samples (7x7).
// The sample positions and weights give a smooth filter whose centre follows 'atlas_uv' continuously (Castano, "Shadow Mapping Summary", 2013).
float FilteredShadow(Texture2D<float> atlas, SamplerComparisonState cmp_sampler, float2 atlas_uv, float z, float2 atlas_dim, int filter_size)
{
	// Find the texel centre nearest to 'atlas_uv' and the sub-texel offset (s,t) from it, in [0,1)
	float2 uv = atlas_uv * atlas_dim;
	float2 base = floor(uv + 0.5f);
	float s = uv.x + 0.5f - base.x;
	float t = uv.y + 0.5f - base.y;
	float2 texel = 1.0f / atlas_dim;
	base = (base - 0.5f) * texel;

	float sum = 0.0f;
	if (filter_size == 7)
	{
		// Four weighted samples per axis for a 7 texel wide filter
		float4 uw = float4(5 * s - 6, 11 * s - 28, -(11 * s + 17), -(5 * s + 1));
		float4 vw = float4(5 * t - 6, 11 * t - 28, -(11 * t + 17), -(5 * t + 1));
		float4 u = float4((4 * s - 5) / uw.x - 3, (4 * s - 16) / uw.y - 1, -(7 * s + 5) / uw.z + 1, -s / uw.w + 3);
		float4 v = float4((4 * t - 5) / vw.x - 3, (4 * t - 16) / vw.y - 1, -(7 * t + 5) / vw.z + 1, -t / vw.w + 3);
		[unroll] for (int j = 0; j != 4; ++j)
		{
			[unroll] for (int i = 0; i != 4; ++i)
			{
				sum += uw[i] * vw[j] * atlas.SampleCmpLevelZero(cmp_sampler, base + float2(u[i], v[j]) * texel, z);
			}
		}
		return sum / 2704.0f;
	}
	else
	{
		// Three weighted samples per axis for a 5 texel wide filter
		float3 uw = float3(4 - 3 * s, 7, 1 + 3 * s);
		float3 vw = float3(4 - 3 * t, 7, 1 + 3 * t);
		float3 u = float3((3 - 2 * s) / uw.x - 2, (3 + s) / uw.y, s / uw.z + 2);
		float3 v = float3((3 - 2 * t) / vw.x - 2, (3 + t) / vw.y, t / vw.z + 2);
		[unroll] for (int j = 0; j != 3; ++j)
		{
			[unroll] for (int i = 0; i != 3; ++i)
			{
				sum += uw[i] * vw[j] * atlas.SampleCmpLevelZero(cmp_sampler, base + float2(u[i], v[j]) * texel, z);
			}
		}
		return sum / 144.0f;
	}
}

// Returns a value in [0,1] where 0 means 'ws_pos' is fully in the shadow of 'light', and 1 means not in shadow.
// 'light' must have shadow views. 'ws_norm' is the surface normal at 'ws_pos' (it does not need to be normalised).
// 'view_depth' is the distance of 'ws_pos' in front of the camera, used to fade out shadows that end at a set distance.
float ShadowVisibility(Texture2D<float> atlas, SamplerComparisonState cmp_sampler, StructuredBuffer<ShadowView> views, Light light, float4 ws_pos, float4 ws_norm, float view_depth)
{
	// The filter reads up to half its width plus one texel from the sample position. Samples are kept this far inside the view's region.
	float2 atlas_dim;
	atlas.GetDimensions(atlas_dim.x, atlas_dim.y);
	float2 texel = 1.0f / atlas_dim;

	// Orient the normal toward the light. Receivers are moved along it by about a shadow texel so that surfaces do not shadow themselves.
	float4 light_to_pos = ws_pos - light.ws_position;
	float4 light_dir = DirectionalLight(light) ? light.ws_direction : light_to_pos;
	float3 norm = normalize(ws_norm.xyz + float3(0, 0, TINY));
	norm = dot(norm, light_dir.xyz) > 0 ? -norm : norm;

	// Choose the view that covers 'ws_pos'. Point lights have one view per cube face. Directional lights have cascades
	// ordered from nearest to furthest, and use the first cascade that contains the point with room for the filter.
	int first = light.info.y;
	int count = DirectionalLight(light) ? light.info.z : 1;
	if (PointLight(light))
		first += ShadowCubeFace(light_to_pos.xyz);

	// Views that end at a set distance fade to fully lit over a range of camera depths, so the edge of the views is not visible.
	// The fade range is the same for all views of a light.
	float2 fade_depth = views[first].bias.zw;
	float fade = fade_depth.y > 0 ? saturate((view_depth - fade_depth.x) / (fade_depth.y - fade_depth.x)) : 0.0f;
	if (fade >= 1.0f)
		return 1.0f;

	for (int i = 0; i != count; ++i)
	{
		ShadowView view = views[first + i];
		float margin = view.bias.y * 0.5f + 1.0f;

		// Perspective views have texels that grow with distance from the light, so their normal offset is scaled by that distance
		float offset = DirectionalLight(light) ? view.bias.x : view.bias.x * length(light_to_pos.xyz);
		float4 pos = float4(ws_pos.xyz + offset * norm, 1.0f);

		// Project into the view
		float4 ss_pos = mul(pos, view.w2s);
		ss_pos.xyz /= ss_pos.w;
		float2 uv = float2(0.5f + 0.5f * ss_pos.x, 0.5f - 0.5f * ss_pos.y);

		// Directional views clamp caster depths into [0,1] when rendering, so they cover every depth along the light. A receiver beyond
		// the far plane is behind every caster in the view, and one in front of the near plane is in front of them all. Compare at the clamped depth.
		if (DirectionalLight(light))
			ss_pos.z = saturate(ss_pos.z);

		// Try the next cascade if the point, with the filter area around it, is not inside this one. The last view is used if it contains the point at all.
		float2 inset = margin * texel / view.atlas_rect.xy;
		bool inside = ss_pos.w > 0 && all(uv >= 0.0f) && all(uv <= 1.0f) && ss_pos.z >= 0.0f && ss_pos.z <= 1.0f;
		bool inside_filter = inside && all(uv >= inset) && all(uv <= 1.0f - inset);
		if (!inside_filter && i + 1 != count)
			continue;

		// Points outside every view are not covered by any caster, so they are lit
		if (!inside)
			return 1.0f;

		// Map to the view's region of the atlas. Keep the filter footprint inside the region so neighbouring views are never sampled.
		float2 atlas_uv = uv * view.atlas_rect.xy + view.atlas_rect.zw;
		atlas_uv = clamp(atlas_uv, view.atlas_rect.zw + margin * texel, view.atlas_rect.zw + view.atlas_rect.xy - margin * texel);
		return lerp(FilteredShadow(atlas, cmp_sampler, atlas_uv, ss_pos.z, atlas_dim, (int)view.bias.y), 1.0f, fade);
	}
	return 1.0f;
}

#endif
