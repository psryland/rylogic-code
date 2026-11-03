// Exercise the production shadow sampler against deliberately disagreeing atlas views.
#include "view3d-12/src/shaders/hlsl/shadow/shadow_cast.hlsli"

Texture2D<float> atlas : register(t0);
StructuredBuffer<ShadowView> views : register(t1);
RWStructuredBuffer<float> results : register(u0);
SamplerComparisonState comparison_sampler : register(s0);

// Sweep every filter-safe map border for cascade blending, fallback, light-type, and distance-fade cases.
numthreads(CSMain, 64, 1, 1)
void CSMain(uint3 id : SV_DispatchThreadID)
{
	// Ignore padded threads outside the requested receiver sweep.
	if (id.x >= 257)
		return;

	// Each row traverses from the fine view's interior through its border into the coarse view.
	float coordinate = 0.6f + 0.45f * id.x / 256.0f;
	float2 xy = id.z < 2 ? float2(coordinate, 0) : id.z < 4 ? float2(0, coordinate) : float2(coordinate, coordinate);
	xy *= id.z < 4 ? ((id.z & 1) != 0 ? -1.0f : 1.0f) : float2((id.z & 1) != 0 ? -1.0f : 1.0f, (id.z & 2) != 0 ? -1.0f : 1.0f);
	float4 position = float4(xy, 0.5f, 1);
	uint mode = id.y % 9;
	if (mode == 8)
		position.xy = float2(coordinate, 0);

	// The no-coverage case is outside every projection.
	if (mode == 7)
		position.xy = float2(100, 100);

	// All rows use the same resource layout; their views differ only in coverage, resolution, and fade range.
	Light light = (Light)0;
	g_shadow_filter_max = id.y >= 18 ? 1 : 7;
	light.info = int4(mode == 5 ? LightType_Spot : mode == 6 ? LightType_Point : LightType_Directional, id.y * 3, mode == 1 ? 1 : mode == 4 ? 3 : 2, 0);
	light.ws_direction = float4(0, 0, -1, 0);
	light.ws_position = float4(-10, 0, 0.5f, 1);
	results[(id.y * 8 + id.z) * 257 + id.x] = ShadowVisibility(atlas, comparison_sampler, views, light, position, float4(0, 0, 1, 0), 15.0f);
}
