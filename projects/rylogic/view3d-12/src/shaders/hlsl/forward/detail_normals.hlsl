// Opt-in simple-material variants that tilt the interpolated normal with world-projected detail-normal layers.
// The stock entry points shade the perturbed fragment unchanged, so materials without detail normals pay nothing.
#include "view3d-12/src/shaders/hlsl/forward/far_clip_fade.hlsl"

// Detail-normal layer constants. The slope map uses the PBR normal-map slot (t12/s7), which simple materials do not otherwise use.
ConstantBuffer<CBufDetailNormals> g_detail : register(b7);

// Return 'In' with its world normal tilted by the summed height gradient of the detail-normal layers.
PSIn ApplyDetailNormals(PSIn In)
{
	// Fragments without an interpolated normal keep the stock fallback normal.
	float3 normal = In.ws_norm.xyz;
	if (dot(normal, normal) == 0.0f)
		return In;

	normal = normalize(normal);

	// Each layer's map slope is per texture unit, so the chain rule through the projection rows gives the world-space height gradient.
	float3 gradient = float3(0, 0, 0);
	for (int i = 0; i != g_detail.info.x; ++i)
	{
		// Sample the layer's slope in its own projection and accumulate its world gradient.
		float4 row_u = g_detail.row_u[i];
		float4 row_v = g_detail.row_v[i];
		float2 uv = float2(dot(In.ws_vert.xyz, row_u.xyz) + row_u.w, dot(In.ws_vert.xyz, row_v.xyz) + row_v.w);
		float2 slope = g_normal_texture.Sample(g_normal_sampler, uv).xy * 2.0f - 1.0f;
		gradient += g_detail.height_scale[i] * (slope.x * row_u.xyz + slope.y * row_v.xyz);
	}

	// A height field displaced along the normal tilts the normal against the tangential part of its gradient.
	gradient -= dot(gradient, normal) * normal;
	In.ws_norm = float4(normalize(normal - gradient), 0);
	return In;
}

// Forward PS with detail normals.
PSOut PSForwardDetail(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	return PSForward(ApplyDetailNormals(In), is_front_face);
}

// Forward PS with detail normals that also writes reflection attributes for RT reflections.
PSReflectionOut PSForwardDetailReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	return PSForwardReflectionAttrs(ApplyDetailNormals(In), is_front_face);
}

// Collect transparent detail-normal fragments into the alpha K-buffer.
void PSForwardDetailAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSForwardAlphaCollect(ApplyDetailNormals(In), is_front_face);
}

// Far-clip-fade variant of PSForwardDetail.
PSOut PSFarFadeDetail(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	return PSFarFade(ApplyDetailNormals(In), is_front_face);
}

// Far-clip-fade variant of PSForwardDetailReflectionAttrs.
PSReflectionOut PSFarFadeDetailReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	return PSFarFadeReflectionAttrs(ApplyDetailNormals(In), is_front_face);
}

// Far-clip-fade variant of PSForwardDetailAlphaCollect.
void PSFarFadeDetailAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSFarFadeAlphaCollect(ApplyDetailNormals(In), is_front_face);
}
