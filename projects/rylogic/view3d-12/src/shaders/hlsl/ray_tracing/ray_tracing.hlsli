//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
#ifndef PR_VIEW3D_SHADER_RAY_TRACING_HLSLI
#define PR_VIEW3D_SHADER_RAY_TRACING_HLSLI

// Helpers for forward passes that write the ray-tracing reflection and alpha side-buffer attributes.
// Requires the forward shader globals and ResolveWorldNormal() to be available at the include site.
float SelectTextureChannel(float4 sample, int channel);
float4 ResolveWorldNormal(PSIn In, bool is_front_face);

// Return the RT reflection side-buffer payload for the visible opaque simple-material surface.
float4 ReflectionAttributes(PSIn In, float4 diff, bool is_front_face)
{
	float reflectivity = saturate(g_nugget.env_reflectivity);
	if (!HasNormals(g_nugget.flags) || reflectivity == 0.0f || diff.a < 0.5f)
		return float4(0, 0, 0, 0);

	float3 normal = ResolveWorldNormal(In, is_front_face).xyz;
	if (dot(normal, normal) == 0.0f)
		return float4(0, 0, 0, 0);

	normal = normalize(normal);
	return float4(0.5f * normal + 0.5f, reflectivity);
}

// Return the RT reflection side-buffer payload for the visible opaque PBR surface.
float4 PbrReflectionAttributes(PSIn In, float4 diff, float2 metallic_uv, float3 normal)
{
	float reflectivity = saturate(g_pbr.metallic);
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasMetallicMap))
		reflectivity *= SelectTextureChannel(g_metallic_texture.Sample(g_metallic_sampler, metallic_uv), g_pbr.metallic_channel);

	if (!HasNormals(g_nugget.flags) || reflectivity == 0.0f || diff.a < 0.5f)
		return float4(0, 0, 0, 0);

	if (dot(normal, normal) == 0.0f)
		return float4(0, 0, 0, 0);

	normal = normalize(normal);
	return float4(0.5f * normal + 0.5f, reflectivity);
}

// Pack the RT alpha side-buffer payload for a resolved world normal and relative reflectivity.
uint AlphaRtAttributesFromNormal(float4 diff, float3 normal, float reflectivity)
{
	// Omit layers that cannot produce a visible reflected contribution.
	if (!HasNormals(g_nugget.flags) || diff.a <= 0.0f)
		return 0;

	if (dot(normal, normal) == 0.0f)
		return 0;

	normal = normalize(normal);
	return PackRGBA8(float4(0.5f * normal + 0.5f, saturate(reflectivity)));
}

// Return the RT alpha side-buffer payload for a transparent simple-material layer.
uint AlphaRtAttributes(PSIn In, float4 diff, bool is_front_face)
{
	// Resolve the simple-material normal before delegating common packing and validation.
	float3 normal = ResolveWorldNormal(In, is_front_face).xyz;
	return AlphaRtAttributesFromNormal(diff, normal, g_nugget.env_reflectivity);
}

#endif
