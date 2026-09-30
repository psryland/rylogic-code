//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
#ifndef PR_VIEW3D_SHADER_PBR_HLSLI
#define PR_VIEW3D_SHADER_PBR_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lights.hlsli"

// GGX/Trowbridge-Reitz normal distribution term.
float PbrDistributionGGX(float3 normal, float3 half_vector, float roughness)
{
	float a = roughness * roughness;
	float a2 = a * a;
	float n_dot_h = saturate(dot(normal, half_vector));
	float n_dot_h2 = n_dot_h * n_dot_h;
	float denom = n_dot_h2 * (a2 - 1.0f) + 1.0f;
	return a2 / max(0.5f * tau * denom * denom, TINY);
}

// Schlick-GGX masking term for one light/view direction.
float PbrGeometrySchlickGGX(float n_dot_v, float roughness)
{
	float r = roughness + 1.0f;
	float k = (r * r) / 8.0f;
	return n_dot_v / max(n_dot_v * (1.0f - k) + k, TINY);
}

// Smith geometry term using Schlick-GGX for both view and light directions.
float PbrGeometrySmith(float3 normal, float3 view, float3 light, float roughness)
{
	float n_dot_v = saturate(dot(normal, view));
	float n_dot_l = saturate(dot(normal, light));
	return PbrGeometrySchlickGGX(n_dot_v, roughness) * PbrGeometrySchlickGGX(n_dot_l, roughness);
}

// Schlick Fresnel approximation.
float3 PbrFresnelSchlick(float cos_theta, float3 f0)
{
	return f0 + (1.0f - f0) * pow(saturate(1.0f - cos_theta), 5.0f);
}

// Evaluate the direct-lighting PBR model for the scene lights.
// 'lights' contains 'light_count' world space lights. Shadows are sampled with 'SampleLightShadow', see lights.hlsli.
float3 PbrIlluminate(StructuredBuffer<Light> lights, int light_count, float3 ambient, float3 ws_pos, float3 normal, float3 view, float3 albedo, float metallic, float roughness, float3 emissive)
{
	// Ambient and emissive light are applied once, independent of the lights
	float n_dot_v = saturate(dot(normal, view));
	float3 f0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
	float3 colour = ambient * albedo + emissive;

	// Accumulate the direct light from each light
	for (int i = 0; i != light_count; ++i)
	{
		// Skip lights that cannot reach this point
		Light light_info = lights[i];
		float attenuation = LightAttenuation(light_info, float4(ws_pos, 1.0f));
		if (attenuation <= 0.0f)
			continue;

		// Skip lights behind the surface
		float3 light = -LightDirectionAt(light_info, float4(ws_pos, 1.0f)).xyz;
		float n_dot_l = saturate(dot(normal, light));
		if (n_dot_l <= 0.0f)
			continue;

		// Cook-Torrance specular plus energy-conserving Lambert diffuse
		float3 half_vector = normalize(view + light);
		float3 fresnel = PbrFresnelSchlick(saturate(dot(half_vector, view)), f0);
		float distribution = PbrDistributionGGX(normal, half_vector, roughness);
		float geometry = PbrGeometrySmith(normal, view, light, roughness);
		float3 specular = distribution * geometry * fresnel / max(4.0f * n_dot_v * n_dot_l, TINY);
		float3 diffuse = (1.0f - fresnel) * (1.0f - metallic) * albedo / (0.5f * tau);

		// Scale by the light colour, intensity, attenuation, and shadowing
		float3 radiance = light_info.colour.rgb * light_info.colour.a * attenuation * LightShadowVisibility(light_info, float4(ws_pos, 1.0f), float4(normal, 0.0f));
		colour += (diffuse + specular) * radiance * n_dot_l;
	}

	return colour;
}

#endif
