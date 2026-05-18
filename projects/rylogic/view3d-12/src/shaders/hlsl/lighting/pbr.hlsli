//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
#ifndef PR_VIEW3D_SHADER_PBR_HLSLI
#define PR_VIEW3D_SHADER_PBR_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

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

// Return a surface-to-light vector for the active light type.
float3 PbrLightDirection(Light light, float3 ws_pos)
{
	return
		DirectionalLight(light) ? normalize(-light.ws_direction.xyz) :
		PointLight(light)       ? normalize(light.ws_position.xyz - ws_pos) :
		SpotLight(light)        ? normalize(light.ws_position.xyz - ws_pos) :
		float3(0, 0, 0);
}

// Return a direct-light attenuation multiplier for the active light type.
float PbrLightAttenuation(Light light, float3 ws_pos)
{
	if (SpotLight(light))
	{
		float3 light_to_pos = ws_pos - light.ws_position.xyz;
		float dist = length(light_to_pos);
		float angle = 2.0f * acos(saturate(dot(light_to_pos, light.ws_direction.xyz) / max(dist, TINY)));
		float cone = saturate((light.spot.y - angle) / max(light.spot.y - light.spot.x, TINY));
		float range = saturate((light.spot.z - dist) * 9.0f / max(light.spot.z, TINY));
		float falloff = saturate(1.0f / (1.0f + light.spot.w * dist));
		return cone * range * falloff;
	}

	return 1.0f;
}

// Evaluate the direct-lighting PBR model for one global light.
float3 PbrIlluminate(Light light_info, float3 ws_pos, float3 normal, float3 view, float light_visible, float3 albedo, float metallic, float roughness, float3 emissive)
{
	float3 light = PbrLightDirection(light_info, ws_pos);
	float3 half_vector = normalize(view + light);
	float n_dot_l = saturate(dot(normal, light));
	float n_dot_v = saturate(dot(normal, view));

	float3 f0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
	float3 fresnel = PbrFresnelSchlick(saturate(dot(half_vector, view)), f0);
	float distribution = PbrDistributionGGX(normal, half_vector, roughness);
	float geometry = PbrGeometrySmith(normal, view, light, roughness);
	float3 specular = distribution * geometry * fresnel / max(4.0f * n_dot_v * n_dot_l, TINY);
	float3 diffuse = (1.0f - fresnel) * (1.0f - metallic) * albedo / (0.5f * tau);

	float attenuation = PbrLightAttenuation(light_info, ws_pos);
	float3 radiance = light_info.colour.rgb * light_visible * attenuation;
	float3 ambient = light_info.ambient.rgb * albedo;
	float light_intensity = light_info.ambient.a;

	return light_intensity * (ambient + (diffuse + specular) * radiance * n_dot_l) + emissive;
}

#endif
