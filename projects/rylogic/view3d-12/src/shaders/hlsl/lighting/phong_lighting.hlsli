//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_PHONG_LIGHTING_HLSLI
#define PR_VIEW3D_SHADER_PHONG_LIGHTING_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lights.hlsli"

// Returns the intensity of light travelling in 'ws_light_dir' that reflects from a surface with normal 'ws_norm' and transparency 'alpha'.
// Semi-transparent surfaces are also lit from behind, in proportion to their transparency.
float LightFacing(in float4 ws_light_dir, in float4 ws_norm, in float alpha)
{
	float brightness = -dot(ws_light_dir, ws_norm);
	return lerp(saturate(brightness), (1.0 - alpha) * abs(brightness), 1.0 - alpha);
}

// Returns the normalised Blinn-Phong specular response for a given incident light direction and surface normal.
float LightSpecular(in float4 ws_light_direction, in float specular_power, in float4 ws_norm, in float4 ws_toeye_norm, in float alpha)
{
	float4 ws_H = normalize(ws_toeye_norm - ws_light_direction);
	float brightness = dot(ws_norm, ws_H);
	brightness = lerp(saturate(brightness), (1.0 - alpha) * abs(brightness), 1.0 - alpha);
	specular_power = max(specular_power, 1.0f);
	return ((specular_power + 8.0f) / (4.0f * tau)) * pow(saturate(brightness), specular_power);
}

// Return the colour due to scene lighting. Returns 'unlit_diff' if 'ws_norm' is zero.
// 'lights' contains 'light_count' world space lights. 'shadow_visibility' is the visibility sampled from the shadow map.
float4 Illuminate(StructuredBuffer<Light> lights, int light_count, float3 ambient, float4 ws_pos, float4 ws_norm, float4 ws_cam, float shadow_visibility, float4 unlit_diff)
{
	// Notes:
	//  - Lighting should not change the alpha value.
	//    If the thing was semi transparent coming in, casting light on it shouldn't change it.
	//  - Simple lighting is calibrated to approximately match PBR for rough dielectric materials under the same light values.
	//  - Light intensity is carried in 'light.colour.a' and scales diffuse and specular light, but not ambient or emissive/unlit colour.

	// Surfaces without normals cannot be lit
	float has_norm = dot(ws_norm, ws_norm); // 1 for normals, 0 for not
	if (has_norm <= TINY)
	{
		return unlit_diff;
	}

	// Ambient light is applied once, independent of the lights
	float4 ws_toeye_norm = normalize(ws_cam - ws_pos);
	float3 lit = ambient * unlit_diff.rgb;

	// Accumulate the diffuse and specular light from each light
	for (int i = 0; i != light_count; ++i)
	{
		// Skip lights that cannot reach this point
		Light light = lights[i];
		float attenuation = LightAttenuation(light, ws_pos);
		if (attenuation <= 0.0f)
			continue;

		// Skip lights that do not face the surface
		float4 ws_light_dir = LightDirectionAt(light, ws_pos);
		float intensity = attenuation * LightFacing(ws_light_dir, ws_norm, unlit_diff.a);
		if (intensity <= 0.0f)
			continue;

		// Lambert diffuse plus Blinn-Phong specular, scaled by the light intensity and shadowing
		float scale = intensity * light.colour.a * LightShadowVisibility(light, shadow_visibility);
		float3 diffuse = (light.colour.rgb * unlit_diff.rgb) / (0.5f * tau);
		float3 specular = light.specular.rgb * LightSpecular(ws_light_dir, light.specular.a, ws_norm, ws_toeye_norm, unlit_diff.a);
		lit += scale * (diffuse + specular);
	}

	return float4(saturate(lit), unlit_diff.a);
}

#endif
