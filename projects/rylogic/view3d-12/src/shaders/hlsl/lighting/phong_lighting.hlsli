//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_PHONG_LIGHTING_HLSLI
#define PR_VIEW3D_SHADER_PHONG_LIGHTING_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Returns the intensity of reflected directional light on a surface with normal 'ws_norm' and transparency 'alpha'
float LightDirectional(in float4 ws_light_direction, in float4 ws_norm, in float alpha)
{
	float brightness = -dot(ws_light_direction, ws_norm);
	return lerp(saturate(brightness), (1.0 - alpha) * abs(brightness), 1.0 - alpha);
}

// Returns the intensity of reflected radial light on a surface at position 'ws_pos' with normal 'ws_norm' and transparency 'alpha'
float LightPoint(in float4 ws_light_position, in float4 ws_norm, in float4 ws_pos, in float alpha)
{
	float4 light_to_pos = ws_pos - ws_light_position;
	float dist = length(light_to_pos);
	float brightness = -dot(light_to_pos, ws_norm) / dist;
	return lerp(saturate(brightness), (1.0 - alpha) * abs(brightness), 1.0 - alpha);
}

// Returns the intensity of reflected radial light on a surface at position 'ws_pos' with normal 'ws_norm' and transparency 'alpha'
float LightSpot(in float4 ws_light_position, in float4 ws_light_direction, in float inner_angle, in float outer_angle, in float range, in float falloff, in float4 ws_norm, in float4 ws_pos, in float alpha)
{
	float brightness = LightPoint(ws_light_position, ws_norm, ws_pos, alpha);
	float4 light_to_pos = ws_pos - ws_light_position;
	float dist = length(light_to_pos);
	float angle = 2.0f * acos(saturate(dot(light_to_pos, ws_light_direction) / dist));
	brightness *= saturate((outer_angle - angle) / (outer_angle - inner_angle));
	brightness *= saturate((range - dist) * 9 / range);
	brightness *= saturate(1.0f / (1.0f + falloff*dist));
	return brightness;
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

// Return calibrated ambient plus Lambert diffuse lighting for simple materials.
float3 LambertLighting(in Light light, float intensity, float light_visible, float3 colour)
{
	float3 ambient = light.ambient.rgb * colour;
	float3 diffuse = (intensity * light_visible * light.colour.rgb * colour) / (0.5f * tau);
	return light.ambient.a * (ambient + diffuse);
}

// Return the colour due to lighting. Returns unlit_diff if ws_norm is zero
float4 Illuminate(in Light light, float4 ws_pos, float4 ws_norm, float4 ws_cam, float light_visible, float4 unlit_diff)
{
	// Notes:
	//  - Lighting should not change the alpha value.
	//    If the thing was semi transparent coming in, casting light on it shouldn't change it.
	//  - Simple lighting is calibrated to approximately match PBR for rough dielectric materials under the same global light values.
	//  - Light intensity is carried in 'light.ambient.a' and scales ambient plus direct light, but not emissive/unlit colour.

	float has_norm = dot(ws_norm, ws_norm); // 1 for normals, 0 for not
	if (has_norm <= TINY)
	{
		return unlit_diff;
	}

	float4 ws_toeye_norm = normalize(ws_cam - ws_pos);
	float4 ws_light_dir =
		DirectionalLight(light) ? light.ws_direction :
		PointLight(light)       ? normalize(ws_pos - light.ws_position) :
		SpotLight(light)        ? normalize(ws_pos - light.ws_position) :
		float4(0, 0, 0, 0);

	float intensity = 0;
	if (DirectionalLight(light))
		intensity = LightDirectional(light.ws_direction, ws_norm, unlit_diff.a);
	else if (PointLight(light))
		intensity = LightPoint(light.ws_position, ws_norm, ws_pos, unlit_diff.a);
	else if (SpotLight(light))
		intensity = LightSpot(light.ws_position, light.ws_direction, light.spot.x, light.spot.y, light.spot.z, light.spot.w, ws_norm, ws_pos, unlit_diff.a);

	float3 diffuse = LambertLighting(light, intensity, light_visible, unlit_diff.rgb);
	float3 specular = intensity != 0.0f
		? light.ambient.a * intensity * light_visible * light.specular.rgb * LightSpecular(ws_light_dir, light.specular.a, ws_norm, ws_toeye_norm, unlit_diff.a)
		: float3(0, 0, 0);

	return float4(saturate(diffuse + specular), unlit_diff.a);
}

#endif
