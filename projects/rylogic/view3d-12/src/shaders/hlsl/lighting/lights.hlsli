//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
// Light model functions shared by the Phong and PBR lighting paths.
#ifndef PR_VIEW3D_SHADER_LIGHTS_HLSLI
#define PR_VIEW3D_SHADER_LIGHTS_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"

// Return the fraction of 'light' that reaches 'ws_pos' due to range, distance falloff, and the spot cone.
// Returns 0 when 'ws_pos' is outside the light's volume, so callers can skip the rest of the lighting for that light.
float LightAttenuation(Light light, float4 ws_pos)
{
	// Directional lights have no position, so they reach everywhere with full strength
	if (DirectionalLight(light))
		return 1.0f;

	// Point and spot lights do not reach beyond their range. A zero range is a light that reaches nothing.
	float4 light_to_pos = ws_pos - light.ws_position;
	float dist = length(light_to_pos);
	float range = light.spot.z;
	if (dist >= range)
		return 0.0f;

	// Fade out over the last part of the range so the light has no hard edge, and apply the distance falloff
	float attenuation = saturate((range - dist) * 9.0f / range);
	attenuation *= saturate(1.0f / (1.0f + light.spot.w * dist));

	// Spot lights fade between the inner and outer cone angles. The angles are full cone angles, hence the factor of 2.
	if (SpotLight(light))
	{
		float angle = 2.0f * acos(saturate(dot(light_to_pos, light.ws_direction) / max(dist, TINY)));
		attenuation *= saturate((light.spot.y - angle) / max(light.spot.y - light.spot.x, TINY));
	}

	return attenuation;
}

// Return the normalised direction that light from 'light' travels when it reaches 'ws_pos'
float4 LightDirectionAt(Light light, float4 ws_pos)
{
	return DirectionalLight(light)
		? light.ws_direction
		: normalize(ws_pos - light.ws_position);
}

// Return the fraction of 'light' that is not blocked by shadow casters.
// 'shadow_visibility' is the visibility sampled from the shadow map, which belongs to the light with 'HasShadow' set.
float LightShadowVisibility(Light light, float shadow_visibility)
{
	// The shadow strength blends between unshadowed and fully shadowed
	return HasShadow(light)
		? lerp(1.0f, shadow_visibility, saturate(light.shadow.x))
		: 1.0f;
}

#endif
