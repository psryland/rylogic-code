//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
// Light and shadow types shared by the forward and ray tracing constant buffers.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_LIGHTING_CBUF_HLSLI
#define PR_VIEW3D_SHADER_LIGHTING_CBUF_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

static const int MaxShadowMaps = 1;

// Lights
struct Light
{
	// x = light type = 0 - ambient, 1 - directional, 2 - point, 3 - spot
	int4   info;         // Encoded info for global lighting
	float4 ws_direction; // The direction of the global light source
	float4 ws_position;  // The position of the global light source
	float4 ambient;      // .rgb = ambient light colour, .a = light intensity scale
	float4 colour;       // The colour of the directional light
	float4 specular;     // The colour of the specular light. alpha channel is specular power
	float4 spot;         // x = inner angle, y = outer angle, z = range, w = falloff
};

// Shadows
struct Shadow
{
	int4 info;  // x = count of smaps, y = smap size
	row_major float4x4 w2l[MaxShadowMaps]; // World space to light space
	row_major float4x4 l2s[MaxShadowMaps]; // Light space to shadow map space
};

// Light types
inline bool AmbientLight(Light light)     { return light.info.x == 0; }
inline bool DirectionalLight(Light light) { return light.info.x == 1; }
inline bool PointLight(Light light)       { return light.info.x == 2; }
inline bool SpotLight(Light light)        { return light.info.x == 3; }

// Shadows
inline int ShadowMapCount(Shadow shdw) { return shdw.info.x; }

#endif
