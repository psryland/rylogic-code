//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
// Light and shadow types shared by the forward and ray tracing constant buffers.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_LIGHTING_CBUF_HLSLI
#define PR_VIEW3D_SHADER_LIGHTING_CBUF_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// The maximum number of lights that shade a frame. Must match 'rdr12::MaxLights' in light.h.
static const int MaxLights = 64;

// Light types. Must match 'rdr12::ELight'.
static const int LightType_Directional = 0;
static const int LightType_Point = 1;
static const int LightType_Spot = 2;

// A world space light. The frame's lights are provided in a structured buffer, with the count in the frame constants.
struct Light
{
	int4   info;         // x = light type, y = first shadow view (-1 = none), z = shadow view count, w = flags (unused)
	float4 ws_direction; // World space direction of directional and spot lights (w = 0)
	float4 ws_position;  // World space position of point and spot lights (w = 1)
	float4 colour;       // .rgb = diffuse colour, .a = intensity. Intensity scales diffuse and specular light
	float4 specular;     // .rgb = specular colour, .a = specular power
	float4 spot;         // x = inner angle, y = outer angle (full cone angles, radians), z = range, w = falloff
	float4 shadow;       // x = shadow strength in [0,1]
};

// The maximum number of shadow views in a frame. Must match 'rdr12::MaxShadowViews' in shadow_view.h.
static const int MaxShadowViews = 32;

// The maximum number of cascades for a directional light. Must match 'rdr12::MaxShadowCascades' in shadow_view.h.
static const int MaxShadowCascades = 4;

// One depth render of the scene from a shadow-casting light, stored in a region of the shadow atlas.
// The frame's shadow views are provided in a structured buffer. Lights refer to their views by index.
// Directional lights have one view per cascade, ordered from nearest to furthest from the camera.
struct ShadowView
{
	row_major float4x4 w2s; // World space to clip space for the view (depth in [0,1])
	float4 atlas_rect;      // Region of the atlas in UV units: xy = size, zw = offset
	float4 bias;            // x = receiver normal offset (world units, or world units per unit distance for spot and point lights), y = filter width in texels (5 or 7),
	                        // zw = distances from the camera (start, end) over which the shadow fades to fully lit (end = 0 means no fade)
};

// Light types
inline bool DirectionalLight(Light light) { return light.info.x == LightType_Directional; }
inline bool PointLight(Light light)       { return light.info.x == LightType_Point; }
inline bool SpotLight(Light light)        { return light.info.x == LightType_Spot; }

// True if the light has shadow views
inline bool HasShadow(Light light) { return light.info.y >= 0; }

#endif
