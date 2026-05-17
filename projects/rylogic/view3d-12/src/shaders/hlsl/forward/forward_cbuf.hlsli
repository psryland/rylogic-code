//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Constant buffer definitions for forward shaders.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_FORWARD_CBUF_HLSL
#define PR_VIEW3D_SHADER_FORWARD_CBUF_HLSL
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Constants per frame.
struct CBufFrame// :reg(b0)
{
	// Camera transform
	Camera cam;
	
	// Global lighting
	Light global_light;

	// EnvMap
	EnvMap env_map;

	// Shadows
	Shadow shadow;

	// Projected textures
	ProjTexture proj_tex;
};

// Constants per render nugget.
struct CBufNugget// :reg(b1)
{
	// Sync with:
	//   forward_cbuf.hlsli
	//   shadow_map_cbuf.hlsli
	//   gbuffer_cbuf.hlsli
	//   ray_cast.hlsli

	// x = Model flags - See types.hlsli
	// y = Texture flags
	// z = Alpha flags
	// w = Instance Id
	int4 flags;

	// Object transform
	row_major float4x4 m2o; // model to object space
	row_major float4x4 o2w; // object to world
	row_major float4x4 o2s; // object to screen
	row_major float4x4 n2w; // normal to world

	// Texture2D
	row_major float4x4 tex2surf0; // texture to surface transform

	// Tinting
	float4 tint; // object tint colour

	// EnvMap
	float env_reflectivity; // Reflectivity of the environment map
	float3 pad0;
};

// Constants used for radial fading.
struct CBufFade// :reg(b2)
{
	// The centre of the fade region. Set to (0,0,0,0) to use the camera position
	float4 fade_centre;
	
	// x = Fade starting radius
	// y = Fade ending radius
	float2 fade_radius;

	// 0 = Spherical fade
	// 1 = Cylindrical fade
	int fade_type;
	int pad0;
};

// Constants used for screen space geometry shaders.
struct CBufScreenSpace// :reg(b3)
{
	float2 screen_dim; // x = screen width, y = screen height, 
	float2 size;       // x = width in pixels, y = height in pixels
	int depth;         // True if depth scaling should be used
	int pad0;
	int pad1;
	int pad2;
};

// Constants for physically based materials.
struct CBufPbrSurface// :reg(b4)
{
	float4 base_colour; // Linear base-colour factor
	float4 emissive;    // Linear emissive factor
	float metallic;
	float roughness;
	float alpha_cutoff;
	int alpha_mode;     // 0 = opaque, 1 = mask, 2 = blend
};

// Constants used for diagnostic shaders.
struct CBufDiag// :reg(b5)
{
	float4 colour;
	float length;
	int pad0;
	int pad1;
	int pad2;
};

#endif
