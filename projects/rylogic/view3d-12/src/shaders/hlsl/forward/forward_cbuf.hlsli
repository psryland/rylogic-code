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
cbuffer CBufFrame :reg(b0)
{
	// Camera transform
	Camera m_cam;
	
	// Global lighting
	Light m_global_light;

	// EnvMap
	EnvMap m_env_map;

	// Shadows
	Shadow m_shadow;

	// Projected textures
	ProjTexture m_proj_tex;
};

// Constants per render nugget.
cbuffer CBufNugget :reg(b1)
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
	int4 m_flags;

	// Object transform
	row_major float4x4 m_m2o; // model to object space
	row_major float4x4 m_o2w; // object to world
	row_major float4x4 m_o2s; // object to screen
	row_major float4x4 m_n2w; // normal to world

	// Texture2D
	row_major float4x4 m_tex2surf0; // texture to surface transform

	// Tinting
	float4 m_tint; // object tint colour

	// EnvMap
	float m_env_reflectivity; // Reflectivity of the environment map
	float3 ng_pad0;
};

// Constants used for radial fading.
cbuffer CBufFade :reg(b2)
{
	// The centre of the fade region. Set to (0,0,0,0) to use the camera position
	float4 m_fade_centre;
	
	// x = Fade starting radius
	// y = Fade ending radius
	float2 m_fade_radius;

	// 0 = Spherical fade
	// 1 = Cylindrical fade
	int m_fade_type;
};

// Constants used for screen space geometry shaders.
cbuffer CBufScreenSpace :reg(b3)
{
	float2 m_screen_dim; // x = screen width, y = screen height, 
	float2 m_size;       // x = width in pixels, y = height in pixels
	int m_depth;         // True if depth scaling should be used
	int ss_pad0, ss_pad1, ss_pad2; // Padding for alignment
};

// Constants used for diagnostic shaders
cbuffer CBufDiag :reg(b3) //can b3 be reused?
{
	float4 m_colour;
	float  m_length;
	int    m_diag_pad[3];
};

#endif
