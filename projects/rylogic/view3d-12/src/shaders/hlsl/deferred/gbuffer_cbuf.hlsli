//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// Constant buffer definitions for gbuffer shader
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_GBUFFER_CBUF_HLSL
#define PR_VIEW3D_SHADER_GBUFFER_CBUF_HLSL

#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Camera to world transform
cbuffer CBufCamera :reg(b0)
{
	Camera m_cam;
	float4 m_frustum[4]; // View frustum corners in camera space
};

// Global lighting
cbuffer CBufLighting :reg(b1)
{
	Light m_light;
};

// Constants per render nugget.
cbuffer CBufNugget :reg(b2)
{
	// Sync with:
	//   forward_cbuf.hlsli
	//   shadow_map_cbuf.hlsli
	//   gbuffer_cbuf.hlsli

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
};

#if SHADER_BUILD

#define HAS_NORMALS (m_flags.x == 1)
#define HAS_TEX0    (m_flags.y == 1)

#endif

#endif