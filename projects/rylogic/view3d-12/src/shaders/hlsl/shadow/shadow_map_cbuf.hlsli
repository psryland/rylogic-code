//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// Constant buffer definitions for shadow map shader
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_SHADOW_MAP_CBUF_HLSL
#define PR_VIEW3D_SHADER_SHADOW_MAP_CBUF_HLSL
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Camera to world transform and view frustum
struct CBufFrame //:reg(b0)
{
	// Not using 'Shadow' from types because the shadow map generation
	// shaders operate on one shadow map at a time. The main render can
	// support multiple shadow maps.
	row_major float4x4 w2l; // World space to light space
	row_major float4x4 l2s; // Light space to screen space
};

// Constants per render nugget.
struct CBufNugget //:reg(b1)
{
	// Sync with:
	//   forward_cbuf.hlsli
	//   shadow_map_cbuf.hlsli
	//   gbuffer_cbuf.hlsli

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
};

#endif

