//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// Constant buffer definitions for shadow map shader
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_SHADOW_MAP_CBUF_HLSL
#define PR_VIEW3D_SHADER_SHADOW_MAP_CBUF_HLSL
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// The shadow views that one draw call renders into. Provided as root constants.
// Each instance of the draw renders into one view: instance 'i' uses view 'views[i/4][i%4]' in the frame's shadow view buffer.
// The instance also renders into viewport 'views[i/4][i%4] - info.x', where 'info.x' is the first view of the current batch of viewports.
struct CBufDrawViews //:reg(b0)
{
	uint4 views[4]; // Indices of the shadow views to render into, one per instance
	uint4 info;     // x = index of the shadow view bound to viewport 0, yzw = unused
};

// Constants per render nugget.
struct CBufNugget //:reg(b1)
{
	// Sync with:
	//   forward_cbuf.hlsli
	//   shadow_map_cbuf.hlsli

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
	float4 colour_blend; // linear override RGB; unused by depth-only shadow shading

	// EnvMap
	float env_reflectivity; // Reflectivity of the environment map
	float3 far_clip_fade; // Reserved to match the forward per-nugget layout
};

#endif
