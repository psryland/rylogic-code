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

// The index of the current draw's entry in the element constants table. Provided as a root constant.
struct CBufElement //:reg(b1)
{
	uint index;
};

// Constants per drawlist element. The render step uploads one entry per drawlist element as a structured buffer (t2).
// Only the data needed for depth and alpha cut-out is included.
struct ElementConstants
{
	// Sync with:
	//   View3DShadowElement in pr/view3d-12/shaders/procedural_vertex.hlsli

	// x = Model flags - See types.hlsli
	// y = Texture flags
	// z = Alpha flags
	// w = Instance Id
	int4 flags;

	// Object transform
	row_major float4x4 m2o; // model to object space
	row_major float4x4 o2w; // object to world

	// Texture2D
	row_major float4x4 tex2surf0; // texture to surface transform

	// Tinting
	float4 tint; // object tint colour
};

#endif
