//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Constant buffer definitions for ray tracing shaders.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_RAY_TRACING_CBUF_HLSLI
#define PR_VIEW3D_SHADER_RAY_TRACING_CBUF_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Constants per ray tracing diagnostic dispatch.
struct CBufFrame// :reg(b0)
{
	// Camera transform
	Camera cam;

	// x = aspect, y = vertical FOV, z = focus distance, w = orthographic flag
	float4 camera;

	// x = near clip, y = far clip
	float4 clip;
};

#endif
