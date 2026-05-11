//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Constant buffer definitions for ray tracing shaders.
// This file is included from C++ source as well
#ifndef PR_VIEW3D_SHADER_RAY_TRACING_CBUF_HLSLI
#define PR_VIEW3D_SHADER_RAY_TRACING_CBUF_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

static const int RayTracingMode_Diagnostic = 0;
static const int RayTracingMode_HardShadows = 1;
static const int RayTracingMode_Reflections = 2;
static const int RayTracingMode_Caustics = 3;
static const int RayTracingMode_ReflectionsAndCaustics = 4;

static const int RayTracingInstanceMask_Default = 0x01;
static const int RayTracingInstanceMask_Caustic = 0x02;
static const int RayTracingInstanceMask_All = 0xFF;

// Constants per ray tracing dispatch.
struct CBufFrame// :reg(b0)
{
	// Camera transform
	Camera cam;

	// Screen to world projection
	row_major float4x4 s2w;

	// Global lighting
	Light global_light;

	// x = aspect, y = vertical FOV, z = focus distance, w = orthographic flag
	float4 camera;

	// x = near clip, y = far clip
	float4 clip;

	// x = shadow strength, y = minimum shadow-ray bias
	float4 shadow;

	// x = reflection strength, y = minimum reflection-ray bias
	float4 reflection;

	// x = caustic strength, y = minimum caustic-ray bias, z = projected caustic pattern scale, w = glass thickness approximation
	float4 caustic;

	// x = RayTracingMode_*
	int4 options;
};

#endif
