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

static const int RayTracingGeometryFlag_HasGeometry = 0x01;
static const int RayTracingGeometryFlag_HasNormals = 0x02;
static const int RayTracingGeometryFlag_Index16 = 0x04;
static const int RayTracingGeometryFlag_Index32 = 0x08;

static const int RayTracingMaterialFlag_Reflective = 0x01;
static const int RayTracingMaterialFlag_Transmissive = 0x02;

// Material data for one TLAS instance geometry.
struct RayTracingMaterial
{
	float4 diffuse;

	// x = reflectivity, y = transmission, z = index of refraction, w = approximate thickness
	float4 optics;

	// x = RayTracingMaterialFlag_*, yzw = reserved
	uint4 flags;
};

// Vertex data copied into a packed RT shading buffer.
struct RayTracingVertex
{
	float4 position;
	float4 colour;
	float4 normal;
	float2 tex0;
	int2 idx0;
};

// Geometry metadata for one TLAS instance geometry, indexed using InstanceID() + GeometryIndex().
struct RayTracingGeometry
{
	// x = vertex offset, y = vertex base, z = index offset, w = index count
	uint4 ranges;

	// x = RayTracingGeometryFlag_*, y = vertex count, zw = reserved
	uint4 flags;

	// Transform interpolated model-space normals into world space.
	row_major float4x4 normal_to_world;
};

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

	// x = reflection strength, y = minimum reflection-ray bias, z = maximum reflection bounces, w = minimum reflection throughput
	float4 reflection;

	// x = caustic strength, y = minimum caustic-ray bias, z = projected caustic pattern scale, w = glass thickness approximation
	float4 caustic;

	// x = RayTracingMode_*, y = material count, z = geometry count, w = geometry fallback count
	int4 options;
};

#endif
