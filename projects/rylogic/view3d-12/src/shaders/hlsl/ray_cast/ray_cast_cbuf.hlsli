//***********************************************
// View 3d
//  Copyright © Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_RAY_CAST_CBUF_HLSL
#define PR_VIEW3D_SHADER_RAY_CAST_CBUF_HLSL
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// The maximum number of rays in a single pass
static const int MaxRays = 16;
static const int MaxIntercepts = 256;

// Keep in sync with ESnapType in 'ray_cast.h'
static const int ESnapType_None = 0;
static const int ESnapType_Vert = 1;
static const int ESnapType_EdgeMiddle = 2;
static const int ESnapType_FaceCentre = 3;
static const int ESnapType_Edge = 4;
static const int ESnapType_Face = 5;

// Keep in sync with ESnapMode in 'ray_cast.h'
static const int ESnapMode_Vert = 1 << 0;
static const int ESnapMode_Edge = 1 << 1;
static const int ESnapMode_Face = 1 << 2;
static const int ESnapMode_Perspective = 1 << 8; // If set, then snap distance scales with distance from the origin

// A single ray to cast
struct Ray
{
	float4 ws_origin;
	float4 ws_direction;
	
	// Combination of 'ESnapMode'. What sort of snapping to perform
	int snap_mode;

	// The snap distance.
	// If 'snap_mode' is not perspective, then this is the distance in world space units.
	// If 'snap_mode' is perspective, then distance is calculated from 'distance-from-ray-origin * snap_distance'.
	float snap_distance;

	int id;
	int pad0;
};

// The intercept between a ray and an object
struct Intercept
{
	float4 ws_intercept; // xyz = intercept, w = distance
	float4 ws_normal;    // Normal at the intercept point (if available)
	int    snap_type;    // See ESnapType
	int    ray_index;    // The index of the input ray that caused this intercept
	voidp  inst_ptr;     // The address of the instance that was hit
};

// Per-frame constants
struct CBufFrame //:reg(b0)
{
	// The rays to cast
	Ray rays[MaxRays];

	// The number of rays to cast
	uint ray_count;

	uint pad0;
	uint pad1;
	uint pad2;
};

// Per-nugget constants
struct CBufNugget //:reg(b1)
{
	// x = Model flags - See types.hlsli
	// y = Texture flags
	// z = Alpha flags
	// w = Instance Id
	int4 flags;

	// Object transform
	row_major float4x4 m2o; // model to object space
	row_major float4x4 o2w; // object to world
	row_major float4x4 n2w; // normal to world

	// Instance pointer
	voidp inst_ptr;
};

#endif