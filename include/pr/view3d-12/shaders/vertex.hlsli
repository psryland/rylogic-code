//*********************************************
// View 3D canonical vertex contract
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#ifndef PR_VIEW3D_VERTEX_HLSLI
#define PR_VIEW3D_VERTEX_HLSLI
#include "pr/hlsl/interop.hlsli"

// Canonical buffered vertex consumed by View3D raster shaders and written by GPU-generated object compute shaders.
struct View3DVertex
{
	float4 vert semantic(POSITION0);
	float4 diff semantic(COLOR0);
	float4 norm semantic(NORMAL0);
	float2 tex0 semantic(TEXCOORD0);
	int2   idx0 semantic(INDICES0);
};

#endif
