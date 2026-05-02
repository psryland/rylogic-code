//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#ifndef PR_VIEW3D_SHADER_KBUFFER_RESOLVE_CBUF_HLSLI
#define PR_VIEW3D_SHADER_KBUFFER_RESOLVE_CBUF_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufKBufferResolve// :reg(b0)
{
	int2 screen_dim;
	int sample_count;
	int pad0;
};

#endif
