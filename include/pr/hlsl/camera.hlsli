//***********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_HLSL_CAMERA_HLSLI
#define PR_HLSL_CAMERA_HLSLI

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Returns the near (x) and far (y) plane distances in camera space from a reversed depth
// projection matrix (either perspective or orthographic), where the near plane maps to depth 1.
inline float2 ClipPlanes(float4x4 c2s)
{
	// These terms give the plane that maps to depth 0 first. Under reversed depth that is the far plane.
	float2 dist;
	dist.x = c2s._43 / c2s._33; // far
	dist.y = c2s._43 / (1 + c2s._33) + c2s._44 * (c2s._43 - c2s._33 - 1) / (c2s._33 * (1 + c2s._33)); // near
	return dist.yx;
}

#ifdef __cplusplus
}
#endif
#endif
