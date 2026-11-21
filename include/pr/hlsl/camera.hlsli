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

// Returns the depth in front of the camera, measured along the camera's view axis in camera space, of a point at reversed screen depth 'depth',
// for a perspective or orthographic projection 'c2s'. This inverts the depth row of the projection, so it is exact at any screen position.
inline float ViewDistance(float4x4 c2s, float depth)
{
	// Screen depth is (z*_33 + _43) / (z*_34 + _44) for camera-space z. Camera space looks down -z, so the distance is -z.
	return (c2s._43 - depth * c2s._44) / (c2s._33 - depth * c2s._34);
}

#ifdef __cplusplus
}
#endif
#endif
