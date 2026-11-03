//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared types for the ambient occlusion passes.
// This file is included from both HLSL and C++ source.
#ifndef PR_VIEW3D_AMBIENT_OCCLUSION_CBUF_HLSLI
#define PR_VIEW3D_AMBIENT_OCCLUSION_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::shaders::post
{
	using namespace pr::hlsl;
#endif

// Ambient occlusion constant buffer. Bound to b0.
// "View space" here has x to the right, y down (matching pixel rows), and z the positive distance in front of the camera.
struct CBufAmbientOcclusion //:reg(b0)
{
	// Converts a depth buffer value 'd' to view-space z: 'z = -(x*d + y) / (z*d + w)'.
	float4 depth_params;

	// Converts an AO texel index (u, v) to view-space xy: '(x*u + z, y*v + w)', multiplied by the view-space z for perspective cameras.
	float4 view_params;

	// The scene viewport on the render target, in pixels (x, y, width, height).
	float4 viewport;

	// Size of the AO buffers in texels, and the number of viewport pixels per AO texel along each axis.
	int2 ao_size;
	int scale;

	// Non-zero when the camera is orthographic.
	int orthographic;

	// The depth buffer clear value. Pixels with this depth have no scene geometry.
	float clear_depth;

	// The world-space search radius, and the AO texels per world unit at view depth 1 (perspective) or at any depth (orthographic).
	float radius;
	float texels_per_unit;

	// Largest search radius in AO texels.
	float max_radius;

	// Occluder weight as a function of distance 'r': 'saturate(r * falloff_mul + falloff_add)'.
	float falloff_mul;
	float falloff_add;

	// Number of search directions, and samples in each direction on each side of the pixel.
	int slice_count;
	int step_count;

	// Power applied to the visibility. Larger values darken more.
	float intensity;
	float3 pad;
};

#ifdef __cplusplus
}
#endif
#endif
