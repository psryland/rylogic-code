//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared types for the underwater post-processing pass.
// This file is included from both HLSL and C++ source.
#ifndef PR_VIEW3D_UNDERWATER_CBUF_HLSLI
#define PR_VIEW3D_UNDERWATER_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::shaders::post
{
	using namespace pr::hlsl;
#endif

// Underwater constant buffer. Bound to b0.
struct CBufUnderwater //:reg(b0)
{
	// Transform from normalised device coordinates (x,y in [-1,+1], z = depth buffer value) to camera space.
	row_major float4x4 s2c;

	// Linear colour multiplied into the scene colour, and the linear colour that distance fades towards.
	float4 tint;
	float4 fog_colour;

	// The scene viewport on the render target, in pixels (x, y, width, height).
	float4 viewport;

	// Camera-space water surface plane (xyz = normal out of the water, w = offset), or zero when the whole view is in water.
	float4 surface;

	// Height of the camera's near plane above the water surface, as 'x*ndc.x + y*ndc.y + z'. Used only when 'split' is non-zero.
	float4 waterline;

	// Distance at which the fog hides 95% of a surface.
	float visibility;

	// Animation phase of the distortion in radians, in [0, 2pi).
	float phase;

	// Largest distortion offset as a fraction of the viewport height, and ripples per viewport height.
	float distortion_amplitude;
	float distortion_frequency;

	// Non-zero when a resolved depth buffer is bound, and when the camera is orthographic.
	int has_depth;
	int orthographic;

	// The depth buffer clear value. Pixels with this depth have no scene geometry.
	float clear_depth;

	// Non-zero when the water surface crosses the near plane. Only pixels below the waterline then show the effect.
	int split;
};

#ifdef __cplusplus
}
#endif
#endif
