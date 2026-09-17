// Surface RGB override shared by raster and ray-traced shading.
#ifndef PR_VIEW3D_SURFACE_COLOUR_HLSLI
#define PR_VIEW3D_SURFACE_COLOUR_HLSLI
#include "pr/hlsl/interop.hlsli"

// Blend linear surface RGB towards override.xyz by override.w in [0,1], preserving surface alpha exactly.
odr float4 SurfaceColourBlend(float4 surface, float4 colour_blend)
{
	// The disabled path is an exact identity, including values outside the display range.
	if (colour_blend.w == 0.0f)
		return surface;

	// Opacity belongs to the original surface, not to the override.
	return float4(lerp(surface.xyz, colour_blend.xyz, colour_blend.w), surface.w);
}
#endif
