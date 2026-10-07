// Stock simple-material forward pixel family.
#include "pr/view3d-12/shaders/forward_pixel.hlsli"

// Shade a simple-material fragment with the stock forward shading.
float4 StockShade(inout PSIn In, bool is_front_face)
{
	// The stock family adds nothing to the shared shading.
	return ForwardShade(In, is_front_face).diff;
}

// Generate the forward pixel family for simple materials.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(Forward, StockShade)
