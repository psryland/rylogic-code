// Stock PBR forward pixel family for materials that read all textures through TEXCOORD_0.
#include "pr/view3d-12/shaders/forward_pixel_pbr.hlsli"

// Shade a PBR fragment with the stock PBR forward shading.
float4 StockShadePbr(inout PSIn In, bool is_front_face)
{
	// The stock family adds nothing to the shared shading.
	return ForwardShadePbr(In, is_front_face).diff;
}

// Generate the forward pixel family for PBR materials.
VIEW3D_FORWARD_PBR_PIXEL_ENTRY_POINTS(ForwardPbr, StockShadePbr)
