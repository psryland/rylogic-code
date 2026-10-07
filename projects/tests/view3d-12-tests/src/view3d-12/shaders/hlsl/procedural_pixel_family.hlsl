// Forward pixel family fixture: stock simple-material shading with its colour channels rotated, scaled by a pixel-stage read of the procedural constants.
#include "pr/view3d-12/shaders/forward_pixel.hlsli"
#include "procedural_vertex_common.hlsli"

ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Rotate the stock colour so the output proves both the stock shading and the caller's pixel stage ran.
float4 RotateShade(inout PSIn In, bool is_front_face)
{
	// 'positions[0].w' is 1 in every fixture; an unbound constant buffer would turn the output black.
	float4 diff = ForwardShade(In, is_front_face).diff;
	diff.rgb = diff.gbr * g_procedural.positions[0].w;
	return diff;
}

// Generate the six forward pixel entry points.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(Rotate, RotateShade)
