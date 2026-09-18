#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DShadowFrame> g_frame : register(VIEW3D_SHADOW_FRAME_REGISTER);
ConstantBuffer<View3DForwardShadowNugget> g_nugget : register(VIEW3D_SHADOW_NUGGET_REGISTER);
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_SHADOW_CONSTANTS_REGISTER);

// Emit generated geometry through the public ShadowMap contract.
View3DShadowVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Preserve the stock shadow pixel stage while sourcing positions from the same logical ID decoder.
	return View3DProceduralShadowVertex(ProceduralPosition(vertex_id, g_procedural), g_procedural.colour, float2(0, 0), g_frame, g_nugget);
}
