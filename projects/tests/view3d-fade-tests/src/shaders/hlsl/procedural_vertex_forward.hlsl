#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DForwardShadowNugget> g_nugget : register(VIEW3D_FORWARD_NUGGET_REGISTER);
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Emit generated geometry through the public Forward contract.
View3DForwardVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Preserve the stock pixel-stage contract while sourcing all geometry from the logical ID.
	return View3DProceduralForwardVertex(ProceduralPosition(vertex_id, g_procedural), float4(0, 0, 1, 0), g_procedural.colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
