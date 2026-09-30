#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DForwardNugget> g_nugget : register(VIEW3D_FORWARD_NUGGET_REGISTER);
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);
ByteAddressBuffer g_buffer : register(VIEW3D_PROCEDURAL_BUFFER_REGISTER);

// Emit generated geometry whose colour comes from the last float4 of the caller's 1024-element immutable buffer.
View3DForwardVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Reading the final element proves the whole caller buffer was copied, not only its first bytes.
	float4 colour = asfloat(g_buffer.Load4(1023 * 16));
	return View3DProceduralForwardVertex(ProceduralPosition(vertex_id, g_procedural), g_procedural.normal, colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
