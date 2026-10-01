#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Emit generated geometry through the public Forward contract.
View3DForwardVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Preserve the stock pixel-stage contract while sourcing all geometry from the logical ID.
	return View3DProceduralForwardVertex(ProceduralPosition(vertex_id, g_procedural), g_procedural.normal, g_procedural.colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
