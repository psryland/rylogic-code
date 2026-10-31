#include "population_procedural_common.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Generate a quad vertex and place it at the population instance of this draw.
View3DForwardVertexOut VSMain(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Find the instance, build the vertex through the procedural contract, then apply the instance placement
	View3DPopulationEntry entry = View3DPopulationFetch(g_pop_view, g_pop_draw, instance_id);
	View3DPopulationInstance inst = g_pop_instances[entry.source];
	View3DForwardVertexOut v = View3DProceduralForwardVertex(PopulationProceduralPosition(vertex_id, g_procedural), g_procedural.normal, g_procedural.colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
	return View3DPopulationForwardVertex(v, inst, entry, g_frame);
}
