#include "population_procedural_common.hlsli"

ConstantBuffer<View3DShadowDrawViews> g_draw : register(VIEW3D_SHADOW_DRAW_VIEWS_REGISTER);
StructuredBuffer<View3DShadowView> g_shadow_views : register(VIEW3D_SHADOW_VIEWS_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_SHADOW_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DShadowElement> g_elements : register(VIEW3D_SHADOW_ELEMENTS_REGISTER);
static const View3DShadowElement g_nugget = g_elements[g_element.index];
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_SHADOW_CONSTANTS_REGISTER);

// Generate a quad vertex with the same deformation as the forward wrapper, and place it at the population instance in the draw's shadow view.
View3DShadowVertexOut VSMain(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Find the instance, then project the generated vertex into the single shadow view of a population draw
	View3DPopulationEntry entry = View3DPopulationFetch(g_pop_view, g_pop_draw, instance_id);
	View3DPopulationInstance inst = g_pop_instances[entry.source];
	return View3DPopulationShadowVertex(PopulationProceduralPosition(vertex_id, g_procedural), g_procedural.colour, float2(0, 0), g_draw, g_shadow_views, g_nugget, inst, entry);
}
