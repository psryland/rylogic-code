#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DShadowDrawViews> g_draw : register(VIEW3D_SHADOW_DRAW_VIEWS_REGISTER);
StructuredBuffer<View3DShadowView> g_shadow_views : register(VIEW3D_SHADOW_VIEWS_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_SHADOW_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DShadowElement> g_elements : register(VIEW3D_SHADOW_ELEMENTS_REGISTER);
static const View3DShadowElement g_nugget = g_elements[g_element.index];
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_SHADOW_CONSTANTS_REGISTER);

// Emit generated geometry through the public ShadowMap contract.
View3DShadowVertexOut VSMain(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Preserve the stock shadow pixel stage while sourcing positions from the same logical ID decoder.
	return View3DProceduralShadowVertex(ProceduralPosition(vertex_id, g_procedural), g_procedural.colour, float2(0, 0), instance_id, g_draw, g_shadow_views, g_nugget);
}
