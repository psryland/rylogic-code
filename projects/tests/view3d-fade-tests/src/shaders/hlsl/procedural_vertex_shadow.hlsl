#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DShadowDrawViews> g_draw : register(VIEW3D_SHADOW_DRAW_VIEWS_REGISTER);
StructuredBuffer<View3DShadowView> g_shadow_views : register(VIEW3D_SHADOW_VIEWS_REGISTER);
ConstantBuffer<View3DShadowNugget> g_nugget : register(VIEW3D_SHADOW_NUGGET_REGISTER);
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_SHADOW_CONSTANTS_REGISTER);

// Emit generated geometry through the public ShadowMap contract.
View3DShadowVertexOut VSMain(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Preserve the stock shadow pixel stage while sourcing positions from the same logical ID decoder.
	return View3DProceduralShadowVertex(ProceduralPosition(vertex_id, g_procedural), g_procedural.colour, float2(0, 0), instance_id, g_draw, g_shadow_views, g_nugget);
}
