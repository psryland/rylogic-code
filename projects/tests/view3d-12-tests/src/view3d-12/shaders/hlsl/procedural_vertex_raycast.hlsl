#include "procedural_vertex_common.hlsli"

ConstantBuffer<View3DRayCastNugget> g_nugget : register(VIEW3D_RAYCAST_NUGGET_REGISTER);
ConstantBuffer<ProceduralConstants> g_procedural : register(VIEW3D_PROCEDURAL_RAYCAST_CONSTANTS_REGISTER);

// Emit generated geometry through the public RayCast contract.
View3DRayCastVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Feed the stock topology geometry shader the same world position used by Forward.
	return View3DProceduralRayCastVertex(ProceduralPosition(vertex_id, g_procedural), g_nugget);
}
