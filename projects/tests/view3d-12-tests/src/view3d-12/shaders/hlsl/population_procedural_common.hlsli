#include "pr/view3d-12/shaders/population.hlsli"
#include "procedural_vertex_common.hlsli"

// Population bindings, shared by the forward and shadow wrappers
StructuredBuffer<View3DPopulationInstance> g_pop_instances : register(VIEW3D_POPULATION_INSTANCES_REGISTER);
ByteAddressBuffer g_pop_view : register(VIEW3D_POPULATION_VIEW_REGISTER);
ConstantBuffer<View3DPopulationDraw> g_pop_draw : register(VIEW3D_POPULATION_DRAW_REGISTER);

// Return the model-space corner 'vertex_id' (in [0,4)) of a 10x10 quad, moved by the offset in 'constants.positions[0]'.
// The offset stands in for per-frame deformation that only the procedural constants know about.
float4 PopulationProceduralPosition(uint vertex_id, ProceduralConstants constants)
{
	// Corners in counter-clockwise order: (-5,-5), (5,-5), (5,5), (-5,5)
	float x = (vertex_id == 1 || vertex_id == 2) ? 5.0f : -5.0f;
	float y = (vertex_id >= 2) ? 5.0f : -5.0f;
	return float4(x + constants.positions[0].x, y + constants.positions[0].y, constants.positions[0].z, 1);
}
