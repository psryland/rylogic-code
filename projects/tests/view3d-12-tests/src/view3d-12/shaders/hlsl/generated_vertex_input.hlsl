#include "pr/view3d-12/shaders/vertex.hlsli"

cbuffer GeneratedVertexConstants : register(b0)
{
	float4 g_colour;
	uint4 g_count;
}

RWStructuredBuffer<View3DVertex> g_vertices : register(u0);
StructuredBuffer<float4> g_positions : register(t0);

// Copy each vertex position from the caller's input buffer, so the rendered triangle exists only if the input reached t0.
numthreads(CSMain, 64, 1, 1)
void CSMain(uint vertex_id : SV_DispatchThreadID)
{
	// Threads past the vertex count belong to the partial final group.
	if (vertex_id >= g_count.x)
		return;

	View3DVertex output;
	output.vert = g_positions[vertex_id];
	output.diff = g_colour;
	output.norm = float4(0, 0, 1, 0);
	output.tex0 = float2(0, 0);
	output.idx0 = int2(0, 0);
	g_vertices[vertex_id] = output;
}
