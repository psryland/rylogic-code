#include "pr/view3d-12/shaders/procedural_vertex.hlsli"

struct ProceduralConstants
{
	float4 positions[3];
	float4 colour;
};

// Decode a sparse logical vertex domain that proves indexed IDs are not limited by the placeholder vertex buffer.
float4 ProceduralPosition(uint vertex_id, ProceduralConstants constants)
{
	// The focused fixture publishes exactly three IDs above the U16 range.
	return constants.positions[vertex_id - 70000];
}
