//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//***********************************************
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/skinned/skinned.hlsli"

struct Vert
{
	float4 vert;
	float4 diff;
	float4 norm;
	float2 tex0;
	int2 idx0;
};

struct CBufSkinning
{
	uint vertex_count;
	uint skin_count;
	uint2 pad;
	row_major float4x4 model_to_object;
	row_major float4x4 object_to_model;
};

ConstantBuffer<CBufSkinning> resource(g_skinning, b0);
StructuredBuffer<Vert> resource(g_rest_vertices, t0);
StructuredBuffer<Skinfluence> resource(g_skin, t1);
StructuredBuffer<Mat4x4> resource(g_pose, t2);
RWStructuredBuffer<Vert> resource(g_output_vertices, u0);

// Write one current-pose vertex, keeping the output in model-space so the existing render transforms still apply.
numthreads(CSSkinning, 128, 1, 1)
void CSSkinning(uint3 DTID(DTid))
{
	uint vertex_index = DTid.x;
	if (vertex_index >= g_skinning.vertex_count)
		return;

	Vert vert = g_rest_vertices[vertex_index];

	// Dead/unreferenced vertex slots can contain NoIndex (-1). Raster skips them, but compute visits every vertex.
	if (vert.idx0.x < 0 || (uint)vert.idx0.x >= g_skinning.skin_count)
	{
		g_output_vertices[vertex_index] = vert;
		return;
	}

	uint skin_index = (uint)vert.idx0.x;
	Skinfluence influence = g_skin[skin_index];

	float4 os_vert = mul(vert.vert, g_skinning.model_to_object);
	float4 os_norm = mul(vert.norm, g_skinning.model_to_object);
	os_vert = SkinVertex(g_pose, influence, os_vert);
	os_norm = SkinNormal(g_pose, influence, os_norm);

	vert.vert = mul(os_vert, g_skinning.object_to_model);
	vert.norm = mul(os_norm, g_skinning.object_to_model);
	g_output_vertices[vertex_index] = vert;
}
