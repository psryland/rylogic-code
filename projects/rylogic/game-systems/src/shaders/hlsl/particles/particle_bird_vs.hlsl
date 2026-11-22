//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Vertex shader of the stock bird particle type. Each instance draws one ParticleBirdRecord as a small procedural bird: a body, two wings that
// bend at the elbow as they flap, and a tail (see particle_bird.hlsli). The batch's index buffer selects the logical vertices below, so the
// batch has no vertex data. Birds are flat silhouettes, so they are drawn unlit in the record's colour.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/view3d-12/shaders/indirect_instances.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];

// The particle system's records buffer, one ParticleBirdRecord per instance.
ByteAddressBuffer g_records : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// The bird's logical vertices in its local frame (+y forward, +x right), in units of the wingspan, before the wings flap:
//  0 = Head, 1 = Rump, 2/5 = Right/left elbow front, 3/6 = Right/left elbow back, 4/7 = Right/left wing tip, 8/9 = Right/left tail tip.
static const float2 BirdOutline[10] =
{
	float2(0.00f, 0.25f), float2(0.00f, -0.15f),
	float2(0.22f, 0.06f), float2(0.22f, -0.10f), float2(0.50f, -0.12f),
	float2(-0.22f, 0.06f), float2(-0.22f, -0.10f), float2(-0.50f, -0.12f),
	float2(0.07f, -0.30f), float2(-0.07f, -0.30f),
};

// Generate logical vertex 'vertex_id' of the bird drawn by 'instance_id'.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 48;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	float4 orientation = asfloat(g_records.Load4(offset + 16));
	uint2 colour_flap = g_records.Load2(offset + 32);
	float4 colour = ParticleUnpackColour(colour_flap.x);
	float flap = asfloat(colour_flap.y);

	// Wing vertices lift by the flap angle about the body's axis. The tip bends a further half of the flap at the elbow, so the wing curves.
	float2 outline = BirdOutline[vertex_id];
	float3 ls_vert = float3(outline, 0.0f);
	if (vertex_id >= 2 && vertex_id <= 7)
	{
		float side = vertex_id <= 4 ? 1.0f : -1.0f;
		float elbow_x = abs(BirdOutline[2].x);
		float s0;
		float c0;
		sincos(flap, s0, c0);
		ls_vert = float3(side * elbow_x * c0, outline.y, elbow_x * s0);
		if (vertex_id == 4 || vertex_id == 7)
		{
			float s1;
			float c1;
			sincos(1.5f * flap, s1, c1);
			float reach = abs(outline.x) - elbow_x;
			ls_vert += float3(side * reach * c1, 0.0f, reach * s1);
		}
	}

	// Scale, turn and place the vertex in the object's space.
	float3 ms_vert = pos_size.xyz + ParticleRotate(orientation, ls_vert * pos_size.w);
	return View3DProceduralForwardVertex(float4(ms_vert, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
