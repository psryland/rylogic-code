//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Vertex shader of the stock flat sprite batch. Each instance draws one ParticleMeshRecord as a soft octagonal disc in the record's local XY plane,
// turned by its orientation, so the disc lies on a surface rather than facing the camera. The disc's diameter is the record's size. The centre
// vertex is opaque and the rim is transparent. The disc uses the sprite index list (see particle_sprite.hlsli).
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/view3d-12/shaders/indirect_instances.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];

// The particle system's records buffer, one ParticleMeshRecord per instance.
ByteAddressBuffer g_records : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// Generate one vertex of the disc drawn by 'instance_id'. 'vertex_id' is the logical vertex: 0-7 on the rim, 8 at the centre.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 48;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	float4 orientation = asfloat(g_records.Load4(offset + 16));
	float4 colour = ParticleUnpackColour(g_records.Load(offset + 32));

	// Rim vertices sit on a circle of the disc's radius in the record's XY plane, and are transparent so the disc's edge is soft.
	float3 offset_ls = float3(0.0f, 0.0f, 0.0f);
	if (vertex_id < ParticleSpriteRimCount)
	{
		float sine;
		float cosine;
		sincos(6.283185307179586f * float(vertex_id) / float(ParticleSpriteRimCount), sine, cosine);
		offset_ls = 0.5f * pos_size.w * float3(cosine, sine, 0.0f);
		colour.a = 0.0f;
	}

	// Turn the disc into the object's space. Its normal is the record's local z.
	float3 ms_vert = pos_size.xyz + ParticleRotate(orientation, offset_ls);
	float3 ms_norm = ParticleRotate(orientation, float3(0.0f, 0.0f, 1.0f));
	return View3DProceduralForwardVertex(float4(ms_vert, 1.0f), float4(ms_norm, 0.0f), colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
