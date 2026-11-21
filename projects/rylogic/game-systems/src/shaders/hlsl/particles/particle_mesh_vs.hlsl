//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Vertex shader of the stock mesh particle batch. Each instance draws one ParticleMeshRecord as a copy of the batch's shared mesh, which is
// a ParticleMeshVertex for each logical vertex (see particle_mesh.hlsli).
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

// The batch's shared mesh, one ParticleMeshVertex per logical vertex.
ByteAddressBuffer g_mesh : register(VIEW3D_PROCEDURAL_BUFFER1_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// Generate mesh vertex 'vertex_id' of the copy drawn by 'instance_id'.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record and the mesh vertex.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 48;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	float4 orientation = asfloat(g_records.Load4(offset + 16));
	float4 colour = ParticleUnpackColour(g_records.Load(offset + 32));
	float4 pos_u = asfloat(g_mesh.Load4(vertex_id * 32 + 0));
	float4 norm_v = asfloat(g_mesh.Load4(vertex_id * 32 + 16));

	// Scale, turn and place the mesh vertex in the object's space.
	float3 ms_vert = pos_size.xyz + ParticleRotate(orientation, pos_u.xyz * pos_size.w);
	float3 ms_norm = ParticleRotate(orientation, norm_v.xyz);
	return View3DProceduralForwardVertex(float4(ms_vert, 1.0f), float4(ms_norm, 0.0f), colour, float2(pos_u.w, norm_v.w), float2(0, 0), g_frame, g_nugget);
}
