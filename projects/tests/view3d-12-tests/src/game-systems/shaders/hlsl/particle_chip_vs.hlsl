//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Vertex shader of the test chip particle type. Each instance draws one ParticleChipRecord as a lit box. The box's proportions come from the
// batch constants, so one record layout serves chips of any shape. See particle_chip.hlsli.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/view3d-12/shaders/indirect_instances.hlsli"
#include "particle_chip.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
ConstantBuffer<ParticleChipDrawConstants> g_chip : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];

// The particle system's records buffer, one ParticleChipRecord per instance.
ByteAddressBuffer g_records : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// Rotate 'v' by the unit quaternion 'q'.
float3 Rotate(float4 q, float3 v)
{
	return v + 2.0f * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

// Generate one vertex of the chip drawn by 'instance_id'. 'vertex_id' is the logical vertex: four corners for each of the six faces.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 48;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	float4 orientation = asfloat(g_records.Load4(offset + 16));
	float4 colour = ParticleUnpackColour(g_records.Load(offset + 32));

	// Face 'f' lies on the box side with normal +/- axis f/2. Its corners go around the face in the other two axes.
	uint face = vertex_id / 4;
	uint corner = vertex_id % 4;
	uint axis = face / 2;
	float sign = (face % 2) == 0 ? 1.0f : -1.0f;
	float2 uv = float2(corner == 1 || corner == 2 ? 1.0f : -1.0f, corner >= 2 ? 1.0f : -1.0f);
	float3 local = axis == 0 ? float3(sign, uv.x, uv.y) : axis == 1 ? float3(uv.x, sign, uv.y) : float3(uv.x, uv.y, sign);
	float3 normal = axis == 0 ? float3(sign, 0, 0) : axis == 1 ? float3(0, sign, 0) : float3(0, 0, sign);

	// Scale the unit box to the chip's proportions, then turn and place it.
	float3 half_extent = 0.5f * pos_size.w * float3(1.0f, g_chip.width, g_chip.thickness);
	float3 ms_vert = pos_size.xyz + Rotate(orientation, local * half_extent);
	float3 ms_norm = Rotate(orientation, normal);
	return View3DProceduralForwardVertex(float4(ms_vert, 1.0f), float4(ms_norm, 0.0f), colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}
