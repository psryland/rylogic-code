//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Vertex shader of the stock sprite particle type. Each instance draws one record as a camera-facing octagon (see particle_sprite.hlsli).
// The centre vertex is opaque and the rim is transparent, so each sprite is a soft disc. A batch with a texture multiplies the disc by the texture,
// fitted around the rim. Record positions are in the object's space; the
// sprite's size is in world units, so sprites keep their size under any object scale.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/view3d-12/shaders/indirect_instances.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];

// The particle system's records buffer, one ParticleSpriteRecord per instance.
ByteAddressBuffer g_records : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// Generate one vertex of the sprite drawn by 'instance_id'. 'vertex_id' is the logical vertex: 0-7 on the rim, 8 at the centre.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 32;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	uint4 colour_angle_streak = g_records.Load4(offset + 16);
	float4 colour = ParticleUnpackColour(colour_angle_streak.x);

	// Rim vertices sit on a circle of the sprite's radius in the camera's view plane, turned by the sprite's angle, and are transparent.
	float radius = 0.5f * pos_size.w;
	float2 offset_cs;
	float2 uv;
	if (ParticleSpriteVertex(vertex_id, asfloat(colour_angle_streak.y), radius, offset_cs, uv))
		colour.a = 0.0f;

	// A streak stretches the disc along the streak's direction on screen, so the sprite spans from its centre back to its tail. Rim points are pushed
	// out along the streak in proportion to how far they already lie along it, which keeps the outline smooth. The streak is in the object's space.
	float3 streak = float3(f16tof32(colour_angle_streak.z), f16tof32(colour_angle_streak.z >> 16), f16tof32(colour_angle_streak.w));
	float3 streak_ws = mul(mul(float4(streak, 0.0f), g_nugget.m2o), g_nugget.o2w).xyz;
	float2 streak_cs = float2(dot(streak_ws, g_frame.cam.c2w[0].xyz), dot(streak_ws, g_frame.cam.c2w[1].xyz));
	float streak_len = length(streak_cs);
	if (streak_len > 1e-6f && radius > 0.0f)
	{
		float2 dir = streak_cs / streak_len;
		offset_cs += dir * dot(offset_cs, dir) * streak_len / (2.0f * radius);
	}

	// Place the centre through the object transform, then offset it in world space so the sprite faces the camera, and back to the middle of the streak.
	View3DForwardVertexOut output = View3DProceduralForwardVertex(float4(pos_size.xyz, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), colour, uv, float2(0, 0), g_frame, g_nugget);
	output.ws_vert.xyz += offset_cs.x * g_frame.cam.c2w[0].xyz + offset_cs.y * g_frame.cam.c2w[1].xyz - 0.5f * streak_ws;
	output.ws_norm = float4(g_frame.cam.c2w[2].xyz, 0.0f);
	output.ss_vert = mul(output.ws_vert, g_frame.cam.w2s);
	return output;
}
