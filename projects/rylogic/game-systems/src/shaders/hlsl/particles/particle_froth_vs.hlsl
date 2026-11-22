//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Vertex shader of the stock froth particle type. Each instance draws one ParticleFrothRecord as a camera-facing octagon (see particle_sprite.hlsli),
// cut off at the plane of the water surface under the puff, so foam below the water is not drawn. The octagon has the same alpha everywhere, because
// the batch's texture gives each puff its outline. Record positions are in the object's space and sizes are in world units.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/view3d-12/shaders/indirect_instances.hlsli"
#include "pr/game-systems/particles/particle_froth.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];

// The particle system's records buffer, one ParticleFrothRecord per instance.
ByteAddressBuffer g_records : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);

// The records that the view may see, as indices into 'g_records'.
ByteAddressBuffer g_instances : register(VIEW3D_INDIRECT_INSTANCES_REGISTER);

// Generate one vertex of the puff drawn by 'instance_id'. 'vertex_id' is the logical vertex: 0-7 on the rim, 8 at the centre.
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID)
{
	// Read the record.
	uint offset = View3DIndirectInstance(g_instances, instance_id) * 32;
	float4 pos_size = asfloat(g_records.Load4(offset + 0));
	uint4 colour_angle_surface = g_records.Load4(offset + 16);
	float4 colour = ParticleUnpackColour(colour_angle_surface.x);

	// Find the vertex's place on the uncut octagon, in world space.
	float2 offset_cs;
	float2 uv;
	ParticleSpriteVertex(vertex_id, asfloat(colour_angle_surface.y), 0.5f * pos_size.w, offset_cs, uv);
	float3 offset_ws = offset_cs.x * g_frame.cam.c2w[0].xyz + offset_cs.y * g_frame.cam.c2w[1].xyz;

	// Cut the octagon off at the surface plane. The plane is moved to world space, which keeps it a plane when the object is scaled evenly.
	// 'centre_above' is the centre's distance above the plane. A rim vertex below the plane is pulled straight towards the centre until it lies on the
	// plane, and its texture coordinate moves with it, so the texture is not stretched. Each fan triangle is then cut along the line between its two rim
	// vertices, which follows the surface closely enough for an octagon. The cut edge lies on the water, where the soft depth fade hides it.
	float4x4 o2w = mul(g_nugget.m2o, g_nugget.o2w);
	float3 normal_ws = normalize(mul(float4(ParticleFrothUnpackNormal(colour_angle_surface.z), 0.0f), o2w).xyz);
	float centre_above = max(0.0f, dot(normal_ws, mul(float4(0.0f, 0.0f, asfloat(colour_angle_surface.w), 0.0f), o2w).xyz));
	float rim_above = centre_above + dot(normal_ws, offset_ws);
	if (rim_above < 0.0f)
	{
		// The fraction of the way from the centre to the rim vertex at which the plane is crossed.
		float s = centre_above / (centre_above - rim_above);
		offset_ws *= s;
		uv = 0.5f + s * (uv - 0.5f);
	}

	// Place the centre through the object transform, then offset it in world space so the puff faces the camera.
	View3DForwardVertexOut output = View3DProceduralForwardVertex(float4(pos_size.xyz, 1.0f), float4(0.0f, 0.0f, 0.0f, 0.0f), colour, uv, float2(0, 0), g_frame, g_nugget);
	output.ws_vert.xyz += offset_ws;
	output.ws_norm = float4(g_frame.cam.c2w[2].xyz, 0.0f);
	output.ss_vert = mul(output.ws_vert, g_frame.cam.w2s);
	return output;
}
