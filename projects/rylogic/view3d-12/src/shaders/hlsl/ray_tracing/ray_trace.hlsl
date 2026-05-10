//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/shaders/hlsl/ray_tracing/ray_tracing_cbuf.hlsli"

#ifdef PR_RDR_LSHADER_ray_trace

ConstantBuffer<CBufFrame> g_frame : register(b0);
RaytracingAccelerationStructure g_scene : register(t0);
RWTexture2D<float4> g_output : register(u0);

struct RayPayload
{
	float4 colour;
};

// Return a stable pseudo-random colour for diagnostic object/primitive ids.
float3 DiagnosticColour(uint seed)
{
	seed ^= seed >> 16;
	seed *= 0x7feb352du;
	seed ^= seed >> 15;
	seed *= 0x846ca68bu;
	seed ^= seed >> 16;

	float3 colour = float3(
		((seed >> 0) & 0xFFu) / 255.0f,
		((seed >> 8) & 0xFFu) / 255.0f,
		((seed >> 16) & 0xFFu) / 255.0f);
	return lerp(colour, float3(1.0f, 1.0f, 1.0f), 0.20f);
}

// Convert the dispatch pixel to a world-space camera ray using the same camera convention as View3D's raster path.
RayDesc MakeCameraRay(uint2 pixel, uint2 dim)
{
	float2 nss = float2(
		(2.0f * (pixel.x + 0.5f) / dim.x) - 1.0f,
		1.0f - (2.0f * (pixel.y + 0.5f) / dim.y));

	float half_height = g_frame.camera.z * tan(0.5f * g_frame.camera.y);
	float half_width = g_frame.camera.x * half_height;

	RayDesc ray = (RayDesc)0;
	ray.TMin = max(g_frame.clip.x, 0.001f);
	ray.TMax = g_frame.clip.y;

	if (g_frame.camera.w != 0.0f)
	{
		ray.Origin = g_frame.cam.c2w[3].xyz + nss.x * half_width * g_frame.cam.c2w[0].xyz + nss.y * half_height * g_frame.cam.c2w[1].xyz;
		ray.Direction = -g_frame.cam.c2w[2].xyz;
	}
	else
	{
		float3 cs_point = float3(nss.x * half_width, nss.y * half_height, -g_frame.camera.z);
		ray.Origin = g_frame.cam.c2w[3].xyz;
		ray.Direction = normalize(cs_point.x * g_frame.cam.c2w[0].xyz + cs_point.y * g_frame.cam.c2w[1].xyz + cs_point.z * g_frame.cam.c2w[2].xyz);
	}

	return ray;
}

[shader("raygeneration")]
void RayGen()
{
	uint2 pixel = DispatchRaysIndex().xy;
	uint2 dim = DispatchRaysDimensions().xy;

	RayPayload payload;
	payload.colour = float4(0.02f, 0.03f, 0.05f, 1.0f);

	RayDesc ray = MakeCameraRay(pixel, dim);
	TraceRay(g_scene, RAY_FLAG_FORCE_OPAQUE, 0xFF, 0, 1, 0, ray, payload);
	g_output[pixel] = payload.colour;
}

[shader("miss")]
void Miss(inout RayPayload payload)
{
	payload.colour = float4(0.02f, 0.03f, 0.05f, 1.0f);
}

[shader("closesthit")]
void ClosestHit(inout RayPayload payload, in BuiltInTriangleIntersectionAttributes attrib)
{
	uint seed = InstanceID() ^ (PrimitiveIndex() * 747796405u);
	float edge = min(attrib.barycentrics.x, min(attrib.barycentrics.y, 1.0f - attrib.barycentrics.x - attrib.barycentrics.y));
	float wire = smoothstep(0.0f, 0.02f, edge);
	payload.colour = float4(lerp(float3(0.02f, 0.02f, 0.02f), DiagnosticColour(seed), wire), 1.0f);
}

#endif

#if defined(PR_RDR_VSHADER_ray_trace_present) || defined(PR_RDR_PSHADER_ray_trace_present)

struct RTPresentIn
{
	float4 ss_vert :SV_Position;
	float2 tex0 :TEXCOORD0;
};

#ifdef PR_RDR_VSHADER_ray_trace_present
RTPresentIn main(uint vid :SV_VertexID)
{
	RTPresentIn Out = (RTPresentIn)0;
	Out.tex0 = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(Out.tex0 * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}
#endif

#ifdef PR_RDR_PSHADER_ray_trace_present
Texture2D<float4> g_rt_output : register(t0);

float4 main(RTPresentIn In) :SV_Target
{
	return g_rt_output.Load(int3(In.ss_vert.xy, 0));
}
#endif

#endif
