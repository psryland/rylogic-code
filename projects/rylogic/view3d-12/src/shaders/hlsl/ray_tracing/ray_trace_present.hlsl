//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************

struct RTPresentIn
{
	float4 ss_vert :SV_Position;
	float2 tex0 :TEXCOORD0;
};

Texture2D<float4> g_rt_output : register(t0);

// Generate a fullscreen triangle for presenting the ray tracing output texture.
RTPresentIn VSRayTracePresent(uint vid :SV_VertexID)
{
	RTPresentIn Out = (RTPresentIn)0;
	Out.tex0 = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(Out.tex0 * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}

// Copy the ray tracing output texture to the current render target.
float4 PSRayTracePresent(RTPresentIn In) :SV_Target
{
	return g_rt_output.Load(int3(In.ss_vert.xy, 0));
}
