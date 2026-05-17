//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "view3d-12/src/shaders/hlsl/forward/kbuffer.hlsli"

Texture2D<float4>  g_opaque_colour :register(t0);
Texture2D<uint4>   g_alpha_colour :register(t1);
Texture2D<uint4>   g_alpha_depth :register(t2);

struct PSIn_KBufferResolve
{
	float4 ss_vert :SV_Position;
	float2 tex0 :TEXCOORD0;
};

PSIn_KBufferResolve VSFullScreenTriangle(uint vid :SV_VertexID)
{
	PSIn_KBufferResolve Out = (PSIn_KBufferResolve)0;
	Out.tex0 = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(Out.tex0 * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}

float4 PSAlphaResolve(PSIn_KBufferResolve In) :SV_Target
{
	int2 pix = int2(In.ss_vert.xy);
	float4 base = g_opaque_colour.Load(int3(pix, 0));
	uint4 alpha_colour = g_alpha_colour.Load(int3(pix, 0));
	uint4 alpha_depth = g_alpha_depth.Load(int3(pix, 0));

	// Merge the OIA alpha pixels over the opaque colour.
	float4 oia = UnpackOia(alpha_depth);
	base.rgb = base.rgb * (1.0f - oia.a) + oia.rgb * oia.a;

	// Merge each alpha layer from back to front.
	if (DepthOf(alpha_depth.w) != KBufferDepthFar)
	{
		float4 src = UnpackRGBA8(alpha_colour.w);
		base.rgb = base.rgb * (1.0f - src.a) + src.rgb * src.a;
	}
	if (DepthOf(alpha_depth.z) != KBufferDepthFar)
	{
		float4 src = UnpackRGBA8(alpha_colour.z);
		base.rgb = base.rgb * (1.0f - src.a) + src.rgb * src.a;
	}
	if (DepthOf(alpha_depth.y) != KBufferDepthFar)
	{
		float4 src = UnpackRGBA8(alpha_colour.y);
		base.rgb = base.rgb * (1.0f - src.a) + src.rgb * src.a;
	}
	if (DepthOf(alpha_depth.x) != KBufferDepthFar)
	{
		float4 src = UnpackRGBA8(alpha_colour.x);
		base.rgb = base.rgb * (1.0f - src.a) + src.rgb * src.a;
	}

	return base;
}

