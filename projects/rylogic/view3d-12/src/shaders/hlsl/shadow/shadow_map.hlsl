//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/camera.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/shadow/shadow_map_cbuf.hlsli"

// Constant buffers
ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufNugget> resource(g_nugget, b1);

// Texture2D /w sampler
Texture2D<float4> resource(m_texture0, t0);
SamplerState      resource(m_sampler0, s0);

struct PSIn_ShadowMap
{
	float4 ss_vert :SV_POSITION;
	float4 ws_vert :POSITION1;
	float4 diff :COLOR0;
	float2 tex0 :TEXCOORD0;
};
struct PSOut
{
	float shade :SV_TARGET;
};

// Default SMAP VS
PSIn_ShadowMap VSShadowMap(VSIn In)
{
	PSIn_ShadowMap Out = (PSIn_ShadowMap)0;
	
	// Transform
	float4 os_vert = mul(In.vert, g_nugget.m2o);
	
	float4 ws_vert = mul(os_vert, g_nugget.o2w);
	float4 ls_vert = mul(ws_vert, g_frame.w2l);
	float2 nf = ClipPlanes(g_frame.l2s);

	// Transform. Set ws_vert.w to normalised distance from light
	Out.ws_vert = ws_vert;
	Out.ws_vert.w = Frac(nf.y, -ls_vert.z, nf.x);
	Out.ss_vert = mul(ls_vert, g_frame.l2s);

	// Tinting
	Out.diff = g_nugget.tint;

	// Per Vertex colour
	Out.diff = In.diff * Out.diff;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0, 0, 1), g_nugget.tex2surf0).xy;

	return Out;
}

// Default SMAP PS
PSOut PSShadowMap(PSIn_ShadowMap In)
{
	PSOut Out = (PSOut)0;
	
	float4 diff = In.diff;

	// Texture2D (with transform)
	if (HasTex0(g_nugget.flags))
		diff = m_texture0.Sample(m_sampler0, In.tex0) * diff;

	// If not alpha blending, clip alpha pixels
	if (!HasAlpha(g_nugget.flags))
		clip(diff.a - 0.5);

	Out.shade = In.ws_vert.w;
	return Out;
}

