//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************

#include "view3d-12/src/shaders/hlsl/deferred/gbuffer_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/deferred/gbuffer.hlsli"

ConstantBuffer<CBufNugget> g_nugget : register(b2);
Texture2D<float4> g_texture0 :register(t0);
SamplerState g_sampler0 :register(s0);

// Vertex shader
#ifdef PR_RDR_VSHADER_gbuffer
PSIn main(VSIn In)
{
	PSIn Out;

	// Transform
	Out.ss_vert = mul(In.vert, g_nugget.o2s);
	Out.ws_vert = mul(In.vert, g_nugget.o2w);
	Out.ws_norm = mul(In.norm, g_nugget.n2w);

	// Tinting
	Out.diff = g_nugget.tint;

	// Per Vertex colour
	Out.diff = In.diff * Out.diff;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0,0,1), g_nugget.tex2surf0).xy;

	return Out;
}
#endif

// Pixel shader
#ifdef PR_RDR_PSHADER_gbuffer
PSOut_GBuffer main(PSIn In)
{
	// Transform
	float4 ws_vert = In.ws_vert;
	float4 ws_norm = HasNormals(g_nugget.flags) ? normalize(In.ws_norm) : float4(0,0,0,0);

	// Tinting
	float4 diff = In.diff;

	// Texture2D (with transform)
	if (HasTex0(g_nugget.flags))
		diff = g_texture0.Sample(g_sampler0, In.tex0) * diff;

	// Generate gbuffer output
	PSOut_GBuffer Out = WriteGBuffer(diff, ws_vert, ws_norm);
	return Out;
}
#endif
