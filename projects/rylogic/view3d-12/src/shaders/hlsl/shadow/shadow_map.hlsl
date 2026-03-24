//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/camera.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/shadow/shadow_map_cbuf.hlsli"

// Texture2D /w sampler
Texture2D<float4> resource(m_texture0, t0);
SamplerState      resource(m_sampler0, s0);

// Skinned Meshes
StructuredBuffer<Mat4x4> resource(m_pose, t4);
StructuredBuffer<Skinfluence> resource(m_skin, t5);

#include "view3d-12/src/shaders/hlsl/skinned/skinned.hlsli"

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
PSIn_ShadowMap VSDefault(VSIn In)
{
	PSIn_ShadowMap Out = (PSIn_ShadowMap)0;
	
	// Transform
	float4 os_vert = mul(In.vert, m_m2o);
	
	if (IsSkinned(m_flags))
	{
		os_vert = SkinVertex(m_pose, m_skin[In.idx0.x], os_vert);
	}

	float4 ws_vert = mul(os_vert, m_o2w);
	float4 ls_vert = mul(ws_vert, m_w2l);
	float2 nf = ClipPlanes(m_l2s);

	// Transform. Set ws_vert.w to normalised distance from light
	Out.ws_vert = ws_vert;
	Out.ws_vert.w = Frac(nf.y, -ls_vert.z, nf.x);
	Out.ss_vert = mul(ls_vert, m_l2s);

	// Tinting
	Out.diff = m_tint;

	// Per Vertex colour
	Out.diff = In.diff * Out.diff;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0, 0, 1), m_tex2surf0).xy;

	return Out;
}

// Default SMAP PS
PSOut PSDefault(PSIn_ShadowMap In)
{
	PSOut Out = (PSOut)0;
	
	float4 diff = In.diff;

	// Texture2D (with transform)
	if (HasTex0(m_flags))
		diff = m_texture0.Sample(m_sampler0, In.tex0) * diff;

	// If not alpha blending, clip alpha pixels
	if (!HasAlpha(m_flags))
		clip(diff.a - 0.5);

	Out.shade = In.ws_vert.w;
	return Out;
}

// Vertex shader
#ifdef PR_RDR_VSHADER_shadow_map
PSIn_ShadowMap main(VSIn In)
{
	PSIn_ShadowMap Out = VSDefault(In);
	return Out;
}
#endif

// Pixel shader
#ifdef PR_RDR_PSHADER_shadow_map
PSOut main(PSIn_ShadowMap In)
{
	PSOut Out = PSDefault(In);
	return Out;
}
#endif
