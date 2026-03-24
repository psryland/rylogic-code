//***********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#include "pr/hlsl/vector.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// Converts point geometry into arrow heads
// Uses ss_vert for centre position, and ws_norm as the arrow forward direction
void GSArrowHead(point PSIn In[1], inout TriangleStream<PSIn> OutStream)
{
	PSIn Out;

	// Size (in pixels) of the sprite. Use the size in the input tex0.xy unless it's zero,
	float w = In[0].tex0.x > 0.0001f ? In[0].tex0.x : m_size.x * 0.5f;
	float h = In[0].tex0.y > 0.0001f ? In[0].tex0.y : m_size.y * 0.5f;
	
	// The output normal is the camera forward vector
	float4 ws_norm = m_cam.m_c2w[2];

	if (m_depth)
	{
		// Arrow head direction is in 'In[0].ws_norm'
		float4 tang = In[0].ws_norm;
		float4 perp = NormaliseOrZero(float4(cross(ws_norm.xyz, tang.xyz), 0));

		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (1.0f * tang) * h;
		Out.ss_vert = mul(Out.ws_vert, m_cam.m_w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);

		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-0.6f * tang + 0.8f * perp) * w;
		Out.ss_vert = mul(Out.ws_vert, m_cam.m_w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	
		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-0.6f * tang - 0.8f * perp) * w;
		Out.ss_vert = mul(Out.ws_vert, m_cam.m_w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	}
	else
	{
		// Arrow head direction is in 'In[0].ws_norm'
		float4 ss_norm = mul(In[0].ws_norm, m_cam.m_w2s);

		// Arrow direction and perpendicular in screen space
		float2 dir = normalize(ss_norm.xy * m_screen_dim.xy);
		float2 tang = dir / m_screen_dim.xy;
		float2 perp = float2(-dir.y, dir.x) / m_screen_dim.xy;

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (1.0f * tang) * h * In[0].ss_vert.w;
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (-0.6f * tang + 0.8f * perp) * w * In[0].ss_vert.w;
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	
		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (-0.6f * tang - 0.8f * perp) * w * In[0].ss_vert.w;
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	}
	OutStream.RestartStrip();
}

#ifdef PR_RDR_GSHADER_arrow_head
[maxvertexcount(3)]
void main(point PSIn In[1], inout TriangleStream<PSIn> OutStream)
{
	GSArrowHead(In, OutStream);
}
#endif
