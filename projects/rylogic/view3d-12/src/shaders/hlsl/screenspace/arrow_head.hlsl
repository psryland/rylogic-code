//***********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#include "pr/hlsl/vector.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

ConstantBuffer<CBufFrame> g_frame : register(b0);
ConstantBuffer<CBufScreenSpace> g_ss : register(b3);

// Converts point geometry into arrow heads
// Uses ss_vert for centre position, and ws_norm as the arrow forward direction
[maxvertexcount(3)]
void GSArrowHead(point PSIn In[1], inout TriangleStream<PSIn> OutStream)
{
	PSIn Out;

	// Size (in pixels) of the sprite. Use the size in the input tex0.xy unless it's zero,
	float w = In[0].tex0.x > 0.0001f ? In[0].tex0.x : g_ss.size.x * 0.5f;
	float h = In[0].tex0.y > 0.0001f ? In[0].tex0.y : g_ss.size.y * 0.5f;
	
	// The output normal is the camera forward vector
	float4 ws_norm = g_frame.cam.c2w[2];

	if (g_ss.depth)
	{
		// Arrow head direction is in 'In[0].ws_norm'
		float4 tang = In[0].ws_norm;
		float4 perp = NormaliseOrZero(float4(cross(ws_norm.xyz, tang.xyz), 0));

		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (1.0f * tang) * h;
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);

		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-0.6f * tang + 0.8f * perp) * w;
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	
		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-0.6f * tang - 0.8f * perp) * w;
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.ws_norm = ws_norm;
		Out.tex0 = float2(0, 0); // null out the tex0
		OutStream.Append(Out);
	}
	else
	{
		// Arrow head direction is in 'In[0].ws_norm'
		float4 ss_norm = mul(In[0].ws_norm, g_frame.cam.w2s);

		// Arrow direction and perpendicular in screen space
		float2 dir = normalize(ss_norm.xy * g_ss.screen_dim.xy);
		float2 tang = dir / g_ss.screen_dim.xy;
		float2 perp = float2(-dir.y, dir.x) / g_ss.screen_dim.xy;

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
