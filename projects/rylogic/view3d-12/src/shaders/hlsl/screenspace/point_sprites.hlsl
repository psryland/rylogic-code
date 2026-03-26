//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#include "pr/hlsl/core.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// Constant buffers
ConstantBuffer<CBufFrame> g_frame : register(b0);
ConstantBuffer<CBufScreenSpace> g_ss : register(b3);

// Converts point geometry into billboard quads
void GSPointSprites(point PSIn In[1], inout TriangleStream<PSIn> OutStream)
{
	PSIn Out;

	// Size (in pixels) of the sprite. Use the size in the input tex0.xy unless it's zero,
	float w = In[0].tex0.x > 0.0001f ? In[0].tex0.x : g_ss.size.x * 0.5f;
	float h = In[0].tex0.y > 0.0001f ? In[0].tex0.y : g_ss.size.y * 0.5f;

	// The output normal is the camera forward vector
	float4 ws_norm = g_frame.cam.c2w[2];

	// Output a camera facing quad: bottom to top 'S' order.
	if (g_ss.depth)
	{
		float4 radx = w * g_frame.cam.c2w[0];
		float4 rady = h * g_frame.cam.c2w[1];

		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-radx - rady);
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.tex0.xy = float2(0, 0);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);
		
		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (+radx - rady);
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.tex0.xy = float2(1, 0);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);
		
		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (-radx + rady);
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.tex0.xy = float2(0, 1);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);
		
		Out = In[0];
		Out.ws_vert = In[0].ws_vert + (+radx + rady);
		Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
		Out.tex0.xy = float2(1, 1);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);
	}
	else
	{
		// Screen space only. No depth test or distance scaling
		float2 radx = float2(w / g_ss.screen_dim.x, 0);
		float2 rady = float2(0, h / g_ss.screen_dim.y);

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (-radx - rady) * In[0].ss_vert.w;
		Out.tex0.xy = float2(0, 0);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (+radx - rady) * In[0].ss_vert.w;
		Out.tex0.xy = float2(1, 0);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (-radx + rady) * In[0].ss_vert.w;
		Out.tex0.xy = float2(0, 1);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);

		Out = In[0];
		Out.ss_vert.xy = In[0].ss_vert.xy + (+radx + rady) * In[0].ss_vert.w;
		Out.tex0.xy = float2(1, 1);
		Out.ws_norm = ws_norm;
		OutStream.Append(Out);
	}
	OutStream.RestartStrip();
}

#ifdef PR_RDR_GSHADER_point_sprites
[maxvertexcount(4)]
void main(point PSIn In[1], inout TriangleStream<PSIn> OutStream)
{
	GSPointSprites(In, OutStream);
}
#endif
