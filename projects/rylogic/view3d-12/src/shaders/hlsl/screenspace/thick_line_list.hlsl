//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************

#include "pr/hlsl/vector.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// ConstantBuffers
ConstantBuffer<CBufScreenSpace> g_ss : register(b3);

// Converts line segments into tristrip
[maxvertexcount(18)]
void GSThickLineList(line PSIn In[2], inout TriangleStream<PSIn> OutStream)
{
	PSIn Out;
	
	// Clip the line against the near plane
	float4 p0 = In[0].ss_vert;
	float4 p1 = In[1].ss_vert;
	if (p0.z < 0.0f)
	{
		float t = -p0.z / (p1.z - p0.z);
		In[0].ss_vert = p0 = lerp(p0, p1, t);
	}
	if (p1.z < 0.0f)
	{
		float t = -p1.z / (p0.z - p1.z);
		In[1].ss_vert = p1 = lerp(p1, p0, t);
	}

	// Normalise the screen space points
	p0 /= p0.w;
	p1 /= p1.w;

	// Line direction and perpendicular in screen space
	float width = max(1.0f, g_ss.size.x * 0.5f);
	float2 lin = (p1 - p0).xy;
	float2 dir = normalize(lin * g_ss.screen_dim.xy);
	float2 tang = dir / g_ss.screen_dim.xy;
	float2 perp = float2(-dir.y, dir.x) / g_ss.screen_dim.xy;

	// {0, cos(tau/16), cos(tau*2/16), cos(tau*3/16)};
	const float X[5] = { 1, 0.92387953251, 0.70710678118, 0.38268343236, 0 };
	const float Y[5] = { 0, 0.38268343236, 0.70710678118, 0.92387953251, 1 };
	
	// The rounded start of the line
	Out = In[0];
	Out.ss_vert.xy = In[0].ss_vert.xy + (-X[0] * tang + Y[0] * perp) * width * In[0].ss_vert.w;
	OutStream.Append(Out);
	for (int i = 0; ++i != 5;)
	{
		Out.ss_vert.xy = In[0].ss_vert.xy + (-X[i] * tang - Y[i] * perp) * width * In[0].ss_vert.w;
		OutStream.Append(Out);
		Out.ss_vert.xy = In[0].ss_vert.xy + (-X[i] * tang + Y[i] * perp) * width * In[0].ss_vert.w;
		OutStream.Append(Out);
	}

	// The rounded end of the line
	Out = In[1];
	for (int j = 5; j-- != 1;)
	{
		Out.ss_vert.xy = In[1].ss_vert.xy + (X[j] * tang - Y[j] * perp) * width * In[1].ss_vert.w;
		OutStream.Append(Out);
		Out.ss_vert.xy = In[1].ss_vert.xy + (X[j] * tang + Y[j] * perp) * width * In[1].ss_vert.w;
		OutStream.Append(Out);
	}
	Out.ss_vert.xy = In[1].ss_vert.xy + (X[0] * tang + Y[0] * perp) * width * In[1].ss_vert.w;
	OutStream.Append(Out);
	OutStream.RestartStrip();
}
