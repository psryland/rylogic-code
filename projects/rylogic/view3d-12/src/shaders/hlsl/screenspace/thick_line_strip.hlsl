//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#include "pr/hlsl/vector.hlsli"
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// Constant buffers
ConstantBuffer<CBufScreenSpace> g_ss : register(b3);

// Converts line geometry into tristrip
void GSThickLineStrip(lineadj PSIn In[4], inout TriangleStream<PSIn> OutStream)
{
	// Notes:
	//  - To use the thick linestrip shader, you need to add an extra vert/index to the start
	//    and end of the strip. The first line segment drawn is from In[1] -> In[2], then from In[2] -> In[3], etc.
	//  - There is no decent way to handle lines that fold back on themselves, so if two adjacent line segments
	//    are degenerate or fold back to accutely, they're treated as start/end sections with end caps.

	PSIn Out;

	float4 p0 = In[0].ss_vert;
	float4 p1 = In[1].ss_vert;
	float4 p2 = In[2].ss_vert;
	float4 p3 = In[3].ss_vert;
	bool line_beg = !any(p0 - p1);
	bool line_end = !any(p2 - p3);

	// Clip the line against the near plane
	if (p1.z < 0.0f)
	{
		float t = -p1.z / (p2.z - p1.z);
		In[1].ss_vert = p1 = lerp(p1, p2, t);
	}
	if (p2.z < 0.0f)
	{
		float t = -p2.z / (p1.z - p2.z);
		In[2].ss_vert = p2 = lerp(p2, p1, t);
	}

	// Normalise the screen space points
	p0 /= p0.w;
	p1 /= p1.w;
	p2 /= p2.w;
	p3 /= p3.w;

	// Line direction and perpendicular in screen space
	float2 lin0 = (p1 - p0).xy;
	float2 lin1 = (p2 - p1).xy;
	float2 lin2 = (p3 - p2).xy;
	float2 dir0 = normalize(lin0 * g_ss.screen_dim.xy);
	float2 dir1 = normalize(lin1 * g_ss.screen_dim.xy);
	float2 dir2 = normalize(lin2 * g_ss.screen_dim.xy);
	float2 perp0 = RotateCCW(dir0);
	float2 perp1 = RotateCCW(dir1);
	float2 perp2 = RotateCCW(dir2);
	float2 bisector1 = perp0 + perp1;
	float2 bisector2 = perp1 + perp2;
	float bi_lensq1 = dot(bisector1, bisector1);
	float bi_lensq2 = dot(bisector2, bisector2);
	float bi_scale1 = dot(bisector1, perp1);
	float bi_scale2 = dot(bisector2, perp1);

	float2 width = max(1.0f, g_ss.size.x * 0.5f) / g_ss.screen_dim.xy;

	// Use end caps if 'lin0' or 'lin1' is too short, or degenerate
	line_beg = line_beg || bi_lensq1 < 0.05f;
	line_end = line_end || bi_lensq2 < 0.05f;

	// {0, cos(tau/16), cos(tau*2/16), cos(tau*3/16)};
	const float X[5] = { 1, 0.92387953251, 0.70710678118, 0.38268343236, 0 };
	const float Y[5] = { 0, 0.38268343236, 0.70710678118, 0.92387953251, 1 };

	// The start of the line
	Out = In[1];
	if (line_beg)
	{
		// The rounded start of the line
		Out.ss_vert.xy = In[1].ss_vert.xy + (-X[0] * dir1 + Y[0] * perp1) * width * In[1].ss_vert.w;
		OutStream.Append(Out);
		for (int i = 1; i < 5; i += 1)
		{
			Out.ss_vert.xy = In[1].ss_vert.xy + (-X[i] * dir1 - Y[i] * perp1) * width * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + (-X[i] * dir1 + Y[i] * perp1) * width * In[1].ss_vert.w;
			OutStream.Append(Out);
		}
	}
	else
	{
		float2 a = bisector1 / bi_scale1; // The concave vertex
		float2 b = perp1; // The line end vertex
		float2 c = bisector1 / sqrt(bi_lensq1); // The convex vectex
		float2 d = normalize(b + c);

		if (dot(dir0, dir1) > 0.9f) // straight
		{
			Out.ss_vert.xy = In[1].ss_vert.xy - width * c * In[1].ss_vert.w;
			OutStream.Append(Out);
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + width * a * In[1].ss_vert.w;
			OutStream.Append(Out);
		}
		else if (dot(dir0, perp1) < 0) // turning left
		{
			Out.ss_vert.xy = In[1].ss_vert.xy - width * c * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy - width * d * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + width * a * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy - width * b * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + width * a * In[1].ss_vert.w;
			OutStream.Append(Out);
		}
		else // turning right
		{
			Out.ss_vert.xy = In[1].ss_vert.xy + width * c * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy - width * a * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + width * d * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy - width * a * In[1].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[1].ss_vert.xy + width * b * In[1].ss_vert.w;
			OutStream.Append(Out);
		}
	}

	// The end of the line
	Out = In[2];
	if (line_end)
	{
		// The rounded end of the line
		for (int i = 4; i > 0; i -= 1)
		{
			Out.ss_vert.xy = In[2].ss_vert.xy + (X[i] * dir1 - Y[i] * perp1) * width * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + (X[i] * dir1 + Y[i] * perp1) * width * In[2].ss_vert.w;
			OutStream.Append(Out);
		}
		Out.ss_vert.xy = In[2].ss_vert.xy + (X[0] * dir1 + Y[0] * perp1) * width * In[2].ss_vert.w;
		OutStream.Append(Out);
	}
	else
	{
		float2 a = bisector2 / bi_scale2; // The concave vertex
		float2 b = perp1; // The line end vertex
		float2 c = bisector2 / sqrt(bi_lensq2); // The convex vectex
		float2 d = normalize(b + c);

		if (dot(dir1, dir2) > 0.9f) // straight
		{
			Out.ss_vert.xy = In[2].ss_vert.xy - width * c * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * a * In[2].ss_vert.w;
			OutStream.Append(Out);
		}
		else if (dot(dir2, perp1) > 0) // turning left
		{
			Out.ss_vert.xy = In[2].ss_vert.xy - width * b * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * a * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy - width * d * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * a * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy - width * c * In[2].ss_vert.w;
			OutStream.Append(Out);
		}
		else // turning right
		{
			Out.ss_vert.xy = In[2].ss_vert.xy - width * a * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * b * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy - width * a * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * d * In[2].ss_vert.w;
			OutStream.Append(Out);
			Out.ss_vert.xy = In[2].ss_vert.xy + width * c * In[2].ss_vert.w;
			OutStream.Append(Out);
		}
	}

	OutStream.RestartStrip();
}

#ifdef PR_RDR_GSHADER_thick_line_strip
[maxvertexcount(16)]
void main(lineadj PSIn In[4], inout TriangleStream<PSIn> OutStream)
{
	GSThickLineStrip(In, OutStream);
}
#endif