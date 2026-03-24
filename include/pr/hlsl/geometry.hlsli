//***********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_HLSL_GEOMETRY_HLSLI
#define PR_HLSL_GEOMETRY_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Project 'pt' onto the plane with normal 'direction'
inline float3 Project(float3 pt, float3 direction)
{
	return pt - dot(pt, direction) * direction;
}
inline float4 Project(float4 pt, float4 direction)
{
	return pt - dot(pt, direction) * direction;
}

// Return the normal for the triangle (a,b,c)
// Returns 'def' if the triangle is degenerate
inline float4 FaceNormal(float4 a, float4 b, float4 c, float4 def = float4(0,0,0,0))
{
	float3 n = cross((b - a).xyz, (c - b).xyz);
	float len = length(n);
	return len > 0 ? float4(n / len, 0) : def;
}

// Measure the coplanarity of the two triangles (a,b,c) and (a,c,d)
// Returns 1 for parallel normals, -1 for opposing normals.
// Returns 0 if either of the triangles are degenerate.
inline float Coplanarity(float4 a, float4 b, float4 c, float4 d)
{
	float4 n0 = FaceNormal(a, b, c);
	float4 n1 = FaceNormal(a, c, d);
	return dot(n0,n1);
}

// Return a point that is the weighted result of verts 'a','b','c' and 'bary'
inline float2 BaryPoint(float2 a, float2 b, float2 c, float3 bary)
{
	float4 pt = bary.x * float4(a, 0, 1) + bary.y * float4(b, 0, 1) + bary.z * float4(c, 0, 1);
	return pt.xy / pt.w;
}
inline float2 BaryPoint(float2 a, float2 b, float2 c, float4 bary)
{
	return BaryPoint(a, b, c, bary.xyz);
}
inline float4 BaryPoint(float4 a, float4 b, float4 c, float3 bary)
{
	float4 pt = bary.x * a + bary.y * b + bary.z * c;
	return pt / pt.w;
}
inline float4 BaryPoint(float4 a, float4 b, float4 c, float4 bary)
{
	return BaryPoint(a, b, c, bary.xyz);
}

// Return the barycentric coordinates for 'point' with respect to triangle a,b,c
inline float4 Barycentric(float2 pos, float2 a, float2 b, float2 c)
{
	float2 ab = b - a;
	float2 ac = c - a;
	float2 ap = pos - a;

	float d00 = dot(ab, ab);
	float d01 = dot(ab, ac);
	float d11 = dot(ac, ac);
	float d20 = dot(ap, ab);
	float d21 = dot(ap, ac);

	// If 'denom' == 0, the triangle has no area
	// Return an invalid coordinate to signal this.
	float denom = d00 * d11 - d01 * d01;
	if (denom == 0)
		return float4(0, 0, 0, 0);
	
	float4 bary;
	bary.y = (d11 * d20 - d01 * d21) / denom;
	bary.z = (d00 * d21 - d01 * d20) / denom;
	bary.x = 1.0f - bary.y - bary.z;
	bary.w = 0.0f;
	return bary;
}
inline float4 Barycentric(float4 pos, float4 a, float4 b, float4 c)
{
	float4 ab = b - a;
	float4 ac = c - a;
	float4 ap = pos - a;

	float d00 = dot(ab, ab);
	float d01 = dot(ab, ac);
	float d11 = dot(ac, ac);
	float d20 = dot(ap, ab);
	float d21 = dot(ap, ac);

	// If 'denom' == 0, the triangle has no area
	// Return an invalid coordinate to signal this.
	float denom = d00 * d11 - d01 * d01;
	if (denom == 0)
		return float4(0, 0, 0, 0);
	
	float4 bary;
	bary.y = (d11 * d20 - d01 * d21) / denom;
	bary.z = (d00 * d21 - d01 * d20) / denom;
	bary.x = 1.0f - bary.y - bary.z;
	bary.w = 0.0f;
	return bary;
}

// Returns 1.0 if point 'p' lies inside triangle (a,b,c), otherwise 0.0.
// 'p' - Point to test (does NOT need to lie in triangle plane)
// a,b,c - Triangle vertices
inline float PointInTriangle(float3 p, float3 a, float3 b, float3 c)
{
	// Notes:
	//   - Internally computes the triangle normal
	//   - Projects 'p' onto the triangle plane implicitly via orientation tests
	//   - Returns 0.0 for degenerate triangles
	float3 ab = b - a;
	float3 ac = c - a;

	float3 n = cross(ab, ac); // triangle normal

	// Degeneracy check
	float area2 = dot(n, n);
	float valid = step(1e-12f, area2);

	float3 bc = c - b;
	float3 ca = a - c;

	float3 ap = p - a;
	float3 bp = p - b;
	float3 cp = p - c;

	float s0 = dot(cross(ab, ap), n);
	float s1 = dot(cross(bc, bp), n);
	float s2 = dot(cross(ca, cp), n);

	float min_s = min(s0, min(s1, s2));
	float max_s = max(s0, max(s1, s2));

	// 's0..s2' scale with the squared triangle area, so use a relative tolerance
	float eps = max(1e-12f, 1e-5f * area2);

	float all_pos = step(-eps, min_s);
	float all_neg = step(max_s, eps);

	return max(all_pos, all_neg) * valid;
}

// Returns 1.0 if 'p' is inside the quad '(a,b,c,d)', otherwise 0.0.
inline float PointInQuad(float3 p, float3 a, float3 b, float3 c, float3 d)
{
	float t0 = PointInTriangle(p, a, b, c);
	float t1 = PointInTriangle(p, a, b, d);
	float t2 = PointInTriangle(p, a, c, d);
	float t3 = PointInTriangle(p, b, c, d);
	return max(max(t0, t1), max(t2, t3));
}

// Returns a spherical direction vector corresponding to the i'th point of a Fibonacci sphere
inline float4 FibonacciSpiral(int i, int N)
{
	// Z goes from -1 to +1
	// Using a half step bias so that there is no point at the poles.
	// This prevents degenerates during 'unmapping' and also results in more evenly
	// spaced points. See "Fibonacci grids: A novel approach to global modelling".
	float z = -1.0f + (2.0f * i + 1.0f) / N;

	// Radius at z
	float r = sqrt(1.0f - z * z);

	// Golden angle increment
	static const float GoldenAngle = 2.39996322972865332223f;
	float theta = i * GoldenAngle;
	
	float x = cos(theta) * r;
	float y = sin(theta) * r;
	return float4(x, y, z, 0);
}

#ifdef __cplusplus
}
#endif
#endif



#if 0

// This needs moving to where 'm_proj_tex_count' is defined
inline float4 ProjTex(float4 ws_pos, float4 in_diff)
{
	float4 out_diff = in_diff;
	for (int i = 0; i < m_proj_tex_count.x; i += 1.0)
	{
		// Project the world space position into projected texture coords
		float2 pt_pos = mul(ws_pos, m_proj_tex[i]).xy;
		float2 pt_pos_sat = saturate(pt_pos);
		float4 diff = m_proj_texture[i].Sample(m_proj_sampler[i], pt_pos_sat);
		if (pt_pos_sat.x == pt_pos.x && pt_pos_sat.y == pt_pos.y)
			out_diff = diff;
	}
	return out_diff;
}
#endif
