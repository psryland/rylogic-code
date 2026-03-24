//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_CLOSEST_POINT_HLSLI
#define PR_HLSL_CLOSEST_POINT_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/geometry.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Notes:
//  - Convention is: ClosestPoint_XToY(...)
//  - Rays are defined as 'start' and 'direction': P(t) = s + t*d
//  - Lines are defined as 'start' and 'end': P(t) = p + t*(e - s)
//  - Returns parametric values.

// Forwards
float4 Intercept_RayVsTriangle(float4 s, float4 d, float4 a, float4 b, float4 c);

// Finds the parametric value of the closest point on a ray to 'pt'
float ClosestPoint_PointVsRay(float4 pt, float4 s, float4 d)
{
	return dot(pt - s, d) / dot(d, d);
}

// Returns the parametric values of the closest points on two rays
// Closest points are: p0 = s0 + return.x * d0, p1 = s1 + return.y * d1
float2 ClosestPoint_RayToRay(float4 s0, float4 d0, float4 s1, float4 d1)
{
	float4 r = s0 - s1;
	float a = dot(d0, d0);
	float b = dot(d0, d1);
	float e = dot(d1, d1);
	float c = dot(d0, r);
	float f = dot(d1, r);
	float denom = a * e - b * b;
	return select(
		abs(denom) > 1e-6f,
		float2((b * f - c * e) / denom, (a * f - b * c) / denom),
		float2(0, -dot(r, d0) / a) // If the lines are parallel, return the 's0' as the nearest point
	);
}

// Returns the parametric values of the closest point between a ray 's -> s+d' and a triangle 'a,b,c'
// The closest point on the triangle is at: BaryPoint(a, b, c, return.xyz)
// The closest point on the ray is at: s + return.w * d
float4 ClosestPoint_RayToTriangle(float4 s, float4 d, float4 a, float4 b, float4 c, out bool intercept)
{
	// If the ray intersects the triangle, then the intersection point is the closest point
	float4 bary = Intercept_RayVsTriangle(s, d, a, b, c);
	intercept = !AllZero(bary) && all(saturate(bary) == bary);

	if (intercept)
	{
		// Find the parameteric value on the ray
		float4 pt = BaryPoint(a, b, c, bary);
		float t = dot(pt - s, d) / dot(d, d);
		return float4(bary.xyz, t);
	}

	// Otherwise, find the closest point between the ray and the triangle edges/vertices
	else
	{
		float4 edges[3][2] = {
			{ a, b - a },
			{ b, c - b },
			{ c, a - c },
		};

		float2 best_t;
		int best_edge = -1;
		float best_dist_sq = float_max;

		// Check distance to each edge
		for (int i = 0; i != 3; ++i)
		{
			float2 t = ClosestPoint_RayToRay(s, d, edges[i][0], edges[i][1]);
			t.y = saturate(t.y); // Clamp t.edge to [0, 1] to stay on the edge segment

			float4 pt_on_ray = s + t.x * d;
			float4 pt_on_edge = edges[i][0] + t.y * edges[i][1];
			float dist_sq = length_sq(pt_on_ray - pt_on_edge);
			if (dist_sq < best_dist_sq)
			{
				best_dist_sq = dist_sq;
				best_edge = i;
				best_t = t;
			}
		}

		// Find the barycentric coords of the closest point on the triangle
		float4 pt = edges[best_edge][0] + best_t.y * edges[best_edge][1];
		bary = Barycentric(pt, a, b, c);
		return float4(bary.xyz, best_t.x);
	}
}
float4 ClosestPoint_RayToTriangle(float4 s, float4 d, float4 a, float4 b, float4 c)
{
	bool intercept;
	return ClosestPoint_RayToTriangle(s, d, a, b, c, intercept);
}

// TODO: there are closest point functions in 'collision.hlsli',
// They aren't general purpose because the require transforming in to primitive space
// Don't move them here, but they might be good starting points for more general purpose functions.

#ifdef __cplusplus
}
#endif
#endif



#if 0 // World space functions
// Finds the closest point on a plane to 'pos'. Returns the normal at the closest point
inline float4 ClosestPoint_PointToPlane(float4 pos, float4 plane)
{
	float dist = dot(pos, plane);
	return pos - dist * plane;
}
inline float4 ClosestPoint_PointToPlane(float4 pos, float4 plane, out float4 normal)
{
	normal = float4(plane.xyz, 0);
	return ClosestPoint_PointToPlane(pos, plane);
}

// Finds the closest point on a sphere to 'pos'
inline float4 ClosestPoint_PointToSphere(float4 pos, float4 sphere)
{
	float4 normal;
	return ClosestPoint_PointToSphere(pos, sphere, normal);
}
inline float4 ClosestPoint_PointToSphere(float4 pos, float4 sphere, out float4 normal)
{
	float4 ray = pos - float4(sphere.xyz, 1);
	float dist_sq = sqr(ray);
	
	normal = select(dist_sq != 0, ray / sqrt(dist_sq), float4(1, 0, 0, 0));
	return float4(sphere.xyz + ray.xyz * sphere.w, 1);
}

// Finds the closest point on a triangle to 'pos'
inline float4 ClosestPoint_PointToTriangle(float4 pos, float4 tri[3])
{
	float4 bary;
	return ClosestPoint_PointToTriangle(pos, tri, bary);
}
inline float4 ClosestPoint_PointToTriangle(float4 pos, float4 tri[3], out float4 bary)
{
	float4 ab = tri[1] - tri[0];
	float4 ac = tri[2] - tri[0];
	float4 ap = pos - tri[0];
	float4 bp = pos - tri[1];
	float4 cp = pos - tri[2];
	float d1 = dot(ab, ap);
	float d2 = dot(ac, ap);
	float d3 = dot(ab, bp);
	float d4 = dot(ac, bp);
	float d5 = dot(ab, cp);
	float d6 = dot(ac, cp);
	float vc = d1*d4 - d3*d2;
	float vb = d5*d2 - d1*d6;
	float va = d3*d6 - d5*d4;

	// Check if P in vertex region outside A
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		bary = float4(1.0f, 0.0f, 0.0f, 0.0f);
		return tri[0];
	}

	// Check if P in vertex region outside B
	if (d3 >= 0.0f && d4 <= d3)
	{
		bary = float4(0.0f, 1.0f, 0.0f, 0.0f);
		return tri[1];
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// Barycentric coordinates (1-v, v, 0)
		float v = d1 / (d1 - d3);
		bary = float4(1.0f - v, v, 0.0f, 0.0f);
		return tri[0] + v * ab;
	}

	// Check if P in vertex region outside C
	if (d6 >= 0.0f && d5 <= d6)
	{
		bary = float4(0.0f, 0.0f, 1.0f, 0.0f);
		return tri[2];
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// Barycentric coordinates (1-w, 0, w)
		float w = d2 / (d2 - d6);
		bary = float4(1.0f - w, 0.0f, w, 0.0f);
		return tri[0] + w * ac;
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
	{
		// Barycentric coordinates (0, 1-w, w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		bary = float4(0.0f, 1.0f - w, w, 0.0f);
		return tri[1] + w * (tri[2] - tri[1]);
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	//'  = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	bary = float4(1.0f - v - w, v, w, 0.0f);
	return tri[0] + ab * v + ac * w;
}
inline float4 ClosestPoint_PointToTriangle(float4 pos, float4 tri[3], out float4 bary, out float4 normal)
{
	float4 pt = ClosestPoint_PointToTriangle(pos, tri, bary);
	float4 ray = pos - pt;
	float dist = length(ray);
	normal = dist != 0 ? ray / dist : float4(normalize(cross((tri[1] - tri[0]).xyz, (tri[2] - tri[0]).xyz)), 0);
	return pt;
}
#endif

