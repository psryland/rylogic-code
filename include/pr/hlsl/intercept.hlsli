//*********************************************
// Intercept
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_INTERCEPT_HLSLI
#define PR_HLSL_INTERCEPT_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Notes:
//  - Convention is: float Intercept_XVsY(...), i.e. Intercept finds the parametric values of the intercept
//  - Favour 'Intercept' over 'Intersect' because the boolean tests don't provide as much information.
//    The optimiser will remove any redundant calculations if only simple boolean tests are needed.
//  - Rays are defined as 'start' and 'direction': P(t) = s + t*d
//  - Lines are defined as 'start' and 'end': P(t) = p + t*(e - s)
//  - Returns parametric values

// Forwards
float4 Intercept_RayVsTriangle(float4 s, float4 d, float4 a, float4 b, float4 c);
float4 Intercept_RayVsTriangle(float4 s, float4 d, float4 a, float4 b, float4 c, out float f2b);

// Calculate the barycentric coordinates for the intersection of a line passing through 's -> s+d' and a triangle 'a-b-c'.
// Returns xyz = bary coords of intercept on the triangle, w = unused.
// Returns (0,0,0,0) if the ray is in the plane of 'a-b-c'.
// The ray intersects the triangle if: !AllZero(para) && all(saturate(para) == para);
// The intersection is at: BaryPoint(a, b, c, return.xyz), return.w is unused.
float4 Intercept_RayVsTriangle(float4 s, float4 d, float4 a, float4 b, float4 c)
{
	float f2b;
	return Intercept_RayVsTriangle(s, d, a, b, c, f2b);
}
float4 Intercept_RayVsTriangle(float4 s, float4 d, float4 a, float4 b, float4 c, out float f2b)
{
	float4 sa = a - s;
	float4 sb = b - s;
	float4 sc = c - s;

	// Test if 'd' is on or inside the edges ab, bc, and ca.
	// Done by testing that the signed tetrahedral volumes are all positive.
	float3 bary = float3(
		Triple(d, sc, sb),
		Triple(d, sa, sc),
		Triple(d, sb, sa)
	);

	// Compute the barycentric coordinates (u, v, w) determining the
	// intersection point u*a + v*b + w*c. Note: If the line lies
	// in the plane of the triangle then 'sum' will be zero
	float sum = SumComponents(bary);
	float denom = (abs(sum) > 0.0001) ? (1 / sum) : 0;
	f2b = (denom > 0.0f) * 2.0f - 1.0f;
	return float4(bary * denom, 0.0f);
}

// Returns the parametric value of the intersection between the line between 's' and 'e', with 'frust'.
// Assumes 's' is within the frustum to start with.
float Intercept_LineVsFrustum(float4 s, float4 e, float4x4 frust)
{
	const float4 T = 1e10f;
	
	// Find the distance from each frustum face for 's' and 'e'
	float4 d0 = mul(s, frust);
	float4 d1 = mul(e, frust);

	// Clip the edge 's-e' to each of the frustum sides (Actually, find the parametric
	// value of the intercept)(min(T,..) protects against divide by zero)
	float4 t0 = step(d1, d0) * min(T, -d0 / (d1 - d0)); // Clip to the frustum sides
	float t1 = step(e.z, s.z) * min(T.x, -s.z / (e.z - s.z)); // Clip to the far plane

	// Set all components that are <= 0.0 to BIG
	t0 += step(t0, float4(0, 0, 0, 0)) * T;
	t1 += step(t1, 0.0f) * T.x;

	// Find the smallest positive parametric value
	// => the closest intercept with a frustum plane
	float t = T.x;
	t = min(t, t0.x);
	t = min(t, t0.y);
	t = min(t, t0.z);
	t = min(t, t0.w);
	t = min(t, t1);
	return t;
}

#ifdef __cplusplus
}
#endif
#endif



#if 0 // World space functions
// Intersects a ray with a plane, returning true if there is an intercept.
// 'plane.xyz' is the (normalised) plane normal, 'plane.w' is the signed distance to the origin from the plane
inline bool Intercept_RayVsPlane(float4 pos, float4 ray, float4 plane, inout float t1, inout float4 normal)
{
	// 'step' is the length of the projection of 'ray' on the normal
	// Is the ray moving away from the triangle?
	float step = dot(ray, plane);
	if (step >= 0)
		return false;
	
	// 'dist' is the distance to the plane (scaled by |n|)
	// Unlike triangles, particles collide from any depth in the plane
	// Is the start point too far away to reach the triangle?
	float dist = dot(pos, plane);
	if (dist >= -step)
		return false;

	// 't' is the parametric value of the intercept point.
	// The condition above guarantees t on the interval [-inf,1)
	float t = dist > 0 ? -dist / step : 0;
	if (t >= t1)
		return false;

	// Return the intercept
	t1 = t;
	normal = float4(plane.xyz, 0);
	return true;
}

// Intersects a ray with a sphere, returning true if there is an intercept
// 'sphere.xyz' is the center of the sphere, 'sphere.w' is the radius
bool Intercept_RayVsSphere(float4 pos, float4 ray, float4 sphere, inout float t1, inout float4 normal)
{
	float4 p = pos - float4(sphere.xyz, 1);
	float4 q = ray;

	// The length of 'q' squared
	float q_sq = sqr(q);
	if (q_sq == 0)
		return false;
	
	// Find the closest point on the infinite line through 'ray'
	float4 c = pos - q * dot(q, pos) / q_sq;
	float c_sq = sqr(c);

	// If the closest point is not within the sphere then there is no intersection
	float radius_sq = sqr(sphere.w);
	if (radius_sq < c_sq)
		return false;

	// Get the distance from the closest point to the intersection with the boundary of the sphere
	float x = sqrt((radius_sq - c_sq) / q_sq); // includes the normalising 1/|q| in x

	// Get the parametric values of the intersection
	float4 half_chord = q * x;
	float4 hit_min = c - half_chord;
	float4 hit_max = c + half_chord;
	float tmin = dot(q, hit_min - pos) / q_sq;
	float tmax = dot(q, hit_max - pos) / q_sq;

	// Does the ray start within the sphere?
	// Is the intercept further than 't1'.
	if (tmin < 0 || tmin >= t1)
		return false;
	
	t1 = tmin;
	normal = normalize(hit_min - float4(sphere.xyz, 1));
	return true;
}

// Intersects a ray with a triangle, returning true if there is an intercept
bool Intercept_RayVsTriangle(float4 pos, float4 ray, float4 tri[3], inout float t1, inout float4 normal)
{
	float4 e0 = tri[1] - tri[0];
	float4 e1 = tri[2] - tri[0];
	float4 p = pos - tri[0];
	float4 q = ray;
	
	// Triangle normal (not yet normalised)
	float4 n = float4(cross(e0.xyz, e1.xyz), 0);
	
	// 'step' is the length of the projection of 'ray' on the normal (scaled by |n|)
	// Is the ray moving away from the triangle?
	float step = dot(q, n);
	if (step >= 0)
		return false;
	
	// 'dist' is the distance to the plane of the triangle (scaled by |n|)
	// Does the ray start behind the triangle?
	// Is the start point too far away to reach the triangle?
	float dist = dot(p, n);
	if (dist < 0 || dist >= -step)
		return false;

	// 't' is the parametric value of the intercept point.
	// The condition above guarantees t on the interval [0,1)
	float t = -dist / step;
	if (t >= t1)
		return false;
	
	// Find the intercept point
	float4 h = pos + t * ray;

	// Compute barycentric coordinate components and test if within bounds.
	// If any coordinate is outside the interval [0,1], the intercept is not within the triangle.
	float4 bary = Barycentric(h, tri);
	if (any(bary) < 0 || any(bary) >= 1 || all(bary) == 0)
		return false;
	
	// Return the intercept
	t1 = t;
	normal = n / length(n);
	return true;
}
#endif
