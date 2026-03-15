//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_COMPUTE_COLLISION_HLSLI
#define PR_COMPUTE_COLLISION_HLSLI

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/geometry.hlsli"
#include "pr/hlsl/intercept.hlsli"
#include "pr/hlsl/closest_point.hlsli"

// Notes:
//  - All calculations are all performed in primitive space because it makes
//    the code much simpler. 'pos' and 'ray' must be transformed into primitive space.
//  - Closest point calculations return a point that is on the surface, even if 'pos'
//    is inside the primitive.
//  - 2D primitives are double-sided.
//  - Intercept calculations return the parametric value of the intercept point and
//    the surface normal at that point.

struct Prim
{
	// Notes:
	//  - Primitives should be positioned using rotations and translations only (no scaling).
	//  - Alignment matters. These are provided in an array so need to be a multiple of 16 bytes.
	
	// The object to world space transform for the primitive.
	row_major float4x4 o2w;
	
	// Primitive data:
	//    Plane: no data needed, plane is XY, normal is Z
	//     Quad: [0].xy = width/height, plane is XY, normal is Z
	// Triangle: [0].xy = a, [0].zw = b, [1].xy = c, plane is XY, normal is Z
	//  Ellipse: [0].xy = radii, plane is XY, normal is Z
	//      Box: [0].xyz = radii, centre is origin
	//   Sphere: [0].xyz = radii (actually an ellipsoid), centre is origin
	// Cylinder: [0].xy = radii, [0].z = half-length (actually an elliptical cylinder), centre is origin, main axis is Z
	float4 data[2];

	// flag.x = primitive type
	// flag.y = unused
	// flag.z = unused
	// flag.w = unused
	uint4 flags;
};

static const uint Prim_Plane = 0;
static const uint Prim_Quad = 1;
static const uint Prim_Triangle = 2;
static const uint Prim_Ellipse = 3;
static const uint Prim_Box = 4;
static const uint Prim_Sphere = 5;
static const uint Prim_Cylinder = 6;

static const float TINY = 0.0001f;
static const float FLT_MAX = 3.402823e38f;

// Forwards
float4 ClosestPoint_PointToInfiniteLine(float4 pos, float4 a, float4 b);
float4 ClosestPoint_PointToInfiniteLine(float4 pos, float4 a, float4 b, out float t);
float4 ClosestPoint_PointToPlane(float4 pos);
float4 ClosestPoint_PointToPlane(float4 pos, out float4 normal);
float4 ClosestPoint_PointToQuad(float4 pos, float2 radii);
float4 ClosestPoint_PointToQuad(float4 pos, float2 radii, out float4 normal);
float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c);
float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c, out float4 bary);
float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c, out float4 bary, out float4 normal);
float4 ClosestPoint_PointToEllipse(float4 pos, float2 radii);
float4 ClosestPoint_PointToEllipse(float4 pos, float2 radii, out float4 normal);
float4 ClosestPoint_PointToBox(float4 pos, float4 radii);
float4 ClosestPoint_PointToBox(float4 pos, float4 radii, out float4 normal);
float4 ClosestPoint_PointToSphere(float4 pos, float4 radii);
float4 ClosestPoint_PointToSphere(float4 pos, float4 radii, out float4 normal);
float4 ClosestPoint_PointToCylinder(float4 pos, float4 radii);
float4 ClosestPoint_PointToCylinder(float4 pos, float4 radii, out float4 normal);

bool Intercept_RayVsPlane(float4 pos, float4 ray, inout float4 normal, inout float t1);
bool Intercept_RayVsQuad(float4 pos, float4 ray, float2 a, float2 b, float2 c, float2 d, inout float4 normal, inout float t1);
bool Intercept_RayVsTriangle(float4 pos, float4 ray, float2 a, float2 b, float2 c, inout float4 normal, inout float t1);
bool Intercept_RayVsEllipse(float4 pos, float4 ray, float2 radii, inout float4 normal, inout float t1);
bool Intercept_RayVsBox(float4 pos, float4 ray, float4 radii, inout float4 normal, inout float t1);
bool Intercept_RayVsSphere(float4 pos, float4 ray, float radius, inout float4 normal, inout float t1);
bool Intercept_RayVsCylinder(float4 pos, float4 ray, float4 radii, inout float4 normal, inout float t1);

// Closest point on an infinite line to 'pos'
inline float4 ClosestPoint_PointToInfiniteLine(float4 pos, float4 a, float4 b)
{
	float t;
	return ClosestPoint_PointToInfiniteLine(pos, a, b, t);
}
inline float4 ClosestPoint_PointToInfiniteLine(float4 pos, float4 a, float4 b, out float t)
{
	float4 ab = b - a;
	t = dot(pos - a, ab) / dot(ab,ab);
	return a + t * ab;
}

// Closest point on a plane to 'pos'
// The plane is the XY plane with normal (0,0,1) and distance 0.
inline float4 ClosestPoint_PointToPlane(float4 pos)
{
	return float4(pos.xy, 0, 1);
}
inline float4 ClosestPoint_PointToPlane(float4 pos, out float4 normal)
{
	normal = float4(0, 0, 1, 0);
	return ClosestPoint_PointToPlane(pos);
}

// Closest point on a quad to 'pos'
// The quad lies in the XY plane with normal pointing down the z-axis
inline float4 ClosestPoint_PointToQuad(float4 pos, float2 radii)
{
	return float4(clamp(pos.x, -radii.x, +radii.x), clamp(pos.y, -radii.y, +radii.y), 0, 1);
}
inline float4 ClosestPoint_PointToQuad(float4 pos, float2 radii, out float4 normal)
{
	float4 pt = ClosestPoint_PointToQuad(pos, radii);
	normal = select(any(pos.xy - pt.xy), normalize(pos - pt), float4(0, 0, sign_nz(pos.z), 0));
	return pt;
}

// Closest point on triangle to 'pos'
// The triangle lies in the XY plane with normal pointing down the z-axis.
inline float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c)
{
	float4 bary;
	return ClosestPoint_PointToTriangle(pos, a, b, c, bary);
}
inline float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c, out float4 bary)
{
	float2 ab = b - a;
	float2 ac = c - a;
	float2 ap = pos.xy - a;
	float2 bp = pos.xy - b;
	float2 cp = pos.xy - c;
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
		return float4(a, 0, 1);
	}

	// Check if P in vertex region outside B
	if (d3 >= 0.0f && d4 <= d3)
	{
		bary = float4(0.0f, 1.0f, 0.0f, 0.0f);
		return float4(b, 0, 1);
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// Barycentric coordinates (1-v, v, 0)
		float v = d1 / (d1 - d3);
		bary = float4(1.0f - v, v, 0.0f, 0.0f);
		return float4(a + v * ab, 0, 1);
	}

	// Check if P in vertex region outside C
	if (d6 >= 0.0f && d5 <= d6)
	{
		bary = float4(0.0f, 0.0f, 1.0f, 0.0f);
		return float4(c, 0, 1);
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// Barycentric coordinates (1-w, 0, w)
		float w = d2 / (d2 - d6);
		bary = float4(1.0f - w, 0.0f, w, 0.0f);
		return float4(a + w * ac, 0, 1);
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
	{
		// Barycentric coordinates (0, 1-w, w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		bary = float4(0.0f, 1.0f - w, w, 0.0f);
		return float4(b + w * (c - b), 0, 1);
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	//'  = u*a + v*b + w*c, u = va * denom = 1.0f - v - w
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	bary = float4(1.0f - v - w, v, w, 0.0f);
	return float4(a + ab * v + ac * w, 0, 1);
}
inline float4 ClosestPoint_PointToTriangle(float4 pos, float2 a, float2 b, float2 c, out float4 bary, out float4 normal)
{
	float4 pt = ClosestPoint_PointToTriangle(pos, a, b, c, bary);
	normal = select(any(pos.xy - pt.xy), normalize(pos - pt), float4(0, 0, sign_nz(pos.z), 0));
	return pt;
}

// Closest point on an ellipse to 'pos'
// The ellipse lies in the XY plane with normal pointing down z-axis and with radii 'radii.xy'.
inline float4 ClosestPoint_PointToEllipse(float4 pos, float2 radii)
{
	// Normalise to the unit circle
	float2 npos = pos.xy / radii;
	
	// If the point is inside the ellipse, then it is the closest point
	if (dot(npos, npos) <= 1)
		return float4(pos.xy, 0, 1);
	
	// If not, then project onto the unit circle, and rescale back to an ellipse
	return float4(normalize(npos) * radii, 0, 1);
}
inline float4 ClosestPoint_PointToEllipse(float4 pos, float2 radii, out float4 normal)
{
	float4 pt = ClosestPoint_PointToEllipse(pos, radii);
	normal = select(any(pos.xy - pt.xy), normalize(pos - pt), float4(0, 0, sign_nz(pos.z), 0));
	return pt;
}

// Closest point on a box to 'pos'
// The box is at the origin with half extents 'radii'.
inline float4 ClosestPoint_PointToBox(float4 pos, float4 radii)
{
	float4 normal;
	return ClosestPoint_PointToBox(pos, radii, normal);
}
inline float4 ClosestPoint_PointToBox(float4 pos, float4 radii, out float4 normal)
{
	normal = float4(0,0,0,0);
	
	// Guarantee that 'radii' is not degenerate
	radii.xyz += TINY * (radii.xyz == 0);

	// If 'pos' is inside or on the surface of the box, push the shortest distance component to the surface
	float3 pt = pos.xyz;
	float3 dist = abs(pt) - radii.xyz;
	if (all(dist <= 0))
	{
		// If 'abs(pt) == radii' => 'dist == 0', but 'pos[i]' is not zero because 'radii' is not zero
		// If 'pt == 0' => 'abs(dist) == radii'. 'i' will be the smallest axis, but 'pos[i]' could be zero.
		int i = min_component_index(abs(dist));
		pt[i] = sign_nz(pos[i]) * radii[i];
		normal[i] = sign_nz(pos[i]);
	}
	else
	{
		pt = clamp(pos.xyz, -radii.xyz, +radii.xyz);
		normal.xyz = normalize(pos.xyz - pt);
	}
	return float4(pt, 1);
}

// Closest point on a sphere (ellipsoid) to 'pos'
// The sphere (ellipsoid) is at the origin, with radii 'radii'.
inline float4 ClosestPoint_PointToSphere(float4 pos, float4 radii)
{
	// If right at the centre, use the smallest axis
	if (all(pos == 0))
	{
		float4 pt = float4(0,0,0,0);
		int i = min_component_index(radii.xyz);
		pt[i] = radii[i];
		return pt;
	}
	
	// Otherwise, project onto the unit sphere and scale back to the ellipsoid
	return float4(normalize(pos.xyz / radii.xyz) * radii.xyz, 1);
}
inline float4 ClosestPoint_PointToSphere(float4 pos, float4 radii, out float4 normal)
{
	float4 pt = ClosestPoint_PointToSphere(pos, radii);

	// Tangents scale correctly between sphere <=> ellipsoid so find orthogonal
	// tangents, scale back to an ellipsoid, then calculate the normal.
	float3 npos = pt.xyz / radii.xyz; // normalised to unit sphere
	float3 tang0 = cross(npos.xyz, NotParallel(npos.xyz));
	float3 tang1 = cross(tang0, npos.xyz);
	normal.xyz = normalize(cross(tang0 * radii.xyz, tang1 * radii.xyz));
	return pt;
}

// Closest point on a cylinder (elliptic cylinder) to 'pos'
// The cylinder (elliptic cylinder) is at the origin, with the main axis along Z, and with radii 'radii'.
inline float4 ClosestPoint_PointToCylinder(float4 pos, float4 radii)
{
	// Normalise to the unit circle
	float2 npos = pos.xy / radii.xy;
	return pos; // todo
	
}
inline float4 ClosestPoint_PointToCylinder(float4 pos, float4 radii, out float4 normal)
{
	normal = float4(0,0,1,0);//todo
	return pos; // todo
}

// Find the closest point on 'prim' to 'pos'. Returns the surface normal at that point.
inline float4 ClosestPoint_PointToPrimitive(float4 pos, Prim prim, out float4 normal)
{
	// Transform 'pos' into primitive space
	float4x4 w2o = InvertOrthonormal(prim.o2w);
	float4 os_pos = mul(pos, w2o);
	float4 os_normal;

	// Get a direction vector that is a ray from the closest point on the surface to 'pos'
	switch (prim.flags.x)
	{
		case Prim_Plane:
		{
			os_pos = ClosestPoint_PointToPlane(os_pos, os_normal);
			break;
		}
		case Prim_Quad:
		{
			os_pos = ClosestPoint_PointToQuad(os_pos, prim.data[0].xy, os_normal);
			break;
		}
		case Prim_Triangle:
		{
			float4 bary;
			os_pos = ClosestPoint_PointToTriangle(os_pos, prim.data[0].xy, prim.data[0].zw, prim.data[1].xy, bary, os_normal);
			break;
		}
		case Prim_Ellipse:
		{
			os_pos = ClosestPoint_PointToEllipse(os_pos, prim.data[0].xy, os_normal);
			break;
		}
		case Prim_Box:
		{
			os_pos = ClosestPoint_PointToBox(os_pos, prim.data[0], os_normal);
			break;
		}
		case Prim_Sphere:
		{
			os_pos = ClosestPoint_PointToSphere(os_pos, prim.data[0], os_normal);
			break;
		}
		case Prim_Cylinder:
		{
			os_pos = ClosestPoint_PointToCylinder(os_pos, prim.data[0], os_normal);
			break;
		}
		default:
		{
			os_pos = float4(0,0,0,0) / 0;
			os_normal = float4(0,0,0,0) / 0;
			break;
		}
	}
		
	// Transform the point and normal back into world space
	normal = mul(os_normal, prim.o2w);
	return mul(os_pos, prim.o2w);
}

// Intersects a ray with a plane, returning true if there is an intercept.
// The plane is the XY plane with normal (0,0,1) and distance 0.
inline bool Intercept_RayVsPlane(float4 pos, float4 ray, inout float4 normal, inout float t1)
{
	// 'step' is the length of the projection of 'ray' on the normal
	// Is the ray moving away from the plane?
	float step = ray.z;
	if (step >= 0)
		return false;
	
	// 'dist' is the distance to the plane (scaled by |n|)
	// Unlike triangles, particles collide from any depth in the plane
	// Is the start point too far away to reach the plane?
	float dist = pos.z;
	if (dist >= -step)
		return false;

	// 't' is the parametric value of the intercept point.
	// The condition above guarantees t on the interval [-inf,1)
	float t = dist > 0 ? -dist / step : 0;
	if (t >= t1)
		return false;

	// Return the intercept
	t1 = t;
	normal = float4(0, 0, 1, 0);
	return true;
}

// Intersects a ray with a quad, returning true if there is an intercept
// The quad lies in the XY plane with normal pointing down the z-axis
inline bool Intercept_RayVsQuad(float4 pos, float4 ray, float2 radii, inout float4 normal, inout float t1)
{
	float4 norm = float4(0,0,0,0); float t = 0;
	if (!Intercept_RayVsPlane(pos, ray, norm, t))
		return false;

	// Find the intercept point
	float2 intercept = (pos + t * ray).xy;

	// Bounding quad test
	if (abs(intercept.x) > radii.x || abs(intercept.y) > radii.y)
		return false;
	
	t1 = t;
	normal = norm;
	return true;
}

// Intersects a ray with a triangle, returning true if there is an intercept
// The triangle lies in the XY plane with normal pointing down the z-axis.
inline bool Intercept_RayVsTriangle(float4 pos, float4 ray, float2 a, float2 b, float2 c, inout float4 normal, inout float t1)
{
	// Plane test first
	float4 norm = float4(0,0,0,0); float t = 0;
	if (!Intercept_RayVsPlane(pos, ray, norm, t))
		return false;

	// Find the intercept point
	float2 intercept = (pos + t * ray).xy;

	// If any coordinate is outside the interval [0,1], the intercept is not within the triangle.
	float4 bary = Barycentric(intercept, a, b, c);
	if (any(bary) < 0 || any(bary) >= 1 || all(bary) == 0)
		return false;

	// Return the intercept
	t1 = t;
	normal = norm;
	return true;
}

// Intersects a ray with an ellipse, returning true if there is an intercept
// The ellipse is in the XY plane with normal pointing down z-axis and with radii 'radii.xy'.
inline bool Intercept_RayVsEllipse(float4 pos, float4 ray, float2 radii, inout float4 normal, inout float t1)
{
	float4 norm = float4(0,0,0,0); float t = 0;
	if (!Intercept_RayVsPlane(pos, ray, norm, t))
		return false;
	
	// Find the intercept point
	float2 intercept = (pos + t * ray).xy;

	// If the point is outside the ellipse, then there is no intercept
	// Ellipse equation: x^2/a^2 + y^2/b^2 = 1
	if (sum(sqr(intercept) / sqr(radii)) > 1)
		return false;

	t1 = t;
	normal = norm;
	return true;
}

// Intersects a ray with a box, returning true if there is an intercept
// The box is at the origin with half extents 'radii'.
inline bool Intercept_RayVsBox(float4 pos, float4 ray, float4 radii, inout float4 normal, inout float t1)
{
	int axis = -1;
	float tmin = -FLT_MAX;
	float tmax = +FLT_MAX;
	
	// For all three slabs
	[unroll]
	for (int i = 0; i != 3; ++i)
	{
		// If the ray is parallel to the slab, then no hit if origin not within slab
		if (ray[i] == 0)
		{
			// On the surface is not a hit because the reflected ray is the same as the incident ray
			if (abs(pos[i]) >= radii[i])
				return false;
		}
		// Only if the ray is pointing into the slab
		else
		{
			// Compute intersection 't' value of ray with near and far plane of slab
			float ood = 1.0f / ray[i];
			float ta = (-radii[i] - pos[i]) * ood;
			float tb = (+radii[i] - pos[i]) * ood;

			// Make 'ta' be intersection with near plane, 'tb' with far plane
			if (ta > tb)
				swap(ta, tb);

			// Compute the intersection of slab intersection intervals
			if (ta > tmin) { tmin = ta; axis = i; }
			if (tb < tmax) { tmax = tb; }

			// Exit with no collision as soon as slab intersection becomes empty
			if (tmax - tmin < TINY)
				return false;
		}
	}

	// No collision if 'tmin' >= t1
	if (axis == -1 || tmin >= t1 || tmax <= 0 || sign(pos[axis]) == sign(ray[axis]))
		return false;
	
	// 'tmin' is the nearest intersection
	t1 = max(0, tmin);
	normal = float4(
		select(axis == 0, sign_nz(pos.x), 0),
		select(axis == 1, sign_nz(pos.y), 0),
		select(axis == 2, sign_nz(pos.z), 0),
	0);
	return true;
}

// Intersects a ray with a (elliptic) sphere, returning true if there is an intercept.
// The sphere is at the origin with radii 'radii'
inline bool Intercept_RayVsSphere(float4 pos, float4 ray, float4 radii, inout float4 normal, inout float t1)
{
	// Ellipsoid equation: x^2/a^2 + y^2/b^2 + z^2/c^2 = 1

	// Normalize to a unit sphere
	float4 npos = pos / radii;
	float4 nray = ray / radii;

	// If the point starts within the ellipsoid
	float x = sum(npos);
	if (x <= 1)
	{
		// 'pos' is at the origin
		if (x == 0)
		{
			t1 = 0;
			normal = Random3N(sum(pos));
			return true;
		}
		
		// Tangents scale correctly between sphere <=> ellipsoid so find orthogonal
		// tangents, scale back to an ellipsoid, then calculate the normal.
		t1 = 0;
		float3 tang0 = cross(npos.xyz, NotParallel(npos.xyz));
		float3 tang1 = cross(tang0, npos.xyz);
		normal.xyz = normalize(cross(tang0 * radii.xyz, tang1 * radii.xyz));
		return true;
	}

	// The point starts outside	the ellipsoid
	// Zero length rays do not intersect.
	float ray_sq = dot(ray, ray);
	if (ray_sq == 0)
		return false;

	// Find the closest point on the infinite line through 'ray'
	// If the closest point is not within the sphere then there is no intersection.
	float4 closest = pos - (dot(ray, pos) / ray_sq) * ray;
	float closest_sq = dot(closest, closest);
	if (closest_sq >= 1)
		return false;

	// Get the distance from the closest point to the intersection with the boundary of the sphere
	float c = sqrt((1 - closest_sq) / ray_sq); // includes the normalising 1/|ray| in x

	// Get the parametric values of the intersection
	float4 half_chord = ray * c;
	float4 hit_min = closest - half_chord;
	float4 hit_max = closest + half_chord;
	float tmin = dot(ray, hit_min - pos) / ray_sq;
	float tmax = dot(ray, hit_max - pos) / ray_sq;

	// Does the ray intersect the sphere?
	// Is the intercept further than 't1'.
	if (tmax < 0 || tmin >= t1)
		return false;
	
	t1 = tmin;
	float3 tang0 = cross(hit_min.xyz, NotParallel(hit_min.xyz));
	float3 tang1 = cross(tang0, hit_min.xyz);
	normal.xyz = normalize(cross(tang0 * radii.xyz, tang1 * radii.xyz));
	return true;
}

// Intersects a ray with an (elliptic) cylinder, returning true if there is an intercept.
// The cylinder has the main axis on the z-axis with radius 'radii.xy' and length 'radii.z'.
inline bool Intercept_RayVsCylinder(float4 pos, float4 ray, float4 radii, inout float4 normal, inout float t1)
{
	// If 'pos' is within the radii of the infinite cylinder...
	float x = sqr(pos.x) / sqr(radii.x) + sqr(pos.y) / sqr(radii.y);
	if (x <= 1)
	{
		// Test the start position against the end caps
		float z = abs(pos.z) - abs(radii.z);
		if (z > 0)
		{
			// The start position is outside the cylinder.
			// Test for intersection with a disc.
			// Adjust 'pos' so that we're in disc space.
			if (pos.z < 0)
			{
				pos.z = -pos.z;
				ray.z = -ray.z;
			}
			pos.z -= radii.z;
			return Intercept_RayVsEllipse(pos, ray, radii.xy, normal, t1);
		}

		// The start position is inside the cylinder.
		// Calculate the normal to the cylinder wall ellipse.
		// The slope of the tangent of an ellipse at point (x,y) is 
			
		//float4 surface = float4(
		//	select(pos.x != 0, pos.x / sqr(radii.x), 0),
			
		//todo
		return false;
	}

	// The start position is outside the cylinder.
		
	//todo
	return false;

	// Find the closest point between two line segments
}

// Perform a ray intersection test with the primitive
inline int Intercept_RayVsPrimitive(float4 pos, float4 ray, Prim prim, inout float4 normal, inout float t1)
{
	// Transform 'pos' and 'ray' into primitive space
	float4x4 w2o = InvertOrthonormal(prim.o2w);
	float4 os_pos = mul(pos, w2o);
	float4 os_ray = mul(ray, w2o);
	
	// Collide against the primitive
	int intercept = 0;
	switch (prim.flags.x)
	{
		case Prim_Plane:
		{
			intercept = Intercept_RayVsPlane(os_pos, os_ray, normal, t1);
			break;
		}
		case Prim_Quad:
		{
			intercept = Intercept_RayVsQuad(os_pos, os_ray, prim.data[0].xy, normal, t1);
			break;
		}
		case Prim_Triangle:
		{
			intercept = Intercept_RayVsTriangle(os_pos, os_ray, prim.data[0].xy, prim.data[0].zw, prim.data[1].xy, normal, t1);
			break;
		}
		case Prim_Ellipse:
		{
			intercept = Intercept_RayVsEllipse(os_pos, os_ray, prim.data[0].xy, normal, t1);
			break;
		}
		case Prim_Box:
		{
			intercept = Intercept_RayVsBox(os_pos, os_ray, prim.data[0], normal, t1);
			break;
		}
		case Prim_Sphere:
		{
			intercept = Intercept_RayVsSphere(os_pos, os_ray, prim.data[0], normal, t1);
			break;
		}
		case Prim_Cylinder:
		{
			intercept = Intercept_RayVsCylinder(os_pos, os_ray, prim.data[0], normal, t1);
			break;
		}
	}
	
	// Transform the normal back into world space
	normal = select(intercept, mul(normal, prim.o2w), normal);
	return intercept;
}

#endif
