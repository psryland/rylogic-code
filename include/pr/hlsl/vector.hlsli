//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_VECTOR_HLSLI
#define PR_HLSL_VECTOR_HLSLI
#include "pr/hlsl/core.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// Returns true if all vector elements are 0
bool AllZero(float2 a)
{
	return !any(a);
}
bool AllZero(float3 a)
{
	return !any(a);
}
bool AllZero(float4 a)
{
	return !any(a);
}

// Returns true if all vector elements are >= 0
bool AllZeroOrPositive(float2 a)
{
	return !any(abs(a) - a);
}
bool AllZeroOrPositive(float3 a)
{
	return !any(abs(a) - a);
}
bool AllZeroOrPositive(float4 a)
{
	return !any(abs(a) - a);
}

// Return the sum of the components of 'vec'
float SumComponents(float2 vec)
{
	return dot(vec, float2(1, 1));
}
float SumComponents(float3 vec)
{
	return dot(vec, float3(1, 1, 1));
}
float SumComponents(float4 vec)
{
	return dot(vec, float4(1,1,1,1));
}

// Triple product
float Triple(float4 a, float4 b, float4 c)
{
	return dot(a, float4(cross(b.xyz, c.xyz), 0));
}

// Cross-product matrix: CPM(r) * v = cross(r, v)
float3x3 CrossProductMatrix(float3 r)
{
	return float3x3(
		float3(0, r.z, -r.y),
		float3(-r.z, 0, r.x),
		float3(r.y, -r.x, 0));
}

// Rotate a 2D vector
float2 RotateCW(float2 a)
{
	return float2(a.y, -a.x);
}
float2 RotateCCW(float2 a)
{
	return float2(-a.y, a.x);
}

// 2D cross product
float Cross2D(float2 lhs, float2 rhs)
{
	return lhs.x * rhs.y - lhs.y * rhs.x;
}

// Return a vector that is not parallel to 'v'
inline float2 NotParallel(float2 v)
{
	v = abs(v);
	return select(v.x > v.y, float2(0,1), float2(1,0));
}
inline float3 NotParallel(float3 v)
{
	v = abs(v);
	return select(v.x > v.y && v.x > v.z, float3(0,0,1), float3(1,0,0));
}
inline float4 NotParallel(float4 v)
{
	v = abs(v);
	return select(v.x > v.y && v.x > v.z, float4(0,0,1,0), float4(1,0,0,0));
}

// Return a vector perpendicular to 'v'
inline float3 Perpendicular(float3 v)
{
	// Choose the axis least aligned with v, then cross
	float3 ax = abs(v.x) < abs(v.y) ? float3(1, 0, 0) : float3(0, 1, 0);
	if (abs(v.z) < abs(ax.x * v.x + ax.y * v.y + ax.z * v.z))
		ax = float3(0, 0, 1);
	
	return cross(v, ax);
}

// Normalise a vector or return zero if the length is zero
inline float2 NormaliseOrZero(float2 vec)
{
	float len = length(vec);
	return select(len != 0, vec / len, float2(0,0));
}
inline float3 NormaliseOrZero(float3 vec)
{
	float len = length(vec);
	return select(len != 0, vec / len, float3(0,0,0));
}
inline float4 NormaliseOrZero(float4 vec)
{
	float len = length(vec);
	return select(len != 0, vec / len, float4(0,0,0,0));
}

// Orthonormalise a rotation matrix
float3x3 Orthonormalise(float3x3 mat)
{
	mat[0] = normalize(mat[0]);
	mat[1] = normalize(cross(mat[2], mat[0]));
	mat[2] = cross(mat[0], mat[1]);
	return mat;
}
float4x4 Orthonormalise(float4x4 mat)
{
	mat[0].xyz = normalize(mat[0].xyz);
	mat[1].xyz = normalize(cross(mat[2].xyz, mat[0].xyz));
	mat[2].xyz = cross(mat[0].xyz, mat[1].xyz);
	return mat;
}

// Invert an orthonormal matrix
float4x4 InvertOrthonormal(float4x4 mat)
{
	// This assumes row_major float4x4's
	return float4x4(
		float4(mat[0].x, mat[1].x, mat[2].x, 0),
		float4(mat[0].y, mat[1].y, mat[2].y, 0),
		float4(mat[0].z, mat[1].z, mat[2].z, 0),
		float4(
			-dot(mat[0], mat[3]),
			-dot(mat[1], mat[3]),
			-dot(mat[2], mat[3]),
			1));
}

// Invert a matrix assuming that it's an affine matrix
float4x4 InvertAffine(float4x4 mat)
{
	float3 t = mat[3].xyz;
	float3x3 r = (float3x3)mat;
	float3 s = float3(length(r[0]), length(r[1]), length(r[2]));
	
	// Remove scale
	r[0] /= s.x;
	r[1] /= s.y;
	r[2] /= s.z;

	// Invert rotation
	float3x3 rt = transpose(r);

	// Invert scale
	rt[0] /= s.x;
	rt[1] /= s.y;
	rt[2] /= s.z;

	// Invert translation
	float3 inv_t = float3(
		-(t.x * rt[0].x + t.y * rt[1].x + t.z * rt[2].x),
		-(t.x * rt[0].y + t.y * rt[1].y + t.z * rt[2].y),
		-(t.x * rt[0].z + t.y * rt[1].z + t.z * rt[2].z)
	);

	// Reconstruct the inverse matrix
	return float4x4(
		float4(rt[0].x, rt[0].y, rt[0].z, 0),
		float4(rt[1].x, rt[1].y, rt[1].z, 0),
		float4(rt[2].x, rt[2].y, rt[2].z, 0),
		float4(inv_t.x, inv_t.y, inv_t.z, 1)
	);
}

// Invert a 3x3 matrix using cofactor expansion
float3x3 Invert(float3x3 m)
{
	float3 c0 = cross(m[1], m[2]);
	float3 c1 = cross(m[2], m[0]);
	float3 c2 = cross(m[0], m[1]);
	float det = dot(m[0], c0);
	float inv_det = 1.0f / det;
	return transpose(float3x3(c0, c1, c2)) * inv_det;
}

// Return a vector representing the approximate rotation between two orthonormal transforms
float3 RotationVectorApprox(float3x3 from, float3x3 to)
{
	// Note: 'from' and 'to' must be orthonormal matrices.
	//
	// Given a vector 'avel' representing a small step of angular velocity, a rotation
	// can be applied to an orthonormal matrix by adding the cross product of each basis
	// vector to the matrix. e.g.,
	//   float3x3 orientation = {...};
	//   orientation[0] += cross(avel, orientation[0]);
	//   orientation[1] += cross(avel, orientation[2]);
	//   orientation[2] += cross(avel, orientation[1]);
	//   Orthonormalize(orientation); // renormalize the basis vectors
	//
	// Vector cross product can also be represented using a cross product matrix:
	//   CPM of avel = 
	//      [   0     avel.z  -avel.y]
	//      [-avel.z    0      avel.x]
	//      [ avel.y -avel.x     0   ]
	// So the rotation can be applied like this:
	//   orientation += CPM(avel) * orientation;
	//
	// This functions uses the inverse of this principle to recreate the 'avel' vector
	// from two orientation matrices, assuming one matrix is a small rotation from the other.
	// i.e.
	//   to = from + CPM(avel) * from
	//   to - from = CPM(avel) * from
	//   (to - from) * fromT = CPM(avel) * from * fromT
	//   (to - from) * fromT = CPM(avel) <-- can read out the components of 'avel' from this
	float3x3 cpm = mul((to - from), transpose(from));
	return float3(cpm[1].z, cpm[2].x, cpm[0].y);
}

#ifdef __cplusplus
}
#endif
#endif
