// Face Normal Compute Shader
//  Demonstrates Shaderator's dual-compilation approach: this file compiles
//  as both HLSL (for GPU dispatch) and C++ (for CPU debugging).
//
//  The algorithm matches pr/hlsl/geometry.hlsli::FaceNormal().
//  See that file for the full Rylogic HLSL geometry library.

#include "pr/hlsl/interop.hlsli"

struct Triangle
{
	float4 a;
	float4 b;
	float4 c;
};

static const int ThreadGroupSize = 32;

struct CbParams
{
	int NumTriangles;
};

// Buffer bindings
ConstantBuffer<CbParams> resource(Constants, b0);
StructuredBuffer<Triangle> resource(InputTriangles, t0);
RWStructuredBuffer<float4> resource(OutputNormals, u0);

// Compute the face normal for triangle (a, b, c).
// Returns a unit-length direction or (0,0,0,0) for degenerate triangles.
float4 FaceNormal(float4 a, float4 b, float4 c)
{
	float4 ab = b - a;
	float4 cb = c - b;

	float3 edge1 = float3(ab.x, ab.y, ab.z);
	float3 edge2 = float3(cb.x, cb.y, cb.z);
	float3 n = cross(edge1, edge2);
	float len = length(n);
	if (len > 0)
		return float4(n.x / len, n.y / len, n.z / len, 0);

	return float4(0, 0, 0, 0);
}

// Compute shader entry point - one thread per triangle
numthreads(CSFaceNormal, ThreadGroupSize, 1, 1)
void CSFaceNormal(int3 DTID(dtid))
{
	if (dtid.x >= Constants.NumTriangles)
		return;
	
	OutputNormals[dtid.x] = FaceNormal(
		InputTriangles[dtid.x].a,
		InputTriangles[dtid.x].b,
		InputTriangles[dtid.x].c);
}
