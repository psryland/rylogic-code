//*********************************************
// HLSL GeometryTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/geometry.hlsli"
#include "pr/geometry/point.h"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(GeometryTests)
	{
		PRUnitTestMethod(FaceNormalSimple)
		{
			// Triangle in XY plane: normal should be +Z
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);
			auto n = FaceNormal(a, b, c);
			PR_EXPECT(FEql(n, float4(0, 0, 1, 0)));
		}
		PRUnitTestMethod(FaceNormalDegenerate)
		{
			// Degenerate triangle (collinear points) should return default
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(2, 0, 0, 1);
			auto def = float4(0, 1, 0, 0);
			auto n = FaceNormal(a, b, c, def);
			PR_EXPECT(FEql(n, def));
		}
		PRUnitTestMethod(BarycentricAtVertices)
		{
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			// At vertex a: bary = (1, 0, 0)
			auto ba = Barycentric(a, a, b, c);
			PR_EXPECT(FEql(ba.x, 1.0f));
			PR_EXPECT(FEql(ba.y, 0.0f));
			PR_EXPECT(FEql(ba.z, 0.0f));
			PR_EXPECT(FEql(ba, pr::geometry::Barycentric(a, a, b, c)));

			// At vertex b: bary = (0, 1, 0)
			auto bb = Barycentric(b, a, b, c);
			PR_EXPECT(FEql(bb.x, 0.0f));
			PR_EXPECT(FEql(bb.y, 1.0f));
			PR_EXPECT(FEql(bb.z, 0.0f));
			PR_EXPECT(FEql(bb, pr::geometry::Barycentric(b, a, b, c)));

			// At vertex c: bary = (0, 0, 1)
			auto bc = Barycentric(c, a, b, c);
			PR_EXPECT(FEql(bc.x, 0.0f));
			PR_EXPECT(FEql(bc.y, 0.0f));
			PR_EXPECT(FEql(bc.z, 1.0f));
			PR_EXPECT(FEql(bc, pr::geometry::Barycentric(c, a, b, c)));
		}
		PRUnitTestMethod(BarycentricCentroid)
		{
			auto a = float4(0, 0, 0, 1);
			auto b = float4(3, 0, 0, 1);
			auto c = float4(0, 3, 0, 1);
			auto centroid = float4(1, 1, 0, 1);

			auto bary = Barycentric(centroid, a, b, c);
			PR_EXPECT(FEql(bary.x, 1.0f / 3.0f));
			PR_EXPECT(FEql(bary.y, 1.0f / 3.0f));
			PR_EXPECT(FEql(bary.z, 1.0f / 3.0f));

			// Compare HLSL vs CPU
			PR_EXPECT(FEql(bary, pr::geometry::Barycentric(centroid, a, b, c)));
		}
		PRUnitTestMethod(BaryPointRoundtrip)
		{
			auto a = float4(1, 2, 3, 1);
			auto b = float4(4, 0, -1, 1);
			auto c = float4(-2, 5, 1, 1);
			auto bary = float3(0.3f, 0.5f, 0.2f);

			auto pt = BaryPoint(a, b, c, bary);
			auto bary2 = Barycentric(pt, a, b, c);
			PR_EXPECT(FEql(bary2.x, bary.x));
			PR_EXPECT(FEql(bary2.y, bary.y));
			PR_EXPECT(FEql(bary2.z, bary.z));

			// Compare HLSL vs CPU
			PR_EXPECT(FEql(bary2, pr::geometry::Barycentric(pt, a, b, c)));
		}
		PRUnitTestMethod(PointInTriangleInside)
		{
			auto a = float3(0, 0, 0);
			auto b = float3(4, 0, 0);
			auto c = float3(0, 4, 0);

			// Centroid is inside
			PR_EXPECT(FEql(PointInTriangle(float3(1, 1, 0), a, b, c), 1.0f));

			// Vertex is inside
			PR_EXPECT(FEql(PointInTriangle(a, a, b, c), 1.0f));
		}
		PRUnitTestMethod(PointInTriangleOutside)
		{
			auto a = float3(0, 0, 0);
			auto b = float3(4, 0, 0);
			auto c = float3(0, 4, 0);

			// Clearly outside
			PR_EXPECT(FEql(PointInTriangle(float3(5, 5, 0), a, b, c), 0.0f));
			PR_EXPECT(FEql(PointInTriangle(float3(-1, -1, 0), a, b, c), 0.0f));
		}
		PRUnitTestMethod(Coplanarity_Parallel)
		{
			// Two triangles sharing edge (a,c) with same normal
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);
			auto d = float4(-1, 0, 0, 1);

			auto cop = Coplanarity(a, b, c, d);
			PR_EXPECT(FEql(cop, 1.0f));
		}
		PRUnitTestMethod(ProjectVector)
		{
			// Project (1,1,0) onto the plane normal to (0,0,1) = (1,1,0)
			auto pt = float4(1, 1, 1, 0);
			auto dir = float4(0, 0, 1, 0);
			auto proj = Project(pt, dir);
			PR_EXPECT(FEql(proj, float4(1, 1, 0, 0)));
		}
		PRUnitTestMethod(Barycentric2D)
		{
			auto a = float2(0, 0);
			auto b = float2(2, 0);
			auto c = float2(0, 2);
			auto mid = float2(0.5f, 0.5f);

			auto bary = Barycentric(mid, a, b, c);
			auto pt = BaryPoint(a, b, c, bary);
			PR_EXPECT(FEql(pt.x, mid.x));
			PR_EXPECT(FEql(pt.y, mid.y));
		}
		PRUnitTestMethod(FibonacciSpiralUnitSphere)
		{
			// All points from FibonacciSpiral should be on the unit sphere
			int N = 32;
			for (int i = 0; i != N; ++i)
			{
				auto pt = FibonacciSpiral(i, N);
				float len = length(float3(pt.x, pt.y, pt.z));
				PR_EXPECT(FEqlAbsolute(len, 1.0f, 1e-4f));
			}
		}
	};
}
