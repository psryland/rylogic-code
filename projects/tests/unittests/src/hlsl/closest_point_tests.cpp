//*********************************************
// HLSL ClosestPointTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/closest_point.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(ClosestPointTests)
	{
		PRUnitTestMethod(PointVsRayOrigin)
		{
			// Point at the ray origin: t = 0
			auto s = float4(0, 0, 0, 1);
			auto d = float4(1, 0, 0, 0);
			auto pt = float4(0, 0, 0, 1);

			auto t = ClosestPoint_PointVsRay(pt, s, d);
			PR_EXPECT(FEql(t, 0.0f));
		}
		PRUnitTestMethod(PointVsRayProjection)
		{
			// Point offset perpendicular to ray
			auto s = float4(0, 0, 0, 1);
			auto d = float4(1, 0, 0, 0);
			auto pt = float4(3, 5, 0, 1);

			auto t = ClosestPoint_PointVsRay(pt, s, d);
			PR_EXPECT(FEql(t, 3.0f));
		}
		PRUnitTestMethod(PointVsRayBehind)
		{
			// Point behind ray start
			auto s = float4(0, 0, 0, 1);
			auto d = float4(1, 0, 0, 0);
			auto pt = float4(-2, 1, 0, 1);

			auto t = ClosestPoint_PointVsRay(pt, s, d);
			PR_EXPECT(FEql(t, -2.0f));
		}
		PRUnitTestMethod(RayToRayParallel)
		{
			// Parallel rays
			auto s0 = float4(0, 0, 0, 1);
			auto d0 = float4(1, 0, 0, 0);
			auto s1 = float4(0, 1, 0, 1);
			auto d1 = float4(1, 0, 0, 0);

			auto t = ClosestPoint_RayToRay(s0, d0, s1, d1);

			// For parallel rays, the distance should be constant.
			// The function returns t0=0 for parallel case.
			auto p0 = s0 + t.x * d0;
			auto p1 = s1 + t.y * d1;
			auto dist = length(p0 - p1);
			PR_EXPECT(FEql(dist, 1.0f));
		}
		PRUnitTestMethod(RayToRayPerpendicular)
		{
			// Two perpendicular skew rays
			auto s0 = float4(0, 0, 0, 1);
			auto d0 = float4(1, 0, 0, 0);
			auto s1 = float4(0, 0, 1, 1);
			auto d1 = float4(0, 1, 0, 0);

			auto t = ClosestPoint_RayToRay(s0, d0, s1, d1);

			// Closest points should be at (0,0,0) and (0,0,1)
			PR_EXPECT(FEql(t.x, 0.0f));
			PR_EXPECT(FEql(t.y, 0.0f));
		}
		PRUnitTestMethod(RayToRayIntersecting)
		{
			// Two rays that actually intersect at (1, 1, 0)
			auto s0 = float4(0, 0, 0, 1);
			auto d0 = float4(1, 1, 0, 0);
			auto s1 = float4(2, 0, 0, 1);
			auto d1 = float4(-1, 1, 0, 0);

			auto t = ClosestPoint_RayToRay(s0, d0, s1, d1);

			auto p0 = s0 + t.x * d0;
			auto p1 = s1 + t.y * d1;
			auto dist = length(p0 - p1);
			PR_EXPECT(FEqlRelative(dist, 0.0f, 1e-3f));
			PR_EXPECT(FEql(p0.x, 1.0f));
			PR_EXPECT(FEql(p0.y, 1.0f));
		}
		PRUnitTestMethod(RayToTriangleHit)
		{
			// Ray aimed at centre of triangle - use the 5-arg overload (no out param)
			auto s = float4(0.2f, 0.2f, 2.0f, 1.0f);
			auto d = float4(0, 0, -1, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto result = ClosestPoint_RayToTriangle(s, d, a, b, c);
			constexpr auto tol = 1e-5f;

			// result.xyz are barycentric, result.w is parametric distance along ray
			PR_EXPECT(FEql(result.w, 2.0f)); // parametric distance along ray
			PR_EXPECT(result.x >= -tol);
			PR_EXPECT(result.y >= -tol);
			PR_EXPECT(result.z >= -tol);
		}
		PRUnitTestMethod(RayToTriangleMiss)
		{
			// Ray that misses the triangle - should find closest edge point
			auto s = float4(2, 2, 2, 1);
			auto d = float4(0, 0, -1, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto result = ClosestPoint_RayToTriangle(s, d, a, b, c);

			// result.w is the parametric value along the ray to the closest point
			PR_EXPECT(FEql(result.w, 2.0f)); // z-distance to the triangle plane
		}
	};
}
