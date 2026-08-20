//*********************************************
// HLSL InterceptTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/intercept.hlsli"
#include "pr/hlsl/geometry.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(InterceptTests)
	{
		static constexpr float tol = 1e-4f;

		PRUnitTestMethod(RayVsTriangleHit, Quick)
		{
			// Ray along -Z hitting a triangle in the XY plane
			auto s = float4(0.25f, 0.25f, 1.0f, 1.0f);
			auto d = float4(0, 0, -1.0f, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto bary = Intercept_RayVsTriangle(s, d, a, b, c);

			// Should not be all-zero (i.e. there is an intercept)
			PR_EXPECT(!AllZero(bary));

			// Barycentric coords should be in [0,1]
			PR_EXPECT(bary.x >= -tol && bary.x <= 1.0f + tol);
			PR_EXPECT(bary.y >= -tol && bary.y <= 1.0f + tol);
			PR_EXPECT(bary.z >= -tol && bary.z <= 1.0f + tol);

			// Sum should be ~1
			PR_EXPECT(FEql(bary.x + bary.y + bary.z, 1.0f));

			// The intercept point should be (0.25, 0.25, 0)
			auto pt = BaryPoint(a, b, c, bary);
			PR_EXPECT(FEql(pt.x, 0.25f));
			PR_EXPECT(FEql(pt.y, 0.25f));
		}
		PRUnitTestMethod(RayVsTriangleMiss, Quick)
		{
			// Ray that misses the triangle
			auto s = float4(5, 5, 1, 1);
			auto d = float4(0, 0, -1, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto bary = Intercept_RayVsTriangle(s, d, a, b, c);

			// Either all-zero or bary coords outside [0,1]
			bool miss = AllZero(bary) || bary.x < 0 || bary.y < 0 || bary.z < 0;
			PR_EXPECT(miss);
		}
		PRUnitTestMethod(RayVsTriangleParallel, Quick)
		{
			// Ray parallel to the triangle plane
			auto s = float4(0, 0, 0, 1);
			auto d = float4(1, 0, 0, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto bary = Intercept_RayVsTriangle(s, d, a, b, c);

			// Should return zero (degenerate / parallel)
			PR_EXPECT(AllZero(bary));
		}
		PRUnitTestMethod(RayVsTriangleAtVertex, Quick)
		{
			// Ray aimed directly at vertex 'a'
			auto s = float4(0, 0, 1, 1);
			auto d = float4(0, 0, -1, 0);
			auto a = float4(0, 0, 0, 1);
			auto b = float4(1, 0, 0, 1);
			auto c = float4(0, 1, 0, 1);

			auto bary = Intercept_RayVsTriangle(s, d, a, b, c);

			// Bary should indicate vertex 'a': (1, 0, 0)
			PR_EXPECT(FEql(bary.x, 1.0f));
			PR_EXPECT(FEql(bary.y, 0.0f));
			PR_EXPECT(FEql(bary.z, 0.0f));
		}
	};
}
