//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(InterpolateTests)
	{
		PRUnitTestMethod(LerpScalar, float, double)
		{
			PR_EXPECT(FEql(Lerp(T(0), T(10), T(0.5)), T(5)));
			PR_EXPECT(FEql(Lerp(T(0), T(10), T(0)), T(0)));
			PR_EXPECT(FEql(Lerp(T(0), T(10), T(1)), T(10)));
			PR_EXPECT(FEql(Lerp(T(-5), T(5), T(0.5)), T(0)));
		}

		PRUnitTestMethod(LerpVector, float, double)
		{
			using V4 = Vec4<T>;

			auto a = V4(0, 0, 0, 1);
			auto b = V4(10, 20, 30, 1);
			auto mid = Lerp(a, b, T(0.5));
			PR_EXPECT(FEql(mid, V4(5, 10, 15, 1)));
		}

		PRUnitTestMethod(InterpolateVectorTest, float, double)
		{
			using V4 = Vec4<T>;
			using Q = Quat<T>;

			// Simple linear interpolation between two positions with zero velocity
			auto p0 = V4(0, 0, 0, 1);
			auto v0 = V4(0, 0, 0, 0);
			auto p1 = V4(10, 0, 0, 1);
			auto v1 = V4(0, 0, 0, 0);
			auto interp = InterpolateVector<T>(p0, v0, p1, v1, T(1));

			// At t=0, should be at p0
			PR_EXPECT(FEql(interp.Eval(T(0)), p0));

			// At t=1, should be at p1
			PR_EXPECT(FEql(interp.Eval(T(1)), p1));

			// At t=0.5, should be somewhere in between
			auto mid = interp.Eval(T(0.5));
			PR_EXPECT(mid.x > T(0) && mid.x < T(10));
		}

		PRUnitTestMethod(InterpolateRotationTest, float, double)
		{
			using V4 = Vec4<T>;
			using Q = Quat<T>;

			// Interpolate between identity and 90-degree rotation about Z
			auto q0 = Identity<Q>();
			auto w0 = V4(0, 0, 0, 0); // zero angular velocity
			auto q1 = Quat<T>(T(0), T(0), Sqrt(T(0.5)), Sqrt(T(0.5))); // 90° about Z
			auto w1 = V4(0, 0, 0, 0);

			auto interp = InterpolateRotation<T>(q0, w0, q1, w1, T(1));

			// At t=0, should be identity
			auto r0 = interp.Eval(T(0));
			PR_EXPECT(FEql(r0, q0));

			// At t=1, should be q1 (or equivalent)
			auto r1 = interp.Eval(T(1));
			PR_EXPECT(FEql(r1, q1) || FEql(r1, -q1));
		}
	};
}
#endif
