//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(DynamicsTests)
	{
		PRUnitTestMethod(ScalarDynamics, float, double)
		{
			// Test with a simple constant velocity motion: x = v*t
			// 5 samples at t=0,1,2,3,4 with dt=1, x = 2*t => x = 0,2,4,6,8
			T const samples[5] = { T(0), T(2), T(4), T(6), T(8) };
			auto [pos, vel, acc] = CalculateScalarDynamics(std::span<T const>(samples), T(1));

			// Position should be the centre sample
			PR_EXPECT(FEql(pos, T(4)));

			// Velocity should be constant at 2
			PR_EXPECT(Abs(vel - T(2)) < T(0.1));

			// Acceleration should be ~0
			PR_EXPECT(Abs(acc) < T(0.5));
		}

		PRUnitTestMethod(TranslationalDynamics, float, double)
		{
			using V4 = Vec4<T>;
			using Q = Quat<T>;
			using X = Xform<T>;

			// 5 keyframes of constant velocity: moving at (1,0,0) per second
			X const samples[5] = {
				X(V4(0, 0, 0, 1), Identity<Q>()),
				X(V4(1, 0, 0, 1), Identity<Q>()),
				X(V4(2, 0, 0, 1), Identity<Q>()),
				X(V4(3, 0, 0, 1), Identity<Q>()),
				X(V4(4, 0, 0, 1), Identity<Q>()),
			};
			auto [pos, vel, acc] = CalculateTranslationalDynamics(std::span<X const, 5>(samples), T(1));

			// Centre position
			PR_EXPECT(FEql(pos, V4(2, 0, 0, 1)));

			// Constant velocity
			PR_EXPECT(FEqlAbsolute(vel, V4(1, 0, 0, 0), T(0.1)));
		}
	};
}
#endif
