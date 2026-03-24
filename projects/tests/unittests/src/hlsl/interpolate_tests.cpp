//*********************************************
// HLSL InterpolateTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/interpolate.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(InterpolateTests)
	{
		// q and -q represent the same rotation
		static bool QuatEql(float4 a, float4 b)
		{
			return FEql(a, b) || FEql(a, -b);
		}

		PRUnitTestMethod(HermiteSplineEndpoints)
		{
			// Hermite spline should pass through its endpoints
			auto x0 = float3(1, 2, 3);
			auto v0 = float3(1, 0, 0);
			auto x1 = float3(4, 5, 6);
			auto v1 = float3(0, 1, 0);

			auto hs = HermiteSpline_Create(x0, v0, x1, v1);

			auto p0 = HermiteSpline_Position(hs, 0.0f);
			auto p1 = HermiteSpline_Position(hs, 1.0f);
			PR_EXPECT(FEql(p0, x0));
			PR_EXPECT(FEql(p1, x1));
		}
		PRUnitTestMethod(HermiteSplineTangents)
		{
			// Tangents at endpoints should match
			auto x0 = float3(0, 0, 0);
			auto v0 = float3(3, 0, 0);
			auto x1 = float3(3, 3, 0);
			auto v1 = float3(0, 3, 0);

			auto hs = HermiteSpline_Create(x0, v0, x1, v1);

			auto t0 = HermiteSpline_Velocity(hs, 0.0f);
			auto t1 = HermiteSpline_Velocity(hs, 1.0f);
			PR_EXPECT(FEql(t0, v0));
			PR_EXPECT(FEql(t1, v1));
		}
		PRUnitTestMethod(HermiteSplineLinear)
		{
			// Linear case: if v0 = v1 = (x1 - x0), the curve should be a straight line
			auto x0 = float3(0, 0, 0);
			auto x1 = float3(1, 1, 1);
			auto v = x1 - x0;
			auto hs = HermiteSpline_Create(x0, v, x1, v);

			// Midpoint should be the linear midpoint
			auto mid = HermiteSpline_Position(hs, 0.5f);
			PR_EXPECT(FEql(mid, float3(0.5f, 0.5f, 0.5f)));
		}
		PRUnitTestMethod(InterpolateVectorEndpoints)
		{
			auto x0 = float3(1, 0, 0);
			auto v0 = float3(0, 0, 0);
			auto x1 = float3(0, 1, 0);
			auto v1 = float3(0, 0, 0);
			float interval = 1.0f;

			auto interp = InterpolateVector_Create(x0, v0, x1, v1, interval);

			auto p0 = InterpolateVector_Eval(interp, 0.0f);
			auto p1 = InterpolateVector_Eval(interp, interval);
			PR_EXPECT(FEql(p0, x0));
			PR_EXPECT(FEql(p1, x1));
		}
		PRUnitTestMethod(InterpolateVectorVelocity)
		{
			auto x0 = float3(0, 0, 0);
			auto v0 = float3(2, 0, 0);
			auto x1 = float3(2, 0, 0);
			auto v1 = float3(2, 0, 0);
			float interval = 1.0f;

			auto interp = InterpolateVector_Create(x0, v0, x1, v1, interval);

			// For constant velocity linear motion, the derivative should be ~v everywhere
			auto dv = InterpolateVector_EvalDerivative(interp, 0.0f);
			PR_EXPECT(FEql(dv, v0));
		}
		PRUnitTestMethod(InterpolateRotationEndpoints)
		{
			// Start at identity, end at 90 degrees around Z
			auto q0 = quat_identity();
			auto w0 = float3(0, 0, 0);
			float angle = 3.14159265f / 2.0f;
			auto q1 = normalize(float4(0, 0, sin(angle / 2), cos(angle / 2)));
			auto w1 = float3(0, 0, 0);
			float interval = 1.0f;

			auto interp = InterpolateRotation_Create(q0, w0, q1, w1, interval);

			auto r0 = InterpolateRotation_Eval(interp, 0.0f);
			auto r1 = InterpolateRotation_Eval(interp, interval);
			PR_EXPECT(QuatEql(r0, q0));
			PR_EXPECT(QuatEql(r1, q1));
		}
		PRUnitTestMethod(InterpolateRotationUnitQuaternion)
		{
			// The interpolated quaternion should always be unit length
			auto q0 = normalize(float4(1, 0, 0, 1));
			auto w0 = float3(0, 0, 1);
			auto q1 = normalize(float4(0, 1, 0, 1));
			auto w1 = float3(0, 0, -1);
			float interval = 1.0f;

			auto interp = InterpolateRotation_Create(q0, w0, q1, w1, interval);

			for (int i = 0; i != 10; ++i)
			{
				float t = i / 9.0f * interval;
				auto q = InterpolateRotation_Eval(interp, t);
				float len = length(float4(q.x, q.y, q.z, q.w));
				PR_EXPECT(FEqlAbsolute(len, 1.0f, 1e-3f));
			}
		}
		PRUnitTestMethod(VelCorrectedHermiteEndpoints)
		{
			auto prev = float3(0, 0, 0);
			auto next = float3(2, 0, 0);
			auto pos = float3(1, 0, 0);
			auto vel = float3(1, 0, 0);
			float interval = 2.0f;
			float T = interval * 0.5f;

			auto interp = VelCorrectedHermite_Create(prev, next, pos, vel, interval);

			// At t=-T => pos_prev, t=0 => pos, t=+T => pos_next
			auto p_prev = VelCorrectedHermite_Eval(interp, -T);
			auto p_mid = VelCorrectedHermite_Eval(interp, 0.0f);
			auto p_next = VelCorrectedHermite_Eval(interp, T);

			PR_EXPECT(FEql(p_prev, prev));
			PR_EXPECT(FEql(p_mid, pos));
			PR_EXPECT(FEql(p_next, next));
		}
		PRUnitTestMethod(VelCorrectedHermiteMidVelocity)
		{
			auto prev = float3(0, 0, 0);
			auto next = float3(2, 0, 0);
			auto pos = float3(1, 0, 0);
			auto vel = float3(1, 0, 0);
			float interval = 2.0f;

			auto interp = VelCorrectedHermite_Create(prev, next, pos, vel, interval);

			// Velocity at the midpoint should match the input velocity
			auto dv = VelCorrectedHermite_EvalDerivative(interp, 0.0f);
			PR_EXPECT(FEql(dv, vel));
		}
	};
}
