//*********************************************
// HLSL InterpolateTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/math/utils/interpolate.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/interpolate.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;
	using quat = pr::math::Quat<float>;
	using V4 = pr::math::Vec4<float>;

	PRUnitTestClass(InterpolateTests)
	{
		// q and -q represent the same rotation
		static bool QuatEql(float4 a, float4 b)
		{
			return FEql(a, b) || FEql(a, -b);
		}

		PRUnitTestMethod(HermiteSplineEndpoints, Quick)
		{
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
		PRUnitTestMethod(HermiteSplineTangents, Quick)
		{
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
		PRUnitTestMethod(HermiteVectorCpuParity, Quick)
		{
			// Compare HLSL HermiteVector against CPU HermiteVector at multiple time steps
			auto x0 = float3(1, -2, 3);
			auto v0 = float3(0.5f, 1, -0.3f);
			auto x1 = float3(-1, 4, 0);
			auto v1 = float3(-0.2f, 0, 1.5f);
			float interval = 1.5f;

			auto hlsl_interp = HermiteVector_Create(x0, v0, x1, v1, interval);
			pr::math::HermiteVector<float> cpu_interp(V4(x0, 1), V4(v0, 0), V4(x1, 1), V4(v1, 0), interval);

			for (int i = 0; i != 11; ++i)
			{
				float t = i / 10.0f * interval;
				auto hlsl_pos = HermiteVector_Eval(hlsl_interp, t);
				auto cpu_pos = cpu_interp.Eval(t);
				PR_EXPECT(FEqlAbsolute(hlsl_pos, cpu_pos.xyz, 1e-4f));

				auto hlsl_vel = HermiteVector_EvalDerivative(hlsl_interp, t);
				auto cpu_vel = cpu_interp.EvalDerivative(t);
				PR_EXPECT(FEqlAbsolute(hlsl_vel, cpu_vel.xyz, 1e-4f));
			}
		}
		PRUnitTestMethod(HermiteQuaternionCpuParity, Quick)
		{
			// Compare HLSL HermiteQuaternion against CPU HermiteQuaternion
			auto q0 = normalize(float4(1, 0, 0, 1));
			auto w0 = float3(0, 0, 1);
			auto q1 = normalize(float4(0, 1, 0, 1));
			auto w1 = float3(0, 0, -1);
			float interval = 1.0f;

			auto hlsl_interp = HermiteQuaternion_Create(q0, w0, q1, w1, interval);
			pr::math::HermiteQuaternion<float> cpu_interp(
				quat(q0.x, q0.y, q0.z, q0.w), V4(w0, 0),
				quat(q1.x, q1.y, q1.z, q1.w), V4(w1, 0),
				interval);

			for (int i = 0; i != 11; ++i)
			{
				float t = i / 10.0f * interval;
				auto hlsl_q = HermiteQuaternion_Eval(hlsl_interp, t);
				auto cpu_q = cpu_interp.Eval(t);
				PR_EXPECT(QuatEql(hlsl_q, float4(cpu_q.x, cpu_q.y, cpu_q.z, cpu_q.w)));
			}

			// Check endpoints match angular velocity
			auto hlsl_w0 = HermiteQuaternion_EvalDerivative(hlsl_interp, 0.0f);
			auto hlsl_w1 = HermiteQuaternion_EvalDerivative(hlsl_interp, interval);
			auto cpu_w0 = cpu_interp.EvalDerivative(0.0f);
			auto cpu_w1 = cpu_interp.EvalDerivative(interval);
			PR_EXPECT(FEqlAbsolute(hlsl_w0, cpu_w0.xyz, 1e-3f));
			PR_EXPECT(FEqlAbsolute(hlsl_w1, cpu_w1.xyz, 1e-3f));
		}
		PRUnitTestMethod(HermiteQuaternionUnitLength, Quick)
		{
			// The interpolated quaternion should always be unit length
			auto q0 = normalize(float4(1, 0, 0, 1));
			auto w0 = float3(0, 0, 1);
			auto q1 = normalize(float4(0, 1, 0, 1));
			auto w1 = float3(0, 0, -1);
			float interval = 1.0f;

			auto interp = HermiteQuaternion_Create(q0, w0, q1, w1, interval);

			for (int i = 0; i != 10; ++i)
			{
				float t = i / 9.0f * interval;
				auto q = HermiteQuaternion_Eval(interp, t);
				float len = length(float4(q.x, q.y, q.z, q.w));
				PR_EXPECT(FEqlAbsolute(len, 1.0f, 1e-3f));
			}
		}
		PRUnitTestMethod(HermiteVectorMidPointCpuParity, Quick)
		{
			// Compare HLSL HermiteVector_MidPoint against CPU HermiteVector_MidPoint
			auto prev = float3(0, 0, 0);
			auto pos = float3(1, 0.5f, 0);
			auto vel = float3(1, 0.2f, 0);
			auto next = float3(2, 0, 0);
			float interval = 2.0f;
			float T = interval * 0.5f;

			auto hlsl_interp = HermiteVector_MidPoint_Create(prev, pos, next, interval);
			math::HermiteVector_MidPoint<float> cpu_interp(V4(prev, 1), V4(pos, 1), V4(next, 1), interval);

			// Check endpoints and midpoint
			for (int i = 0; i != 11; ++i)
			{
				float t = -T + i / 10.0f * interval;
				auto hlsl_pos = HermiteVector_MidPoint_Eval(hlsl_interp, t);
				auto cpu_pos = cpu_interp.Eval(t);
				PR_EXPECT(FEqlAbsolute(hlsl_pos, cpu_pos.xyz, 1e-4f));

				auto hlsl_vel = HermiteVector_MidPoint_EvalDerivative(hlsl_interp, t);
				auto cpu_vel = cpu_interp.EvalDerivative(t);
				PR_EXPECT(FEqlAbsolute(hlsl_vel, cpu_vel.xyz, 1e-4f));
			}
		}
		PRUnitTestMethod(HermiteQuaternionMidPointCpuParity, Quick)
		{
			// Compare HLSL HermiteQuaternion_MidPoint against CPU HermiteQuaternion_MidPoint
			auto rot_prev = normalize(float4(0, 0, 0, 1));
			auto rot_mid = normalize(float4(0, 0, 0.3827f, 0.9239f)); // ~45 deg about Z
			auto avl_mid = float3(0, 0, 0.5f);
			auto rot_next = normalize(float4(0, 0, 0.7071f, 0.7071f)); // ~90 deg about Z
			float interval = 2.0f;
			float T = interval * 0.5f;

			auto hlsl_interp = HermiteQuaternion_MidPoint_Create(rot_prev, rot_mid, rot_next, interval);
			math::HermiteQuaternion_MidPoint<float> cpu_interp(
				quat(rot_prev.x, rot_prev.y, rot_prev.z, rot_prev.w),
				quat(rot_mid.x, rot_mid.y, rot_mid.z, rot_mid.w),
				quat(rot_next.x, rot_next.y, rot_next.z, rot_next.w),
				interval);

			for (int i = 0; i != 11; ++i)
			{
				float t = -T + i / 10.0f * interval;
				auto hlsl_q = HermiteQuaternion_MidPoint_Eval(hlsl_interp, t);
				auto cpu_q = cpu_interp.Eval(t);
				PR_EXPECT(QuatEql(hlsl_q, float4(cpu_q.x, cpu_q.y, cpu_q.z, cpu_q.w)));
			}
		}
	};
}
