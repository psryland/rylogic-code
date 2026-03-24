//*********************************************
// HLSL QuaternionTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/quaternions.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(QuaternionTests)
	{
		static bool QuatEql(float4 a, float4 b)
		{
			return FEql(a, b) || FEql(a, -b);
		}
		PRUnitTestMethod(Identity)
		{
			auto q = quat_identity();
			PR_EXPECT(FEql(q, float4(0, 0, 0, 1)));
		}
		PRUnitTestMethod(Conjugate)
		{
			auto q = float4(0.1f, 0.2f, 0.3f, 0.9f);
			auto qc = quat_conjugate(q);
			PR_EXPECT(FEql(qc, float4(-0.1f, -0.2f, -0.3f, 0.9f)));
		}
		PRUnitTestMethod(MulIdentity)
		{
			// q * identity == q
			auto q = normalize(float4(1, 2, 3, 4));
			auto r = quat_mul(q, quat_identity());
			PR_EXPECT(QuatEql(r, q));

			r = quat_mul(quat_identity(), q);
			PR_EXPECT(QuatEql(r, q));
		}
		PRUnitTestMethod(MulInverse)
		{
			// q * conjugate(q) == identity (for unit quaternions)
			auto q = normalize(float4(1, 2, 3, 4));
			auto r = quat_mul(q, quat_conjugate(q));
			PR_EXPECT(QuatEql(r, quat_identity()));
		}
		PRUnitTestMethod(MulAssociativity)
		{
			auto a = normalize(float4(1, 0, 0, 1));
			auto b = normalize(float4(0, 1, 0, 1));
			auto c = normalize(float4(0, 0, 1, 1));

			auto ab_c = quat_mul(quat_mul(a, b), c);
			auto a_bc = quat_mul(a, quat_mul(b, c));
			PR_EXPECT(QuatEql(ab_c, a_bc));
		}
		PRUnitTestMethod(RotateIdentity)
		{
			// Rotating by identity leaves vector unchanged
			auto v = float3(1, 2, 3);
			auto r = quat_rotate(quat_identity(), v);
			PR_EXPECT(FEql(r, v));
		}
		PRUnitTestMethod(Rotate90Z)
		{
			// 90 degree rotation around Z: (1,0,0) -> (0,1,0)
			float half = 3.14159265f / 4.0f; // pi/4
			auto q = float4(0, 0, sin(half), cos(half));
			auto r = quat_rotate(q, float3(1, 0, 0));
			PR_EXPECT(FEql(r, float3(0, 1, 0)));

			// Compare HLSL rotate vs CPU rotate
			auto cpu_q = pr::math::Quat<float>(q.x, q.y, q.z, q.w);
			auto cpu_r = pr::math::Rotate(cpu_q, float3(1, 0, 0));
			PR_EXPECT(FEql(r, cpu_r));
		}
		PRUnitTestMethod(RotateUnrotateRoundtrip)
		{
			auto q = normalize(float4(1, 2, 3, 4));
			auto v = float3(5, -3, 7);
			auto rotated = quat_rotate(q, v);

			// Compare HLSL rotate vs CPU rotate
			auto cpu_q = pr::math::Quat<float>(q.x, q.y, q.z, q.w);
			auto cpu_rotated = pr::math::Rotate(cpu_q, v);
			PR_EXPECT(FEql(rotated, cpu_rotated));

			auto back = quat_unrotate(q, rotated);
			PR_EXPECT(FEql(back, v));
		}
		PRUnitTestMethod(ToFloat3x3Roundtrip)
		{
			// Convert to matrix and back
			auto q = normalize(float4(1, 2, 3, 4));
			auto m = quat_to_float3x3(q);

			// Compare HLSL quat_to_float3x3 vs CPU ToMatrix
			auto cpu_q = pr::math::Quat<float>(q.x, q.y, q.z, q.w);
			auto cpu_m = pr::math::ToMatrix<float3x3>(cpu_q);
			PR_EXPECT(FEql(m, cpu_m));

			auto q2 = quat_from_float3x3(m);

			// Compare HLSL quat_from_float3x3 vs CPU ToQuat
			auto cpu_q2 = pr::math::ToQuat<pr::math::Quat<float>>(m);
			PR_EXPECT(QuatEql(q2, float4(cpu_q2.x, cpu_q2.y, cpu_q2.z, cpu_q2.w)));

			PR_EXPECT(QuatEql(q, q2));
		}
		PRUnitTestMethod(ToFloat3x3RotatesLikeQuat)
		{
			// Matrix rotation should match quaternion rotation
			auto q = normalize(float4(1, -1, 2, 3));
			auto m = quat_to_float3x3(q);

			// Compare HLSL vs CPU matrix conversion
			auto cpu_q = pr::math::Quat<float>(q.x, q.y, q.z, q.w);
			auto cpu_m = pr::math::ToMatrix<float3x3>(cpu_q);
			PR_EXPECT(FEql(m, cpu_m));

			auto v = float3(4, -2, 7);

			auto qr = quat_rotate(q, v);
			auto mr = mul(v, m);
			PR_EXPECT(FEql(qr, mr));

			// Compare HLSL rotate vs CPU rotate
			auto cpu_r = pr::math::Rotate(cpu_q, v);
			PR_EXPECT(FEql(qr, cpu_r));
		}
		PRUnitTestMethod(ToFloat4x4)
		{
			auto q = normalize(float4(1, 2, 3, 4));
			auto pos = float3(10, 20, 30);
			auto m = quat_to_float4x4(q, pos);

			// The translation row should be the position
			PR_EXPECT(FEql(m[3].x, pos.x));
			PR_EXPECT(FEql(m[3].y, pos.y));
			PR_EXPECT(FEql(m[3].z, pos.z));
			PR_EXPECT(FEql(m[3].w, 1.0f));

			// The rotation part should match the 3x3
			auto m3 = quat_to_float3x3(q);
			for (int r = 0; r != 3; ++r)
				for (int c = 0; c != 3; ++c)
					PR_EXPECT(FEql(m[r][c], m3[r][c]));
		}
		PRUnitTestMethod(FromFloat3x3AxisAligned)
		{
			// Identity matrix -> identity quaternion
			auto m = float3x3(
				float3(1, 0, 0),
				float3(0, 1, 0),
				float3(0, 0, 1));
			auto q = quat_from_float3x3(m);
			PR_EXPECT(QuatEql(q, quat_identity()));

			// Compare HLSL vs CPU
			auto cpu_q = pr::math::ToQuat<pr::math::Quat<float>>(m);
			PR_EXPECT(QuatEql(q, float4(cpu_q.x, cpu_q.y, cpu_q.z, cpu_q.w)));

			// 180 degree rotation around X
			auto mx = float3x3(
				float3(1, 0, 0),
				float3(0, -1, 0),
				float3(0, 0, -1));
			auto qx = quat_from_float3x3(mx);
			PR_EXPECT(FEql(abs(qx.x), 1.0f));
			PR_EXPECT(FEql(qx.y, 0.0f));
			PR_EXPECT(FEql(qx.z, 0.0f));
			PR_EXPECT(FEql(qx.w, 0.0f));

			// Compare HLSL vs CPU
			auto cpu_qx = pr::math::ToQuat<pr::math::Quat<float>>(mx);
			PR_EXPECT(QuatEql(qx, float4(cpu_qx.x, cpu_qx.y, cpu_qx.z, cpu_qx.w)));
		}
		PRUnitTestMethod(LogExpRoundtrip)
		{
			auto q = normalize(float4(1, 2, 3, 4));
			auto v = quat_log(q);
			auto q2 = quat_exp(v);
			PR_EXPECT(QuatEql(q, q2));
		}
		PRUnitTestMethod(LogExpIdentity)
		{
			// log(identity) should be zero
			auto v = quat_log(quat_identity());
			PR_EXPECT(FEql(v, float3(0, 0, 0)));

			// exp(zero) should be identity
			auto q = quat_exp(float3(0, 0, 0));
			PR_EXPECT(QuatEql(q, quat_identity()));
		}
		PRUnitTestMethod(RotationVector)
		{
			// Rotation from identity to a known rotation
			float angle = 1.0f; // 1 radian around Z
			auto q = float4(0, 0, sin(angle / 2), cos(angle / 2));
			auto rv = quat_rotation_vector(quat_identity(), q);

			// Should be approximately (0, 0, angle)
			PR_EXPECT(FEql(rv.x, 0.0f));
			PR_EXPECT(FEql(rv.y, 0.0f));
			PR_EXPECT(FEql(rv.z, angle));
		}
		PRUnitTestMethod(RotationVectorSameQuat)
		{
			// Rotation from q to q should be zero
			auto q = normalize(float4(1, 2, 3, 4));
			auto rv = quat_rotation_vector(q, q);
			PR_EXPECT(FEql(rv, float3(0, 0, 0)));
		}
	};
}
