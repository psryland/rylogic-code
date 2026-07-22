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
	PRUnitTestClass(Quaternion)
	{
		std::default_random_engine rng = std::default_random_engine(1u);

		PRUnitTestMethod(Construction, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;
			using Vec3 = Vec3<T>;

			// Identity
			auto Q0 = Identity<Quat>();
			PR_EXPECT(Q0.x == T(0) && Q0.y == T(0) && Q0.z == T(0) && Q0.w == T(1));

			// From axis-angle (Vec4)
			auto axis4 = Vec4::Normal(T(0), T(0), T(1), T(0));
			auto Q1 = Quat(axis4, DegreesToRadians(T(90)));
			PR_EXPECT(FEql(LengthSq(Q1.xyzw), T(1)));

			// From axis-angle (Vec3)
			auto axis3 = Vec3::Normal(T(0), T(0), T(1));
			auto Q2 = Quat(axis3, DegreesToRadians(T(90)));
			PR_EXPECT(FEql(Q1, Q2));

			// From Euler angles
			auto Q3 = Quat(T(0), T(0), DegreesToRadians(T(90)));
			PR_EXPECT(FEql(LengthSq(Q3.xyzw), T(1)));

			// From two direction vectors
			auto from = Vec4::XAxis();
			auto to = Vec4::YAxis();
			auto Q4 = Quat(from, to);
			PR_EXPECT(FEql(LengthSq(Q4.xyzw), T(1)));

			// Array access
			PR_EXPECT(Q0[3] == T(1));
		}
		PRUnitTestMethod(Operators, float, double)
		{
			using Quat = Quat<T>;

			auto Q0 = Quat(T(1), T(2), T(3), T(4));

			// Unary plus
			auto pos = +Q0;
			PR_EXPECT(pos.x == T(1) && pos.y == T(2) && pos.z == T(3) && pos.w == T(4));

			// Unary negate (NOT conjugate)
			auto neg = -Q0;
			PR_EXPECT(neg.x == T(-1) && neg.y == T(-2) && neg.z == T(-3) && neg.w == T(-4));

			// Conjugate (~)
			auto conj = ~Q0;
			PR_EXPECT(conj.x == T(-1) && conj.y == T(-2) && conj.z == T(-3) && conj.w == T(4));

			// Scalar multiply
			auto scaled = Q0 * T(2);
			PR_EXPECT(scaled.x == T(2) && scaled.y == T(4) && scaled.z == T(6) && scaled.w == T(8));
			PR_EXPECT(T(2) * Q0 == scaled);

			// Scalar divide
			auto halved = Q0 / T(2);
			PR_EXPECT(FEql(halved.x, T(0.5)) && FEql(halved.y, T(1)) && FEql(halved.z, T(1.5)) && FEql(halved.w, T(2)));

			// Quaternion multiply: identity * q = q
			auto I = Identity<Quat>();
			auto Q1 = I * Q0;
			PR_EXPECT(FEql(Q1, Q0));

			// q * conj(q) should produce a real quaternion (xyz ≈ 0)
			auto Q2 = Normalise(Q0);
			auto Q3 = Q2 * ~Q2;
			PR_EXPECT(FEqlAbsolute(Q3.x, T(0), T(0.0001)));
			PR_EXPECT(FEqlAbsolute(Q3.y, T(0), T(0.0001)));
			PR_EXPECT(FEqlAbsolute(Q3.z, T(0), T(0.0001)));
			PR_EXPECT(FEqlAbsolute(Q3.w, T(1), T(0.0001)));
		}
		PRUnitTestMethod(AxisAngle, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;

			auto axis = Vec4::Normal(T(1), T(1), T(1), T(0));
			auto angle = DegreesToRadians(T(60));
			auto Q = Quat(axis, angle);

			// Extract axis and angle
			auto [aa_axis, aa_angle] = math::AxisAngle(Q);
			PR_EXPECT(FEqlAbsolute(aa_angle, angle, T(0.001)));
			PR_EXPECT(FEql(aa_axis, axis));

			// Member functions
			PR_EXPECT(FEqlAbsolute(Q.Angle(), angle, T(0.001)));
		}
		PRUnitTestMethod(Rotate, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;
			using Vec3 = Vec3<T>;
			using Mat3x3 = Mat3x3<T>;

			// Rotating (1,0,0) by 90 deg around Z should give (0,1,0)
			auto Q = Quat(Vec4(T(0), T(0), T(1), T(0)), DegreesToRadians(T(90)));
			auto v3 = Vec3(T(1), T(0), T(0));
			auto r3 = math::Rotate(Q, v3);
			PR_EXPECT(FEql(r3, Vec3(T(0), T(1), T(0))));

			// Same with Vec4 (w preserved)
			auto v4 = Vec4(T(1), T(0), T(0), T(0));
			auto r4 = math::Rotate(Q, v4);
			PR_EXPECT(FEql(r4, Vec4(T(0), T(1), T(0), T(0))));
		}
		PRUnitTestMethod(ToMatrixRoundTrip, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;
			using Mat3x3 = Mat3x3<T>;

			// Create a quaternion from axis-angle
			std::uniform_real_distribution<T> rng_angle(T(-3.14), T(+3.14));
			for (int i = 0; i != 20; ++i)
			{
				auto ang = rng_angle(rng);
				auto axis = Vec4::Normal(T(1), T(2), T(3), T(0));
				auto q0 = Quat(axis, ang);
				auto mat = ToMatrix<Mat3x3>(q0);
				auto q1 = ToQuat<Quat>(mat);

				// Rotate a test vector with both and compare
				auto v = Vec4::Normal(T(-1), T(3), T(2), T(0));
				auto r0 = math::Rotate(q0, v);
				auto r1 = math::Rotate(q1, v);
				auto r2 = (mat * v).w0();
				PR_EXPECT(FEql(r0, r1));
				PR_EXPECT(FEql(r0, r2));
			}
		}
		PRUnitTestMethod(Slerp, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;

			auto axis = Vec4(T(0), T(0), T(1), T(0));

			// 0 degrees and 90 degrees around Z
			auto q0 = Identity<Quat>();
			auto q1 = Quat(axis, DegreesToRadians(T(90)));

			// Slerp at 0 should return q0
			auto s0 = math::Slerp(q0, q1, T(0));
			PR_EXPECT(FEql(s0, q0));

			// Slerp at 1 should return q1
			auto s1 = math::Slerp(q0, q1, T(1));
			PR_EXPECT(FEqlOrientation(s1, q1));

			// Slerp at 0.5 should be 45 degrees
			auto s05 = math::Slerp(q0, q1, T(0.5));
			auto q_half = Quat(axis, DegreesToRadians(T(45)));
			PR_EXPECT(FEqlOrientation(s05, q_half));
		}
		PRUnitTestMethod(LogMapExpMap, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;

			// Round-trip test: ExpMap(LogMap(q)) ≈ q
			for (int i = 0; i != 30; ++i)
			{
				auto axis = Vec4::Normal(T(1 + i), T(2 - i), T(3 + i * 2), T(0));
				auto angle = T(0.1) * i; // stay within [0, pi)
				auto q0 = Quat(axis, angle);

				auto v = LogMap<Vec4>(q0);
				auto q1 = ExpMap<Quat>(v);

				// Compare orientations (q and -q are equivalent)
				PR_EXPECT(FEqlOrientation(q0, q1, T(0.001)));
			}
		}
		PRUnitTestMethod(ScaleRotation, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;

			// Scale a 90-degree rotation by 0.5 to get 45 degrees
			auto axis = Vec4(T(0), T(0), T(1), T(0));
			auto q = Quat(axis, DegreesToRadians(T(90)));
			auto q_half = math::Scale(q, T(0.5));
			auto q_expected = Quat(axis, DegreesToRadians(T(45)));
			PR_EXPECT(FEqlOrientation(q_half, q_expected, T(0.001)));

			// Scale by 0 should be identity
			auto q_zero = math::Scale(q, T(0));
			PR_EXPECT(FEqlOrientation(q_zero, Identity<Quat>(), T(0.001)));
		}

		PRUnitTestMethod(RotationAt, float, double)
		{
			using Quat = Quat<T>;
			using Vec3 = Vec3<T>;
			using Vec4 = Vec4<T>;
			using Mat3x3 = Mat3x3<T>;
			auto constexpr tol = T(0.001);

			// Compare against the exact angular displacement integral for the fixed-axis cases.
			auto expect_parallel = [&](float time, Vec3 avel, Vec3 aacc)
			{
				auto const dt = static_cast<T>(time);
				auto const expected_disp = (avel + T(0.5) * aacc * dt) * dt;
				auto const actual = math::RotationAt(time, Identity<Quat>(), avel, aacc);
				auto const actual_disp = LogMap<Vec3>(actual) * T(2);

				PR_EXPECT(FEqlAbsolute<Vec3>(actual_disp, expected_disp, tol));
			};

			// Zero time should preserve the starting orientation.
			auto const ori = Quat(Vec4::XAxis(), T(0.25));
			auto const unchanged = math::RotationAt(0.0f, ori, Vec3::ZAxis(), Vec3(T(0), T(0), T(2)));
			PR_EXPECT(FEqlOrientation(unchanged, ori, tol));

			// Zero angular acceleration should reduce to constant angular velocity.
			expect_parallel(1.0f, Vec3(T(0), T(0), T(1)), Vec3(T(0)));

			// Parallel angular acceleration should integrate to 0.5*a*t^2 from rest.
			expect_parallel(1.0f, Vec3(T(0)), Vec3(T(0), T(0), T(2)));

			// Non-zero initial angular velocity should add linearly to the quadratic term.
			expect_parallel(1.0f, Vec3(T(0), T(0), T(1)), Vec3(T(0), T(0), T(2)));

			// Negative time should follow the same integral and support exact cancellation cases.
			expect_parallel(-1.0f, Vec3(T(0), T(0), T(1)), Vec3(T(0), T(0), T(2)));

			// Matrix and quaternion paths should agree for the non-parallel SPIRAL(6) integration case.
			auto const time_np = 0.35f;
			auto const avel_np = Vec3(T(0.7), T(-0.2), T(0.4));
			auto const aacc_np = Vec3(T(-0.3), T(0.6), T(0.1));
			auto const rot_q = math::RotationAt(time_np, ori, avel_np, aacc_np);
			auto const rot_m = math::RotationAt(time_np, ToMatrix<Mat3x3>(ori), avel_np, aacc_np);
			PR_EXPECT(FEqlOrientation(rot_q, ToQuat<Quat>(rot_m), tol));
		}

		PRUnitTestMethod(FEqlOrientation, float, double)
		{
			using Quat = Quat<T>;

			// q and -q represent the same orientation
			auto Q0 = Normalise(Quat(T(1), T(2), T(3), T(4)));
			PR_EXPECT(math::FEqlOrientation(Q0, -Q0));

			// Identity quaternion
			PR_EXPECT(math::FEqlOrientation(Identity<Quat>(), Identity<Quat>()));

			// Slightly different quaternions should not be equal
			auto Q1 = Normalise(Quat(T(1), T(2), T(3), T(3)));
			PR_EXPECT(!math::FEqlOrientation(Q0, Q1));
		}
	};
}
#endif
