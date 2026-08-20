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

		PRUnitTestMethod(Construction, Quick, float, double)
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

		// Keep the free ToMatrix helper on the proxy access path for quaternions constructed from raw components.
		PRUnitTestMethod(ConstexprToMatrix, Quick, float, double)
		{
			using Quat = Quat<T>;
			using Vec3 = Vec3<T>;
			using Mat3x3 = Mat3x3<T>;

			constexpr auto q = Quat(T(0), T(0), T(1), T(1));
			constexpr auto m = ToMatrix<Mat3x3>(q);
			constexpr auto expected = Mat3x3(
				Vec3(T(0), T(1), T(0)),
				Vec3(T(-1), T(0), T(0)),
				Vec3(T(0), T(0), T(1)));
			static_assert(All(m == expected));
		}

		PRUnitTestMethod(Operators, Quick, float, double)
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
		PRUnitTestMethod(AxisAngle, Quick, float, double)
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
		PRUnitTestMethod(Rotate, Quick, float, double)
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
		PRUnitTestMethod(ToMatrixRoundTrip, Quick, float, double)
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
		PRUnitTestMethod(Slerp, Quick, float, double)
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
		PRUnitTestMethod(LogMapExpMap, Quick, float, double)
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
		PRUnitTestMethod(ScaleRotation, Quick, float, double)
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

		// Regression: Axis(), Angle(), and SinAngle() must canonicalize consistently for w < 0.
		PRUnitTestMethod(MemberAxisSinAngle, Quick, float, double)
		{
			using quat_t = Quat<T>;
			using vec4_t = Vec4<T>;
			auto constexpr tol = T(0.001);

			// 270-degree +Z rotation has w < 0; q270 and -q270 represent the same orientation.
			auto q270 = quat_t(vec4_t::ZAxis(), DegreesToRadians(T(270)));
			PR_EXPECT(q270.w < T(0));

			auto axis = q270.Axis();
			auto angle = q270.Angle();

			// q and -q give the same shortest-arc axis, angle, and sine.
			PR_EXPECT(FEqlAbsolute<vec4_t>(axis, (-q270).Axis(), tol));
			PR_EXPECT(FEqlAbsolute(angle, (-q270).Angle(), tol));
			PR_EXPECT(FEqlAbsolute(q270.SinAngle(), (-q270).SinAngle(), tol));

			// Axis() + Angle() round-trip reconstructs the correct orientation.
			PR_EXPECT(FEqlOrientation(quat_t(axis, angle), q270, tol));

			// SinAngle() equals sin(Angle()).
			PR_EXPECT(FEqlAbsolute(q270.SinAngle(), std::sin(angle), tol));
		}

		// Tests for quaternions in the negative-w hemisphere (w < 0), i.e. rotations > 180 degrees
		// (half-angle > π/2). AxisAngle, Scale, and LogMap must all canonicalize to the positive-w
		// form and return the equivalent shortest-arc result without changing the orientation.
		PRUnitTestMethod(NegativeHemisphere, Quick, float, double)
		{
			using Quat = Quat<T>;
			using Vec4 = Vec4<T>;

			// A 270-degree rotation around Z encodes as w = cos(135°) < 0 — the negative hemisphere.
			// Its shortest-arc equivalent is -90 degrees (same orientation, positive-w quaternion).
			auto z_axis = Vec4::ZAxis();
			auto q270 = Quat(z_axis, DegreesToRadians(T(270)));
			PR_EXPECT(q270.w < T(0));

			// AxisAngle: must reconstruct a shortest-arc orientation whose angle is 90 deg, not 270 deg.
			// The axis is flipped to preserve the correct rotation sense.
			{
				auto [axis, angle] = math::AxisAngle(q270);
				PR_EXPECT(FEqlAbsolute(angle, DegreesToRadians(T(90)), T(0.001)));

				// Recovering the quaternion from the extracted axis and angle must give the same orientation.
				auto q_recovered = Quat(axis, angle);
				PR_EXPECT(math::FEqlOrientation(q_recovered, q270, T(0.001)));
			}

			// q and -q represent the same orientation; AxisAngle must return the same axis and angle for both.
			{
				auto [axis_pos, angle_pos] = math::AxisAngle(q270);
				auto [axis_neg, angle_neg] = math::AxisAngle(-q270);
				PR_EXPECT(FEqlAbsolute(angle_pos, angle_neg, T(0.001)));
				PR_EXPECT(FEqlAbsolute<Vec4>(axis_pos, axis_neg, T(0.001)));
			}

			// Scale(q, f) and Scale(-q, f) must be orientation-equivalent.
			{
				auto q_scaled_pos = math::Scale(q270, T(0.5));
				auto q_scaled_neg = math::Scale(-q270, T(0.5));
				PR_EXPECT(math::FEqlOrientation(q_scaled_pos, q_scaled_neg, T(0.001)));
			}

			// LogMap(q) and LogMap(-q) must produce the same principal tangent vector.
			{
				auto v_pos = LogMap<Vec4>(q270);
				auto v_neg = LogMap<Vec4>(-q270);
				PR_EXPECT(FEqlAbsolute<Vec4>(v_pos, v_neg, T(0.001)));
			}

			// Scale: shortest-arc scaling by 0.5 must bisect the -90-degree arc, giving -45 degrees.
			{
				auto q_half = math::Scale(q270, T(0.5));
				auto q_expected = Quat(z_axis, DegreesToRadians(T(-45)));
				PR_EXPECT(math::FEqlOrientation(q_half, q_expected, T(0.001)));
			}

			// LogMap / ExpMap round-trip: orientation must survive even when w < 0.
			{
				auto v = LogMap<Vec4>(q270);
				auto q_rt = ExpMap<Quat>(v);
				PR_EXPECT(math::FEqlOrientation(q270, q_rt, T(0.001)));
			}

			// Identity round-trip through Scale must remain identity.
			{
				auto q_id = Identity<Quat>();
				PR_EXPECT(math::FEqlOrientation(math::Scale(q_id, T(0.5)), q_id, T(0.001)));
			}

			// Near-identity: tiny positive rotation must round-trip cleanly.
			{
				auto q_tiny = Quat(z_axis, T(0.001));
				auto v = LogMap<Vec4>(q_tiny);
				auto q_rt = ExpMap<Quat>(v);
				PR_EXPECT(math::FEqlOrientation(q_tiny, q_rt, T(0.001)));
			}

			// Near-pi (just below 180 deg): AxisAngle must return the correct angle magnitude.
			{
				auto q_near_pi = Quat(z_axis, DegreesToRadians(T(179)));
				auto [axis, angle] = math::AxisAngle(q_near_pi);
				PR_EXPECT(FEqlAbsolute(angle, DegreesToRadians(T(179)), T(0.01)));
			}

			// Exact pi boundary: construct with w = 0 explicitly to avoid cos(pi/2) rounding in
			// the axis-angle constructor. At 180 degrees the rotation axis is inherently ambiguous
			// — +Z and -Z are both valid and equally short — so q_pi and -q_pi may produce
			// different square roots from Scale(q, 0.5). Both roots must square back to q_pi's
			// orientation, but they need not equal each other.
			{
				// q_pi  is a 180-degree rotation around +Z with w = 0 exactly.
				// q_mpi is its orientation-equivalent antipodal form {x,y,z} negated.
				auto q_pi  = Quat{T(0), T(0),  T(1), T(0)};
				auto q_mpi = Quat{T(0), T(0), T(-1), T(0)};

				// AxisAngle must reconstruct q_pi's orientation for both inputs.
				{
					auto [axis, angle] = math::AxisAngle(q_pi);
					PR_EXPECT(math::FEqlOrientation(Quat(axis, angle), q_pi, T(0.001)));
				}
				{
					auto [axis, angle] = math::AxisAngle(q_mpi);
					PR_EXPECT(math::FEqlOrientation(Quat(axis, angle), q_pi, T(0.001)));
				}

				// ExpMap(LogMap(…)) round-trip must be orientation-equivalent to q_pi for both.
				PR_EXPECT(math::FEqlOrientation(ExpMap<Quat>(LogMap<Vec4>(q_pi)),  q_pi, T(0.001)));
				PR_EXPECT(math::FEqlOrientation(ExpMap<Quat>(LogMap<Vec4>(q_mpi)), q_pi, T(0.001)));

				// Scale(q, 0.5) produces one of two equally valid 90-degree roots. The root from
				// q_pi uses axis +Z; the root from q_mpi uses axis -Z. Each must square back to
				// q_pi's orientation via quaternion multiplication.
				auto sqrt_pos = math::Scale(q_pi,  T(0.5));
				auto sqrt_neg = math::Scale(q_mpi, T(0.5));
				PR_EXPECT(math::FEqlOrientation(sqrt_pos * sqrt_pos, q_pi, T(0.001)));
				PR_EXPECT(math::FEqlOrientation(sqrt_neg * sqrt_neg, q_pi, T(0.001)));
			}
		}

		PRUnitTestMethod(RotationAt, Quick, float, double)
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

		PRUnitTestMethod(FEqlOrientation, Quick, float, double)
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
