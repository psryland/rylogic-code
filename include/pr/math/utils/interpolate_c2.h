//*****************************************************************************
// C2 interpolation
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include "pr/math/utils/interpolate_c1.h"

namespace pr::math
{
	// A quintic Hermite vector interpolator constrained by endpoint position, velocity, and acceleration.
	template <ScalarTypeFP S>
	struct Hermite5Vector
	{
		using Vec4 = Vec4<S>;

		QuinticCurve3<S> m_p;
		S m_interval;

		// Construct a stationary interpolator at the origin over one second.
		Hermite5Vector() noexcept
			: Hermite5Vector(
				Origin<Vec4>(), Zero<Vec4>(), Zero<Vec4>(),
				Origin<Vec4>(), Zero<Vec4>(), Zero<Vec4>(),
				S(1))
		{
		}

		// Construct an interpolator with derivatives expressed in world units per second.
		Hermite5Vector(
			Vec4 x0, Vec4 v0, Vec4 a0,
			Vec4 x1, Vec4 v1, Vec4 a1,
			S interval) noexcept
			: m_p(x0, v0 * interval, a0 * Sqr(interval), x1, v1 * interval, a1 * Sqr(interval))
			, m_interval(interval)
		{
			pr_assert(interval != 0);
		}

		// Evaluate position at time t.
		Vec4 Eval(S t) const noexcept
		{
			return m_p.Eval(t / m_interval);
		}

		// Evaluate velocity at time t.
		Vec4 EvalDerivative(S t) const noexcept
		{
			return m_p.EvalDerivative(t / m_interval) / m_interval;
		}

		// Evaluate acceleration at time t.
		Vec4 EvalDerivative2(S t) const noexcept
		{
			return m_p.EvalDerivative2(t / m_interval) / Sqr(m_interval);
		}
	};

	// A quintic SO(3) interpolator constrained by endpoint orientation, world angular velocity, and world angular acceleration.
	template <ScalarTypeFP S>
	struct Hermite5Quaternion
	{
		using Hermite3Quat = math::Hermite3Quaternion<S>;
		using Quat = Quat<S>;
		using Vec4 = Vec4<S>;

		QuinticCurve3<S> m_p;
		Quat m_q1;
		S m_interval;

		// Construct a stationary identity rotation over one second.
		Hermite5Quaternion() noexcept
			: Hermite5Quaternion(
				Identity<Quat>(), Zero<Vec4>(), Zero<Vec4>(),
				Identity<Quat>(), Zero<Vec4>(), Zero<Vec4>(),
				S(1))
		{
		}

		// Construct an interpolator whose angular derivatives are expressed in world space.
		Hermite5Quaternion(
			Quat q0, Vec4 w0, Vec4 a0,
			Quat q1, Vec4 w1, Vec4 a1,
			S interval) noexcept
			: m_p()
			, m_q1(q1)
			, m_interval(interval)
		{
			pr_assert(interval != 0);

			// Use the endpoint orientation as a fixed frame and canonicalize the relative rotation to the shortest arc.
			auto dq = ~q1 * q0;
			if (vec(dq).w < 0)
				dq = -dq;

			auto q_identity = Identity<Quat>();
			auto u0 = LogMap<Vec4>(dq);
			auto u1 = Zero<Vec4>();

			// Convert world angular velocity into physical-time derivatives of the local logarithmic coordinates.
			auto u0_dot = Hermite3Quat::Tangent(dq, Rotate(~q1, w0));
			auto u1_dot = Hermite3Quat::Tangent(q_identity, Rotate(~q1, w1));

			// The log-coordinate acceleration includes a curvature term even when its second derivative is zero.
			auto u0_ddot = LogAcceleration(dq, u0_dot, Rotate(~q1, a0));
			auto u1_ddot = LogAcceleration(q_identity, u1_dot, Rotate(~q1, a1));

			m_p = QuinticCurve3<S>(
				u0, u0_dot * interval, u0_ddot * Sqr(interval),
				u1, u1_dot * interval, u1_ddot * Sqr(interval));
		}

		// Evaluate orientation at time t.
		Quat Eval(S t) const noexcept
		{
			auto u = m_p.Eval(t / m_interval);
			return m_q1 * ExpMap<Quat>(u);
		}

		// Evaluate world-space angular velocity at time t.
		Vec4 EvalDerivative(S t) const noexcept
		{
			auto u = m_p.Eval(t / m_interval);
			auto u_dot = m_p.EvalDerivative(t / m_interval) / m_interval;
			return Rotate(m_q1, Hermite3Quat::AngularVelocity(u, u_dot));
		}

		// Evaluate world-space angular acceleration at time t.
		Vec4 EvalDerivative2(S t) const noexcept
		{
			auto u = m_p.Eval(t / m_interval);
			auto u_dot = m_p.EvalDerivative(t / m_interval) / m_interval;
			auto u_ddot = m_p.EvalDerivative2(t / m_interval) / Sqr(m_interval);
			return Rotate(m_q1, AngularAcceleration(u, u_dot, u_ddot));
		}

		// Map local angular acceleration to the second derivative of logarithmic coordinates.
		static Vec4 LogAcceleration(Quat q, Vec4 u_dot, Vec4 angular_acceleration) noexcept
		{
			auto u = LogMap<Vec4>(q);
			auto bias = AngularAcceleration(u, u_dot, Zero<Vec4>());
			return Hermite3Quat::Tangent(q, angular_acceleration - bias);
		}

		// Map logarithmic orientation derivatives to angular acceleration in the same frame.
		static Vec4 AngularAcceleration(Vec4 u, Vec4 u_dot, Vec4 u_ddot) noexcept
		{
			// Express sin(r)/r as a function of r² to avoid singular radial derivatives near the identity.
			auto r_sq = LengthSq(u);
			auto r = Sqrt(r_sq);
			auto f = S(0);
			auto f_s = S(0);
			auto f_ss = S(0);
			if (r < S(0.2))
			{
				f = S(1) - r_sq / S(6) + Sqr(r_sq) / S(120);
				f_s = -S(1) / S(6) + r_sq / S(60) - Sqr(r_sq) / S(1680);
				f_ss = S(1) / S(60) - r_sq / S(840) + Sqr(r_sq) / S(30240);
			}
			else
			{
				auto sin_r = Sin(r);
				auto cos_r = Cos(r);
				f = sin_r / r;
				f_s = (r * cos_r - sin_r) / (S(2) * r * r_sq);
				f_ss = (S(3) * (sin_r - r * cos_r) - r_sq * sin_r) / (S(4) * r * r_sq * r_sq);
			}

			// Differentiate the exponential-map quaternion twice without dividing by the rotation angle.
			auto u_dot_dot = Dot(u, u_dot);
			auto u_dot_dot_derivative = Dot(u_dot, u_dot) + Dot(u, u_ddot);
			auto qv = f * u;
			auto qw = Cos(r);
			auto qv_ddot =
				f * u_ddot +
				S(4) * f_s * u_dot_dot * u_dot +
				(S(4) * f_ss * Sqr(u_dot_dot) + S(2) * f_s * u_dot_dot_derivative) * u;
			auto qw_ddot = -(S(2) * f_s * Sqr(u_dot_dot) + f * u_dot_dot_derivative);

			// q'' * inverse(q) yields angular acceleration because the quadratic q' term is purely scalar.
			return S(2) * (qw * qv_ddot - qw_ddot * qv - Cross(qv_ddot, qv));
		}
	};

	// A C2 transform interpolator combining independent quintic translation and SO(3) rotation curves.
	template <ScalarTypeFP S>
	struct Hermite5Transform
	{
		using Xform = Xform<S>;
		using Vec4 = Vec4<S>;
		using Quat = Quat<S>;

		Hermite5Vector<S> pos;
		Hermite5Quaternion<S> rot;

		// Construct a stationary identity transform over one second.
		Hermite5Transform() noexcept
			: pos()
			, rot()
		{
		}

		// Construct a transform interpolator with world-space linear and angular derivatives.
		Hermite5Transform(
			Vec4 pos0, Vec4 vel0, Vec4 acc0, Quat rot0, Vec4 avl0, Vec4 aac0,
			Vec4 pos1, Vec4 vel1, Vec4 acc1, Quat rot1, Vec4 avl1, Vec4 aac1,
			S interval) noexcept
			: pos(pos0, vel0, acc0, pos1, vel1, acc1, interval)
			, rot(rot0, avl0, aac0, rot1, avl1, aac1, interval)
		{
		}

		// Evaluate the transform at time t.
		Xform Eval(S t) const noexcept
		{
			return Xform{ pos.Eval(t), rot.Eval(t) };
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(InterpolatorC2Tests)
	{
		using S = float;
		using V4 = Vec4<S>;
		using Q = Quat<S>;

		inline static constexpr S Tol = S(0.002);
		inline static constexpr bool CreateVisuals = true;

		// Verify the normalised quintic polynomial independently of the physical-time wrapper.
		PRUnitTestMethod(QuinticCurve3Constraints)
		{
			using VD = Vec4<double>;

			auto const value0 = VD(+0.4, -1.3, +2.1, 0);
			auto const derivative0 = VD(-0.7, +0.2, +1.4, 0);
			auto const derivative20 = VD(+1.1, -0.9, +0.3, 0);
			auto const value1 = VD(+2.6, +0.5, -1.7, 0);
			auto const derivative1 = VD(+0.8, -1.2, +0.6, 0);
			auto const derivative21 = VD(-0.4, +1.5, -0.8, 0);
			auto const curve = QuinticCurve3(
				value0, derivative0, derivative20,
				value1, derivative1, derivative21);

			// The curve directly satisfies all six constraints in its normalised parameter.
			PR_EXPECT(FEqlAbsolute(curve.Eval(0.0), value0, 1E-12));
			PR_EXPECT(FEqlAbsolute(curve.EvalDerivative(0.0), derivative0, 1E-12));
			PR_EXPECT(FEqlAbsolute(curve.EvalDerivative2(0.0), derivative20, 1E-12));
			PR_EXPECT(FEqlAbsolute(curve.Eval(1.0), value1, 1E-12));
			PR_EXPECT(FEqlAbsolute(curve.EvalDerivative(1.0), derivative1, 1E-12));
			PR_EXPECT(FEqlAbsolute(curve.EvalDerivative2(1.0), derivative21, 1E-11));

			// Finite differences check both derivative polynomials away from the endpoint constraints.
			auto const t = 0.37;
			auto const h = 1E-5;
			auto const numerical_derivative = (curve.Eval(t + h) - curve.Eval(t - h)) / (2.0 * h);
			auto const numerical_derivative2 = (curve.EvalDerivative(t + h) - curve.EvalDerivative(t - h)) / (2.0 * h);
			PR_EXPECT(FEqlAbsolute(numerical_derivative, curve.EvalDerivative(t), 1E-8));
			PR_EXPECT(FEqlAbsolute(numerical_derivative2, curve.EvalDerivative2(t), 1E-8));
		}

		// Verify all six vector endpoint constraints over a non-unit interval.
		PRUnitTestMethod(Hermite5Vec)
		{
			auto x0 = V4(-2, 1, 3, 1);
			auto v0 = V4(1, -2, 0.5f, 0);
			auto a0 = V4(0.25f, 1, -0.5f, 0);
			auto x1 = V4(5, -3, 2, 1);
			auto v1 = V4(-1, 0.5f, 2, 0);
			auto a1 = V4(1.5f, -0.25f, 0.75f, 0);
			auto interval = S(2.5);
			auto interp = Hermite5Vector<S>(x0, v0, a0, x1, v1, a1, interval);

			// All six endpoint constraints define the quintic uniquely.
			PR_EXPECT(FEqlAbsolute(interp.Eval(0), x0, Tol));
			PR_EXPECT(FEqlAbsolute(interp.Eval(interval), x1, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative(0), v0, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative(interval), v1, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative2(0), a0, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative2(interval), a1, Tol));
		}

		// Verify quaternion endpoint constraints and analytic angular acceleration.
		PRUnitTestMethod(Hermite5Quat)
		{
			auto q0 = Q(Normalise(V4(1, 2, -1, 0)), S(0.8));
			auto w0 = V4(0.6f, -0.4f, 0.3f, 0);
			auto a0 = V4(-0.2f, 0.7f, 0.5f, 0);
			auto q1 = Q(Normalise(V4(-2, 1, 3, 0)), S(1.4));
			auto w1 = V4(-0.3f, 0.8f, -0.5f, 0);
			auto a1 = V4(0.9f, -0.1f, 0.4f, 0);
			auto interval = S(1.7);
			auto interp = Hermite5Quaternion<S>(q0, w0, a0, q1, w1, a1, interval);

			// Quaternion signs are equivalent, while angular derivatives are ordinary world vectors.
			auto r0 = interp.Eval(0);
			auto r1 = interp.Eval(interval);
			auto sign0 = Dot(r0.xyzw, q0.xyzw) < 0 ? S(-1) : S(+1);
			auto sign1 = Dot(r1.xyzw, q1.xyzw) < 0 ? S(-1) : S(+1);
			PR_EXPECT(FEqlAbsolute(r0.xyzw, sign0 * q0.xyzw, Tol));
			PR_EXPECT(FEqlAbsolute(r1.xyzw, sign1 * q1.xyzw, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative(0), w0, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative(interval), w1, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative2(0), a0, Tol));
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative2(interval), a1, Tol));

			// Independently differentiate angular velocity to verify the analytic acceleration away from endpoint constraints.
			auto t = S(0.63) * interval;
			auto epsilon = S(0.001);
			auto numeric_acceleration = (interp.EvalDerivative(t + epsilon) - interp.EvalDerivative(t - epsilon)) / (S(2) * epsilon);
			PR_EXPECT(FEqlAbsolute(interp.EvalDerivative2(t), numeric_acceleration, S(0.005)));

			// Near-identity and near-180-degree relative rotations exercise both conditioned Jacobian branches.
			for (auto angle : {S(0.01), constants<S>::tau_by_2 - S(0.0001)})
			{
				auto boundary_q0 = Q(V4::YAxis(), angle);
				auto boundary_q1 = Identity<Q>();
				auto boundary_w0 = V4(0.4f, -0.2f, 0.3f, 0);
				auto boundary_a0 = V4(-0.1f, 0.5f, 0.2f, 0);
				auto boundary_interp = Hermite5Quaternion<S>(
					boundary_q0, boundary_w0, boundary_a0,
					boundary_q1, w1, a1,
					interval);
				PR_EXPECT(FEqlAbsolute(boundary_interp.EvalDerivative(0), boundary_w0, Tol));
				PR_EXPECT(FEqlAbsolute(boundary_interp.EvalDerivative2(0), boundary_a0, S(0.005)));
			}
		}

		// Recover angular derivatives from orientation samples to check the complete SO(3) mapping independently.
		PRUnitTestMethod(Hermite5QuatOrientationDerivatives)
		{
			using VD = Vec4<double>;
			using QD = Quat<double>;

			auto const h = 2E-4;

			// A centred relative rotation gives world angular velocity at the sample time.
			auto numerical_velocity = [h](auto const& interp, double t)
			{
				auto const rotation = interp.Eval(t + h) * ~interp.Eval(t - h);
				return LogMap<VD>(rotation) / h;
			};

			// Adjacent centred orientation increments give world angular acceleration without using either derivative evaluator.
			auto numerical_acceleration = [h](auto const& interp, double t)
			{
				auto const rotation0 = interp.Eval(t) * ~interp.Eval(t - 2.0 * h);
				auto const rotation1 = interp.Eval(t + 2.0 * h) * ~interp.Eval(t);
				auto const velocity0 = LogMap<VD>(rotation0) / h;
				auto const velocity1 = LogMap<VD>(rotation1) / h;
				return (velocity1 - velocity0) / (2.0 * h);
			};

			// General non-collinear endpoint derivatives exercise all Jacobian terms.
			{
				auto const q0 = QD(Normalise(VD(+0.3, -0.7, +0.2, 0)), 0.9);
				auto const q1 = QD(Normalise(VD(-0.4, +0.1, +0.8, 0)), 1.4);
				auto const w0 = VD(+0.7, -0.4, +0.2, 0);
				auto const w1 = VD(-0.3, +0.6, +0.5, 0);
				auto const a0 = VD(+0.4, +0.2, -0.5, 0);
				auto const a1 = VD(-0.2, +0.7, +0.3, 0);
				auto const interp = Hermite5Quaternion(q0, w0, a0, q1, w1, a1, 1.7);

				for (auto const t : {0.17, 0.63, 1.31, 1.55})
				{
					PR_EXPECT(FEqlAbsolute(numerical_velocity(interp, t), interp.EvalDerivative(t), 2E-6));
					PR_EXPECT(FEqlAbsolute(numerical_acceleration(interp, t), interp.EvalDerivative2(t), 2E-5));
				}
			}

			// Transverse dynamics close to the shortest-arc pi boundary exercise the conditioned inverse Jacobian.
			{
				auto const q1 = QD(Normalise(VD(+0.2, +0.6, -0.5, 0)), 0.8);
				auto const relative = QD(Normalise(VD(-0.7, +0.4, +0.1, 0)), constants<double>::tau_by_2 - 1E-8);
				auto const q0 = q1 * relative;
				auto const w0 = VD(+0.5, -0.8, +0.3, 0);
				auto const w1 = VD(-0.6, +0.2, +0.7, 0);
				auto const a0 = VD(+0.4, +0.5, -0.2, 0);
				auto const a1 = VD(-0.3, +0.1, +0.6, 0);
				auto const interp = Hermite5Quaternion(q0, w0, a0, q1, w1, a1, 1.3);

				for (auto const t : {0.02, 0.41, 0.97, 1.21})
				{
					PR_EXPECT(FEqlAbsolute(numerical_velocity(interp, t), interp.EvalDerivative(t), 3E-6));
					PR_EXPECT(FEqlAbsolute(numerical_acceleration(interp, t), interp.EvalDerivative2(t), 3E-5));
				}
			}
		}

		// Demonstrate that cubic vector joins are C1 while quintic joins are C2.
		PRUnitTestMethod(C1VersusC2Continuity)
		{
			auto x0 = V4(-2, 0, 0, 1);
			auto x1 = V4(0, 1, 0, 1);
			auto x2 = V4(3, 0, 1, 1);
			auto v0 = V4(0.5f, 0, 0, 0);
			auto v1 = V4(1, -0.25f, 0.5f, 0);
			auto v2 = V4(0, 0.5f, 0, 0);
			auto a0 = V4(0, 0.25f, 0, 0);
			auto a1 = V4(-0.5f, 0.75f, 0.25f, 0);
			auto a2 = V4(0.5f, 0, -0.25f, 0);
			auto dt0 = S(1.25);
			auto dt1 = S(2.0);

			// Cubic segments sharing position and velocity remain C1 but generally disagree in acceleration.
			auto cubic0 = Hermite3Vector<S>(x0, v0, x1, v1, dt0);
			auto cubic1 = Hermite3Vector<S>(x1, v1, x2, v2, dt1);
			PR_EXPECT(FEqlAbsolute(cubic0.Eval(dt0), cubic1.Eval(0), Tol));
			PR_EXPECT(FEqlAbsolute(cubic0.EvalDerivative(dt0), cubic1.EvalDerivative(0), Tol));
			PR_EXPECT(!FEqlAbsolute(cubic0.EvalDerivative2(dt0), cubic1.EvalDerivative2(0), Tol));

			// Quintic segments additionally share the prescribed acceleration and are therefore C2.
			auto quintic0 = Hermite5Vector<S>(x0, v0, a0, x1, v1, a1, dt0);
			auto quintic1 = Hermite5Vector<S>(x1, v1, a1, x2, v2, a2, dt1);
			PR_EXPECT(FEqlAbsolute(quintic0.Eval(dt0), quintic1.Eval(0), Tol));
			PR_EXPECT(FEqlAbsolute(quintic0.EvalDerivative(dt0), quintic1.EvalDerivative(0), Tol));
			PR_EXPECT(FEqlAbsolute(quintic0.EvalDerivative2(dt0), quintic1.EvalDerivative2(0), Tol));
		}

		// Demonstrate that cubic rotation joins are C1 while quintic joins are C2.
		PRUnitTestMethod(C1VersusC2RotationContinuity)
		{
			auto q0 = Q(V4::XAxis(), S(-0.4));
			auto q1 = Q(Normalise(V4(1, 2, -1, 0)), S(0.8));
			auto q2 = Q(Normalise(V4(-2, 1, 3, 0)), S(1.3));
			auto w0 = V4(0.2f, -0.3f, 0.1f, 0);
			auto w1 = V4(-0.4f, 0.5f, 0.3f, 0);
			auto w2 = V4(0.1f, 0.2f, -0.5f, 0);
			auto a0 = V4(0.3f, 0.2f, -0.1f, 0);
			auto a1 = V4(-0.2f, 0.6f, 0.4f, 0);
			auto a2 = V4(0.5f, -0.3f, 0.2f, 0);
			auto dt0 = S(1.4);
			auto dt1 = S(1.9);
			auto epsilon = S(0.001);

			// Cubic segments share angular velocity, but their independently implied angular accelerations disagree.
			auto cubic0 = Hermite3Quaternion<S>(q0, w0, q1, w1, dt0);
			auto cubic1 = Hermite3Quaternion<S>(q1, w1, q2, w2, dt1);
			auto cubic_acceleration0 = (cubic0.EvalDerivative(dt0) - cubic0.EvalDerivative(dt0 - epsilon)) / epsilon;
			auto cubic_acceleration1 = (cubic1.EvalDerivative(epsilon) - cubic1.EvalDerivative(0)) / epsilon;
			PR_EXPECT(FEqlAbsolute(cubic0.EvalDerivative(dt0), cubic1.EvalDerivative(0), Tol));
			PR_EXPECT(!FEqlAbsolute(cubic_acceleration0, cubic_acceleration1, S(0.01)));

			// Quintic segments meet at the explicitly shared world angular acceleration.
			auto quintic0 = Hermite5Quaternion<S>(q0, w0, a0, q1, w1, a1, dt0);
			auto quintic1 = Hermite5Quaternion<S>(q1, w1, a1, q2, w2, a2, dt1);
			PR_EXPECT(FEqlAbsolute(quintic0.EvalDerivative(dt0), quintic1.EvalDerivative(0), Tol));
			PR_EXPECT(FEqlAbsolute(quintic0.EvalDerivative2(dt0), quintic1.EvalDerivative2(0), Tol));
		}

		// Verify complete transform continuity across adjacent unequal-duration segments.
		PRUnitTestMethod(Hermite5XformContinuity)
		{
			auto p0 = V4(-2, 0, 0, 1);
			auto v0 = V4(1, 0, 0, 0);
			auto a0 = V4(0, 0.5f, 0, 0);
			auto q0 = Q(V4::XAxis(), S(-0.6));
			auto w0 = V4(0.2f, -0.4f, 0.1f, 0);
			auto aa0 = V4(0.3f, 0.1f, -0.2f, 0);

			auto p1 = V4(0, 2, 1, 1);
			auto v1 = V4(0.5f, -0.25f, 0.75f, 0);
			auto a1 = V4(-0.4f, 0.6f, 0.2f, 0);
			auto q1 = Q(Normalise(V4(1, 2, 1, 0)), S(0.7));
			auto w1 = V4(-0.3f, 0.5f, 0.4f, 0);
			auto aa1 = V4(0.7f, -0.2f, 0.3f, 0);

			auto p2 = V4(3, 1, -1, 1);
			auto v2 = V4(0, 0.5f, -0.25f, 0);
			auto a2 = V4(0.2f, -0.3f, 0.5f, 0);
			auto q2 = Q(Normalise(V4(-1, 1, 2, 0)), S(1.2));
			auto w2 = V4(0.6f, 0.1f, -0.5f, 0);
			auto aa2 = V4(-0.1f, 0.4f, 0.2f, 0);
			auto dt0 = S(1.3);
			auto dt1 = S(2.1);

			auto lhs = Hermite5Transform<S>(
				p0, v0, a0, q0, w0, aa0,
				p1, v1, a1, q1, w1, aa1,
				dt0);
			auto rhs = Hermite5Transform<S>(
				p1, v1, a1, q1, w1, aa1,
				p2, v2, a2, q2, w2, aa2,
				dt1);

			// Both independent transform components meet with equal values and first two physical derivatives.
			auto x_lhs = lhs.Eval(dt0);
			auto x_rhs = rhs.Eval(0);
			PR_EXPECT(FEqlAbsolute(x_lhs.pos, x_rhs.pos, Tol));
			PR_EXPECT(Abs(Dot(x_lhs.rot.xyzw, x_rhs.rot.xyzw)) > S(1) - Tol);
			PR_EXPECT(FEqlAbsolute(lhs.pos.EvalDerivative(dt0), rhs.pos.EvalDerivative(0), Tol));
			PR_EXPECT(FEqlAbsolute(lhs.pos.EvalDerivative2(dt0), rhs.pos.EvalDerivative2(0), Tol));
			PR_EXPECT(FEqlAbsolute(lhs.rot.EvalDerivative(dt0), rhs.rot.EvalDerivative(0), Tol));
			PR_EXPECT(FEqlAbsolute(lhs.rot.EvalDerivative2(dt0), rhs.rot.EvalDerivative2(0), Tol));
		}

		#if PR_UNITTESTS_VISUALISE
		// Generate an LDraw scene showing position, orientation, and derivative continuity across a C2 join.
		PRUnitTestMethod(LdrHermite5Continuity)
		{
			if constexpr (CreateVisuals)
			{
				using namespace pr::ldraw;

				auto p0 = V4(-3, 0, 0, 1);
				auto v0 = V4(1, 0, 0, 0);
				auto a0 = V4(0, 1, 0, 0);
				auto q0 = Q(V4::XAxis(), -0.7f);
				auto w0 = V4(0.4f, 0, 0.2f, 0);
				auto aa0 = V4(0, 0.4f, 0, 0);
				auto p1 = V4(0, 2, 1, 1);
				auto v1 = V4(1, -0.25f, 0.5f, 0);
				auto a1 = V4(-0.5f, 0.75f, 0.25f, 0);
				auto q1 = Q(V4::YAxis(), 0.5f);
				auto w1 = V4(-0.2f, 0.5f, 0.3f, 0);
				auto aa1 = V4(0.5f, -0.2f, 0.4f, 0);
				auto p2 = V4(4, 0, -1, 1);
				auto v2 = V4(0.5f, 0.5f, 0, 0);
				auto a2 = V4(0, -0.5f, 0.5f, 0);
				auto q2 = Q(Normalise(V4(1, 1, 1, 0)), 1.3f);
				auto w2 = V4(0.3f, 0.2f, -0.4f, 0);
				auto aa2 = V4(-0.2f, 0.3f, 0.1f, 0);
				auto dt0 = 2.0f;
				auto dt1 = 2.5f;
				auto curves = std::array{
					Hermite5Transform<S>(p0, v0, a0, q0, w0, aa0, p1, v1, a1, q1, w1, aa1, dt0),
					Hermite5Transform<S>(p1, v1, a1, q1, w1, aa1, p2, v2, a2, q2, w2, aa2, dt1),
				};
				auto intervals = std::array{dt0, dt1};

				Builder builder;
				auto& grp = builder.Group("Hermite5_C2_Continuity");
				auto& track = grp.Line("position", 0xFF00FF00).width(3.0f).strip(p0);
				auto& boxes = grp.Group("orientation");
				auto& velocities = grp.Group("velocity");
				auto& accelerations = grp.Group("acceleration");
				auto& angular_velocities = grp.Group("angular_velocity");
				auto& angular_accelerations = grp.Group("angular_acceleration");
				auto box_size = V4(0.4f, 0.6f, 0.8f, 0);
				auto derivative_scale = 0.25f;

				// Connect each derivative field's tips so discontinuities appear as breaks or corners in the comb profile.
				auto& velocity_tips = velocities.Line("velocity_tips", 0xFFFFFF00).strip(p0 + derivative_scale * v0);
				auto& acceleration_tips = accelerations.Line("acceleration_tips", 0xFFFF00FF).strip(p0 + derivative_scale * a0);
				auto& angular_velocity_tips = angular_velocities.Line("angular_velocity_tips", 0xFF00FFFF).strip(p0 + derivative_scale * w0);
				auto& angular_acceleration_tips = angular_accelerations.Line("angular_acceleration_tips", 0xFFFF8000).strip(p0 + derivative_scale * aa0);

				// Draw both segments with common derivative arrows at their shared keyframe.
				for (int segment = 0; segment != ssize(curves); ++segment)
				{
					auto const& curve = curves[segment];
					auto interval = intervals[segment];
					auto sample_count = 200;
					for (int i = 0; i != sample_count + 1; ++i)
					{
						auto t = interval * i / sample_count;
						auto x = curve.Eval(t);
						track.line_to(x.pos);
						if (i % 5 != 0)
							continue;

						auto velocity = curve.pos.EvalDerivative(t);
						auto acceleration = curve.pos.EvalDerivative2(t);
						auto angular_velocity = curve.rot.EvalDerivative(t);
						auto angular_acceleration = curve.rot.EvalDerivative2(t);
						boxes.Box("frame", 0x600080FF).box(box_size).o2w().quat(x.rot).pos(x.pos);
						velocities.Line("velocity", 0xFFFFFF00).strip(x.pos).line_to(x.pos + derivative_scale * velocity);
						accelerations.Line("acceleration", 0xFFFF00FF).strip(x.pos).line_to(x.pos + derivative_scale * acceleration);
						angular_velocities.Line("angular_velocity", 0xFF00FFFF).strip(x.pos).line_to(x.pos + derivative_scale * angular_velocity);
						angular_accelerations.Line("angular_acceleration", 0xFFFF8000).strip(x.pos).line_to(x.pos + derivative_scale * angular_acceleration);

						// The first point is already present, and the second segment reuses the first segment's join sample.
						if (i == 0)
							continue;

						velocity_tips.line_to(x.pos + derivative_scale * velocity);
						acceleration_tips.line_to(x.pos + derivative_scale * acceleration);
						angular_velocity_tips.line_to(x.pos + derivative_scale * angular_velocity);
						angular_acceleration_tips.line_to(x.pos + derivative_scale * angular_acceleration);
					}
				}

				grp.Sphere("C2_join", 0xFFFF0000).sphere(0.12f).pos(p1);
				builder.Save(temp_dir() / "interpolation_c2.ldr");
			}
		}
		#endif
	};
}
#endif
