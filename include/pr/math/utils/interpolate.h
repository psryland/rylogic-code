//***********************************************************************
// Interpolation
//  Copyright (c) Rylogic Ltd 2014
//***********************************************************************
#pragma once
#include "pr/math/types/vector4.h"
#include "pr/math/types/quaternion.h"
#include "pr/math/primitives/spline.h"

namespace pr::math
{
	template <ScalarTypeFP S>
	struct HermiteVector
	{
		using CurveType = CurveType<S>;
		using CubicCurve3 = CubicCurve3<S>;
		using Vec4 = Vec4<S>;

		CubicCurve3 m_p;
		Vec4 m_x1;
		S m_interval;

		HermiteVector() noexcept
			:HermiteVector(Origin<Vec4>(), Zero<Vec4>(), Origin<Vec4>(), Zero<Vec4>(), S(1))
		{}
		HermiteVector(Vec4 x0, Vec4 v0, Vec4 x1, Vec4 v1, S interval) noexcept
			: m_p(x0 - x1, v0 * interval, Zero<Vec4>(), v1 * interval, CurveType::Hermite)
			, m_x1(x1)
			, m_interval(interval)
		{
			pr_assert(interval != 0);
		}
		Vec4 Eval(S t) const noexcept
		{
			return m_x1 + m_p.Eval(t / m_interval);
		}
		Vec4 EvalDerivative(S t) const noexcept
		{
			return m_p.EvalDerivative(t / m_interval) / m_interval;
		}
		Vec4 EvalDerivative2(S t) const noexcept
		{
			return m_p.EvalDerivative2(t / m_interval) / m_interval;
		}
	};

	template <ScalarTypeFP S>
	struct HermiteQuaternion
	{
		// Notes:
		// - This is C1-continuous interpolation using a Hermite cubic in SO(3). I.e, orientation changes smoothly
		//   through key frmes, and angular velocity has no step changes (but does have corners, angular acceleration
		//   isn't continuous)
		// - Important identity:
		//     If:
		//          q(t) = Exp(u(t)),
		//     then the angular velocity w satisfies:
		//          w = J(u) * u`, where J(u) is the left Jacobian, SO(3).
		//     So:
		//          u` = J^{-1}(w)
		using CurveType = CurveType<S>;
		using CubicCurve3 = CubicCurve3<S>;
		using Quat = Quat<S>;
		using Vec4 = Vec4<S>;

		static constexpr S TinyAngle = S(1e-8);
		static constexpr S SmallAngle = S(1e-5);

		CubicCurve3 m_p;
		Quat m_q1;
		S m_interval;
		
		HermiteQuaternion() noexcept
			:HermiteQuaternion(Identity<Quat>(), Zero<Vec4>(), Identity<Quat>(), Zero<Vec4>(), S(1))
		{}
		HermiteQuaternion(Quat q0, Vec4 w0, Quat q1, Vec4 w1, S interval) noexcept
			: m_p()
			, m_q1(q1)
			, m_interval(interval)
		{
			pr_assert(interval != 0);

			// Compute relative quaternion, ensuring w >= 0 so that
			// LogMap (which uses |w|) round-trips correctly via ExpMap.
			auto dq = ~q1 * q0;
			if (vec(dq).w < S(0)) dq = -dq;

			m_p = CubicCurve3(
				LogMap<Vec4>(dq),
				Tangent(dq, Rotate(~q1, w0)) * interval,
				math::Zero<Vec4>(),
				Tangent(math::Identity<Quat>(), Rotate(~q1, w1)) * interval,
				CurveType::Hermite);
		}
		Quat Eval(S t) const noexcept
		{
			// Evaluate the curve in the log domain and convert to quaternion
			auto u = m_p.Eval(t / m_interval);
			return m_q1 * ExpMap<Quat>(u);
		}
		Vec4 EvalDerivative(S t) const noexcept
		{
			// To calculate 'W' from log(q) and log(q)`:   (x` means derivative of x)
			// Say:
			//   u = log(q), r = |u| = angle / 2
			//   q = [qv, qw] = [(u/r) * sin(r), cos(r)] = [u*f(r), cos(r)]
			//     where f(r) = sin(r) / r
			// Also:
			//   u  == m_p.Eval(t)
			//   u` == m_p.EvalDerivative(t)
			//   r` == Dot(u, u`) / r  (where r > 0) (i.e. tangent amount in direction of u)
			//
			// Differentiating:
			//   f`(r) = (r*cos(r) - sin(r)) / r² (product rule)
			//   q` = [qv`, qw`] = [u`*qv + u*qv`*r`, -sin(r)*r`]
			// Also:
			//   q` = 0.5 x [w,0] x q  (quaternion derivative)
			//    => [w,0] = 2*(q` x ~q) = 2*(qw*qv` - qw`*qv - Cross(qv`, qv)) (expanded quaternion multiply)
			//
			// For small 'r' can use expansion for sine:
			//   f(r) = sin(r)/r ~= 1 - r²/6 +...
			//   f`(r) = -r/3 + ...
			// For really small 'r' use:
			//   W ~= 2 * u`  (comes from: if q = [u, 1] => q` ~= [u`, 0]

			// Note: using full angle
			auto u = m_p.Eval(t / m_interval);
			auto u_dot = m_p.EvalDerivative(t / m_interval) / m_interval;

			// Tiny-angle approximation: J(u) = I + 0.5*u + (1/6)u^2 ~= I, so w ~= u`
			auto r = Length(u);
			if (r < TinyAngle)
				return Rotate(m_q1, S(2) * u_dot);

			// Derivative of angle
			auto r_dot = Dot(u, u_dot) / r;
			auto sin_r = std::sin(r);
			auto cos_r = std::cos(r);

			// Derivative of axis
			auto f     = r > SmallAngle ? (sin_r / r) : (S(1) - r * r / S(6));
			auto f_dot = r > SmallAngle ? (r * cos_r - sin_r) / (r * r) : (-r / S(3));

			// q
			auto qv = u * f; // vector part
			auto qw = cos_r; // scalar part

			// q`
			auto qw_dot = -sin_r * r_dot;
			auto qv_dot = u_dot * f + u * (f_dot * r_dot);

			// Vector part of (q` * ~q): vw = qw*qv` - qw`*qv - qv` x qv
			auto omega = S(2) * (qw * qv_dot - qw_dot * qv - Cross(qv_dot, qv));
			return Rotate(m_q1, omega);
		}

		// Returns the tangent of 'q' in SO(3) based on angular velocity 'w'
		static Vec4 Tangent(Quat q, Vec4 w) noexcept
		{
			// Using the inverse left-Jacobian to map angular velocity 'w' to tangent space at 'q'
			// The factor of 0.5 applied on return is because the Exp/Log functions use the
			// convention that lengths in log space are angle/2.

			// 'u' = axis * full_angle (radians)
			// 'r' = |u| = angle / 2
			auto u = S(2) * LogMap<Vec4>(q);
			auto r = Length(u);

			// Tiny-angle approximation: J^{-1}(u) = I - 0.5*u + (1/12)u^2 ~= I, so tangent ~= w
			if (r < TinyAngle)
				return S(0.5) * w;

			// Left Jacobian J(u) multiplied by a vector w:
			//   J(u).w = w + a * (u x w) + b * (u x (u x w))
			// where:
			//   a = (1 - cos r) / r^2
			//   b = (r - sin r) / r^3
			//   r = |u|
			auto u_x_w = Cross(u, w);
			auto u_x_u_x_w = Cross(u, u_x_w);
			auto sin_r = std::sin(r);
			auto cos_r = std::cos(r);

			// Exact alpha term for J^{-1}: alpha = 1/r^2 - (1+cos(r)) / (2*r*sin(r))
			// If sin(r) ~= zero, use series expansion for alpha ~ 1/12 when r -> 0
			auto alpha = Abs(sin_r) > SmallAngle
				? (S(1) / Sqr(r)) - (S(1) + cos_r) / (S(2) * r * sin_r)
				: (S(1) / S(12));

			// J^{-1} * w = w - 0.5 * u x w + alpha * (u x (u x w))
			auto tangent = w - S(0.5) * u_x_w + alpha * u_x_u_x_w;
			return S(0.5) * tangent;
		}
	};

	template <ScalarTypeFP S>
	struct HermiteTransform
	{
		using Xform = Xform<S>;
		using Vec4 = Vec4<S>;
		using Quat = Quat<S>;

		HermiteVector<S> pos;
		HermiteQuaternion<S> rot;

		HermiteTransform() noexcept
			: pos()
			, rot()
		{}
		HermiteTransform(
			Vec4 pos0, Vec4 vel0, Quat rot0, Vec4 angvel0,
			Vec4 pos1, Vec4 vel1, Quat rot1, Vec4 angvel1,
			S interval) noexcept
			: pos(pos0, vel0, pos1, vel1, interval)
			, rot(rot0, angvel0, rot1, angvel1, interval)
		{}

		Xform Eval(S t) const noexcept
		{
			return Xform{ pos.Eval(t), rot.Eval(t) };
		}
	};

	template <ScalarTypeFP S>
	struct HermiteVector_MidPoint
	{
		// Notes:
		//  - Velocity-corrected Hermite spline interpolator.
		//  - Constructs a cubic Hermite spline from actual positions at t±T and (pos, vel) at the midpoint.
		//  - The endpoint tangents are derived so the cubic exactly passes through (pos, vel) at u=0.5.
		using CurveType = CurveType<S>;
		using CubicCurve3 = CubicCurve3<S>;
		using Vec4 = Vec4<S>;

		// Construct a vel-corrected Hermite from:
		//   pos_prev, pos_next: actual positions at t-T and t+T
		//   pos:    actual position at the time t
		//   vel:    velocity at the time t
		//   interval: total time span from 'pos_prev' to 'pos_next' (= 2*T)
		// Derive endpoint tangents V0, V1 in parameter space that force P(0.5) = pos, P'(0.5)/interval = vel
		HermiteVector<S> pos;

		HermiteVector_MidPoint() noexcept
			: pos()
		{}
		HermiteVector_MidPoint(Vec4 pos_prev, Vec4 pos_next, Vec4 pos_mid, Vec4 vel_mid, S interval) noexcept
			: pos()
		{
			// --- Position: velocity-corrected Hermite ---
			// Standard Hermite: P(u) = x1 + H(u; p0, m0, 0, m1) where p0 = pos_prev - pos_next
			// Constraints at u=0.5: P(0.5) = pos_mid and P'(0.5)/interval = vel_mid
			// Solving gives the same tangent formulas as VelCorrectedHermite:
			//   m0 = 4*pos_mid - 5*pos_prev + pos_next - 2*vel_mid*interval
			//   m1 = -pos_prev + 5*pos_next - 4*pos_mid - 2*vel_mid*interval
			pos.m_p = CubicCurve3(
				pos_prev - pos_next,
				S(4) * pos_mid - S(5) * pos_prev + pos_next - S(2) * vel_mid * interval,
				Zero<Vec4>(),
				-pos_prev + S(5) * pos_next - S(4) * pos_mid - S(2) * vel_mid * interval,
				CurveType::Hermite
			);
			pos.m_x1 = pos_next;
			pos.m_interval = interval;
		}

		Vec4 Eval(S t) const noexcept
		{
			// Evaluate position. 't' is time relative to the midpoint (t=0 at PDP time, t=-T at pos_prev, t=+T at pos_next).
			S T = S(0.5) * pos.m_interval;
			S u = (t + T) / pos.m_interval;
			return pos.m_x1 + pos.m_p.Eval(u);
		}
		Vec4 EvalDerivative(S t) const noexcept
		{
			// Evaluate velocity (in world-space units per second).
			S T = S(0.5) * pos.m_interval;
			S u = (t + T) / pos.m_interval;
			return pos.m_p.EvalDerivative(u) / pos.m_interval;
		}
		Vec4 EvalDerivative2(S t) const noexcept
		{
			// Evaluate acceleration (in world-space units per second^2).
			S T = S(0.5) * pos.m_interval;
			S u = (t + T) / pos.m_interval;
			return pos.m_p.EvalDerivative2(u) / pos.m_interval;
		}
	};

	template <ScalarTypeFP S>
	struct HermiteQuaternion_MidPoint
	{
		// Notes:
		//  - Velocity-corrected Hermite spline interpolator.
		//  - Constructs a cubic Hermite spline from actual orientations at t±T and (rot, angvel) at the midpoint.
		//  - The endpoint tangents are derived so the cubic exactly passes through (rot, angvel) at u=0.5.
		using CurveType = CurveType<S>;
		using CubicCurve3 = CubicCurve3<S>;
		using Quat = Quat<S>;
		using Vec4 = Vec4<S>;

		HermiteQuaternion<S> rot;

		HermiteQuaternion_MidPoint() noexcept
			: rot()
		{}
		HermiteQuaternion_MidPoint(Quat rot_prev, Quat rot_mid,  Vec4 angvel_mid, Quat rot_next, S interval) noexcept
			: rot()
		{
			// --- Rotation: velocity-corrected in log domain ---
			// HermiteQuaternion works in the log domain relative to rot_next:
			//   u(t) = log(~rot_next * q(t)), so u0 = log(~rot_next * rot_prev), u1 = 0
			// The tangent in log space is: u' = J^{-1}(u) * w_local, where w_local = ~rot_next * w
			//
			// The midpoint constraint is: u(0.5) = log(~rot_next * rot_mid) = u_mid
			// and: u'(0.5) = J^{-1}(u_mid) * (~rot_next * angvel_mid)
			//
			// Using the same Hermite velocity-correction as position (but in log space):
			auto u0 = LogMap<Vec4>(~rot_next * rot_prev);
			auto u_mid = LogMap<Vec4>(~rot_next * rot_mid);
			auto t_mid = Tangent(~rot_next * rot_mid, Rotate(~rot_next, angvel_mid)) * interval;

			// Solve for endpoint tangents m0_r, m1_r such that the Hermite curve in log space
			// passes through u_mid at u=0.5 with derivative t_mid:
			//   m0_r = 4*u_mid - 5*u0 + 0 - 2*t_mid = 4*u_mid - 5*u0 - 2*t_mid
			//   m1_r = -u0 + 5*0 - 4*u_mid - 2*t_mid = -u0 - 4*u_mid - 2*t_mid
			auto m0_r = S(4) * u_mid - S(5) * u0 - S(2) * t_mid;
			auto m1_r = -u0 - S(4) * u_mid - S(2) * t_mid;

			rot.m_p = CubicCurve3(u0, m0_r, Zero<Vec4>(), m1_r, CubicCurve3::Hermite);
			rot.m_q1 = rot_next;
			rot.m_interval = interval;
		}

		// Evaluate rotation at time t (t=0 at midpoint, t=-T at rot_prev, t=+T at rot_next, where T = interval/2)
		Quat Eval(S t) const noexcept
		{
			auto u = (t + S(0.5) * rot.m_interval) / rot.m_interval;
			auto log_u = rot.m_p.Eval(u);
			return rot.m_q1 * ExpMap<Quat>(log_u);
		}
		Vec4 EvalDerivative(S t) const noexcept
		{
			auto u = (t + S(0.5) * rot.m_interval) / rot.m_interval;
			auto log_u = rot.m_p.Eval(u);
			auto log_u_dot = rot.m_p.EvalDerivative(u) / rot.m_interval;
			return Rotate(rot.m_q1, S(2) * (rot.m_p.EvalDerivative(u) / rot.m_interval));
		}
	};

	template <ScalarTypeFP S>
	struct HermiteTransform_MidPoint
	{
		using Xform = Xform<S>;
		using Vec4 = Vec4<S>;
		using Quat = Quat<S>;

		HermiteVector_MidPoint<S> pos;
		HermiteQuaternion_MidPoint<S> rot;

		HermiteTransform_MidPoint() noexcept
			: pos()
			, rot()
		{}
		HermiteTransform_MidPoint(
			Vec4 pos_prev, Vec4 pos_next, Vec4 pos, Vec4 vel,
			Quat rot_prev, Quat rot_mid, Vec4 angvel_mid, Quat rot_next,
			S interval) noexcept
			: pos(pos_prev, pos_next, pos, vel, interval)
			, rot(rot_prev, rot_mid, angvel_mid, rot_next, interval)
		{}

		// Evaluate the transform at time t (t=0 at midpoint, t=-T at prev, t=+T at next, where T = interval/2)
		Xform Eval(S t) const noexcept
		{
			return Xform{ pos.Eval(t), rot.Eval(t) };
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/primitives/bbox.h"
namespace pr::math::tests
{
	PRUnitTestClass(InterpolatorTests)
	{
		using V4 = Vec4<float>;
		using Q = Quat<float>;

		struct Sample
		{
			Q rot;
			V4 pos;
			V4 vel;
			V4 avel;
			float t;
		};
		std::vector<Sample> GenerateTestData() noexcept
		{
			std::vector<Sample> samples = {
				{Q{-0.2060304f, +0.15757678f, +0.51549790f, +0.81669027f}, V4{+5.3832355f, -3.1496096f, +4.6114840f, 1}, {}, {}, 0.00f},
				{Q{-0.2060304f, +0.15757678f, +0.51549790f, +0.81669027f}, V4{+5.3832355f, -3.1496096f, +4.6114840f, 1}, {}, {}, 0.00f},
				{Q{+0.5922902f, +0.71645330f, -0.12527874f, -0.34668744f}, V4{-2.9858220f, -1.5808580f, +3.1302433f, 1}, {}, {}, 0.80f},
				{Q{+0.3082197f, +0.94610740f, +0.09847915f, -0.01353369f}, V4{-3.0188310f, +1.5786452f, -2.0896165f, 1}, {}, {}, 1.25f},
				{Q{-0.0669388f, +0.10722360f, +0.03925635f, -0.99120194f}, V4{+1.7809376f, +5.5453405f, -1.0587717f, 1}, {}, {}, 1.82f},
				{Q{-0.0783455f, -0.10309572f, -0.03717095f, -0.99088424f}, V4{-2.3707523f, +7.7151650f, -3.6624610f, 1}, {}, {}, 2.50f},
				{Q{+0.4898636f, -0.36353540f, -0.53090600f, +0.58822990f}, V4{-8.2285420f, +1.3285245f, -5.5122820f, 1}, {}, {}, 2.98f},
				{Q{-0.8481584f, -0.15545239f, -0.46448958f, +0.20177048f}, V4{+1.6425184f, -3.8140318f, -3.1722434f, 1}, {}, {}, 3.45f},
				{Q{+0.6668535f, -0.00048639f, -0.12051684f, +0.73537874f}, V4{-3.1473906f, -5.4645233f, +0.9023293f, 1}, {}, {}, 3.75f},
				{Q{+0.0807532f, +0.05050637f, -0.02930553f, -0.99502224f}, V4{+6.1364255f, -0.0826566f, +3.2148716f, 1}, {}, {}, 4.50f},
				{Q{+0.1880102f, +0.01393424f, -0.01297668f, +0.98198247f}, V4{-0.0229692f, +0.8555210f, -3.6779673f, 1}, {}, {}, 5.00f},
				{Q{+0.1880102f, +0.01393424f, -0.01297668f, +0.98198247f}, V4{-0.0229692f, +0.8555210f, -3.6779673f, 1}, {}, {}, 5.00f},
			};

			// Calculate dynamics for each sample point, using finite differences
			for (int i = 1; i != ssize(samples) - 1; ++i)
			{
				int p1 = i - 1, c0 = i + 0, n1 = i + 1;

				// Delta time
				auto t_p1 = samples[c0].t - samples[p1].t;
				auto t_n1 = samples[n1].t - samples[c0].t;
				auto t_pn = t_p1 + t_n1;

				auto x_p1 = samples[p1].pos;
				auto x_c0 = samples[c0].pos;
				auto x_n1 = samples[n1].pos;

				samples[i].vel = t_pn > 0 ? (x_n1 - x_p1) / t_pn : V4{};

				auto ori_p1 = samples[p1].rot;
				auto ori_n1 = samples[n1].rot;
				auto ori_p1n1 = ori_n1 * ~ori_p1;  // Change in orientation from p1 to n1
				samples[i].avel = t_pn > 0 ? LogMap<V4>(ori_p1n1) / t_pn : V4{}; // (N1 - P1) / 2dT = angular velocity at C0 (comes from q` = 0.5 w q)
			}

			return samples;
		}

		PRUnitTestMethod(HermiteVec)
		{
			auto tol = 0.001f;

			// "S" curve
			{
				auto x0 = V4(0, 0, 0, 1);
				auto x1 = V4(1, 1, 0, 1);
				auto v0 = V4(0.3f, 0, 0, 0);
				auto v1 = V4(0.3f, 0, 0, 0);
				HermiteVector<float> interp(x0, v0, x1, v1, 1.0f);
				for (float t = 0; t <= 1.0f; t += 0.1f)
				{
					auto pos = interp.Eval(t);
					PR_EXPECT(IsWithin(BoundingBox<float>::Make(x0, x1), pos, 0.0001f));
				}
				auto X0 = interp.Eval(0);
				auto X1 = interp.Eval(1);
				auto V0 = interp.EvalDerivative(0);
				auto V1 = interp.EvalDerivative(1);
				PR_EXPECT(FEqlAbsolute(X0, x0, tol));
				PR_EXPECT(FEqlAbsolute(X1, x1, tol));
				PR_EXPECT(FEqlAbsolute(V0, v0, tol));
				PR_EXPECT(FEqlAbsolute(V1, v1, tol));
			}

			// x0 == x1 special case
			{
				auto x0 = V4(0, 0, 0, 1);
				auto v0 = V4(0.3f, 0, 0, 0);
				HermiteVector<float> interp(x0, v0, x0, v0, 1.0f);
				auto X0 = interp.Eval(0);
				auto V0 = interp.EvalDerivative(0);
				PR_EXPECT(FEqlAbsolute(X0, x0, tol));
				PR_EXPECT(FEqlAbsolute(V0, v0, tol));
			}

			// Random curves
			std::default_random_engine rng(1u);
			for (int i = 0; i < 100; ++i)
			{
				auto x0 = Random<V4>(rng, Origin<V4>(), 10.0f).w1();
				auto x1 = Random<V4>(rng, Origin<V4>(), 10.0f).w1();
				auto v0 = Random<V4>(rng, Origin<V4>(), 3.0f).w0();
				auto v1 = Random<V4>(rng, Origin<V4>(), 3.0f).w0();
				HermiteVector<float> interp(x0, v0, x1, v1, 1.0f);
				auto X0 = interp.Eval(0);
				auto X1 = interp.Eval(1);
				auto V0 = interp.EvalDerivative(0);
				auto V1 = interp.EvalDerivative(1);
				PR_EXPECT(FEqlAbsolute(X0, x0, tol));
				PR_EXPECT(FEqlAbsolute(X1, x1, tol));
				PR_EXPECT(FEqlAbsolute(V0, v0, tol));
				PR_EXPECT(FEqlAbsolute(V1, v1, tol));
			}
		}
		PRUnitTestMethod(HermiteQuat)
		{
			auto tol = 0.001f;

			// "S" curve
			{
				auto q0 = Q(0, 0, 0, 1);
				auto q1 = Q(V4::ZAxis(), constants<float>::tau_by_4); // 90 about Z
				auto w0 = V4(0, constants<float>::tau_by_4, 0, 0);    // 90 deg/s about Y
				auto w1 = V4(0, 0, 0, 0);
				HermiteQuaternion<float> interp(q0, w0, q1, w1, 1.0f);
				auto Q0 = interp.Eval(0);
				auto Q1 = interp.Eval(1);
				auto W0 = interp.EvalDerivative(0);
				auto W1 = interp.EvalDerivative(1);
				PR_EXPECT(FEqlAbsolute(Q0.xyzw, q0.xyzw, tol));
				PR_EXPECT(FEqlAbsolute(Q1.xyzw, q1.xyzw, tol));
				PR_EXPECT(FEqlAbsolute(W0, w0, tol));
				PR_EXPECT(FEqlAbsolute(W1, w1, tol));
			}

			// q0 == q1 special case
			{
				auto q0 = Q(0, 0, 0, 1);
				auto w0 = V4(0, 0, 0.3f, 0);
				HermiteQuaternion<float> interp(q0, w0, q0, w0, 1.0f);
				auto Q0 = interp.Eval(0);
				auto W0 = interp.EvalDerivative(0);
				PR_EXPECT(FEqlAbsolute(Q0.xyzw, q0.xyzw, tol));
				PR_EXPECT(FEqlAbsolute(W0, w0, tol));
			}

			// Test avel outside [-tau, +tau]
			{
				for (float w = 0.0f; w < 10.f; w += 0.5f)
				{
					auto q0 = Q(V4::ZAxis(), constants<float>::tau_by_4);
					auto w0 = V4(0, 0, w, 0);
					HermiteQuaternion<float> interp(q0, w0, q0, w0, 1.0f);
					auto Q0 = interp.Eval(0);
					auto W0 = interp.EvalDerivative(0);
					PR_EXPECT(FEqlAbsolute(Q0.xyzw, q0.xyzw, tol));
					PR_EXPECT(FEqlAbsolute(W0, w0, tol));
				}
			}

			// Random curves
			std::default_random_engine rng(1u);
			for (int i = 0; i < 100; ++i)
			{
				auto axis0 = RandomN<Vec3<float>>(rng);
				auto axis1 = RandomN<Vec3<float>>(rng);
				auto q0 = Q(Vec4<float>(axis0, 0), std::uniform_real_distribution<float>(0, constants<float>::tau)(rng));
				auto q1 = Q(Vec4<float>(axis1, 0), std::uniform_real_distribution<float>(0, constants<float>::tau)(rng));
				auto w0 = Random<V4>(rng, Origin<V4>(), 3.0f).w0();
				auto w1 = Random<V4>(rng, Origin<V4>(), 3.0f).w0();
				HermiteQuaternion<float> interp(q0, w0, q1, w1, 1.0f);
				auto Q0 = interp.Eval(0);
				auto Q1 = interp.Eval(1);
				auto W0 = interp.EvalDerivative(0);
				auto W1 = interp.EvalDerivative(1);

				// Quaternion sign ambiguity: q and -q represent the same rotation
				auto sign0 = Dot(Q0.xyzw, q0.xyzw) < 0 ? -1.0f : 1.0f;
				auto sign1 = Dot(Q1.xyzw, q1.xyzw) < 0 ? -1.0f : 1.0f;
				PR_EXPECT(FEqlAbsolute(Q0.xyzw, q0.xyzw * sign0, tol));
				PR_EXPECT(FEqlAbsolute(Q1.xyzw, q1.xyzw * sign1, tol));
				PR_EXPECT(FEqlAbsolute(W0, w0, tol));
				PR_EXPECT(FEqlAbsolute(W1, w1, tol));
			}
		}
		PRUnitTestMethod(HermiteXform)
		{
			// @Copilot, please at unit tests here
		}
		PRUnitTestMethod(HermiteVec_MidPoint)
		{
			using S = float;
			using vec4_t = Vec4<S>;

			// Simple straight-line case: pos_prev → pos → pos_next equally spaced
			{
				auto pos_prev = vec4_t(S(0), S(0), S(0), S(1));
				auto pos_next = vec4_t(S(2), S(0), S(0), S(1));
				auto pos = vec4_t(S(1), S(0), S(0), S(1));
				auto vel = vec4_t(S(1), S(0), S(0), S(0));
				auto interval = S(2);

				HermiteVector_MidPoint<S> interp(pos_prev, pos_next, pos, vel, interval);

				// Boundary: Eval(-T) = pos_prev, Eval(+T) = pos_next, where T = interval/2
				auto T = interval / S(2);
				PR_EXPECT(FEql(interp.Eval(-T), pos_prev));
				PR_EXPECT(FEql(interp.Eval(+T), pos_next));

				// Midpoint: Eval(0) = pos
				PR_EXPECT(FEql(interp.Eval(S(0)), pos));

				// Midpoint velocity: EvalDerivative(0) = vel
				PR_EXPECT(FEql(interp.EvalDerivative(S(0)), vel));
			}

			// Non-trivial curve with vertical motion
			{
				auto pos_prev = vec4_t(S(0), S(0), S(0), S(1));
				auto pos_next = vec4_t(S(4), S(0), S(0), S(1));
				auto pos = vec4_t(S(2), S(1), S(0), S(1));
				auto vel = vec4_t(S(2), S(0), S(0), S(0));
				auto interval = S(2);

				HermiteVector_MidPoint<S> interp(pos_prev, pos_next, pos, vel, interval);

				// Boundary positions
				auto T = interval / S(2);
				PR_EXPECT(FEql(interp.Eval(-T), pos_prev));
				PR_EXPECT(FEql(interp.Eval(+T), pos_next));

				// Midpoint constraint
				PR_EXPECT(FEql(interp.Eval(S(0)), pos));
				PR_EXPECT(FEql(interp.EvalDerivative(S(0)), vel));
			}

			// Smoothness: verify no discontinuities by checking values at close sample points
			{
				auto pos_prev = vec4_t(S(0), S(1), S(0), S(1));
				auto pos_next = vec4_t(S(3), S(2), S(-1), S(1));
				auto pos = vec4_t(S(1.5), S(2), S(0.5), S(1));
				auto vel = vec4_t(S(1.5), S(0.5), S(-0.5), S(0));
				auto interval = S(2);
				auto T = interval / S(2);
				auto eps = S(0.001);

				HermiteVector_MidPoint<S> interp(pos_prev, pos_next, pos, vel, interval);

				// Check continuity at several sample points
				for (S t = -T + eps; t < T - eps; t += S(0.1))
				{
					auto p0 = interp.Eval(t);
					auto p1 = interp.Eval(t + eps);

					// Consecutive samples should be close together
					auto diff = p1 - p0;
					PR_EXPECT(Length(diff) < S(0.1));
				}
			}
		}
		PRUnitTestMethod(HermiteQuat_MidPoint)
		{
			// @Copilot, please at unit tests here
		}
		PRUnitTestMethod(HermiteXform_MidPoint)
		{
			// @Copilot, please at unit tests here
		}
		PRUnitTestMethod(LdrDump)
		{
			#if PR_UNITTESTS_VISUALISE
			using namespace pr::rdr12::ldraw;
			auto samples = GenerateTestData();

			Builder builder;
			V4 const box_dim = 0.5f * V4{ 1.0f, 1.5f, 2.0f, 0.0f };
			float vel_scale = 0.1f;
			float avel_scale = 0.1f;

			// Input data
			{
				builder.Sphere("start", 0xFFFF0000).radius(0.1f).pos(samples[0].pos);
				auto& track = builder.Line("track0").smooth().strip(samples[0].pos);
				auto& boxes = builder.Group("boxes0");
				auto& grp_vel = builder.Group("vel0");
				auto& grp_avel = builder.Group("avel0");
				for (auto& sample : samples)
				{
					track.line_to(sample.pos);
					grp_vel.Line("vel", 0xFFFFFF00).arrow().strip(sample.pos).line_to(sample.pos + vel_scale * sample.vel);
					grp_avel.Line("avel", 0xFF00FFFF).arrow().strip(sample.pos).line_to(sample.pos + avel_scale * sample.avel);
					boxes.Box("obj", 0x80008000).dim(box_dim).ori(sample.rot).pos(sample.pos);
				}
			}

			// Interpolated data
			{
				int i = 0;
				auto curr = &samples[i + 0];
				auto next = &samples[i + 1];
				auto T = std::max(next->t - curr->t, 0.001f);

				auto interpolateV = HermiteVector<float>(samples[i].pos, samples[i].vel, samples[i + 1].pos, samples[i + 1].vel, T);
				auto interpolateQ = HermiteQuaternion<float>(samples[i].rot, samples[i].avel, samples[i + 1].rot, samples[i + 1].avel, T);

				auto& boxes = builder.Group("boxes1"); int box_index = 0;
				auto& track = builder.Line("track1", 0xFF00FF00).strip(samples[0].pos);
				auto& grp_vel = builder.Group("vel1");
				auto& grp_avel = builder.Group("avel1");
				for (float dt = 0.0f; ; dt += 0.001f)
				{
					if (dt > samples[i + 1].t - samples[i].t)
					{
						if (++i == ssize(samples) - 1)
							break;

						++curr; ++next;
						T = std::max(samples[i + 1].t - samples[i].t, 0.001f);
						interpolateV = HermiteVector<float>(samples[i].pos, samples[i].vel, samples[i + 1].pos, samples[i + 1].vel, T);
						interpolateQ = HermiteQuaternion<float>(samples[i].rot, samples[i].avel, samples[i + 1].rot, samples[i + 1].avel, T);
						dt = 0.0f;
						box_index = 0;
					}

					auto pos = interpolateV.Eval(dt);
					auto vel = interpolateV.EvalDerivative(dt);
					auto qr = interpolateQ.Eval(dt);
					auto avel = interpolateQ.EvalDerivative(dt);

					track.line_to(pos);

					auto N = 20;
					auto frac = N * dt / (samples[i + 1].t - samples[i].t);
					if (frac > box_index + 1)
					{
						grp_vel.Line("vel", 0xFFFFFF00).arrow().strip(pos).line_to(pos + vel_scale * vel);
						grp_avel.Line("avel", 0xFF00FFFF).arrow().strip(pos).line_to(pos + avel_scale * avel);
						boxes.Box("obj", 0x8000FF00).dim(box_dim).ori(qr).pos(pos);
						++box_index;
					}
				}
			}

			builder.Save("E:\\Dump\\LDraw\\interpolation.ldr", ESaveFlags::Pretty);
			#endif
		}
	};
}
#endif