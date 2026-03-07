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
	struct InterpolateVector
	{
		using CurveType = CurveType<S>;
		using CubicCurve3 = CubicCurve3<S>;
		using Vec4 = Vec4<S>;

		CubicCurve3 m_p;
		Vec4 m_x1;
		S m_interval;

		InterpolateVector() noexcept
			:InterpolateVector(Origin<Vec4>(), Zero<Vec4>(), Origin<Vec4>(), Zero<Vec4>(), S(1))
		{}
		InterpolateVector(Vec4 x0, Vec4 v0, Vec4 x1, Vec4 v1, S interval) noexcept
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
	struct InterpolateRotation
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
		
		InterpolateRotation() noexcept
			:InterpolateRotation(Identity<Quat>(), Zero<Vec4>(), Identity<Quat>(), Zero<Vec4>(), S(1))
		{}
		InterpolateRotation(Quat q0, Vec4 w0, Quat q1, Vec4 w1, S interval) noexcept
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

		PRUnitTestMethod(Vector)
		{
			auto tol = 0.001f;

			// "S" curve
			{
				auto x0 = V4(0, 0, 0, 1);
				auto x1 = V4(1, 1, 0, 1);
				auto v0 = V4(0.3f, 0, 0, 0);
				auto v1 = V4(0.3f, 0, 0, 0);
				InterpolateVector<float> interp(x0, v0, x1, v1, 1.0f);
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
				InterpolateVector<float> interp(x0, v0, x0, v0, 1.0f);
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
				InterpolateVector<float> interp(x0, v0, x1, v1, 1.0f);
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
		PRUnitTestMethod(Rotation)
		{
			auto tol = 0.001f;

			// "S" curve
			{
				auto q0 = Q(0, 0, 0, 1);
				auto q1 = Q(V4::ZAxis(), constants<float>::tau_by_4); // 90 about Z
				auto w0 = V4(0, constants<float>::tau_by_4, 0, 0);    // 90 deg/s about Y
				auto w1 = V4(0, 0, 0, 0);
				InterpolateRotation<float> interp(q0, w0, q1, w1, 1.0f);
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
				InterpolateRotation<float> interp(q0, w0, q0, w0, 1.0f);
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
					InterpolateRotation<float> interp(q0, w0, q0, w0, 1.0f);
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
				InterpolateRotation<float> interp(q0, w0, q1, w1, 1.0f);
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
		#if 0
		PRUnitTestMethod(LdrDump)
		{
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

				auto interpolateV = InterpolateVector<float>(samples[i].pos, samples[i].vel, samples[i + 1].pos, samples[i + 1].vel, T);
				auto interpolateQ = InterpolateRotation<float>(samples[i].rot, samples[i].avel, samples[i + 1].rot, samples[i + 1].avel, T);

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
						interpolateV = InterpolateVector<float>(samples[i].pos, samples[i].vel, samples[i + 1].pos, samples[i + 1].vel, T);
						interpolateQ = InterpolateRotation<float>(samples[i].rot, samples[i].avel, samples[i + 1].rot, samples[i + 1].avel, T);
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
		}
		#endif
	};
}
#endif