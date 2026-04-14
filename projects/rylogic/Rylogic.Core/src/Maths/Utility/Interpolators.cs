//***************************************************
// Interpolators
//  Copyright (c) Rylogic Ltd 2008
//***************************************************
// Port of C++ interpolators from include/pr/math/utils/interpolate.h.
// These use Hermite cubic curves in R3 and SO(3) for C1-continuous interpolation.

using System;
using System.Diagnostics;

namespace Rylogic.Maths
{
	/// <summary>Hermite interpolation of position and velocity using a cubic curve in R3.</summary>
	public struct HermiteVector
	{
		public CubicCurve3 m_p;
		public v4 m_x1;
		public float m_interval;

		public HermiteVector(v4 x0, v4 v0, v4 x1, v4 v1, float interval)
		{
			Debug.Assert(interval != 0);
			m_p = CubicCurve3.Hermite(x0 - x1, v0 * interval, v4.Zero, v1 * interval);
			m_x1 = x1;
			m_interval = interval;
		}

		/// <summary>Evaluate position at time t.</summary>
		public v4 Eval(float t)
		{
			return m_x1 + m_p.Eval(t / m_interval);
		}

		/// <summary>Evaluate velocity at time t.</summary>
		public v4 EvalDerivative(float t)
		{
			return m_p.EvalDerivative(t / m_interval) / m_interval;
		}

		/// <summary>Evaluate acceleration at time t.</summary>
		public v4 EvalDerivative2(float t)
		{
			return m_p.EvalDerivative2(t / m_interval) / m_interval;
		}
	}

	/// <summary>C1-continuous Hermite interpolation of quaternion orientation using a cubic curve in SO(3) log space.</summary>
	/// <remarks>
	/// Orientation changes smoothly through key frames, and angular velocity has no step changes
	/// (but does have corners — angular acceleration isn't continuous).
	/// </remarks>
	public struct HermiteQuaternion
	{
		private const float TinyAngle = 1e-8f;
		private const float SmallAngle = 1e-5f;

		public CubicCurve3 m_p;
		public Quat m_q1;
		public float m_interval;

		public HermiteQuaternion(Quat q0, v4 w0, Quat q1, v4 w1, float interval)
		{
			Debug.Assert(interval != 0);

			// Compute relative quaternion, ensuring w >= 0 so that
			// LogMap (which uses |w|) round-trips correctly via ExpMap.
			var dq = ~q1 * q0;
			if (dq.w < 0) dq = -dq;

			m_p = CubicCurve3.Hermite(
				Math_.LogMap(dq),
				Tangent(dq, Math_.Rotate(~q1, w0)) * interval,
				v4.Zero,
				Tangent(Quat.Identity, Math_.Rotate(~q1, w1)) * interval);
			m_q1 = q1;
			m_interval = interval;
		}

		/// <summary>Evaluate orientation at time t.</summary>
		public Quat Eval(float t)
		{
			var u = m_p.Eval(t / m_interval);
			return m_q1 * Math_.ExpMap(u);
		}

		/// <summary>Evaluate angular velocity at time t (world-space).</summary>
		public v4 EvalDerivative(float t)
		{
			// Note: using full angle
			var u = m_p.Eval(t / m_interval);
			var u_dot = m_p.EvalDerivative(t / m_interval) / m_interval;

			// Tiny-angle approximation: J(u) ~= I, so w ~= u`
			var r = u.w0.Length;
			if (r < TinyAngle)
				return Math_.Rotate(m_q1, 2f * u_dot);

			// Derivative of angle
			var r_dot = Math_.Dot(u, u_dot) / r;
			var sin_r = (float)Math.Sin(r);
			var cos_r = (float)Math.Cos(r);

			// Derivative of axis
			var f = r > SmallAngle ? (sin_r / r) : (1f - r * r / 6f);
			var f_dot = r > SmallAngle ? (r * cos_r - sin_r) / (r * r) : (-r / 3f);

			// q
			var qv = u * f;  // vector part
			var qw = cos_r;  // scalar part

			// q`
			var qw_dot = -sin_r * r_dot;
			var qv_dot = u_dot * f + u * (f_dot * r_dot);

			// Vector part of (q` * ~q): vw = qw*qv` - qw`*qv - qv` x qv
			var omega = 2f * (qw * qv_dot - qw_dot * qv - Math_.Cross(qv_dot, qv));
			return Math_.Rotate(m_q1, omega);
		}

		/// <summary>
		/// Returns the tangent of 'q' in SO(3) based on angular velocity 'w'.
		/// Uses the inverse left-Jacobian to map angular velocity to tangent space.
		/// </summary>
		public static v4 Tangent(Quat q, v4 w)
		{
			// The factor of 0.5 is because Exp/Log use the convention that lengths in log space are angle/2.
			var u = 2f * Math_.LogMap(q);
			var r = u.w0.Length;

			// Tiny-angle approximation: J^{-1}(u) ~= I, so tangent ~= w
			if (r < TinyAngle)
				return 0.5f * w;

			var u_x_w = Math_.Cross(u, w);
			var u_x_u_x_w = Math_.Cross(u, u_x_w);
			var sin_r = (float)Math.Sin(r);
			var cos_r = (float)Math.Cos(r);

			// Exact alpha term for J^{-1}: alpha = 1/r² - (1+cos(r)) / (2*r*sin(r))
			var alpha = Math.Abs(sin_r) > SmallAngle
				? (1f / Math_.Sqr(r)) - (1f + cos_r) / (2f * r * sin_r)
				: (1f / 12f);

			// J^{-1} * w = w - 0.5 * u x w + alpha * (u x (u x w))
			var tangent = w - 0.5f * u_x_w + alpha * u_x_u_x_w;
			return 0.5f * tangent;
		}
	}

	/// <summary>Hermite interpolation of a combined position + rotation transform.</summary>
	public struct HermiteTransform
	{
		public HermiteVector pos;
		public HermiteQuaternion rot;

		public HermiteTransform(
			v4 pos0, v4 vel0, Quat rot0, v4 avl0,
			v4 pos1, v4 vel1, Quat rot1, v4 avl1,
			float interval)
		{
			pos = new HermiteVector(pos0, vel0, pos1, vel1, interval);
			rot = new HermiteQuaternion(rot0, avl0, rot1, avl1, interval);
		}

		/// <summary>Evaluate the transform at time t.</summary>
		public Xform Eval(float t)
		{
			return new Xform(pos.Eval(t), rot.Eval(t));
		}
	}
}
