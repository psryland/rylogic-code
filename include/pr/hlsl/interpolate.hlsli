//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_INTERPOLATORS_HLSLI
#define PR_HLSL_INTERPOLATORS_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/hermite_spline.hlsli"

#ifdef __cplusplus
namespace pr::hlsl {
#endif

// ------------------------------------------------------------------------------------------------

// Vector interpolator using Hermite splines
struct HermiteVector
{
	HermiteSpline m_p;
	float3 m_x1;
	float m_interval;
};
inline HermiteVector HermiteVector_Create(float3 x0, float3 v0, float3 x1, float3 v1, float interval)
{
	HermiteVector interp;
	interp.m_p = HermiteSpline_Create(x0 - x1, v0 * interval, float3(0, 0, 0), v1 * interval);
	interp.m_x1 = x1;
	interp.m_interval = interval;
	return interp;
}
inline float3 HermiteVector_Eval(HermiteVector interp, float t)
{
	return interp.m_x1 + HermiteSpline_Position(interp.m_p, t / interp.m_interval);
}
inline float3 HermiteVector_EvalDerivative(HermiteVector interp, float t)
{
	return HermiteSpline_Velocity(interp.m_p, t / interp.m_interval) / interp.m_interval;
}
inline float3 HermiteVector_EvalDerivative2(HermiteVector interp, float t)
{
	return HermiteSpline_Acceleration(interp.m_p, t / interp.m_interval) / interp.m_interval;
}

// ------------------------------------------------------------------------------------------------

// Rotation interpolator using Hermite splines in SO(3) log space
struct HermiteQuaternion
{
	HermiteSpline m_p;
	float4 m_q1;
	float m_interval;
};
inline HermiteQuaternion HermiteQuaternion_Create(float4 q0, float3 w0, float4 q1, float3 w1, float interval)
{
	float4 q1_inv = quat_conjugate(q1);
	float4 q_delta = quat_mul(q1_inv, q0);

	// Ensure w >= 0 so that quat_log (which uses |w|) round-trips correctly via quat_exp
	q_delta = q_delta.w < 0 ? -q_delta : q_delta;

	HermiteQuaternion interp;
	interp.m_p = HermiteSpline_Create(
		quat_log(q_delta),                                                // u0 (half angle)
		quat_tangent(q_delta, quat_rotate(q1_inv, w0)) * interval,        // u0' = J^{-1}(u) * w0
		float3(0, 0, 0),                                                  // u1
		quat_tangent(quat_identity(), quat_rotate(q1_inv, w1)) * interval // u1' = J^{-1}(0) * w1
	);
	interp.m_q1 = q1;
	interp.m_interval = interval;
	return interp;
}
inline float4 HermiteQuaternion_Eval(HermiteQuaternion interp, float t)
{
	// Evaluate the curve in the log domain and convert to quaternion
	float3 u = HermiteSpline_Position(interp.m_p, t / interp.m_interval);
	return quat_mul(interp.m_q1, quat_exp(u));
}
inline float3 HermiteQuaternion_EvalDerivative(HermiteQuaternion interp, float t)
{
	// To calculate 'W' from log(q) and log(q)':   (x' means derivative of x)
	// Say:
	//   u = log(q), r = |u| = angle / 2
	//   q = [qv, qw] = [(u/r) * sin(r), cos(r)] = [u*f(r), cos(r)]
	//     where f(r) = sin(r) / r
	// Also:
	//   u  == m_p.Eval(t)
	//   u' == m_p.EvalDerivative(t)
	//   r' == dot(u, u') / r  (where r > 0) (i.e. tangent amount in direction of u)
	//
	// Differentiating:
	//   f'(r) = (r*cos(r) - sin(r)) / r^2 (product rule)
	//   q' = [qv', qw'] = [u'*f + u*f'*r', -sin(r)*r']
	// Also:
	//   q' = 0.5 x [w,0] x q  (quaternion derivative)
	//    => [w,0] = 2*(q' x ~q) = 2*(qw*qv' - qw'*qv - cross(qv', qv))
	//
	// For small 'r' can use expansion for sine:
	//   f(r) = sin(r)/r ~= 1 - r^2/6 +...
	//   f'(r) = -r/3 + ...
	// For really small 'r' use:
	//   W ~= 2 * u'  (comes from: if q = [u, 1] => q' ~= [u', 0])
	float3 u = HermiteSpline_Position(interp.m_p, t / interp.m_interval);
	float3 u_dot = HermiteSpline_Velocity(interp.m_p, t / interp.m_interval) / interp.m_interval;

	// Tiny-angle approximation: J(u) ~= I, so w ~= 2*u'
	float r = length(u);
	if (r < TinyAngle)
		return quat_rotate(interp.m_q1, 2.0 * u_dot);

	// Derivative of angle
	float r_dot = dot(u, u_dot) / r;
	float sin_r = sin(r);
	float cos_r = cos(r);

	// f(r) and f'(r)
	float f     = r > SmallAngle ? (sin_r / r) : (1.0f - r * r / 6.0f);
	float f_dot = r > SmallAngle ? (r * cos_r - sin_r) / (r * r) : (-r / 3.0f);

	// q = [u*f, cos(r)]
	float3 qv = u * f;
	float qw = cos_r;

	// q' = [u'*f + u*f'*r', -sin(r)*r']
	float qw_dot = -sin_r * r_dot;
	float3 qv_dot = u_dot * f + u * (f_dot * r_dot);

	// Vector part of (q' * ~q): omega = 2*(qw*qv' - qw'*qv - cross(qv', qv))
	float3 omega = 2.0 * (qw * qv_dot - qw_dot * qv - cross(qv_dot, qv));
	return quat_rotate(interp.m_q1, omega);
}

// ------------------------------------------------------------------------------------------------

// Combined transform interpolator
struct HermiteTransform
{
	HermiteVector pos;
	HermiteQuaternion rot;
};
inline HermiteTransform HermiteTransform_Create(
	float3 x0, float3 v0, float3 x1, float3 v1,
	float4 q0, float3 w0, float4 q1, float3 w1,
	float interval)
{
	HermiteTransform interp;
	interp.pos = HermiteVector_Create(x0, v0, x1, v1, interval);
	interp.rot = HermiteQuaternion_Create(q0, w0, q1, w1, interval);
	return interp;
}
inline Transform HermiteTransform_Eval(HermiteTransform interp, float t)
{
	Transform xform;
	xform.translation = float4(HermiteVector_Eval(interp.pos, t), 1);
	xform.rotation = HermiteQuaternion_Eval(interp.rot, t);
	xform.scale = float4(1, 1, 1, 1);
	return xform;
}

// ------------------------------------------------------------------------------------------------

// Hermite spline through three positions with position-derived tangents.
// The midpoint tangent is the finite-difference velocity: (pos_next - pos_prev) / interval.
struct HermiteVector_MidPoint
{
	HermiteVector pos;
};
inline HermiteVector_MidPoint HermiteVector_MidPoint_Create(float3 pos_prev, float3 pos_mid, float3 pos_next, float interval)
{
	// Hermite from u=0 (pos_prev) to u=1 (pos_next), passing through pos_mid at u=0.5.
	// Tangent at midpoint: finite-difference velocity = (pos_next - pos_prev) / interval
	//
	// Solving the Hermite midpoint constraints P(0.5) = pos_mid and P'(0.5) = vel_mid:
	//   m0 = 4*pos_mid - 5*pos_prev + pos_next - 2*vel_mid*interval
	//   m1 = -pos_prev + 5*pos_next - 4*pos_mid - 2*vel_mid*interval
	//
	// With vel_mid = (pos_next - pos_prev) / interval, substituting:
	//   vel_mid * interval = pos_next - pos_prev
	//   m0 = 4*pos_mid - 3*pos_prev - pos_next
	//   m1 = pos_prev + 3*pos_next - 4*pos_mid
	HermiteVector_MidPoint interp;
	interp.pos.m_p = HermiteSpline_Create(
		pos_prev - pos_next,
		4.0f * pos_mid - 3.0f * pos_prev - pos_next,
		float3(0, 0, 0),
		pos_prev + 3.0f * pos_next - 4.0f * pos_mid
	);
	interp.pos.m_x1 = pos_next;
	interp.pos.m_interval = interval;
	return interp;
}
inline float3 HermiteVector_MidPoint_Eval(HermiteVector_MidPoint interp, float t)
{
	// Evaluate position. 't' is time relative to the midpoint (t=0 at midpoint, t=-T at pos_prev, t=+T at pos_next).
	float T = interp.pos.m_interval * 0.5f;
	float u = (t + T) / interp.pos.m_interval;
	return interp.pos.m_x1 + HermiteSpline_Position(interp.pos.m_p, u);
}
inline float3 HermiteVector_MidPoint_EvalDerivative(HermiteVector_MidPoint interp, float t)
{
	// Evaluate velocity (in world-space units per second).
	float T = interp.pos.m_interval * 0.5f;
	float u = (t + T) / interp.pos.m_interval;
	return HermiteSpline_Velocity(interp.pos.m_p, u) / interp.pos.m_interval;
}
inline float3 HermiteVector_MidPoint_EvalDerivative2(HermiteVector_MidPoint interp, float t)
{
	// Evaluate acceleration (in world-space units per second^2).
	float T = interp.pos.m_interval * 0.5f;
	float u = (t + T) / interp.pos.m_interval;
	return HermiteSpline_Acceleration(interp.pos.m_p, u) / interp.pos.m_interval;
}

// ------------------------------------------------------------------------------------------------

// Hermite interpolation for quaternions through three orientations with orientation-derived angular velocity.
struct HermiteQuaternion_MidPoint
{
	HermiteQuaternion rot;
};
inline HermiteQuaternion_MidPoint HermiteQuaternion_MidPoint_Create(float4 rot_prev, float4 rot_mid, float4 rot_next, float interval)
{
	// Work in the log domain relative to rot_next.
	// Derive the midpoint angular velocity from orientations only:
	//   avl_mid = 2 * log(~rot_prev * rot_next) / interval
	float4 rot_next_inv = quat_conjugate(rot_next);

	float4 dq0 = quat_mul(rot_next_inv, rot_prev);
	dq0 = dq0.w < 0 ? -dq0 : dq0;
	float4 dq_mid = quat_mul(rot_next_inv, rot_mid);
	dq_mid = dq_mid.w < 0 ? -dq_mid : dq_mid;

	float3 u0 = quat_log(dq0);
	float3 u_mid = quat_log(dq_mid);

	// Finite-difference angular velocity at midpoint
	float4 dq_prev_to_next = quat_mul(quat_conjugate(rot_prev), rot_next);
	dq_prev_to_next = dq_prev_to_next.w < 0 ? -dq_prev_to_next : dq_prev_to_next;
	float3 avl_mid_world = 2.0f * quat_log(dq_prev_to_next) / interval;
	float3 t_mid = quat_tangent(dq_mid, quat_rotate(rot_next_inv, avl_mid_world)) * interval;

	// Solve for endpoint tangents
	float3 m0_r = 4.0f * u_mid - 5.0f * u0 - 2.0f * t_mid;
	float3 m1_r = -u0 - 4.0f * u_mid - 2.0f * t_mid;

	HermiteQuaternion_MidPoint interp;
	interp.rot.m_p = HermiteSpline_Create(u0, m0_r, float3(0, 0, 0), m1_r);
	interp.rot.m_q1 = rot_next;
	interp.rot.m_interval = interval;
	return interp;
}
inline float4 HermiteQuaternion_MidPoint_Eval(HermiteQuaternion_MidPoint interp, float t)
{
	// Evaluate rotation at time t (t=0 at midpoint, t=-T at rot_prev, t=+T at rot_next, where T = interval/2)
	float u = (t + 0.5f * interp.rot.m_interval) / interp.rot.m_interval;
	float3 log_u = HermiteSpline_Position(interp.rot.m_p, u);
	return quat_mul(interp.rot.m_q1, quat_exp(log_u));
}
inline float3 HermiteQuaternion_MidPoint_EvalDerivative(HermiteQuaternion_MidPoint interp, float t)
{
	float u = (t + 0.5f * interp.rot.m_interval) / interp.rot.m_interval;
	return quat_rotate(interp.rot.m_q1, 2.0f * (HermiteSpline_Velocity(interp.rot.m_p, u) / interp.rot.m_interval));
}

// ------------------------------------------------------------------------------------------------

// Combined transform interpolator through three transforms with position/orientation-derived tangents.
struct HermiteTransform_MidPoint
{
	HermiteVector_MidPoint pos;
	HermiteQuaternion_MidPoint rot;
};
inline HermiteTransform_MidPoint HermiteTransform_MidPoint_Create(
	float3 pos_prev, float3 pos_mid, float3 pos_next,
	float4 rot_prev, float4 rot_mid, float4 rot_next,
	float interval)
{
	HermiteTransform_MidPoint interp;
	interp.pos = HermiteVector_MidPoint_Create(pos_prev, pos_mid, pos_next, interval);
	interp.rot = HermiteQuaternion_MidPoint_Create(rot_prev, rot_mid, rot_next, interval);
	return interp;
}
inline Transform HermiteTransform_MidPoint_Eval(HermiteTransform_MidPoint interp, float t)
{
	Transform xform;
	xform.translation = float4(HermiteVector_MidPoint_Eval(interp.pos, t), 1);
	xform.rotation = HermiteQuaternion_MidPoint_Eval(interp.rot, t);
	xform.scale = float4(1, 1, 1, 1);
	return xform;
}

#ifdef __cplusplus
}
#endif
#endif
