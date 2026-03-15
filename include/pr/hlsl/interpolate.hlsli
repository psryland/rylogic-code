//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_INTERPOLATORS_HLSLI
#define PR_HLSL_INTERPOLATORS_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/hermite_spline.hlsli"

// Vector interpolator using Hermite splines
struct InterpolateVector
{
	HermiteSpline m_p;
	float3 m_x1;
	float m_interval;
};
InterpolateVector InterpolateVector_Create(float3 x0, float3 v0, float3 x1, float3 v1, float interval)
{
	InterpolateVector interp;
	interp.m_p = HermiteSpline_Create(x0 - x1, v0 * interval, float3(0, 0, 0), v1 * interval);
	interp.m_x1 = x1;
	interp.m_interval = interval;
	return interp;
}
float3 InterpolateVector_Eval(InterpolateVector interp, float t)
{
	return interp.m_x1 + HermiteSpline_Position(interp.m_p, t / interp.m_interval);
}
float3 InterpolateVector_EvalDerivative(InterpolateVector interp, float t)
{
	return HermiteSpline_Velocity(interp.m_p, t / interp.m_interval) / interp.m_interval;
}
float3 InterpolateVector_EvalDerivative2(InterpolateVector interp, float t)
{
	return HermiteSpline_Acceleration(interp.m_p, t / interp.m_interval) / interp.m_interval;
}

// Rotation interpolator using Hermite splines in SO(3) log space
struct InterpolateRotation
{
	HermiteSpline m_p;
	float4 m_q1;
	float m_interval;
};
InterpolateRotation InterpolateRotation_Create(float4 q0, float3 w0, float4 q1, float3 w1, float interval)
{
	float4 q1_inv = quat_conjugate(q1);
	float4 q_delta = quat_mul(q1_inv, q0);

	InterpolateRotation interp;
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
float4 InterpolateRotation_Eval(InterpolateRotation interp, float t)
{
	// Evaluate the curve in the log domain and convert to quaternion
	float3 u = HermiteSpline_Position(interp.m_p, t / interp.m_interval);
	return quat_mul(interp.m_q1, quat_exp(u));
}
float3 InterpolateRotation_EvalDerivative(InterpolateRotation interp, float t)
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
	//   W ~= 2 * u'  (comes from: if q = [u, 1] => q' ~= [u', 0]
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
	float f     = r > SmallAngle ? (sin_r / r) : (1.0 - r * r / 6.0);
	float f_dot = r > SmallAngle ? (r * cos_r - sin_r) / (r * r) : (-r / 3.0);

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

// Combined interpolators
struct Interpolators
{
	InterpolateVector pos;
	InterpolateRotation rot;
};
Interpolators Interpolators_Create(
	float3 x0, float3 v0, float3 x1, float3 v1,
	float4 q0, float3 w0, float4 q1, float3 w1,
	float interval)
{
	Interpolators interp;
	interp.pos = InterpolateVector_Create(x0, v0, x1, v1, interval);
	interp.rot = InterpolateRotation_Create(q0, w0, q1, w1, interval);
	return interp;
}
Transform Interpolators_Eval(Interpolators interp, float t)
{
	Transform xform;
	xform.translation = float4(InterpolateVector_Eval(interp.pos, t),1);
	xform.rotation = InterpolateRotation_Eval(interp.rot, t);
	xform.scale = float4(1,1,1,1);
	return xform;
}

// ------------------------------------------------------------------------------------------------

// Velocity-corrected Hermite spline interpolator.
struct VelCorrectedHermite
{
	// Constructs a cubic Hermite spline from actual positions at t±T and (pos, vel) at the midpoint.
	// The endpoint tangents are derived so the cubic exactly passes through (pos, vel) at u=0.5.
	HermiteSpline m_p;
	float3 m_x1;
	float m_interval;
};
VelCorrectedHermite VelCorrectedHermite_Create(float3 pos_prev, float3 pos_next, float3 pos, float3 vel, float interval)
{
	// Construct a vel-corrected Hermite from:
	//   pos_prev, pos_next: actual positions at t-T and t+T
	//   pos:    actual position at the PDP time t
	//   vel:    velocity at the PDP time t
	//   interval: total time span from pos_prev to pos_next (= 2*T)

	// Derive endpoint tangents V0, V1 in parameter space that force P(0.5) = pos, P'(0.5)/interval = vel
	float3 V0 = 4.0f * pos - 5.0f * pos_prev + pos_next - 2.0f * vel * interval;
	float3 V1 = -pos_prev + 5.0f * pos_next - 4.0f * pos - 2.0f * vel * interval;

	VelCorrectedHermite interp;
	interp.m_p = HermiteSpline_Create(pos_prev - pos_next, V0, float3(0, 0, 0), V1);
	interp.m_x1 = pos_next;
	interp.m_interval = interval;
	return interp;
}
float3 VelCorrectedHermite_Eval(VelCorrectedHermite interp, float t)
{
	// Evaluate position. 't' is time relative to the midpoint (t=0 at PDP time, t=-T at pos_prev, t=+T at pos_next).
	float T = interp.m_interval * 0.5f;
	float u = (t + T) / interp.m_interval;
	return interp.m_x1 + HermiteSpline_Position(interp.m_p, u);
}
float3 VelCorrectedHermite_EvalDerivative(VelCorrectedHermite interp, float t)
{
	// Evaluate velocity (in world-space units per second).
	float T = interp.m_interval * 0.5f;
	float u = (t + T) / interp.m_interval;
	return HermiteSpline_Velocity(interp.m_p, u) / interp.m_interval;
}
float3 VelCorrectedHermite_EvalDerivative2(VelCorrectedHermite interp, float t)
{
	// Evaluate acceleration (in world-space units per second^2).
	float T = interp.m_interval * 0.5f;
	float u = (t + T) / interp.m_interval;
	return HermiteSpline_Acceleration(interp.m_p, u) / interp.m_interval;
}

#endif
