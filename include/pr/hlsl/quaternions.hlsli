//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_QUATERNIONS_HLSLI
#define PR_HLSL_QUATERNIONS_HLSLI

// Quaternion math utilities for HLSL.
// Quaternions are represented as float4 with layout {x, y, z, w}.
static const float TinyAngle = 1e-8;
static const float SmallAngle = 1e-5;

// Return the identity quaternion
float4 quat_identity()
{
	return float4(0, 0, 0, 1);
}

// Return the conjugate of a quaternion (inverse for unit quaternions)
float4 quat_conjugate(float4 q)
{
	return float4(-q.xyz, q.w);
}

// Hamilton product of two quaternions
float4 quat_mul(float4 a, float4 b)
{
	return float4(
		a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz),
		a.w * b.w - dot(a.xyz, b.xyz)
	);
}

// Rotate a vector by a quaternion: q * v * q'
// Uses the optimized form: v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
float3 quat_rotate(float4 q, float3 v)
{
	float3 t = 2.0 * cross(q.xyz, v);
	return v + q.w * t + cross(q.xyz, t);
}

// Rotate a vector by the conjugate of a quaternion (inverse rotation)
float3 quat_unrotate(float4 q, float3 v)
{
	return quat_rotate(quat_conjugate(q), v);
}

// Exact rotation vector between two unit quaternions.
// Returns the axis-angle vector (axis * angle) representing the rotation from q_from to q_to.
float3 quat_rotation_vector(float4 q_from, float4 q_to)
{
	float4 qd = quat_mul(q_to, quat_conjugate(q_from));

	// Ensure shortest arc
	qd = qd.w < 0 ? -qd : qd;

	float half_angle = atan2(length(qd.xyz), qd.w);
	float len = length(qd.xyz);

	// When len is near zero the rotation is near-identity, use the limit: 2*half_angle/sin(half_angle) -> 2
	float scale = len > 1e-6 ? (2.0 * half_angle / len) : 2.0;
	return scale * qd.xyz;
}

// Convert a unit quaternion to a 3x3 rotation matrix (row-major)
float3x3 quat_to_float3x3(float4 q)
{
	float x2 = 2.0 * q.x * q.x;
	float y2 = 2.0 * q.y * q.y;
	float z2 = 2.0 * q.z * q.z;
	float xy = 2.0 * q.x * q.y;
	float xz = 2.0 * q.x * q.z;
	float yz = 2.0 * q.y * q.z;
	float wx = 2.0 * q.w * q.x;
	float wy = 2.0 * q.w * q.y;
	float wz = 2.0 * q.w * q.z;

	return float3x3(
		1.0 - y2 - z2,       xy + wz,       xz - wy,
		      xy - wz, 1.0 - x2 - z2,       yz + wx,
		      xz + wy,       yz - wx, 1.0 - x2 - y2
	);
}

// Convert a unit quaternion + translation to a 4x4 affine matrix (row-major)
float4x4 quat_to_float4x4(float4 q, float3 pos)
{
	float3x3 r = quat_to_float3x3(q);
	return float4x4(
		r[0].x, r[0].y, r[0].z, 0,
		r[1].x, r[1].y, r[1].z, 0,
		r[2].x, r[2].y, r[2].z, 0,
		pos.x,  pos.y,  pos.z,  1
	);
}

// Convert a rotation matrix (row-major) to a unit quaternion.
// Uses the Shepperd method to avoid singularities.
float4 quat_from_float3x3(float3x3 m)
{
	float4 q;
	float trace = m[0][0] + m[1][1] + m[2][2];
	if (trace >= 0)
	{
		float s = 0.5 * rsqrt(1.0 + trace);
		q.x = (m[1][2] - m[2][1]) * s;
		q.y = (m[2][0] - m[0][2]) * s;
		q.z = (m[0][1] - m[1][0]) * s;
		q.w = 0.25 / s;
	}
	else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
	{
		float s = 0.5 * rsqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
		q.x = 0.25 / s;
		q.y = (m[0][1] + m[1][0]) * s;
		q.z = (m[2][0] + m[0][2]) * s;
		q.w = (m[1][2] - m[2][1]) * s;
	}
	else if (m[1][1] > m[2][2])
	{
		float s = 0.5 * rsqrt(1.0 - m[0][0] + m[1][1] - m[2][2]);
		q.x = (m[0][1] + m[1][0]) * s;
		q.y = 0.25 / s;
		q.z = (m[1][2] + m[2][1]) * s;
		q.w = (m[2][0] - m[0][2]) * s;
	}
	else
	{
		float s = 0.5 * rsqrt(1.0 - m[0][0] - m[1][1] + m[2][2]);
		q.x = (m[2][0] + m[0][2]) * s;
		q.y = (m[1][2] + m[2][1]) * s;
		q.z = 0.25 / s;
		q.w = (m[0][1] - m[1][0]) * s;
	}
	return q;
}

// Log map: quaternion -> float3 (axis * half_angle)
// Convention: log space uses Length = angle/2
float3 quat_log(float4 q)
{
	// Ensure shortest arc
	q = q.w < 0 ? -q : q;

	float cos_half_ang = clamp(q.w, -1.0, 1.0);
	float sin_half_ang = length(q.xyz);
	float ang_by_2 = acos(cos_half_ang);
	return sin_half_ang > TinyAngle
		? q.xyz * (ang_by_2 / sin_half_ang)
		: q.xyz;
}

// Exp map: float3 (axis * half_angle) -> quaternion
float4 quat_exp(float3 v)
{
	float ang_by_2 = length(v);
	float cos_half_ang = cos(ang_by_2);
	float sin_half_ang = sin(ang_by_2);
	float s = ang_by_2 > TinyAngle ? (sin_half_ang / ang_by_2) : 1.0;
	return float4(v * s, cos_half_ang);
}

// Returns the tangent of 'q' in SO(3) based on angular velocity 'w'.
// Uses the inverse left-Jacobian to map angular velocity 'w' to tangent space at 'q'.
// The factor of 0.5 on return is because Exp/Log use the convention that lengths in log space are angle/2.
float3 quat_tangent(float4 q, float3 w)
{
	// 'u' = axis * full_angle (radians)
	// 'r' = |u| = angle
	float3 u = 2.0 * quat_log(q);
	float r = length(u);

	// Tiny-angle approximation: J^{-1}(u) ~= I, so tangent ~= 0.5 * w
	if (r < TinyAngle)
		return 0.5 * w;

	float3 u_x_w = cross(u, w);
	float3 u_x_u_x_w = cross(u, u_x_w);
	float sin_r = sin(r);
	float cos_r = cos(r);

	// Exact alpha term for J^{-1}: alpha = 1/r^2 - (1+cos(r)) / (2*r*sin(r))
	// If sin(r) ~= zero, use series expansion for alpha ~ 1/12 when r -> 0
	float alpha = abs(sin_r) > SmallAngle
		? (1.0 / (r * r)) - (1.0 + cos_r) / (2.0 * r * sin_r)
		: (1.0 / 12.0);

	// J^{-1} * w = w - 0.5 * u x w + alpha * (u x (u x w))
	float3 tangent = w - 0.5 * u_x_w + alpha * u_x_u_x_w;
	return 0.5 * tangent;
}

#endif
