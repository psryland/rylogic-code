//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_CONSTRAINT_COORDINATES_HLSLI
#define PR_PHYSICS_CONSTRAINT_COORDINATES_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Return the world angular covector of a principal rotation-vector component measured along frame A's axis.
// Both arguments are world-space vectors. The coordinate is differentiable for relative rotation angles strictly below pi.
inline float3 RotationVectorGradient(float3 rotation_vector, float3 axis)
{
	// Transpose the inverse left rotation-log differential because row impulses are dual to angular velocities.
	float angle_sq = dot(rotation_vector, rotation_vector);
	float weight;
	if (angle_sq < 0.01f)
	{
		// Avoid cancellation of 1 - (angle/2)*cot(angle/2); omitted terms are below FP32 resolution in this interval.
		weight = 1.0f / 12.0f + angle_sq * (1.0f / 720.0f + angle_sq / 30240.0f);
	}
	else
	{
		// The half-angle expression stays finite up to pi; the principal-coordinate branch itself is discontinuous there.
		float half_angle = 0.5f * sqrt(angle_sq);
		weight = (1.0f - half_angle * cos(half_angle) / sin(half_angle)) / angle_sq;
	}
	float3 cross_axis = cross(rotation_vector, axis);
	return axis + 0.5f * cross_axis + weight * cross(rotation_vector, cross_axis);
}

// Swing-log X/Z coordinates and signed Y twist for a unit child-to-parent joint quaternion.
// Gradients act on parent-joint-frame relative angular velocity. At the swing antipode the split is not regular.
struct SphericalCoordinates
{
	float2 m_swing;
	float m_twist;
	float3 m_swing_x_gradient;
	float3 m_swing_z_gradient;
	float3 m_twist_gradient;
	bool m_regular;
};

// Measure q = swing * twist, using the first nonzero component of (w,y,x,z) to choose the quaternion sign.
// Twist gradients describe the local unwrapped angle, not a derivative across its principal +/-pi branch cut.
inline SphericalCoordinates MeasureSphericalCoordinates(float4 q)
{
	// Canonicalization makes equivalent quaternion signs identical, including exact half-turns.
	bool negate = q.w != 0.0f ? q.w < 0.0f : q.y != 0.0f ? q.y < 0.0f : q.x != 0.0f ? q.x < 0.0f : q.z < 0.0f;
	q = negate ? -q : q;
	float twist_norm_sq = q.y * q.y + q.w * q.w;
	SphericalCoordinates result;
	result.m_swing = float2(0, 0);
	result.m_twist = 0.0f;
	result.m_swing_x_gradient = float3(0, 0, 0);
	result.m_swing_z_gradient = float3(0, 0, 0);
	result.m_twist_gradient = float3(0, 0, 0);
	result.m_regular = twist_norm_sq > 0.0f;
	if (!result.m_regular)
	{
		// At a no-twist half-turn the diagnostic convention is twist zero; no finite split differential is claimed.
		result.m_swing = 3.14159265358979323846f * float2(q.x, q.z);
		return result;
	}

	// Project onto the Y-twist subgroup, leaving the shortest swing carrying Y to the rotated Y direction.
	float twist_norm = sqrt(twist_norm_sq);
	float swing_x = (q.w * q.x + q.y * q.z) / twist_norm;
	float swing_z = (q.w * q.z - q.x * q.y) / twist_norm;
	float sine_half_swing = length(float2(swing_x, swing_z));
	float swing_angle = 2.0f * atan2(sine_half_swing, twist_norm);
	float swing_scale = sine_half_swing > 0.0f ? swing_angle / sine_half_swing : 2.0f;
	result.m_swing = swing_scale * float2(swing_x, swing_z);
	result.m_twist = 2.0f * atan2(q.y, q.w);

	// Relative angular velocity is swing velocity plus rotated Y times twist rate. Remove that twist before differentiating swing log.
	result.m_twist_gradient = float3(-swing_z / twist_norm, 1.0f, swing_x / twist_norm);
	float3 swung_y = quat_rotate(float4(swing_x, 0.0f, swing_z, twist_norm), float3(0, 1, 0));
	float3 swing_vector = float3(result.m_swing.x, 0.0f, result.m_swing.y);
	float3 gradient_x = RotationVectorGradient(swing_vector, float3(1, 0, 0));
	float3 gradient_z = RotationVectorGradient(swing_vector, float3(0, 0, 1));
	result.m_swing_x_gradient = gradient_x - dot(gradient_x, swung_y) * result.m_twist_gradient;
	result.m_swing_z_gradient = gradient_z - dot(gradient_z, swung_y) * result.m_twist_gradient;
	return result;
}

// Return the unit quarter-circle point, keeping the two axis endpoints exact in float arithmetic.
inline float2 SwingQuarterUnit(float angle)
{
	// Exact endpoints avoid manufacturing a cross-axis component from cos(pi/2)'s rounding error.
	if (angle == 0.0f)
		return float2(1, 0);

	if (angle == 1.57079632679489661923f)
		return float2(0, 1);

	return float2(cos(angle), sin(angle));
}

// Retain the nearer boundary candidate; equal distances preserve a deterministic quadrant/endpoint choice.
inline void KeepCloserSwingPoint(float2 query, float2 candidate, inout_(float2) nearest)
{
	// Squared distances suffice for comparing candidates in the bounded swing-angle plane.
	float2 candidate_delta = candidate - query;
	float2 nearest_delta = nearest - query;
	if (dot(candidate_delta, candidate_delta) < dot(nearest_delta, nearest_delta))
		nearest = candidate;
}

// Return the closest point on one quarter ellipse, including degenerate line segments and its two endpoints.
// Swing coordinates and half-axes are in radians, bounded by pi; half-axes are nonnegative.
inline float2 ClosestSwingQuarter(float2 query, float2 radii)
{
	// A zero-width quarter is a segment, not a divided-by-zero ellipse.
	if (radii.x == 0.0f)
		return float2(0, clamp(query.y, 0.0f, radii.y));

	if (radii.y == 0.0f)
		return float2(clamp(query.x, 0.0f, radii.x), 0);

	// Endpoints are candidates even when a stationary point inside the quarter is farther away.
	float2 nearest = float2(radii.x, 0);
	KeepCloserSwingPoint(query, float2(0, radii.y), nearest);
	float a = radii.y * radii.y - radii.x * radii.x;
	float b = radii.x * query.x;
	float c = radii.y * query.y;

	// For t=tan(angle)>0, the distance derivative has the sign of
	// g(t)=a+b*sqrt(1+t*t)-c*sqrt(1+t*t)/t. Its derivative has the sign of b*t^3+c.
	// Thus g has at most one turning point: partition there, rather than assuming the distance derivative is monotone.
	bool split = (b < 0.0f && c > 0.0f) || (b > 0.0f && c < 0.0f);
	float turn = split ? atan2(pow(abs(c), 1.0f / 3.0f), pow(abs(b), 1.0f / 3.0f)) : 1.57079632679489661923f;
	int segment_count = split ? 2 : 1;
	for (int segment = 0; segment != segment_count; ++segment)
	{
		// There is at most one interior root in each interval. A tangent root at the turning point is also a candidate.
		float lower = segment == 0 ? 0.0f : turn;
		float upper = segment == 0 ? turn : 1.57079632679489661923f;
		float2 lower_unit = SwingQuarterUnit(lower);
		float2 upper_unit = SwingQuarterUnit(upper);
		float lower_value = a * lower_unit.x * lower_unit.y + b * lower_unit.y - c * lower_unit.x;
		float upper_value = a * upper_unit.x * upper_unit.y + b * upper_unit.y - c * upper_unit.x;

		// On an axis, the raw derivative vanishes at the endpoint because sin*cos vanishes. Use g's one-sided limit
		// instead: g(0)=a+b when c=0, and g(infinity)=a-c when b=0, so an interior stationary point is not lost.
		if (lower == 0.0f && c == 0.0f)
			lower_value = a + b;

		if (upper == 1.57079632679489661923f && b == 0.0f)
			upper_value = a - c;

		// Keep endpoints regardless of whether the open interval contains another candidate.
		KeepCloserSwingPoint(query, radii * lower_unit, nearest);
		KeepCloserSwingPoint(query, radii * upper_unit, nearest);
		if (!((lower_value < 0.0f && upper_value > 0.0f) || (lower_value > 0.0f && upper_value < 0.0f)))
			continue;

		// Use at most 32 halvings, stopping earlier when the bracket endpoints are adjacent floats.
		for (int iteration = 0; iteration != 32; ++iteration)
		{
			// Only opposite signs are used; multiplying them could erase a sign through underflow.
			float middle = 0.5f * (lower + upper);
			if (middle == lower || middle == upper)
				break;

			float2 unit = SwingQuarterUnit(middle);
			float value = a * unit.x * unit.y + b * unit.y - c * unit.x;
			if (value == 0.0f)
			{
				// A represented zero is an interior stationary candidate.
				lower = middle;
				upper = middle;
				break;
			}
			if ((value < 0.0f) == (lower_value < 0.0f))
			{
				// Preserve the sign-changing bracket's lower endpoint.
				lower = middle;
				lower_value = value;
			}
			else
			{
				// Preserve the sign-changing bracket's upper endpoint.
				upper = middle;
			}
		}
		KeepCloserSwingPoint(query, radii * SwingQuarterUnit(0.5f * (lower + upper)), nearest);
	}
	return nearest;
}

// Nearest boundary and signed Euclidean distance in the swing-vector plane; negative distances are inside.
struct SwingBoundary
{
	float2 m_point;
	float2 m_outward;
	float m_distance;
};

// Project one component into the normal cone of an axis-aligned swing boundary feature.
inline float SwingAxisNormal(float component, float coordinate, float2 range)
{
	// A collapsed interval admits either transverse normal; an endpoint admits only its outward half-line.
	if (range.x == range.y)
		return component;

	if (coordinate == range.x)
		return min(component, 0.0f);

	if (coordinate == range.y)
		return max(component, 0.0f);

	return 0.0f;
}

// Measure the quadrant-wise swing ellipse, using ordered X/Z ranges that contain zero and lie within [-pi,pi].
// At a nonunique nearest point or a corner, return one consistent outward direction rather than claiming a unique differential.
inline SwingBoundary MeasureSwingBoundary(float2 query, float2 x_range, float2 z_range)
{
	// Every quarter contributes its complete arc (or zero-width segment), so an interior point may choose another quadrant.
	SwingBoundary result;
	result.m_point = float2(x_range.y, 0);
	result.m_outward = float2(1, 0);
	for (int quadrant = 0; quadrant != 4; ++quadrant)
	{
		// Mirror into the positive quarter without changing Euclidean distances.
		float2 signs = float2((quadrant & 1) == 0 ? 1.0f : -1.0f, (quadrant & 2) == 0 ? 1.0f : -1.0f);
		float2 radii = float2(signs.x > 0.0f ? x_range.y : -x_range.x, signs.y > 0.0f ? z_range.y : -z_range.x);
		float2 local = ClosestSwingQuarter(signs * query, radii);
		float2 candidate = signs * local;
		float2 delta = query - candidate;
		float2 old_delta = query - result.m_point;
		if (quadrant != 0 && dot(delta, delta) >= dot(old_delta, old_delta))
			continue;

		// A boundary point has a well-defined arc normal except on a collapsed side, where the side normal is used.
		result.m_point = candidate;
		if (radii.x == 0.0f)
			result.m_outward = float2(signs.x, 0);
		else if (radii.y == 0.0f)
			result.m_outward = float2(0, signs.y);
		else if (local.x == 0.0f)
			result.m_outward = float2(0, signs.y);
		else if (local.y == 0.0f)
			result.m_outward = float2(signs.x, 0);
		else
		{
			// Scale the ellipse gradient before normalization; neither a squared tiny radius nor a large inverse radius is needed.
			float2 unit = local / radii;
			float2 normal = radii.x <= radii.y ? float2(unit.x, unit.y * (radii.x / radii.y)) : float2(unit.x * (radii.y / radii.x), unit.y);
			normal /= max(abs(normal.x), abs(normal.y));
			result.m_outward = signs * normalize(normal);
		}
	}

	// Containment uses the point's own quadrant; a zero-width side only contains its zero coordinate.
	float2 radii = float2(query.x < 0.0f ? -x_range.x : x_range.y, query.y < 0.0f ? -z_range.x : z_range.y);
	float2 scaled = float2(radii.x > 0.0f ? query.x / radii.x : 0.0f, radii.y > 0.0f ? query.y / radii.y : 0.0f);
	bool inside = (radii.x > 0.0f || query.x == 0.0f) && (radii.y > 0.0f || query.y == 0.0f) && dot(scaled, scaled) <= 1.0f;
	float distance = length(query - result.m_point);
	result.m_distance = inside ? -distance : distance;
	if (result.m_point.x == 0.0f || result.m_point.y == 0.0f)
	{
		// Axis endpoints and collapsed segments have the bounding rectangle's local normal cone. Project the displacement into
		// that cone; interior arc points instead keep their analytic gradient, even when a tiny rounded displacement points elsewhere.
		float2 displacement = inside ? result.m_point - query : query - result.m_point;
		float2 direction = float2(SwingAxisNormal(displacement.x, result.m_point.x, x_range), SwingAxisNormal(displacement.y, result.m_point.y, z_range));
		float scale = max(abs(direction.x), abs(direction.y));
		if (scale > 0.0f)
			result.m_outward = normalize(direction / scale);
	}

	return result;
}

// Canonical spherical slots: up to two swing boundary faces, twist, and three orthonormal physical dampers.
// Axes are world angular covectors. Identities distinguish a swing face from a curved boundary in the same storage slot.
struct SphericalAngularRows
{
	GpuConstraintAxisDesc m_parameters[6];
	float3 m_axes[6];
	float m_positions[6];
	uint m_identities;
};

// Define one hard angular row, with an unlimited physical impulse and radian coordinate bounds.
inline void SetSphericalAngularRow(inout_(SphericalAngularRows) rows, uint slot, uint identity, float3 axis, float position, float2 limits)
{
	// The common scalar law handles both one-sided faces and a collapsed bilateral interval.
	rows.m_parameters[slot].mode = GpuConstraintAxisMode_Limited;
	rows.m_parameters[slot].lower_limit = limits.x;
	rows.m_parameters[slot].upper_limit = limits.y;
	rows.m_axes[slot] = axis;
	rows.m_positions[slot] = position;
	rows.m_identities |= identity << (4u * slot);
}

// Prepare the spherical primitive for the existing scalar/block solvers. Limited split coordinates require a regular swing/twist chart.
// Fully unrestricted angular motion and isotropic damping do not require that chart, including at its antipode.
inline bool PrepareSphericalAngularRows(float4 frame_a, float4 frame_b, float4 swing_limits, float2 twist_limits, float damping, out_(SphericalAngularRows) rows)
{
	// Every slot is initialized even when a later chart check rejects the complete primitive.
	float infinity = asfloat(0x7f800000u);
	rows.m_identities = 0u;
	for (uint slot = 0u; slot != 6u; ++slot)
	{
		// Free slots contain no residual or Jacobian, rather than stale data from a previous active boundary.
		rows.m_parameters[slot].mode = GpuConstraintAxisMode_Free;
		rows.m_parameters[slot].lower_limit = 0.0f;
		rows.m_parameters[slot].upper_limit = 0.0f;
		rows.m_parameters[slot].target_position = 0.0f;
		rows.m_parameters[slot].target_velocity = 0.0f;
		rows.m_parameters[slot].stiffness = 0.0f;
		rows.m_parameters[slot].damping = 0.0f;
		rows.m_parameters[slot].max_force = infinity;
		rows.m_axes[slot] = float3(0, 0, 0);
		rows.m_positions[slot] = 0.0f;
	}

	// Equal coefficients in an orthonormal parent basis give exactly -c*omega_relative, not coordinate-gradient damping.
	if (damping > 0.0f)
	{
		// These three rows remain separate from all limit covectors and therefore preserve their physical viscous law.
		for (uint axis = 0u; axis != 3u; ++axis)
		{
			// Driven zero-speed rows with gamma=1/(dt*c) are the existing implicit viscous equation.
			float3 basis = float3(axis == 0u ? 1.0f : 0.0f, axis == 1u ? 1.0f : 0.0f, axis == 2u ? 1.0f : 0.0f);
			rows.m_parameters[3u + axis].mode = GpuConstraintAxisMode_Driven;
			rows.m_parameters[3u + axis].damping = damping;
			rows.m_axes[3u + axis] = quat_rotate(frame_a, basis);
		}
	}

	// Full principal ranges impose no artificial wall at a coordinate branch cut.
	float pi = 3.14159265358979323846f;
	bool swing_limited = any(swing_limits != float4(-pi, pi, -pi, pi));
	bool twist_limited = any(twist_limits != float2(-pi, pi));
	if (!swing_limited && !twist_limited)
		return true;

	// The relative quaternion maps frame B into frame A, matching child-to-parent joint coordinates.
	SphericalCoordinates coordinates = MeasureSphericalCoordinates(quat_mul(quat_conjugate(frame_a), frame_b));
	if (!coordinates.m_regular)
		return false;

	float3 gradient_x = quat_rotate(frame_a, coordinates.m_swing_x_gradient);
	float3 gradient_z = quat_rotate(frame_a, coordinates.m_swing_z_gradient);
	float3 gradient_twist = quat_rotate(frame_a, coordinates.m_twist_gradient);
	for (int component = 0; component != 3; ++component)
	{
		// A regular split can still overflow in finite precision; reject the complete row preparation.
		if (!isfinite(gradient_x[component]) || !isfinite(gradient_z[component]) || !isfinite(gradient_twist[component]))
			return false;
	}
	if (!isfinite(coordinates.m_swing.x) || !isfinite(coordinates.m_swing.y) || !isfinite(coordinates.m_twist))
		return false;

	// Twist is a separate signed interval; a zero-width interval is a bilateral angular lock.
	if (twist_limited)
		SetSphericalAngularRow(rows, 2u, 4u, gradient_twist, coordinates.m_twist, twist_limits);

	if (swing_limited)
	{
		// A fully collapsed axis leaves a genuine line segment or point, enforced by two independent coordinate rows.
		float2 x_range = swing_limits.xy;
		float2 z_range = float2(swing_limits.z, swing_limits.w);
		float2 query = coordinates.m_swing;
		bool locked_x = all(x_range == float2(0, 0));
		bool locked_z = all(z_range == float2(0, 0));
		if (locked_x || locked_z)
		{
			// Both obligations survive even when twist and all three damping rows also occupy the six-row block.
			SetSphericalAngularRow(rows, 0u, 1u, gradient_x, query.x, x_range);
			SetSphericalAngularRow(rows, 1u, 2u, gradient_z, query.y, z_range);
		}
		else
		{
			// Zero-width sides contribute independent half-plane faces. Their two-face corner cannot be replaced by one distance normal.
			bool face_x = (x_range.x == 0.0f && query.x <= 0.0f) || (x_range.y == 0.0f && query.x >= 0.0f);
			bool face_z = (z_range.x == 0.0f && query.y <= 0.0f) || (z_range.y == 0.0f && query.y >= 0.0f);
			if (face_x)
				SetSphericalAngularRow(rows, 0u, 1u, gradient_x, query.x, x_range.x == 0.0f ? float2(0, infinity) : float2(-infinity, 0));

			if (face_z)
				SetSphericalAngularRow(rows, 1u, 2u, gradient_z, query.y, z_range.x == 0.0f ? float2(0, infinity) : float2(-infinity, 0));

			// Project only forbidden half-plane components before measuring the curved obligation. At most one face can coexist
			// with a curved boundary: with two faces, the projected point is the origin and lies strictly inside the outer arc.
			float2 projected = float2(face_x ? 0.0f : query.x, face_z ? 0.0f : query.y);
			float2 radii = float2(projected.x < 0.0f ? -x_range.x : x_range.y, projected.y < 0.0f ? -z_range.x : z_range.y);
			float2 scaled = float2(radii.x > 0.0f ? projected.x / radii.x : 0.0f, radii.y > 0.0f ? projected.y / radii.y : 0.0f);
			if (dot(scaled, scaled) >= 1.0f)
			{
				// At a face/arc corner the curve's independent normal is the other axis; elsewhere retain the analytic smooth normal.
				SwingBoundary boundary = MeasureSwingBoundary(projected, x_range, z_range);
				float2 normal = face_x ? float2(0, projected.y < 0.0f ? -1.0f : 1.0f) :
					face_z ? float2(projected.x < 0.0f ? -1.0f : 1.0f, 0) : boundary.m_outward;
				SetSphericalAngularRow(rows, face_x ? 1u : 0u, 3u, normal.x * gradient_x + normal.y * gradient_z, max(boundary.m_distance, 0.0f), float2(-infinity, 0));
			}
		}
	}
	return true;
}

#ifdef __cplusplus
}
#endif
#endif
