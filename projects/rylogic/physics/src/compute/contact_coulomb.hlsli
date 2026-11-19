//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_CONTACT_COULOMB_HLSLI
#define PR_PHYSICS_CONTACT_COULOMB_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Return whether all components of a contact-space value are finite.
inline bool CoulombFinite(float3 value)
{
	return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

// Make an impulse feasible about a unit normal, preserving nonnegative normal support and already feasible inputs.
// Friction is finite and nonnegative; the normal/tangent decomposition must be representable.
inline float3 ProjectCoulombImpulse(float3 impulse, float3 axis, float friction)
{
	// Negative support leaves a zero-radius disc, so no tangent impulse can remain.
	float normal = dot(impulse, axis);
	if (normal < 0.0f)
		return float3(0, 0, 0);

	// Normalize the tangent before measuring its length; its squared unscaled norm may overflow or underflow.
	float3 tangent = impulse - normal * axis;
	float scale = max(abs(tangent.x), max(abs(tangent.y), abs(tangent.z)));
	if (scale == 0.0f)
		return impulse;

	if (normal == 0.0f || friction == 0.0f)
		return normal * axis;

	float3 scaled_tangent = tangent / scale;
	float scaled_length = length(scaled_tangent);

	// A required coefficient above the finite range means every finite supplied coefficient needs projection.
	if (friction >= (scale / normal) * scaled_length)
		return impulse;

	// Divide before multiplying by friction so an overflowing disc radius need not erase a representable tangent.
	float projected_scale = (normal / scaled_length) * friction;
	return normal * axis + projected_scale * scaled_tangent;
}

// Measure the ordinary Coulomb law in velocity units, without treating sliding speed as an error.
// Coordinates are (normal, tangent 0, tangent 1); tangent_response is the positive mean tangent mobility.
inline float CoulombLawError(float3 impulse, float3 velocity, float friction, float normal_response, float tangent_response)
{
	// Evaluate the disc residual in velocity units without subtracting nearly equal impulses.
	float scaled_normal = normal_response * impulse.x;
	float normal_error = abs(min(scaled_normal, velocity.x));
	float2 slip = float2(velocity.y, velocity.z);
	float2 tangent = tangent_response * float2(impulse.y, impulse.z) - slip;
	float limit = tangent_response * (friction * max(impulse.x, 0.0f));
	float tangent_length = length(tangent);
	if (!isfinite(scaled_normal) || !isfinite(normal_error) || !isfinite(limit) || !isfinite(tangent_length))
		return asfloat(0x7F800000u);

	// Inside the disc the residual is exactly slip. Outside, add only the radial excess instead of cancelling two large impulses.
	float2 tangent_error = slip;
	if (tangent_length > limit && tangent_length > 0.0f)
		tangent_error += (tangent_length - limit) * (tangent / tangent_length);

	return max(normal_error, length(tangent_error));
}

// Evaluate the normal-eliminated sliding equations. The compact multiplier s represents r=scale*s/(1-s).
// Positive r gives v_t=-r*lambda_t, so friction opposes the actual final slip rather than a pre-update velocity.
inline bool CoulombSlidingPoint(float normal_response, float2 coupling, float3 schur, float3 free_velocity, float scale, float s, out_(float3) impulse)
{
	// Multiplying the two tangent equations by (1-s) keeps their coefficients bounded as the slip multiplier grows.
	float one_minus_s = 1.0f - s;
	float xx = one_minus_s * schur.x + s;
	float xy = one_minus_s * schur.y;
	float yy = one_minus_s * schur.z + s;
	float determinant = xx * yy - xy * xy;
	float2 rhs = -one_minus_s * (float2(free_velocity.y, free_velocity.z) - coupling * (free_velocity.x / normal_response)) / scale;
	impulse = float3(0.0f, 0.0f, 0.0f);
	if (!(determinant > 0.0f) || !isfinite(determinant))
		return false;

	// Recover normal support from its own equality, not from a cone projection that can manufacture support.
	impulse.y = (yy * rhs.x - xy * rhs.y) / determinant;
	impulse.z = (xx * rhs.y - xy * rhs.x) / determinant;
	impulse.x = -(free_velocity.x + dot(coupling, float2(impulse.y, impulse.z))) / normal_response;
	return CoulombFinite(impulse);
}

// Find a feasible sticking impulse along a unit null direction of the tangent Schur block.
// The minimum-norm tangent is perpendicular to this direction; null motion changes support but not contact velocity.
inline bool CoulombNullSupport(float normal_response, float2 coupling, float3 schur, float friction, inout_(float3) impulse)
{
	float2 null_axis = schur.x >= schur.z ? float2(-schur.y, schur.x) : float2(schur.z, -schur.y);
	float null_length = length(null_axis);
	if (!(null_length > 0.0f))
		return false;

	null_axis /= null_length;
	float normal_slope = -dot(coupling, null_axis) / normal_response;
	if (normal_slope < 0.0f)
	{
		null_axis = -null_axis;
		normal_slope = -normal_slope;
	}

	// Solve sqrt(T^2+q^2)=p+h*q with q>=0, where increasing q increases available support.
	float tangent_squared = impulse.y * impulse.y + impulse.z * impulse.z;
	float p = friction * impulse.x;
	float h = friction * normal_slope;
	float a = 1.0f - h * h;
	float discriminant = p * p - a * tangent_squared;
	if (!(h > 0.0f) || discriminant < 0.0f || (h <= 1.0f && p <= 0.0f))
		return false;

	float root = sqrt(discriminant);
	float denominator = h * p + root;
	float q = 0.0f;
	if (denominator > 0.0f)
		q = (tangent_squared - p * p) / denominator;
	else if (a < 0.0f)
		q = (h * p - root) / a;
	else
		return false;

	if (!isfinite(q) || q < 0.0f)
		return false;

	impulse.y += q * null_axis.x;
	impulse.z += q * null_axis.y;
	impulse.x += q * normal_slope;
	return CoulombFinite(impulse);
}

// Solve one contact with a symmetric positive-semidefinite response and positive normal mobility.
// Free velocity excludes this contact's accumulated impulse. Friction is nonnegative; tolerance is in velocity units.
// Return false on non-finite arithmetic or an unresolved law, leaving the caller responsible for reporting the failed solve.
inline bool SolveCoulombContact(float3x3 response, float3 free_velocity, float3 axis, float friction, float tolerance, out_(float3) impulse)
{
	// Reject raw non-finite input before unilateral projection can conceal it.
	impulse = float3(0.0f, 0.0f, 0.0f);
	if (!CoulombFinite(response[0]) || !CoulombFinite(response[1]) || !CoulombFinite(response[2]) ||
		!CoulombFinite(free_velocity) || !CoulombFinite(axis) || !isfinite(friction) || !isfinite(tolerance))
		return false;

	// Choose an orthonormal contact basis without an angle parameter or a preferred world tangent.
	float3 seed = abs(axis.x) < abs(axis.y) ? float3(1.0f, 0.0f, 0.0f) : float3(0.0f, 1.0f, 0.0f);
	float3 tangent_0 = normalize(cross(axis, seed));
	float3 tangent_1 = cross(axis, tangent_0);
	float3 response_n = mul(response, axis);
	float3 response_t0 = mul(response, tangent_0);
	float3 response_t1 = mul(response, tangent_1);
	float normal_response = dot(axis, response_n);
	float2 coupling = float2(dot(tangent_0, response_n), dot(tangent_1, response_n));
	float3 tangent_response = float3(dot(tangent_0, response_t0), dot(tangent_0, response_t1), dot(tangent_1, response_t1));
	float3 b = float3(dot(axis, free_velocity), dot(tangent_0, free_velocity), dot(tangent_1, free_velocity));
	float scale = max(normal_response, max(tangent_response.x, tangent_response.z));
	float tangent_weight = 0.5f * tangent_response.x + 0.5f * tangent_response.z;
	if (tangent_weight == 0.0f)
		tangent_weight = normal_response;

	if (!CoulombFinite(response_n) || !CoulombFinite(response_t0) || !CoulombFinite(response_t1) ||
		!CoulombFinite(tangent_response) || !CoulombFinite(b) || !isfinite(normal_response) || !isfinite(scale) ||
		!(normal_response > 0.0f) || !(scale > 0.0f) || friction < 0.0f || !(tolerance > 0.0f))
		return false;

	// A separating free state needs no support. A frictionless contact needs only the normal scalar equation.
	if (b.x >= 0.0f)
		return true;

	if (friction == 0.0f)
	{
		impulse = (-b.x / normal_response) * axis;
		float3 velocity = free_velocity + mul(response, impulse);
		return CoulombFinite(impulse) && CoulombFinite(velocity) && abs(dot(axis, velocity)) <= tolerance;
	}

	// Eliminating the normal equation leaves a positive-semidefinite 2x2 response, normalized for the multiplier search.
	float3 schur = (tangent_response - float3(coupling.x * coupling.x, coupling.x * coupling.y, coupling.y * coupling.y) / normal_response) / scale;
	float determinant = schur.x * schur.z - schur.y * schur.y;
	float3 candidate = float3(0.0f, 0.0f, 0.0f);
	bool have_stick = false;
	if (determinant > 0.0f)
	{
		have_stick = CoulombSlidingPoint(normal_response, coupling, schur, b, scale, 0.0f, candidate);
	}
	else if (determinant == 0.0f && schur.x >= 0.0f && schur.z >= 0.0f)
	{
		// For a singular tangent response, test the minimum-norm sticking solution without discarding the friction disc.
		float trace = schur.x + schur.z;
		float2 rhs = -(float2(b.y, b.z) - coupling * (b.x / normal_response)) / scale;
		if (trace > 0.0f)
		{
			candidate.y = (schur.x * rhs.x + schur.y * rhs.y) / (trace * trace);
			candidate.z = (schur.y * rhs.x + schur.z * rhs.y) / (trace * trace);
		}

		candidate.x = -(b.x + dot(coupling, float2(candidate.y, candidate.z))) / normal_response;
		have_stick = CoulombFinite(candidate);
		if (have_stick && length(float2(candidate.y, candidate.z)) > friction * candidate.x && trace > 0.0f)
			have_stick = CoulombNullSupport(normal_response, coupling, schur, friction, candidate);
	}
	else
	{
		return false;
	}

	// Check the actual reconstructed velocity; a singular system's incompatible right-hand side cannot be called sticking.
	if (have_stick && candidate.x >= 0.0f)
	{
		float3 world_impulse = candidate.x * axis + candidate.y * tangent_0 + candidate.z * tangent_1;
		float3 world_velocity = free_velocity + mul(response, world_impulse);
		float3 velocity = float3(dot(axis, world_velocity), dot(tangent_0, world_velocity), dot(tangent_1, world_velocity));
		float error = CoulombLawError(candidate, velocity, friction, normal_response, tangent_weight);
		if (!CoulombFinite(world_impulse) || !CoulombFinite(world_velocity) || !isfinite(error))
			return false;

		if (error <= tolerance)
		{
			impulse = world_impulse;
			return true;
		}
	}

	// At s=1 the tangent impulse vanishes and support is positive. Descend until the impulse is outside its friction disc.
	// This avoids evaluating a singular s=0 endpoint and does not regularize the accepted physical response.
	float upper = 1.0f;
	float lower = 0.5f;
	bool bracketed = false;
	for (int iteration = 0; iteration != 64; ++iteration)
	{
		if (!CoulombSlidingPoint(normal_response, coupling, schur, b, scale, lower, candidate))
			return false;

		float cone_error = length(float2(candidate.y, candidate.z)) - friction * candidate.x;
		if (!isfinite(cone_error))
			return false;

		if (cone_error > 0.0f)
		{
			bracketed = true;
			break;
		}
		upper = lower;
		lower *= 0.5f;
	}
	if (!bracketed)
		return false;

	// Bisection preserves a sign-changing bracket even when the ratio is not monotone. Accept only feasible-side points.
	for (int iteration = 0; iteration != 64; ++iteration)
	{
		float middle = 0.5f * (lower + upper);
		if (!CoulombSlidingPoint(normal_response, coupling, schur, b, scale, middle, candidate))
			return false;

		float cone_error = length(float2(candidate.y, candidate.z)) - friction * candidate.x;
		if (!isfinite(cone_error))
			return false;

		if (cone_error > 0.0f)
		{
			lower = middle;
			continue;
		}
		upper = middle;
		float3 world_impulse = candidate.x * axis + candidate.y * tangent_0 + candidate.z * tangent_1;
		float3 world_velocity = free_velocity + mul(response, world_impulse);
		float3 velocity = float3(dot(axis, world_velocity), dot(tangent_0, world_velocity), dot(tangent_1, world_velocity));
		float error = CoulombLawError(candidate, velocity, friction, normal_response, tangent_weight);
		if (!CoulombFinite(world_impulse) || !CoulombFinite(world_velocity) || !isfinite(error))
			return false;

		if (error <= tolerance)
		{
			impulse = world_impulse;
			return true;
		}
	}
	return false;
}

#ifdef __cplusplus
}
#endif
#endif
