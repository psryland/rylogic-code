//*********************************************
// Ordinary non-associated contact-law derivative
// Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_CONTACT_COULOMB_LINEARISATION_HLSLI
#define PR_PHYSICS_CONTACT_COULOMB_LINEARISATION_HLSLI
#include "physics/src/compute/contact_coulomb.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Residual and selected piecewise derivative in one orthonormal coordinate system.
// The normal equation has no friction multiplier term: this is not the derivative of a cone-energy objective.
struct CoulombLinearisation
{
	float3 residual;
	float3x3 velocity_derivative;
	float3x3 impulse_derivative;
	float normal_weight;
	float tangent_weight;
};

// Apply positive normal/tangent coordinate scales about a unit normal, without subtracting the scale coefficients.
inline float3 ScaleCoulombCoordinates(float3 value, float3 axis, float normal_scale, float tangent_scale)
{
	// Separate the components first so a large tangent scale cannot cancel a smaller pure normal result.
	float3 normal = dot(axis, value) * axis;
	return normal_scale * normal + tangent_scale * (value - normal);
}

// Split a finite scalar into a signed mantissa and base-two exponent.
inline float2 CoulombSplitScale(float value)
{
#ifdef __cplusplus
	int exponent;
	float mantissa = std::frexp(value, &exponent);
#else
	// HLSL frexp returns an unsigned mantissa; preserve the signed scalar contract shared with std::frexp.
	float exponent;
	float mantissa = frexp(value, exponent);
	mantissa = value < 0.0f ? -mantissa : mantissa;
#endif
	return float2(mantissa, (float)exponent);
}

// Restore a power-of-two scale without first constructing an overflowing power at the largest normal exponent.
inline float CoulombJoinScale(float value, float exponent)
{
	if (value == 0.0f)
		return 0.0f;

#ifdef __cplusplus
	return std::ldexp(value, (int)exponent);
#else
	float2 parts = CoulombSplitScale(value);
	value = parts.x;
	exponent += parts.y;
	return exponent > 127.0f ? ldexp(2.0f * value, exponent - 1.0f) : ldexp(value, exponent);
#endif
}

// Nearest feasible impulse in the metric |S^-1*(p-value)|^2, where S has positive normal/tangent scales.
// In scaled coordinates the cone slope is friction*S_normal/S_tangent. This constructs a search point, not a physical contact law.
inline bool ProjectCoulombMetricImpulse(float3 value, float3 axis, float friction, float2 scales, out_(float3) result)
{
	// Preserve already feasible values and avoid measuring an unscaled tangent norm.
	result = float3(0, 0, 0);
	float normal = dot(value, axis);
	float3 tangent = value - normal * axis;
	float tangent_scale = max(abs(tangent.x), max(abs(tangent.y), abs(tangent.z)));
	if (!CoulombFinite(value) || !CoulombFinite(tangent) || !isfinite(normal) ||
		!isfinite(friction) || friction < 0.0f || !isfinite(scales.x) || !isfinite(scales.y) || !(scales.x > 0) || !(scales.y > 0))
		return false;

	if (tangent_scale == 0.0f || friction == 0.0f)
	{
		result = max(normal, 0.0f) * axis;
		return CoulombFinite(result);
	}
	float2 mu = CoulombSplitScale(friction);
	float2 n = CoulombSplitScale(normal);
	float2 t = CoulombSplitScale(tangent_scale);
	// Divide only normalized mantissas: GPU division by a large tangent scale can flush its reciprocal before multiplication.
	float3 unit_tangent = float3(CoulombJoinScale(tangent.x, -t.y), CoulombJoinScale(tangent.y, -t.y), CoulombJoinScale(tangent.z, -t.y)) / t.x;
	float tangent_length = length(unit_tangent);
	if (normal > 0.0f)
	{
		// Compare |t| with mu*n without an overflowing product or a subnormal ratio that HLSL flushes to zero.
		float radius_exponent = mu.y + n.y;
		float comparison_exponent = max(t.y, radius_exponent);
		float tangent_size = CoulombJoinScale(t.x * tangent_length, t.y - comparison_exponent);
		float radius_size = CoulombJoinScale(mu.x * n.x, radius_exponent - comparison_exponent);
		if (tangent_size <= radius_size)
		{
			result = value;
			return true;
		}
	}

	// Boundary support is (s_t^2*n + mu*s_n^2*|t|)/(s_t^2 + mu^2*s_n^2), or zero in the polar cone.
	// Keep product exponents separately: neither squared scales, cone slope, nor the numerator's individual terms need be representable.
	float2 sn = CoulombSplitScale(scales.x);
	float2 st = CoulombSplitScale(scales.y);
	float numerator_a = st.x * st.x * n.x;
	float numerator_b = mu.x * sn.x * sn.x * t.x * tangent_length;
	float exponent_a = 2.0f * st.y + n.y;
	float exponent_b = mu.y + 2.0f * sn.y + t.y;
	float numerator_exponent = numerator_a == 0.0f ? exponent_b : max(exponent_a, exponent_b);
	float numerator = CoulombJoinScale(numerator_a, exponent_a - numerator_exponent) + CoulombJoinScale(numerator_b, exponent_b - numerator_exponent);
	if (numerator <= 0.0f)
		return true;

	float denominator_a = st.x * st.x;
	float denominator_b = mu.x * mu.x * sn.x * sn.x;
	exponent_a = 2.0f * st.y;
	exponent_b = 2.0f * mu.y + 2.0f * sn.y;
	float denominator_exponent = max(exponent_a, exponent_b);
	float denominator = CoulombJoinScale(denominator_a, exponent_a - denominator_exponent) + CoulombJoinScale(denominator_b, exponent_b - denominator_exponent);
	float2 support = CoulombSplitScale(numerator / denominator);
	float projected_normal = CoulombJoinScale(support.x, support.y + numerator_exponent - denominator_exponent);
	if (!isfinite(projected_normal))
		return false;

	// Form each tangent component from the represented support without materializing a tiny ratio or losing a small direction component.
	float2 represented_support = CoulombSplitScale(projected_normal);
	float2 tangent_factor = CoulombSplitScale(mu.x * represented_support.x / (t.x * tangent_length));
	float tangent_exponent = tangent_factor.y + mu.y + represented_support.y - t.y;
	float3 projected_tangent;
	for (int component = 0; component != 3; ++component)
	{
		float2 original = CoulombSplitScale(tangent[component]);
		projected_tangent[component] = CoulombJoinScale(tangent_factor.x * original.x, tangent_exponent + original.y);
	}
	result = projected_normal * axis + projected_tangent;
	return CoulombFinite(result);
}

// Linearise the normal complementarity and tangent-disc residual at the actual velocity and accumulated impulse.
// Response is positive semidefinite with positive normal mobility. Its entries supply units, not a diagonal approximation to coupling.
inline bool LineariseCoulombLaw(float3x3 response, float3 velocity, float3 impulse, float3 axis, float friction, out_(CoulombLinearisation) result)
{
	// Match the existing full-law evaluator's positive mobility scales, including a completely locked tangent plane.
	float3 seed = abs(axis.x) < abs(axis.y) ? float3(1, 0, 0) : float3(0, 1, 0);
	float3 tangent_0 = normalize(cross(axis, seed));
	float3 tangent_1 = cross(axis, tangent_0);
	float a = dot(axis, mul(response, axis));
	float c = 0.5f * dot(tangent_0, mul(response, tangent_0)) + 0.5f * dot(tangent_1, mul(response, tangent_1));
	if (c == 0.0f)
		c = a;

	// Invalid arithmetic is a transaction failure, never an inactive or recovered contact.
	result.residual = float3(0, 0, 0);
	result.velocity_derivative = float3x3(float3(0,0,0), float3(0,0,0), float3(0,0,0));
	result.impulse_derivative = result.velocity_derivative;
	result.normal_weight = a;
	result.tangent_weight = c;
	if (!CoulombFinite(velocity) || !CoulombFinite(impulse) || !CoulombFinite(axis) || !isfinite(friction) ||
		!isfinite(a) || !isfinite(c) || !(a > 0.0f) || !(c > 0.0f) || friction < 0.0f)
		return false;

	// At a zero/zero normal tie use the velocity equation, allowing support to propagate through a resting contact chain.
	float3x3 normal = float3x3(axis.x * axis, axis.y * axis, axis.z * axis);
	float3x3 tangent = float3x3(float3(1,0,0), float3(0,1,0), float3(0,0,1)) - normal;
	float lambda_n = dot(axis, impulse);
	float velocity_n = dot(axis, velocity);
	result.residual = axis * min(a * lambda_n, velocity_n);
	if (velocity_n <= a * lambda_n)
	{
		// Supported or approaching contacts constrain velocity without imposing an associated normal-separation condition.
		result.velocity_derivative = normal;
	}
	else
	{
		// Separating unsupported contacts constrain their normal impulse to zero.
		result.impulse_derivative = a * normal;
	}

	// The disc projection supplies both sticking and supported sliding, including zero and rank-one tangent response.
	float3 lambda_t = impulse - lambda_n * axis;
	float3 velocity_t = velocity - velocity_n * axis;
	float3 z = c * lambda_t - velocity_t;
	float z_length = length(z);
	float radius = c * friction * max(lambda_n, 0.0f);
	if (!CoulombFinite(z) || !isfinite(z_length) || !isfinite(radius))
		return false;

	if (friction == 0.0f)
	{
		// A zero-radius disc fixes tangent impulse, even at the otherwise ambiguous zero-slip origin.
		result.residual += c * lambda_t;
		result.impulse_derivative += c * tangent;
	}
	else if (z_length <= radius)
	{
		// Interior sticking constrains tangent velocity; no tangent inverse or regularisation is introduced.
		result.residual += velocity_t;
		result.velocity_derivative += tangent;
	}
	else
	{
		// Differentiate c*lambda_t - radius*z/|z|, including the radius's dependence on positive normal support.
		float3 unit_z = z / z_length;
		float3x3 projection_derivative = (radius / z_length) * (tangent - float3x3(unit_z.x * unit_z, unit_z.y * unit_z, unit_z.z * unit_z));
		result.residual += velocity_t + (z_length - radius) * unit_z;
		result.velocity_derivative += projection_derivative;
		result.impulse_derivative += c * (tangent - projection_derivative);
		if (lambda_n >= 0.0f)
		{
			// Select the positive-support derivative at zero so an initially unsupported sliding impact can acquire friction.
			result.impulse_derivative -= c * friction * float3x3(unit_z.x * axis, unit_z.y * axis, unit_z.z * axis);
		}
	}
	return CoulombFinite(result.residual) &&
		CoulombFinite(result.velocity_derivative[0]) && CoulombFinite(result.velocity_derivative[1]) && CoulombFinite(result.velocity_derivative[2]) &&
		CoulombFinite(result.impulse_derivative[0]) && CoulombFinite(result.impulse_derivative[1]) && CoulombFinite(result.impulse_derivative[2]);
}

#ifdef __cplusplus
}
#endif
#endif
