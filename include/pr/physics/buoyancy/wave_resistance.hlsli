//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Steady wave-making resistance of a floating body, shared by C++ and HLSL.
//
// A body moving along the water surface pushes the surface down over its waterplane, and the waves it leaves behind
// (the V-shaped wake) carry energy away. The body pays for that energy with a drag force, the wave-making resistance.
// The body is modelled as a pressure distribution moving over deep water (Havelock's moving pressure patch), and
// linear wave theory gives the resistance of such a patch exactly, without simulating the wake:
//   R = k0^3 / (2*pi*rho*g) * integral(-pi/2..pi/2) |P(k0*sec^2(theta), theta)|^2 * sec^5(theta) dtheta
// where k0 = g/U^2 is the wave number of waves that travel at the body's speed U, and P(k, theta) is the Fourier transform
// of the pressure at wave number k in direction theta from the motion. The pressure under a floating body is rho*g*(draft),
// so its total is rho*g*V, the weight of the displaced water. It is modelled as an elliptical Gaussian with standard deviations
// sigma_along (in the direction of motion) and sigma_across, for which
//   R = rho * g * V^2 * k0^3 / (2*pi) * I(k0*sigma_along, k0*sigma_across, k0*depth)
// with the dimensionless integral I (see WaveResistanceIntegral). The spread is the horizontal spread of the displaced volume,
// because the pressure is the weight of the displaced water above each point.
//
// The displaced volume is below the surface, not on it. A disturbance at depth f reaches the surface reduced by exp(-k*f) for
// waves of wave number k, so the waves it makes carry energy reduced by exp(-2*k*f). 'depth' is the mean depth of the displaced
// volume below the surface. This keeps the resistance finite when the waterplane is very small, for example for a dense body
// that is nearly or fully submerged, which would otherwise act as a point load on the surface. A deeply submerged body makes
// almost no waves.
//
// The resistance depends on the speed through k0*sigma, which is 1/Fr^2 for a Froude number Fr = U/sqrt(g*sigma):
//   - slow bodies (k0*sigma >> 1) make waves much shorter than themselves, which cancel, and the resistance vanishes,
//   - it peaks near k0*sigma ~ 1 (the "hump speed"), where the body is as long as the waves it makes,
//   - fast bodies make waves much longer than themselves, and the resistance falls as U^-2.
// The Gaussian shape gives a smooth curve without the oscillations that a sharp-edged hull adds at low speed.
#ifndef PR_PHYSICS_BUOYANCY_WAVE_RESISTANCE_HLSLI
#define PR_PHYSICS_BUOYANCY_WAVE_RESISTANCE_HLSLI
#include "pr/hlsl/core.hlsli"

#ifdef __cplusplus
namespace pr::physics::buoyancy
{
#endif

// Dimensionless wave-resistance integral of a Gaussian pressure patch at depth, with a_along = k0*sigma_along,
// a_across = k0*sigma_across and c_depth = k0*depth:
//   I = integral(-pi/2..pi/2) exp(-sec^4(theta)*(a_along^2*cos^2(theta) + a_across^2*sin^2(theta)) - 2*c_depth*sec^2(theta)) * sec^5(theta) dtheta
// Requires a_across > 0, c_depth >= 0, and a_along > 0 or c_depth > 0. The result is accurate to a fraction of a percent; it is
// zero where it is below e^-40.
odr float WaveResistanceIntegral(float a_along, float a_across, float c_depth)
{
	// With t = tan(theta) the integral becomes 2 * integral(0..inf) exp(-E(t)) * (1 + t^2)^(3/2) dt, where
	// E(t) = (1 + t^2)*(a2 + a_across^2*t^2) and a2 = a_along^2 + 2*c_depth, so depth has the same effect as length along the motion.
	// E is at least a2*(1 + t^2), a2 + a_across^2*t^2, and a_across^2*t^4, so the integrand is negligible beyond the smallest t at
	// which one of these bounds reaches 'e_max'. Using all three keeps the sample spacing small compared with the width of the
	// integrand for any patch shape.
	static const float e_max = 40.0f;
	static const int count = 32;
	float a2 = a_along * a_along + 2.0f * c_depth;
	float b2 = a_across * a_across;
	if (a2 >= e_max)
		return 0.0f;

	// The integrand is even in t, so the midpoint rule over [0, t_max] is the trapezoid rule over [-t_max, t_max], which
	// converges very quickly for smooth integrands that decay to zero at both ends.
	float t_max = min(min(sqrt(e_max / a2 - 1.0f), sqrt((e_max - a2) / b2)), sqrt(sqrt(e_max / b2)));
	float dt = t_max / count;
	float sum = 0.0f;
	for (int i = 0; i != count; ++i)
	{
		// Midpoint sample of the transformed integrand.
		float t = (i + 0.5f) * dt;
		float s = 1.0f + t * t;
		sum += exp(-s * (a2 + b2 * t * t)) * s * sqrt(s);
	}
	return 2.0f * sum * dt;
}

// Steady wave-making resistance (N) of a floating body that displaces 'volume' (m^3) of fluid of density 'density' and moves
// horizontally at 'speed' (m/s) relative to the water, under gravity 'gravity' (m/s^2). 'sigma_along' and 'sigma_across' are the
// standard deviations (m) of the displaced volume along and across the direction of motion, and 'depth' (m) is its mean depth
// below the surface. 'sigma_across' must be positive, 'depth' must not be negative, and 'sigma_along' or 'depth' must be positive.
odr float WaveResistance(float density, float gravity, float volume, float speed, float sigma_along, float sigma_across, float depth)
{
	// A body at rest makes no waves. Otherwise evaluate the integral first, because k0 overflows long before the
	// integral becomes non-zero at low speed.
	if (speed <= 0.0f)
		return 0.0f;

	float k0 = gravity / (speed * speed);
	float integral = WaveResistanceIntegral(k0 * sigma_along, k0 * sigma_across, k0 * depth);
	if (integral == 0.0f)
		return 0.0f;

	return density * gravity * volume * volume * k0 * k0 * k0 * integral / tau;
}

#ifdef __cplusplus
}
#endif
#endif
