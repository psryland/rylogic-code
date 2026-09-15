//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_TERRAIN_BASELINE_SURFACE_HLSLI
#define PR_TERRAIN_BASELINE_SURFACE_HLSLI
#include "pr/physics/terrain/landscape/baseline_types.hlsli"
#include "pr/algorithm/perlin_noise_derivatives.hlsli"
#ifdef __cplusplus
namespace pr::physics::terrain::landscape::shared
{
	using double2 = math::Vec2<double>;
	using double3 = math::Vec3<double>;
	using namespace pr::algorithm::shared;
	using BaselineReal = NoiseReal;
	using BaselineField = NoiseReal3;
#else
typedef NoiseReal BaselineReal;
typedef NoiseReal3 BaselineField;
#endif

// This evaluator is stage-neutral: no resources, registers, barriers or dispatch intrinsics.
// SM6.0+, Int64ShaderOps, DoublePrecisionFloatShaderOps and extended doubles are required.
// Compile HLSL with -Gis: no contraction/reassociation and no implicit float literal rounding.
// Recipes come from BaselineSurface::Recipe(). Each query uses at most 64 noise octaves and constant local storage.
// CPU fields remain FP64; shader fields use FP32 unless PR_TERRAIN_FP32=0 is defined before inclusion.
// Coordinates, lattice/seed phase, height datum and upload/result layouts remain FP64.
// BaselineField holds (value, d/dx, d/dy), not a spatial vector.
// double(...L) gives both compilers a binary64 constant: HLSL's unsuffixed decimals otherwise round through float.

// Return a constant scalar field.
inline BaselineField BaselineConstant(BaselineReal value)
{
	return BaselineField(value, 0, 0);
}

// Multiply scalar fields with the product rule.
inline BaselineField BaselineProduct(BaselineField lhs, BaselineField rhs)
{
	return BaselineField(lhs.x * rhs.x, lhs.y * rhs.x + rhs.y * lhs.x, lhs.z * rhs.x + rhs.z * lhs.x);
}

// Clamp a scalar field and zero its derivative outside the active interval.
inline BaselineField BaselineClamp(BaselineField sample)
{
	if (sample.x <= 0.0L)
		return BaselineConstant(0);
	if (sample.x >= 1.0L)
		return BaselineConstant(1);

	return sample;
}

// Map signed noise into the unit interval.
inline BaselineField BaselineUnit(BaselineField sample)
{
	return BaselineClamp(BaselineConstant(BaselineReal(0.5L)) + BaselineReal(0.5L) * sample);
}

// Evaluate a smooth transition and its derivative; internal edges are strictly ordered constants.
inline BaselineField BaselineSmooth(BaselineReal edge0, BaselineReal edge1, BaselineField sample)
{
	BaselineReal t = (sample.x - edge0) / (edge1 - edge0);
	if (t <= 0.0L)
		return BaselineConstant(0);
	if (t >= 1.0L)
		return BaselineConstant(1);

	BaselineReal value = t * t * (3 - 2 * t);
	BaselineReal deriv = (6 * t * (1 - t)) / (edge1 - edge0);
	return BaselineField(value, deriv * sample.y, deriv * sample.z);
}

// Normalize a positive regional weight, including the derivative of its shared denominator.
inline BaselineField BaselineWeight(BaselineField weight, BaselineField sum)
{
	BaselineReal inv_sum = BaselineReal(1) / sum.x;
	return BaselineField(weight.x * inv_sum, (weight.y * sum.x - sum.y * weight.x) / (sum.x * sum.x), (weight.z * sum.x - sum.z * weight.x) / (sum.x * sum.x));
}

// Refine a bounded float seed to double precision without double-precision division.
inline double BaselineReciprocalSqrt(double value)
{
#ifdef __cplusplus
	return 1 / std::sqrt(value);
#else
	if (value < 0 || !(value == value))
		return NoiseNaN();
	if (value == 0)
	{
		uint low, high;
		asuint(value, low, high);
		return asdouble(0u, high | 0x7ff00000u);
	}
	if (!NoiseFinite(value))
		return 0;

	// Normalize into [1,4) before the float conversion, retaining the entire double exponent range.
	double scaled = value;
	double factor = 1.0L;
	if (scaled < 2.2250738585072014e-308L)
	{
		scaled *= 18014398509481984.0L;
		factor = 134217728.0L;
	}
	uint low, high;
	asuint(scaled, low, high);
	uint exponent = (high >> 20) & 0x7ffu;
	uint odd = (exponent + 1u) & 1u;
	double mantissa = asdouble(low, (high & 0xfffffu) | ((1023u + odd) << 20));
	double inverse = (double)rsqrt((float)mantissa);
	for (int iteration = 0; iteration != 2; ++iteration)
		inverse *= 1.5L - 0.5L * mantissa * inverse * inverse;

	double scale = asdouble(0u, (2046u - ((exponent + 1023u) >> 1)) << 20);
	return inverse * scale * factor;
#endif
}

// Preserve double square-root precision rather than using HLSL's float-only sqrt overload.
inline double BaselineSqrt(double value)
{
#ifdef __cplusplus
	return std::sqrt(value);
#else
	if (value < 0)
		return NoiseNaN();
	if (value == 0 || !NoiseFinite(value))
		return value;

	return value * BaselineReciprocalSqrt(value);
#endif
}

// Evaluate a rounded ridge and its derivative without a cusp at signed-noise zero crossings.
inline BaselineField BaselineRidge(BaselineField sample, BaselineReal roundness)
{
#if !defined(__cplusplus) && defined(PR_TERRAIN_FP32) && PR_TERRAIN_FP32
	BaselineReal magnitude = sqrt(sample.x * sample.x + roundness * roundness);
#else
	BaselineReal magnitude = BaselineSqrt(sample.x * sample.x + roundness * roundness);
#endif
	BaselineReal deriv = magnitude > 0 ? sample.x / magnitude : BaselineReal(0);
	BaselineField unit = BaselineClamp(BaselineField(1 - magnitude, -deriv * sample.y, -deriv * sample.z));
	return BaselineProduct(unit, unit);
}

// Evaluate one bounded fractal band; ridged bands propagate the weight derivative at every octave.
inline BaselineField BaselineFractal(double2 xy, BaselineBand band, bool ridged)
{
	BaselineField sample = BaselineConstant(0);
	BaselineField weight = BaselineConstant(1);
	BaselineReal amplitude = (BaselineReal)band.m_amplitude;
	double frequency = double(1) / band.m_wavelength_m;
	double slice = double(0.17320508075688773L) + (double)band.m_seed * (double(1) / double(65537));
	if (ridged)
		slice = double(0.38196601125010515L) + (double)band.m_seed * (double(1) / double(65539));

	// A malformed upload cannot cause an unbounded shader loop.
	if (band.m_octave_count < 1 || band.m_octave_count > BaselineMaxOctaveCount)
		return BaselineField(NoiseNaN(), 0, 0);

	for (int octave = 0; octave != band.m_octave_count; ++octave)
	{
		double z = slice + octave * double(0.6180339887498949L);
		if (ridged)
			z = slice + octave * double(0.4142135623730950L);

		NoiseSample noise = NoiseWithDerivatives(band.m_seed, xy.x * frequency, xy.y * frequency, z, 0);
		if (ridged)
		{
			BaselineField ridge = BaselineRidge(BaselineField(noise.m_value, (BaselineReal)frequency * noise.m_dx, (BaselineReal)frequency * noise.m_dy), (BaselineReal)band.m_roundness);
			sample = sample + amplitude * BaselineProduct(weight, ridge);
			weight = BaselineClamp((BaselineReal)band.m_weight_gain * ridge);
		}
		else
		{
			sample.x += amplitude * noise.m_value;
			sample.y += amplitude * (BaselineReal)frequency * noise.m_dx;
			sample.z += amplitude * (BaselineReal)frequency * noise.m_dy;
		}
		frequency *= band.m_lacunarity;
		amplitude *= (BaselineReal)band.m_persistence;
	}
	return sample;
}

// Map warped-space derivatives back into world XY through the domain-warp Jacobian.
inline BaselineField BaselineApplyWarp(BaselineField sample, BaselineField warp_x, BaselineField warp_y)
{
	return BaselineField(sample.x, (1 + warp_x.y) * sample.y + warp_y.y * sample.z, warp_x.z * sample.y + (1 + warp_y.z) * sample.z);
}

// Sample a CPU-prepared recipe in any shader stage or directly on the CPU; inspect m_status before using the result.
#ifdef __cplusplus
inline BaselineResult BaselineEvaluate(BaselineRecipe const& recipe, double2 xy)
#else
BaselineResult BaselineEvaluate(BaselineRecipe recipe, double2 xy)
#endif
{
	BaselineResult result;
	result.m_height = result.m_dx = result.m_dy = 0;
	result.m_material_id = -1;
	result.m_status = 1;
	if (!NoiseFinite(xy.x) || !NoiseFinite(xy.y) ||
		!(xy.x >= -recipe.m_supported_coordinate_abs_m && xy.x <= recipe.m_supported_coordinate_abs_m &&
		xy.y >= -recipe.m_supported_coordinate_abs_m && xy.y <= recipe.m_supported_coordinate_abs_m))
		return result;

	// Apply the same smooth domain warp to all regional and detail fields.
	result.m_status = 2;
	BaselineField warp_x = BaselineFractal(xy, recipe.m_fields[6], false);
	BaselineField warp_y = BaselineFractal(xy, recipe.m_fields[7], false);
	double2 warped = xy + double2(warp_x.x, warp_y.x);
	BaselineField regional = BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[0], false), warp_x, warp_y);
	BaselineField uplift = BaselineUnit(BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[2], false), warp_x, warp_y));
	BaselineField selector = BaselineUnit(BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[1], false), warp_x, warp_y));
	selector = BaselineClamp(selector + BaselineReal(0.35L) * (uplift - BaselineConstant(BaselineReal(0.5L))));

	// Blend the terrain families with smooth normalized weights so regions meet without seams.
	BaselineField w0 = BaselineConstant(1) - BaselineSmooth(BaselineReal(0.28L), BaselineReal(0.58L), selector);
	BaselineField w1 = BaselineSmooth(BaselineReal(0.18L), BaselineReal(0.45L), selector) - BaselineSmooth(BaselineReal(0.55L), BaselineReal(0.82L), selector);
	BaselineField w2 = BaselineSmooth(BaselineReal(0.52L), BaselineReal(0.82L), selector);
	BaselineField sum = w0 + w1 + w2;
	if (!(sum.x > 2.2204460492503131e-16L))
		return result;

	w0 = BaselineWeight(w0, sum);
	w1 = BaselineWeight(w1, sum);
	w2 = BaselineWeight(w2, sum);
	BaselineField plains = regional + (BaselineReal)recipe.m_uplift_height_m * (uplift - BaselineConstant(BaselineReal(0.35L))) + BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[3], false), warp_x, warp_y);
	BaselineField hills = regional + BaselineConstant(35) + (BaselineReal)recipe.m_uplift_height_m * (uplift - BaselineConstant(BaselineReal(0.20L))) + BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[4], false), warp_x, warp_y);
	BaselineField mountains = regional + BaselineConstant((BaselineReal)recipe.m_mountain_base_height_m) + (BaselineReal)recipe.m_uplift_height_m * uplift + BaselineApplyWarp(BaselineFractal(warped, recipe.m_fields[5], true), warp_x, warp_y);
	// Keep the world-space height datum outside local field rounding, including near-zero cancellation and high-altitude sources.
	BaselineField height = BaselineProduct(w0, plains) + BaselineProduct(w1, hills) + BaselineProduct(w2, mountains);
	double absolute_height = recipe.m_sea_level_bias_m + height.x;
	if (!NoiseFinite(absolute_height) || !NoiseFinite(height.y) || !NoiseFinite(height.z))
		return result;

	result.m_height = absolute_height;
	result.m_dx = height.y;
	result.m_dy = height.z;
	result.m_material_id = recipe.m_material_id;
	result.m_status = 0;
	return result;
}
#ifdef __cplusplus
}
#endif
#endif
