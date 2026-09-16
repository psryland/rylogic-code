//*********************************************
// Coherent noise: shared C++ / HLSL arithmetic
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PERLIN_NOISE_DERIVATIVES_HLSLI
#define PR_PERLIN_NOISE_DERIVATIVES_HLSLI
#ifndef PR_TERRAIN_FP32
#define PR_TERRAIN_FP32 1
#endif
#ifdef __cplusplus
#include <limits>
#include "pr/math/math.h"
namespace pr::algorithm::shared
{
	using uint = uint32_t;
	using double3 = math::Vec3<double>;
	using NoiseReal = double;
	using NoiseReal3 = double3;
#elif defined(PR_TERRAIN_FP32) && PR_TERRAIN_FP32
typedef float NoiseReal;
typedef float3 NoiseReal3;
#else
typedef double NoiseReal;
typedef double3 NoiseReal3;
#endif

// One coherent-noise sample with analytic derivatives in lattice units.
struct NoiseSample
{
	NoiseReal m_value;
	NoiseReal m_dx;
	NoiseReal m_dy;
	NoiseReal m_dz;
};

// Test finiteness without HLSL's float-only isfinite overload.
inline bool NoiseFinite(double value)
{
	return value >= -1.7976931348623157e308L && value <= 1.7976931348623157e308L;
}

// Return a quiet NaN without a float conversion.
inline double NoiseNaN()
{
#ifdef __cplusplus
	return std::numeric_limits<double>::quiet_NaN();
#else
	return asdouble(0u, 0x7ff80000u);
#endif
}

// Floor a finite coordinate in the signed 64-bit lattice range without the HLSL float floor overload.
inline double NoiseFloor(double value)
{
	int64_t integer = (int64_t)value;
	double truncated = (double)integer;
	return truncated > value ? truncated - 1 : truncated;
}

// Preserve the original unsigned 64-bit coordinate hash, including negative two's-complement coordinates.
inline uint NoiseHash(uint seed, int64_t x, int64_t y, int64_t z)
{
	uint64_t hash = (uint64_t)seed + 0x9E3779B97F4A7C15ull;
	hash ^= (uint64_t)x * 0x9E3779B185EBCA87ull;
	hash ^= (uint64_t)y * 0xC2B2AE3D27D4EB4Full;
	hash ^= (uint64_t)z * 0x165667B19E3779F9ull;
	hash ^= hash >> 33;
	hash *= 0xFF51AFD7ED558CCDull;
	hash ^= hash >> 33;
	hash *= 0xC4CEB9FE1A85EC53ull;
	hash ^= hash >> 33;
	return (uint)hash;
}

// Wrap signed lattice coordinates into a positive period; zero selects the nonperiodic field.
inline int64_t NoiseWrap(int64_t coordinate, int period)
{
	if (period == 0)
		return coordinate;

	int64_t wrapped = coordinate % period;
	return wrapped >= 0 ? wrapped : wrapped + period;
}

// Select the original deterministic corner gradient.
inline NoiseReal3 NoiseGradient(uint hash)
{
	// Two bits encode each original component plus one, avoiding divergent corner-gradient branches.
	uint shift = (hash & 15u) * 2u;
	return NoiseReal3((int)((0x46552222u >> shift) & 3u) - 1, (int)((0x2222550au >> shift) & 3u) - 1, (int)((0x190a0a55u >> shift) & 3u) - 1);
}

// Ease one local coordinate so neighbouring cells meet with continuous derivatives.
inline NoiseReal NoiseFade(NoiseReal t)
{
	// Reflect around the midpoint to avoid cancellation near one without changing the easing polynomial.
	NoiseReal u = t <= NoiseReal(0.5L) ? t : 1 - t;
	NoiseReal value = u * u * u * (u * (u * 6 - 15) + 10);
	return t <= NoiseReal(0.5L) ? value : 1 - value;
}

// Differentiate the easing polynomial.
inline NoiseReal NoiseFadeDerivative(NoiseReal t)
{
	NoiseReal complement = 1 - t;
	return 30 * t * t * complement * complement;
}

// Interpolate in the generator's original arithmetic order.
inline NoiseReal NoiseLerp(NoiseReal t, NoiseReal a, NoiseReal b)
{
	return a + t * (b - a);
}

// Evaluate one corner ramp without relying on language-specific dot accumulation order.
inline NoiseReal NoiseDot(NoiseReal3 gradient, NoiseReal x, NoiseReal y, NoiseReal z)
{
	return gradient.x * x + gradient.y * y + gradient.z * z;
}

// Evaluate the canonical noise and derivative blend with full lattice phase; unsupported lattice inputs return NaN.
inline NoiseSample NoiseWithDerivatives(uint seed, double x, double y, double z, int period)
{
	NoiseSample result;
	result.m_value = (NoiseReal)NoiseNaN();
	result.m_dx = result.m_dy = result.m_dz = result.m_value;
	if (!(x >= -9223372036854775808.0L && x < 9223372036854775808.0L &&
		y >= -9223372036854775808.0L && y < 9223372036854775808.0L &&
		z >= -9223372036854775808.0L && z < 9223372036854775808.0L))
		return result;

	// Keep full coordinate phase before hashing and optional periodic wrapping.
	double floor_x = NoiseFloor(x);
	double floor_y = NoiseFloor(y);
	double floor_z = NoiseFloor(z);
	int64_t cell_x0 = NoiseWrap((int64_t)floor_x, period);
	int64_t cell_y0 = NoiseWrap((int64_t)floor_y, period);
	int64_t cell_z0 = NoiseWrap((int64_t)floor_z, period);
	int64_t cell_x1 = NoiseWrap((int64_t)floor_x + 1, period);
	int64_t cell_y1 = NoiseWrap((int64_t)floor_y + 1, period);
	int64_t cell_z1 = NoiseWrap((int64_t)floor_z + 1, period);
	NoiseReal local_x = (NoiseReal)(x - floor_x);
	NoiseReal local_y = (NoiseReal)(y - floor_y);
	NoiseReal local_z = (NoiseReal)(z - floor_z);

	// Evaluate the corner ramps once and differentiate their nested interpolation.
	NoiseReal u = NoiseFade(local_x), v = NoiseFade(local_y), w = NoiseFade(local_z);
	NoiseReal du = NoiseFadeDerivative(local_x), dv = NoiseFadeDerivative(local_y), dw = NoiseFadeDerivative(local_z);
	NoiseReal3 g000 = NoiseGradient(NoiseHash(seed, cell_x0, cell_y0, cell_z0));
	NoiseReal3 g100 = NoiseGradient(NoiseHash(seed, cell_x1, cell_y0, cell_z0));
	NoiseReal3 g010 = NoiseGradient(NoiseHash(seed, cell_x0, cell_y1, cell_z0));
	NoiseReal3 g110 = NoiseGradient(NoiseHash(seed, cell_x1, cell_y1, cell_z0));
	NoiseReal3 g001 = NoiseGradient(NoiseHash(seed, cell_x0, cell_y0, cell_z1));
	NoiseReal3 g101 = NoiseGradient(NoiseHash(seed, cell_x1, cell_y0, cell_z1));
	NoiseReal3 g011 = NoiseGradient(NoiseHash(seed, cell_x0, cell_y1, cell_z1));
	NoiseReal3 g111 = NoiseGradient(NoiseHash(seed, cell_x1, cell_y1, cell_z1));
	NoiseReal n000 = NoiseDot(g000, local_x, local_y, local_z);
	NoiseReal n100 = NoiseDot(g100, local_x - 1, local_y, local_z);
	NoiseReal n010 = NoiseDot(g010, local_x, local_y - 1, local_z);
	NoiseReal n110 = NoiseDot(g110, local_x - 1, local_y - 1, local_z);
	NoiseReal n001 = NoiseDot(g001, local_x, local_y, local_z - 1);
	NoiseReal n101 = NoiseDot(g101, local_x - 1, local_y, local_z - 1);
	NoiseReal n011 = NoiseDot(g011, local_x, local_y - 1, local_z - 1);
	NoiseReal n111 = NoiseDot(g111, local_x - 1, local_y - 1, local_z - 1);
	NoiseReal nx00 = NoiseLerp(u, n000, n100);
	NoiseReal nx10 = NoiseLerp(u, n010, n110);
	NoiseReal nx01 = NoiseLerp(u, n001, n101);
	NoiseReal nx11 = NoiseLerp(u, n011, n111);
	NoiseReal dnx00_dx = NoiseLerp(u, g000.x, g100.x) + du * (n100 - n000);
	NoiseReal dnx10_dx = NoiseLerp(u, g010.x, g110.x) + du * (n110 - n010);
	NoiseReal dnx01_dx = NoiseLerp(u, g001.x, g101.x) + du * (n101 - n001);
	NoiseReal dnx11_dx = NoiseLerp(u, g011.x, g111.x) + du * (n111 - n011);
	NoiseReal dnx00_dy = NoiseLerp(u, g000.y, g100.y);
	NoiseReal dnx10_dy = NoiseLerp(u, g010.y, g110.y);
	NoiseReal dnx01_dy = NoiseLerp(u, g001.y, g101.y);
	NoiseReal dnx11_dy = NoiseLerp(u, g011.y, g111.y);
	NoiseReal dnx00_dz = NoiseLerp(u, g000.z, g100.z);
	NoiseReal dnx10_dz = NoiseLerp(u, g010.z, g110.z);
	NoiseReal dnx01_dz = NoiseLerp(u, g001.z, g101.z);
	NoiseReal dnx11_dz = NoiseLerp(u, g011.z, g111.z);
	NoiseReal nxy0 = NoiseLerp(v, nx00, nx10);
	NoiseReal nxy1 = NoiseLerp(v, nx01, nx11);
	NoiseReal dnxy0_dx = NoiseLerp(v, dnx00_dx, dnx10_dx);
	NoiseReal dnxy1_dx = NoiseLerp(v, dnx01_dx, dnx11_dx);
	NoiseReal dnxy0_dy = NoiseLerp(v, dnx00_dy, dnx10_dy) + dv * (nx10 - nx00);
	NoiseReal dnxy1_dy = NoiseLerp(v, dnx01_dy, dnx11_dy) + dv * (nx11 - nx01);
	NoiseReal dnxy0_dz = NoiseLerp(v, dnx00_dz, dnx10_dz);
	NoiseReal dnxy1_dz = NoiseLerp(v, dnx01_dz, dnx11_dz);
	result.m_value = NoiseLerp(w, nxy0, nxy1);
	result.m_dx = NoiseLerp(w, dnxy0_dx, dnxy1_dx);
	result.m_dy = NoiseLerp(w, dnxy0_dy, dnxy1_dy);
	result.m_dz = NoiseLerp(w, dnxy0_dz, dnxy1_dz) + dw * (nxy1 - nxy0);
	return result;
}
#ifdef __cplusplus
}
#endif
#endif
