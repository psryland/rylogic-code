//*****************************************************************************
// Perlin Noise Generator
// Coherent noise function over 3 dimensions
// (copyright Ken Perlin) - This is the improved version
//*****************************************************************************
// Usage:
//  PerlinNoiseGenerator perlin(seed);
//  float x,y,z = sample coordinates;
//  float freq = the 'frequency' of the noise
//  float amp = the amplitude of the noise
//  float offset = bias for the noise.
//  [-1, 1] * amp + offset = perlin.Noise(x * freq, y * freq, z * freq) * amp + offset;

#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace pr::algorithm
{
	// One coherent-noise sample and its analytic derivatives with respect to the input coordinates.
	template <typename S>
	struct PerlinNoiseSample
	{
		S m_value;
		S m_dx;
		S m_dy;
		S m_dz;
	};

	// Generates coherent gradient noise without a short repeating permutation table.
	class PerlinNoiseGenerator
	{
		uint32_t m_seed;

	public:

		// Creates a deterministic noise field identified by 'seed'.
		explicit PerlinNoiseGenerator(uint32_t seed)
			:m_seed(seed)
		{
		}

		// Samples the effectively non-periodic noise field at the given coordinates.
		float Noise(float x, float y, float z) const
		{
			return NoiseImpl(x, y, z, 0);
		}

		// Samples the effectively non-periodic noise field and returns its analytic derivatives.
		PerlinNoiseSample<double> NoiseWithDerivatives(double x, double y, double z) const
		{
			return NoiseWithDerivativesImpl(x, y, z, 0);
		}

		// Samples a seamlessly repeating noise field whose period is measured in lattice cells on every axis.
		float NoisePeriodic(float x, float y, float z, int period) const
		{
			if (period <= 0)
				throw std::invalid_argument("Perlin noise period must be greater than zero");

			return NoiseImpl(x, y, z, period);
		}

		// Samples a seamlessly repeating noise field and returns its analytic derivatives.
		PerlinNoiseSample<double> NoisePeriodicWithDerivatives(double x, double y, double z, int period) const
		{
			if (period <= 0)
				throw std::invalid_argument("Perlin noise period must be greater than zero");

			return NoiseWithDerivativesImpl(x, y, z, period);
		}

	private:

		// Samples one cube, optionally wrapping its lattice coordinates to create a seamless period.
		template <typename S>
		S NoiseImpl(S x, S y, S z, int period) const
		{
			// Split the sample into its containing lattice cell and position within that cell.
			auto const floor_x = std::floor(x);
			auto const floor_y = std::floor(y);
			auto const floor_z = std::floor(z);
			auto cell_x0 = static_cast<int64_t>(floor_x);
			auto cell_y0 = static_cast<int64_t>(floor_y);
			auto cell_z0 = static_cast<int64_t>(floor_z);
			auto cell_x1 = cell_x0 + 1;
			auto cell_y1 = cell_y0 + 1;
			auto cell_z1 = cell_z0 + 1;
			x -= floor_x;
			y -= floor_y;
			z -= floor_z;

			// Wrapping the corner coordinates preserves the same interpolation across opposite boundaries.
			if (period != 0)
			{
				cell_x0 = Wrap(cell_x0, period);
				cell_y0 = Wrap(cell_y0, period);
				cell_z0 = Wrap(cell_z0, period);
				cell_x1 = Wrap(cell_x1, period);
				cell_y1 = Wrap(cell_y1, period);
				cell_z1 = Wrap(cell_z1, period);
			}

			// Ease each local coordinate so adjacent cells meet without visible slope changes.
			auto const u = Fade(x);
			auto const v = Fade(y);
			auto const w = Fade(z);

			// Blend the gradient contributions from the eight corners of the containing cube.
			return Lerp(w,
				Lerp(v,
					Lerp(u, Grad(Hash(cell_x0, cell_y0, cell_z0), x, y, z), Grad(Hash(cell_x1, cell_y0, cell_z0), x - 1, y, z)),
					Lerp(u, Grad(Hash(cell_x0, cell_y1, cell_z0), x, y - 1, z), Grad(Hash(cell_x1, cell_y1, cell_z0), x - 1, y - 1, z))),
				Lerp(v,
					Lerp(u, Grad(Hash(cell_x0, cell_y0, cell_z1), x, y, z - 1), Grad(Hash(cell_x1, cell_y0, cell_z1), x - 1, y, z - 1)),
					Lerp(u, Grad(Hash(cell_x0, cell_y1, cell_z1), x, y - 1, z - 1), Grad(Hash(cell_x1, cell_y1, cell_z1), x - 1, y - 1, z - 1))));
		}

		// Samples one cube and differentiates the trilinear blend analytically.
		PerlinNoiseSample<double> NoiseWithDerivativesImpl(double x, double y, double z, int period) const
		{
			// Split the sample into its containing lattice cell and position within that cell.
			auto const floor_x = std::floor(x);
			auto const floor_y = std::floor(y);
			auto const floor_z = std::floor(z);
			auto cell_x0 = static_cast<int64_t>(floor_x);
			auto cell_y0 = static_cast<int64_t>(floor_y);
			auto cell_z0 = static_cast<int64_t>(floor_z);
			auto cell_x1 = cell_x0 + 1;
			auto cell_y1 = cell_y0 + 1;
			auto cell_z1 = cell_z0 + 1;
			x -= floor_x;
			y -= floor_y;
			z -= floor_z;

			// Wrapping the corner coordinates preserves the same interpolation across opposite boundaries.
			if (period != 0)
			{
				cell_x0 = Wrap(cell_x0, period);
				cell_y0 = Wrap(cell_y0, period);
				cell_z0 = Wrap(cell_z0, period);
				cell_x1 = Wrap(cell_x1, period);
				cell_y1 = Wrap(cell_y1, period);
				cell_z1 = Wrap(cell_z1, period);
			}

			// Ease each local coordinate and keep the fade derivatives for the interpolation chain rule.
			auto const u = Fade(x);
			auto const v = Fade(y);
			auto const w = Fade(z);
			auto const du = dFade(x);
			auto const dv = dFade(y);
			auto const dw = dFade(z);

			// Evaluate the corner ramps and their local derivatives once so every interpolant can reuse them.
			auto const g000 = GradVec(Hash(cell_x0, cell_y0, cell_z0));
			auto const g100 = GradVec(Hash(cell_x1, cell_y0, cell_z0));
			auto const g010 = GradVec(Hash(cell_x0, cell_y1, cell_z0));
			auto const g110 = GradVec(Hash(cell_x1, cell_y1, cell_z0));
			auto const g001 = GradVec(Hash(cell_x0, cell_y0, cell_z1));
			auto const g101 = GradVec(Hash(cell_x1, cell_y0, cell_z1));
			auto const g011 = GradVec(Hash(cell_x0, cell_y1, cell_z1));
			auto const g111 = GradVec(Hash(cell_x1, cell_y1, cell_z1));
			auto const n000 = Dot(g000, x,     y,     z);
			auto const n100 = Dot(g100, x - 1, y,     z);
			auto const n010 = Dot(g010, x,     y - 1, z);
			auto const n110 = Dot(g110, x - 1, y - 1, z);
			auto const n001 = Dot(g001, x,     y,     z - 1);
			auto const n101 = Dot(g101, x - 1, y,     z - 1);
			auto const n011 = Dot(g011, x,     y - 1, z - 1);
			auto const n111 = Dot(g111, x - 1, y - 1, z - 1);

			// Differentiate the nested lerps in the same order as the value evaluation.
			auto const nx00 = Lerp(u, n000, n100);
			auto const nx10 = Lerp(u, n010, n110);
			auto const nx01 = Lerp(u, n001, n101);
			auto const nx11 = Lerp(u, n011, n111);

			auto const dnx00_dx = Lerp(u, g000[0], g100[0]) + du * (n100 - n000);
			auto const dnx10_dx = Lerp(u, g010[0], g110[0]) + du * (n110 - n010);
			auto const dnx01_dx = Lerp(u, g001[0], g101[0]) + du * (n101 - n001);
			auto const dnx11_dx = Lerp(u, g011[0], g111[0]) + du * (n111 - n011);
			auto const dnx00_dy = Lerp(u, g000[1], g100[1]);
			auto const dnx10_dy = Lerp(u, g010[1], g110[1]);
			auto const dnx01_dy = Lerp(u, g001[1], g101[1]);
			auto const dnx11_dy = Lerp(u, g011[1], g111[1]);
			auto const dnx00_dz = Lerp(u, g000[2], g100[2]);
			auto const dnx10_dz = Lerp(u, g010[2], g110[2]);
			auto const dnx01_dz = Lerp(u, g001[2], g101[2]);
			auto const dnx11_dz = Lerp(u, g011[2], g111[2]);

			auto const nxy0 = Lerp(v, nx00, nx10);
			auto const nxy1 = Lerp(v, nx01, nx11);

			auto const dnxy0_dx = Lerp(v, dnx00_dx, dnx10_dx);
			auto const dnxy1_dx = Lerp(v, dnx01_dx, dnx11_dx);
			auto const dnxy0_dy = Lerp(v, dnx00_dy, dnx10_dy) + dv * (nx10 - nx00);
			auto const dnxy1_dy = Lerp(v, dnx01_dy, dnx11_dy) + dv * (nx11 - nx01);
			auto const dnxy0_dz = Lerp(v, dnx00_dz, dnx10_dz);
			auto const dnxy1_dz = Lerp(v, dnx01_dz, dnx11_dz);

			auto const value = Lerp(w, nxy0, nxy1);
			auto const dx = Lerp(w, dnxy0_dx, dnxy1_dx);
			auto const dy = Lerp(w, dnxy0_dy, dnxy1_dy);
			auto const dz = Lerp(w, dnxy0_dz, dnxy1_dz) + dw * (nxy1 - nxy0);
			return PerlinNoiseSample<double>{ .m_value = value, .m_dx = dx, .m_dy = dy, .m_dz = dz };
		}

		// Hashes a lattice coordinate directly so the gradient field has no short table-defined period.
		uint32_t Hash(int64_t x, int64_t y, int64_t z) const
		{
			auto hash = uint64_t{m_seed} + 0x9E3779B97F4A7C15ull;
			hash ^= static_cast<uint64_t>(x) * 0x9E3779B185EBCA87ull;
			hash ^= static_cast<uint64_t>(y) * 0xC2B2AE3D27D4EB4Full;
			hash ^= static_cast<uint64_t>(z) * 0x165667B19E3779F9ull;

			// Avalanche all coordinate bits before selecting a gradient from the low bits.
			hash ^= hash >> 33;
			hash *= 0xFF51AFD7ED558CCDull;
			hash ^= hash >> 33;
			hash *= 0xC4CEB9FE1A85EC53ull;
			hash ^= hash >> 33;
			return static_cast<uint32_t>(hash);
		}

		// Wraps negative and positive lattice coordinates into one period.
		static int64_t Wrap(int64_t coordinate, int period)
		{
			auto const wrapped = coordinate % period;
			return wrapped >= 0 ? wrapped : wrapped + period;
		}

		// Produces a smooth interpolation weight for a coordinate within one lattice cell.
		template <typename S>
		static S Fade(S t)
		{
			return t * t * t * (t * (t * 6 - 15) + 10);
		}

		// Produces the derivative of Fade(t).
		template <typename S>
		static S dFade(S t)
		{
			return S(30) * t * t * (t * (t - S(2)) + S(1));
		}

		// Linearly interpolates from 'a' to 'b' by 't'.
		template <typename S>
		static S Lerp(S t, S a, S b)
		{
			return a + t * (b - a);
		}

		// Returns the deterministic gradient vector selected by the hash.
		static std::array<double, 3> GradVec(uint32_t hash)
		{
			auto const h = hash & 15;
			switch (h)
			{
				default:
				case 0:  return {+1.0, +1.0,  0.0};
				case 1:  return {-1.0, +1.0,  0.0};
				case 2:  return {+1.0, -1.0,  0.0};
				case 3:  return {-1.0, -1.0,  0.0};
				case 4:  return {+1.0,  0.0, +1.0};
				case 5:  return {-1.0,  0.0, +1.0};
				case 6:  return {+1.0,  0.0, -1.0};
				case 7:  return {-1.0,  0.0, -1.0};
				case 8:  return { 0.0, +1.0, +1.0};
				case 9:  return { 0.0, -1.0, +1.0};
				case 10: return { 0.0, +1.0, -1.0};
				case 11: return { 0.0, -1.0, -1.0};
				case 12: return {+1.0, +1.0,  0.0};
				case 13: return { 0.0, -1.0, +1.0};
				case 14: return {-1.0, +1.0,  0.0};
				case 15: return { 0.0, -1.0, -1.0};
			}
		}

		// Returns the corner ramp value for a local coordinate.
		template <typename S>
		static S Dot(std::array<double, 3> const& grad, S x, S y, S z)
		{
			return static_cast<S>(grad[0]) * x + static_cast<S>(grad[1]) * y + static_cast<S>(grad[2]) * z;
		}

		// Selects a deterministic corner gradient and returns its contribution toward the sample.
		template <typename S>
		static S Grad(uint32_t hash, S x, S y, S z)
		{
			return Dot(GradVec(hash), x, y, z);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::tests
{
	PRUnitTestClass(PerlinNoiseTests)
	{
		PRUnitTestMethod(NonPeriodicNoiseDoesNotRepeatAtTheLegacyTablePeriod, Quick)
		{
			PerlinNoiseGenerator noise(42);

			// Coordinate hashing must not retain the removed permutation table's visible repeat interval.
			auto const sample = noise.Noise(-0.25f, -3.75f, -12.5f);
			auto const legacy_repeat = noise.Noise(1023.75f, 1020.25f, 1011.5f);
			PR_EXPECT(std::abs(sample - legacy_repeat) > 1e-4f);
		}

		PRUnitTestMethod(PeriodicNoiseRepeatsSeamlessly, Quick)
		{
			PerlinNoiseGenerator noise(42);

			// Explicit periodic sampling must wrap consistently in every direction, including below zero.
			constexpr auto period = 37;
			auto const sample = noise.NoisePeriodic(-0.25f, 3.75f, -12.5f, period);
			auto const wrapped = noise.NoisePeriodic(-0.25f + period, 3.75f - period, -12.5f + period, period);
			PR_EXPECT(std::abs(sample - wrapped) < 1e-6f);
			PR_THROWS(noise.NoisePeriodic(0.0f, 0.0f, 0.0f, 0), std::invalid_argument);
		}

		PRUnitTestMethod(AnalyticDerivativesMatchFiniteDifferences, Quick)
		{
			PerlinNoiseGenerator noise(1337);
			auto const sample = noise.NoiseWithDerivatives(0.37, -1.23, 2.61);
			auto const step = 1.0e-5;
			auto const dx = (noise.NoiseWithDerivatives(0.37 + step, -1.23, 2.61).m_value - noise.NoiseWithDerivatives(0.37 - step, -1.23, 2.61).m_value) / (2.0 * step);
			auto const dy = (noise.NoiseWithDerivatives(0.37, -1.23 + step, 2.61).m_value - noise.NoiseWithDerivatives(0.37, -1.23 - step, 2.61).m_value) / (2.0 * step);
			auto const dz = (noise.NoiseWithDerivatives(0.37, -1.23, 2.61 + step).m_value - noise.NoiseWithDerivatives(0.37, -1.23, 2.61 - step).m_value) / (2.0 * step);
			PR_EXPECT(std::abs(sample.m_dx - dx) < 5.0e-5);
			PR_EXPECT(std::abs(sample.m_dy - dy) < 5.0e-5);
			PR_EXPECT(std::abs(sample.m_dz - dz) < 5.0e-5);
		}
	};
}
#endif
