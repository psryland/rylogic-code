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
#include <cstdint>
#include <cmath>
#include <stdexcept>

namespace pr::algorithm
{
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

		// Samples a seamlessly repeating noise field whose period is measured in lattice cells on every axis.
		float NoisePeriodic(float x, float y, float z, int period) const
		{
			if (period <= 0)
				throw std::invalid_argument("Perlin noise period must be greater than zero");

			return NoiseImpl(x, y, z, period);
		}

	private:

		// Samples one cube, optionally wrapping its lattice coordinates to create a seamless period.
		float NoiseImpl(float x, float y, float z, int period) const
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
		static float Fade(float t)
		{
			return t * t * t * (t * (t * 6 - 15) + 10);
		}

		// Linearly interpolates from 'a' to 'b' by 't'.
		static float Lerp(float t, float a, float b)
		{
			return a + t * (b - a);
		}

		// Selects a deterministic corner gradient and returns its contribution toward the sample.
		static float Grad(uint32_t hash, float x, float y, float z)
		{
			auto const h = hash & 15; // Convert the lower 4 bits of the hash code into one of 12 gradient directions
			auto const u = h < 8 ? x : y;
			auto const v = h < 4 ? y : h == 12 || h == 14 ? x : z;
			return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
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
	};
}
#endif
