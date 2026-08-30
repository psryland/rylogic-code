//*****************************************************************************
// Perlin Noise Generator
// Coherent noise function over 3 dimensions
// (copyright Ken Perlin) - This is the improved version
//*****************************************************************************
// Usage:
//  std::default_random_engine rng(seed);
//  PerlinNoiseGenerator perlin(rng);
//  float x,y,z = sample coordinates;
//  float freq = the 'frequency' of the noise
//  float amp = the amplitude of the noise
//  float offset = bias for the noise.
//  [-1, 1] * amp + offset = perlin.Noise(x * freq, y * freq, z * freq) * amp + offset;

#pragma once
#include <random>
#include <algorithm>
#include <cmath>

namespace pr::algorithm
{
	template <typename Rng = std::default_random_engine>
	class PerlinNoiseGenerator
	{
		enum
		{
			PermTableSize = 1 << 10,
			PermTableMask = PermTableSize - 1,
		};

		int  m_perm[PermTableSize * 2];

	public:

		explicit PerlinNoiseGenerator(Rng& rng)
		{
			// Can also use this pre-generated one if you want...
			//const int table[PermTableSize] =
			//{
			//	151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
			//	140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
			//	247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
			//	 57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
			//	 74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
			//	 60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
			//	 65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
			//	200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
			//	 52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
			//	207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
			//	119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
			//	129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
			//	218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
			//	 81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
			//	184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
			//	222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180
			//};

			// Generate a permutation table
			for (int i = 0; i != PermTableSize; ++i)
				m_perm[i] = i;

			// Give every permutation equal probability so seeds do not favour particular gradient layouts.
			std::shuffle(m_perm, m_perm + PermTableSize, rng);
			for (int i = 0; i != PermTableSize; ++i)
				m_perm[i+PermTableSize] = m_perm[i];
		}

		float Noise(float x, float y, float z) const
		{
			// Pick valid points within the permutation table
			int X = static_cast<int>(std::floor(x)) & PermTableMask;
			int Y = static_cast<int>(std::floor(y)) & PermTableMask;
			int Z = static_cast<int>(std::floor(z)) & PermTableMask;
			
			// Find the relative x, y, z of the point in the cube
			x -= std::floor(x);
			y -= std::floor(y);
			z -= std::floor(z);
			
			// Compute the Fade curves for each axis
			float u = Fade(x);
			float v = Fade(y);
			float w = Fade(z);

			// Hash the coordinates of the 8 cube corners
			int A = m_perm[X    ] + Y, AA = m_perm[A] + Z, AB = m_perm[A + 1] + Z;
			int B = m_perm[X + 1] + Y, BA = m_perm[B] + Z, BB = m_perm[B + 1] + Z;

			// Add the blended results from the 8 corners of the cube
			return  Lerp(w, Lerp(v, Lerp(u, Grad(m_perm[AA  ], x  , y  , z   ),
											Grad(m_perm[BA  ], x-1, y  , z   )),
									Lerp(u, Grad(m_perm[AB  ], x  , y-1, z   ),
											Grad(m_perm[BB  ], x-1, y-1, z   ))),
							Lerp(v, Lerp(u, Grad(m_perm[AA+1], x  , y  , z-1 ),
											Grad(m_perm[BA+1], x-1, y  , z-1 )),
									Lerp(u, Grad(m_perm[AB+1], x  , y-1, z-1 ),
											Grad(m_perm[BB+1], x-1, y-1, z-1 ))));
		}

	private:

		float Fade(float t) const
		{
			return t * t * t * (t * (t * 6 - 15) + 10);
		}
		float Lerp(float t, float a, float b) const
		{
			return a + t * (b - a);
		}
		float Grad(int hash, float x, float y, float z) const
		{
			int h = hash & 15; // Convert the lower 4 bits of the hash code into one of 12 gradient directions
			float u = h < 8 ? x : y;
			float v = h < 4 ? y : h == 12 || h == 14 ? x : z;
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
		PRUnitTestMethod(NegativeCoordinatesUseTheContainingCell, Quick)
		{
			std::default_random_engine rng(42);
			PerlinNoiseGenerator noise(rng);

			// The permutation table makes noise periodic, including coordinates below zero.
			constexpr auto period = 1024.0f;
			auto const negative = noise.Noise(-0.25f, -3.75f, -12.5f);
			auto const wrapped = noise.Noise(-0.25f + period, -3.75f + period, -12.5f + period);
			PR_EXPECT(std::abs(negative - wrapped) < 1e-6f);
		}
	};
}
#endif
