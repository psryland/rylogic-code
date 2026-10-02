//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

namespace pr::algorithm
{
	// Notes:
	//  - A Fibonacci sphere is basically a spiral from (0,0,-1) to (0,0,+1). Over evenly distributed
	//    z-steps from -1 to +1, the phase angle moves in steps of the 'golden_angle' (~137.5 degrees).
	// 
	// Future:
	//  - It should be possible to algorithmically determine the adjacent points and create quads that
	//    cover the sphere.
	//  - If I knew the adjacency, it would probably be possible to make the 'unmapping' faster.

	// Returns a spherical direction vector corresponding to the ith point of a Fibonacci sphere
	inline v4 FibonacciSphericalMapping(int i, int N)
	{
		pr_assert(i >= 0 && i < N && "index value out of range");

		// Z goes from -1 to +1
		// Using a half step bias so that there is no point at the poles.
		// This prevents degenerates during 'unmapping' and also results in more evenly
		// spaced points. See "Fibonacci grids: A novel approach to global modelling".
		auto z = -1.0 + (2.0 * i + 1.0) / N;

		// Radius at z
		auto r = sqrt(1.0 - z * z);

		// Golden angle increment
		auto theta = i * constants<double>::golden_angle;
		auto x = cos(theta) * r;
		auto y = sin(theta) * r;
		return v4{(float)x, (float)y, (float)z, 0};
	}

	// Inverse mapping from a spherical direction vector to the nearest point of a Fibonacci sphere. 'dir' must be normalised.
	inline int FibonacciSphericalMapping(v4 dir, int N)
	{
		// Notes:
		//  - The search region is a spherical cap of angular radius 'cap' centred on 'dir'. The z range and the
		//    phase range that contain the cap can be calculated exactly, so only those points are tested.
		//  - Point 'i' has z = -1 + (2i+1)/N, so a z range maps directly to an index range.
		//  - The phase angle of the i'th point is: i * golden_angle (mod tau).
		//  - The nearest point found within the cap is the true nearest point only if it is no further than 'cap'.
		//    Otherwise a closer point might lie just outside the cap, so the cap is doubled and the search repeated.
		//    A cap of 'tau/2' covers the whole sphere, so the loop always ends.
		constexpr double tau = constants<double>::tau;
		auto polar = acos(std::clamp<double>(dir.z, -1.0, +1.0));
		auto azimuth = fmod(atan2(dir.y, dir.x) + tau, tau);

		// Start with a cap a bit larger than the typical spacing between points
		for (auto cap = 1.5 * Sqrt(4.0 / N);; cap *= 2.0)
		{
			// Find the index range of the points within the z range of the cap
			auto z_min = cos(std::min(tau / 2, polar + cap));
			auto z_max = cos(std::max(0.0, polar - cap));
			auto i0 = std::clamp(static_cast<int>(floor((N * (z_min + 1.0) - 1.0) / 2.0)), 0, N);
			auto i1 = std::clamp(static_cast<int>(ceil((N * (z_max + 1.0) - 1.0) / 2.0)) + 1, 0, N);

			// Find the phase half-width of the cap. If the cap contains a pole, it covers all phases.
			// Otherwise 'sin(cap) < sin(polar)', so the ratio is less than one.
			auto contains_pole = polar - cap <= 0.0 || polar + cap >= tau / 2;
			auto half_width = contains_pole ? tau : asin(sin(cap) / sin(polar));

			// Test the points that fall within the phase range
			auto nearest = -1;
			auto distsq = limits<double>::infinity();
			auto phase = fmod(i0 * constants<double>::golden_angle, tau);
			for (auto i = i0; i != i1; ++i)
			{
				// Phase difference wrapped into [-tau/2, +tau/2]. Both phases are in [0, tau).
				auto dphase = phase - azimuth;
				dphase += (dphase < -tau / 2) * tau - (dphase > tau / 2) * tau;
				if (Abs(dphase) <= half_width)
				{
					auto p = FibonacciSphericalMapping(i, N);
					auto d = LengthSq(p - dir);
					if (d < distsq)
					{
						nearest = i;
						distsq = d;
					}
				}

				phase += constants<double>::golden_angle;
				phase -= (phase >= tau) * tau;
			}

			// Accept the nearest point if no closer point can lie outside the cap. Convert the chord length to an angle to compare with 'cap'.
			if (nearest != -1 && 2.0 * asin(std::min(1.0, Sqrt(distsq) / 2.0)) <= cap)
				return nearest;
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::tests
{
	PRUnitTest(FibonacciSphereTests, Stress)
	{
		{// Test round trip of fib points
			constexpr int N = 65536;
			for (int i = 0; i != N; ++i)
			{
				auto pt = FibonacciSphericalMapping(i, N);
				auto idx = FibonacciSphericalMapping(pt, N);
				PR_EXPECT(idx == i);
			}
		}
		{// Test random sampling
			constexpr int N = 65536;
			std::default_random_engine rng(5);
	
			auto max_dist = 0.0;
			auto max_i = -1;
			for (int i = 0; i != N; ++i)
			{
				auto pt = RandomN<v3>(rng).w0();
				auto idx = FibonacciSphericalMapping(pt, N);
				auto fpt = FibonacciSphericalMapping(idx, N);
				auto dist = Length(fpt - pt);
				if (dist > max_dist)
				{
					max_dist = dist;
					max_i = i;
				}
			}
			PR_EXPECT(max_dist < 0.02f);
		}
		{// Compare with a brute force search, including the poles and small point counts
			auto BruteForce = [](v4 dir, int N)
			{
				// Test every point
				auto nearest = -1;
				auto distsq = limits<float>::infinity();
				for (int i = 0; i != N; ++i)
				{
					auto d = LengthSq(FibonacciSphericalMapping(i, N) - dir);
					if (d < distsq)
					{
						nearest = i;
						distsq = d;
					}
				}
				return nearest;
			};

			std::default_random_engine rng(1);
			for (auto N : { 1, 2, 10, 100, 1000 })
			{
				// Directions at and near the poles, then random directions
				std::vector<v4> dirs = { v4::ZAxis(), -v4::ZAxis(), Normalise(v4{0.001f, 0, 1, 0}), Normalise(v4{0, -0.001f, -1, 0}) };
				for (int i = 0; i != 1000; ++i)
					dirs.push_back(RandomN<v3>(rng).w0());

				for (auto dir : dirs)
				{
					auto idx = FibonacciSphericalMapping(dir, N);
					auto best = BruteForce(dir, N);
					PR_EXPECT(FEql(LengthSq(FibonacciSphericalMapping(idx, N) - dir), LengthSq(FibonacciSphericalMapping(best, N) - dir)));
				}
			}
		}
	}
}
#endif
