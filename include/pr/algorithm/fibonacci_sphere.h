//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/maths.h"

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

	// Inverse mapping from a spherical direction vector to the nearest point of a Fibonacci sphere
	inline int FibonacciSphericalMapping(v4 dir, int N)
	{
		// Notes:
		//  - If N points are distributed evenly over the sphere, then each point can be associated with
		//    an equal amount of surface area, equal to:
		//      patch_area = sphere_surface_area / N = (2*tau*r^2) / N.
		//  - Approximating each spherical patch as a circle of equal area means we can estimate the
		//    distance between points:
		//      patch_area = circle_area = 0.5 * tau * patch_radius^2
		//      patch_radius = sqrt(2.0 * patch_area / tau)
		//  - The patch circle gives us a range on the Z-axis that should contain the nearest point.
		//      radius_at_z = sqrt(1 - z*z)
		//      dz = patch_radius * radius_at_z
		//  - The phase angle of the i'th point is:
		//      i * golden_angle (mod tau)
		//  - The range of phase angles for the patch centred on 'dir' is:
		//      tang = cross(ZAxis, dir) / radius_at_z
		//      dir0 = dir - patch_radius * tang
		//      dir1 = dir + patch_radius * tang
		//      phase_range = [atan2(dir0.y, dir0.x), atan2(dir1.y, dir1.x)] (mod tau)
		//  - Iterate over the values of 'i' that satisfy:
		//      i * golden_angle >=< [phase_range] (mod tau) (i.e. i's that fall within the phase range)

		// Find the patch on the sphere that contains the nearest point
		constexpr double Inflate = 1.5;
		auto patch_area = 2.0 * constants<double>::tau / N;
		auto patch_radius = Sqrt(2.0 * patch_area / constants<double>::tau) * Inflate;
		auto radius_at_z = Sqrt(1.0 - Sqr(dir.z));
		auto dz = std::max(patch_radius * radius_at_z, 0.0001);

		// Find the phase range of the patch
		auto tang = Cross(v4::ZAxis(), dir) / static_cast<float>(radius_at_z);
		auto dir0 = dir - static_cast<float>(patch_radius) * tang;
		auto dir1 = dir + static_cast<float>(patch_radius) * tang;
		auto phase0 = fmod((atan2(dir0.y, dir0.x) + constants<double>::tau), constants<double>::tau);
		auto phase1 = fmod((atan2(dir1.y, dir1.x) + constants<double>::tau), constants<double>::tau);
		auto phase_span = phase1 - phase0;
		if (phase_span < 0) phase_span += constants<double>::tau;

		// Get the Fibonacci sphere index range to search
		auto ZtoI = [=](double z) { return (int)Lerp<double>(0.0, N, Frac(-1.0, z, +1.0)); };
		auto i0 = ZtoI(std::max(-1.0, dir.z - dz));
		auto i1 = ZtoI(std::min(+1.0, dir.z + dz));

		auto nearest = -1;
		auto distsq = limits<double>::infinity();
		auto phase = fmod(i0 * constants<double>::golden_angle, constants<double>::tau);
		for (auto i = i0; i != i1; ++i)
		{
			auto dphase = phase - phase0;
			if (dphase < 0) dphase += constants<double>::tau;
			if (dphase < phase_span)
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
			phase -= (phase > constants<double>::tau) * constants<double>::tau;
		}
		
		// Exception here means no points fell within the search patch. It probably means 'Inflate' needs to be bigger
		return nearest != -1 ? nearest : throw std::runtime_error("No nearest point found in fibonacci sphere mapping");
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::tests
{
	PRUnitTest(FibonacciSphereTests)
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
				auto pt = v4::RandomN(rng, 0.0f);
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
	}
}
#endif
