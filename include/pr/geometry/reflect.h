//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/common/cast.h"
#include "pr/geometry/common.h"
#include "pr/common/fmt.h"

namespace pr
{
	// Reflect a direction vector through a plane
	inline v4 pr_vectorcall Reflect_Direction(v4 plane, v4 direction)
	{
		return direction - 2 * Dot(plane, direction) * plane.w0();
	}

	//TODO
	/*
	// Clip a line segment against a plane
	inline float pr_vectorcall Clip_LineToPlane(v4 plane, v4 point, v4 direction)
	{
		auto d = Dot(plane, direction);
		if (d == 0.0f) return 0.0f;
		auto p = Dot(plane, point);
		return -p / d;
	}


	//// Reflect a ray (of length 't') off a plane ('t' is an in/out parameter).
	//// Returns true if the ray intersects the plane. On return, 't' is the distance from the
	//// initial position to the intercept. Rays that point out of the plane do not reflect.
	//inline bool pr_vectorcall Reflect_RayToPlane(v4 plane, v4 point, v4 direction, float& t)
	//{
	//	assert(point.w == 1.0f);
	//	assert(direction.w == 0.0f);

	//	auto d = Dot(plane, direction);

	//	// Ray is pointing out of the plane, or is parallel => no intersection
	//	if (d >= 0.0f)
	//		return false;

	//	auto p = Dot(plane, point);

	//	// If the ray starts below the plane, then the intersect is at t = 0
	//	if (p <= 0.0f)
	//	{
	//		t = 0.0f;
	//		return true;
	//	}

	//	// If the ray starts above the plane, then the intersect is at t = -p/d
	//	auto intercept = -p / d;
	//	if (intercept < t)
	//	{
	//		t = intercept;
	//		return true;
	//	}

	//	return false;
	//};

	// Reflect 'ray' (of length 'length') off a collection of planes, to calculate the final position.
	// Returns the final position (equivalent to point travelling a distance of 'distance')
	// and the direction that the ray is travelling at the final position.
	inline bool pr_vectorcall Reflect_RayToPlanes(std::span<v4 const> planes, v4 point, v4 direction, float length, v4& out_position, v4& out_direction)
	{
		out_position = point;
		out_direction = direction;
		auto length_remaining = length;
		auto t = length;

		// Find the plane that intersects the 'ray' with the smallest 't' value.
		v4 const* nearest = nullptr;
		for (auto& plane : planes)
		{
			auto t = Clip_LineToPlane(plane, out_position, out_direction);



			if (Reflect_RayToPlane(plane, out_position, out_direction, t))
			{
				nearest = &plane;
			}
		}





		// Keep cycling through the planes until all report no intersection.
		// 'p' is the rolling plane index, 'loops' is the number of loops since an intersection.
		// When we've tested all planes and none have an interestion, then we're done.
		auto length_remaining = length;
		int p = 0, pcount = isize(planes);
		for (int loops = 0; loops != pcount; ++p)
		{
			auto& plane = planes[p % pcount];

			auto t = length_remaining;
			if (!Reflect_RayToPlane(plane, out_position, out_direction, t))
			{
				loops++;
				continue;
			}

			// We've hit a plane, so update the position and direction
			out_position += t * out_direction;
			out_direction = Reflect_Direction(plane, out_direction);
			length_remaining -= t;
			loops = 0;
		}

		// p == pcount only if all planes have been tested and none have an intersection
		return p == pcount;
	}
	*/
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry
{
	PRUnitTest(ReflectionTests)
	{
		/*
		constexpr v4 Walls[] = { v4::YAxis(), v4(1, 0, 0, 1), v4(-1, 0, 0, -1), v4(0, 0, 1, 1), v4(0, 0, -1, -1) };
		
		auto position = v4(2, 2, 2, 1);
		auto direction = v4(1, 1, 1, 0);
		auto distance = 10.0f;

		v4 out_position, out_direction;
		auto reflected = Reflect_RayToPlanes(Walls, position, direction, distance, out_position, out_direction);
		PR_EXPECT(reflected);
		*/
	}
}
#endif