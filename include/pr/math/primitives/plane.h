//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/functions.h"
#include "pr/math/types/vector4.h"

namespace pr::math
{
	template <ScalarTypeFP S>
	struct Plane3
	{
		// Notes:
		//  - Plane3.w should be positive if the normal faces the origin.
		//    Another way to think of it is, how far is the origin above the plane.
		//    Then, when using dot(plane, point), > 0 means above the plane.
		using Vec4 = Vec4<S>;

		Vec4 m_dir_dist; // xyz = direction, w = distance

		// Default construct to the plane x = 0
		constexpr Plane3() noexcept
			:m_dir_dist(0, 0, 1, 0)
		{}

		// Construct from raw dir_dist vector (xyz = direction, w = distance)
		constexpr explicit Plane3(Vec4 dir_dist) noexcept
			: m_dir_dist(dir_dist)
		{}

		// Construct from a direction and distance (direction not necessarily unit length)
		constexpr Plane3(S dx, S dy, S dz, S dist) noexcept
			: m_dir_dist(dx, dy, dz, dist)
		{}

		// Construct from a normal direction and distance
		constexpr Plane3(Vec4 norm, S dist) noexcept
		{
			auto p = norm;
			p.w = -dist;
			m_dir_dist = p;
		}

		// Construct from a point and a direction (not necessarily unit length)
		constexpr Plane3(Vec4 point, Vec4 direction) noexcept
		{
			auto p = direction;
			p.w = -Dot3(point, direction);
			m_dir_dist = p;
		}

		// Allow implicit conversion to Vec4
		constexpr operator Vec4() const noexcept
		{
			return m_dir_dist;
		}

		// Return the direction vector component of the plane
		constexpr Vec4 direction() const noexcept
		{
			return m_dir_dist.w0();
		}

		// Return the signed distance component of the plane
		constexpr S distance() const noexcept
		{
			return -m_dir_dist.w;
		}

		// Construct from 3 points in 3D space
		static Plane3 FromTriangle(Vec4 a, Vec4 b, Vec4 c) noexcept
		{
			auto p = Normalise(Cross(b-a, c-a));
			p.w = -Dot(a, p);
			return Plane3{ p };
		}

		// Make a best fit plane for a set of points. (designed for polygons really)
		// This is using Newell's method of projecting the points into the yz, xz, and xy planes
		static Plane3 FromBestFit(std::span<Vec4 const> points) noexcept
		{
			auto p = Vec4{};
			auto centre = Vec4{};
			auto begin = points.data();
			auto end = begin + points.size();
			for (Vec4 const* i = end - 1, *j = begin; j != end; i = j, ++j)
			{
				// Compute the normal as being proportional to the projected areas
				// of the polygon onto the yz, xz, and xy planes.
				// Also, compute centroid as representative point on the plane
				p.x += (i->y - j->y) * (i->z + j->z); // Projection onto YZ
				p.y += (i->z - j->z) * (i->x + j->x); // Projection onto XZ
				p.z += (i->x - j->x) * (i->y + j->y); // Projection onto XY
				centre += *j;
			}
			p = Normalise(p);
			p.w = Dot(centre, p) / (end - begin); // Centre / (end - begin) is the true centre
			return Plane3{ p };
		}
	};

	#pragma region Functions

	// Normalise (Canonicalise a plane)
	template <ScalarTypeFP S> constexpr Plane3<S> pr_vectorcall Normalise(Plane3<S> plane) noexcept
	{
		auto len = Length(plane.direction());
		return Plane3<S>(Vec4<S>(plane) / len); // This scales the w-component as well
	}

	// Return the signed distance of 'v' from the plane.
	template <ScalarTypeFP S> constexpr S pr_vectorcall Distance(Plane3<S> plane, Vec4<S> v) noexcept
	{
		return Dot(plane, v);
	}

	// Returns 'v' projected onto 'plane'
	// So if plane.w == -dist, if v.w == 1 the returned point will lie on the plane at 'dist'
	// from the origin. if v.w == 0, the returned vector will lie in a plane parallel to 'plane'.
	template <ScalarTypeFP S> constexpr Vec4<S> pr_vectorcall Project(Plane3<S> plane, Vec4<S> v) noexcept
	{
		return v - Dot(plane, v) * plane.direction();
	}

	#pragma endregion
}
