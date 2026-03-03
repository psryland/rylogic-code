//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector4.h"

namespace pr
{
	// Notes:
	//  - Plane.w should be positive if the normal faces the origin.
	//    Another way to think of it is, how far is the origin above the plane.
	//    Then, when using dot(plane, point), > 0 means above the plane.
	struct Plane : Vec4f<struct PlaneType>
	{
		Plane() = default;
		Plane(float dx, float dy, float dz, float dist)
			: Vec4f<struct PlaneType>{dx, dy, dz, dist}
		{}
		Plane(v4 const& v)
			: Vec4f<struct PlaneType>{v}
		{}
		v4 direction() const { return static_cast<v4>(w0()); }
	};

	// Define the dot product for planes
	inline float Dot(Plane plane, v4 const& rhs)
	{
		return Dot4(static_cast<v4>(plane), rhs);
	}

	namespace plane
	{
		// Create a plane from components
		inline Plane make(float dx, float dy, float dz, float dist)
		{
			return Plane{dx, dy, dz, dist};
		}

		// Create a point from a point and direction (not necessarily unit length)
		inline Plane make(v4 const& point, v4 const& direction)
		{
			auto p = direction;
			p.w = -Dot3(point, direction);
			return Plane{p};
		}

		// Create a point from 3 points in 3D space
		inline Plane make(v4 const& a, v4 const& b, v4 const& c)
		{
			auto p = Normalise(Cross3(b-a, c-a));
			p.w = -Dot3(a, p);
			return Plane{p};
		}

		// Create from a normal direction and distance
		inline Plane make(v4 const& norm, float dist)
		{
			auto p = norm;
			p.w = -dist;
			return Plane{p};
		}

		// Make a best fit plane for a set of points. (designed for polygons really)
		// This is using Newell's method of projecting the points into the yz, xz, and xy planes
		inline Plane make(v4 const* begin, v4 const* end)
		{
			auto p = v4{};
			auto centre = v4{};
			for (v4 const *i = end - 1, *j = begin; j != end; i = j, ++j)
			{
				// Compute the normal as being proportional to the projected areas
				// of the polygon onto the yz, xz, and xy planes.
				// Also, compute centroid as representative point on the plane
				p.x += (i->y - j->y) * (i->z + j->z); // Projection onto YZ
				p.y += (i->z - j->z) * (i->x + j->x); // Projection onto XZ
				p.z += (i->x - j->x) * (i->y + j->y); // Projection onto XY
				centre  += *j;
			}
			p = Normalise(p);
			p.w = Dot4(centre, p) / (end - begin);	// Centre / (end - begin) is the true centre
			return Plane{p};
		}

		// Normalise (Canonicalise a plane)
		inline Plane Normalise(Plane plane)
		{
			return Plane(plane / Length(plane.xyz)); // This scales the w-component as well
		}

		// Return the direction vector component of a plane
		inline v4 Direction(Plane plane)
		{
			return static_cast<v4>(plane.w0());
		}

		// Return the distance component of a plane
		inline float Distance(Plane plane)
		{
			return -plane.w;
		}

		// Return the signed distance of 'v' from the plane.
		inline float Distance(Plane plane, v4 const& v)
		{
			return Dot(plane, v);
		}

		// Returns 'v' projected onto 'plane'
		// So if plane.w == -dist, if v.w == 1 the returned point will lie on the plane at 'dist'
		// from the origin. if v.w == 0, the returned vector will lie in a plane parallel to 'plane'.
		inline v4 Project(Plane plane, v4 const& v)
		{
			return v - Dot(plane, v) * Direction(plane);
		}
	}
}
