//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_RAY_H
#define PR_PHYSICS_RAY_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		// A representation of a ray
		struct Ray
		{
			v4		m_point;
			v4		m_direction;
			float	m_thickness;

			Ray()                                                      : m_thickness(0.0f) {}
			Ray(v4 point, v4 direction)                  : m_point(point), m_direction(direction), m_thickness(0.0f) {}
			Ray(v4 point, v4 direction, float thickness) : m_point(point), m_direction(direction), m_thickness(thickness) {}
		};
		inline Ray operator * (m4x4 const& lhs, Ray const& rhs) { return Ray(lhs*rhs.m_point, lhs*rhs.m_direction, rhs.m_thickness); }

	}
}

#endif

