//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/shape.h"

namespace pr::collision
{
	struct Ray
	{
		v4    m_point;     // The origin of the ray
		v4    m_direction; // The directory of the ray away from the origin
		float m_thickness; // The thickness of the ray

		Ray()
			:m_thickness(0.0f)
		{}
		Ray(v4 point, v4 direction)
			:m_point(point)
			,m_direction(direction)
			,m_thickness(0.0f)
		{}
		Ray(v4 point, v4 direction, float thickness)
			:m_point(point)
			,m_direction(direction)
			,m_thickness(thickness)
		{}

		// Operators
		friend Ray operator * (m4x4 const& lhs, Ray const& rhs)
		{
			return Ray(lhs*rhs.m_point, lhs*rhs.m_direction, rhs.m_thickness);
		}
	};
}
