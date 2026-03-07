//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include "pr/collision/forward.h"

namespace pr::collision
{
	struct RayCastResult
	{
		v4 m_normal;          // The normal of the incident face
		Shape const* m_shape; // The primitive that was hit or nullptr
		float m_t0, m_t1;     // The parametric range of the portion of the ray inside the shape

		RayCastResult()
			:m_normal()
			,m_shape()
			,m_t0()
			,m_t1()
		{}
	};
}
