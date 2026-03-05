//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_GLOBAL_FUNCTIONS_H
#define PR_PHYSICS_GLOBAL_FUNCTIONS_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		// Transform an inertia tensor using the parallel axis theorem.
		// 'offset' is the distance from (or toward) the centre of mass (determined by 'translate_type')
		// 'inertia' and 'offset' must be in the same frame.
		namespace ParallelAxisTranslate { enum Type { TowardCoM, AwayFromCoM }; }
		void ParallelAxisTranslateInertia(m3x4& inertia, pr::v4 offset, float mass, ParallelAxisTranslate::Type translate_type);
		
		// Create inertia tensors
		inline m3x4 InertiaTensorWS(m3x4 const& orientation, m3x4 const& os_inertia_tensor)
		{
			return orientation * os_inertia_tensor * Transpose(orientation);
		}
		inline m3x4 InvInertiaTensorWS(m3x4 const& orientation, m3x4 const& os_inv_inertia_tensor)
		{
			return orientation * os_inv_inertia_tensor * Transpose(orientation);
		}
	}
}

#endif
