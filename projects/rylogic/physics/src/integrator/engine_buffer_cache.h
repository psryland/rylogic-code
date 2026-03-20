//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/diagnostics/body_history.h"
#include "src/compute/physics_types.h"
#include "src/collision/shape_cache.h"
#include "src/diagnostics/physics_log.h"

namespace pr::physics
{
	struct EngineBufferCache
	{
		// Persists across frames
		ShapeCache m_shape_cache;

		// Staging buffer for packing body dynamics
		std::vector<GpuRigidBody> m_rb_dynamics;

		// Diagnostics
		#if PR_DBG_PHYSICS
		BodyHistory m_history;
		PhysicsLog m_log;
		#endif
	};
		
	inline void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}
}
