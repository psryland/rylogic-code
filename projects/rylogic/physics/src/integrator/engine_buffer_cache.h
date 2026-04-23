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

		// Staging buffer for collision contacts
		std::vector<GpuResolveContact> m_contacts;

		// Contact information in CPU format
		std::vector<RbContact> m_contacts_cpu;

		// Diagnostics
		#if PR_DBG_PHYSICS
		BodyHistory m_history;
		PhysicsLog m_log;
		#endif

		EngineBufferCache()
			: m_shape_cache()
			, m_rb_dynamics()
			, m_contacts()
			, m_contacts_cpu()
			#if PR_DBG_PHYSICS
			, m_history()
			, m_log()
			#endif
		{}

		void NewFrame(std::span<RigidBody*> rigid_bodies, int max_contacts)
		{
			// Purge unused shapes from the cache
			m_shape_cache.BeginFrame();
			
			// Reset the GPU staging buffer for body dynamics.
			m_rb_dynamics.resize(0);

			// Reset the GPU staging buffer for contacts.
			m_contacts.resize(0);
			m_contacts.resize(max_contacts);

			#if PR_DBG_PHYSICS
			m_history.BeginFrame(rigid_bodies);
			#endif
		}
	};
}
