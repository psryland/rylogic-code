//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine.h"
#include "pr/physics/integrator/engine_config.h"
#include "src/integrator/engine_buffer_cache.h"
#include "src/collision/shape_cache.h"

namespace pr::physics
{
	// Debugging utilities for inspecting GPU results.
	struct DbgPhysics
	{
		Engine& me;
		DbgPhysics(Engine& engine) : me(engine)
		{
			if constexpr (PR_PHYSICS_DIAGNOSTICS)
			{
				if (!me.m_cache->m_log.IsActive() && me.m_cache->m_log.m_frame == 0)
					me.m_cache->m_log.Open("dump\\physics_pipeline.log");
			}
		}

		// Debugging inspection of GPU results.
		void ReadbackIntegrate(int body_count)
		{
			if constexpr (PR_PHYSICS_DIAGNOSTICS)
			{
				auto& log = me.m_cache->m_log;
				if (log.IsActive())
				{
					std::vector<GpuRigidBody> bodies(body_count);
					std::vector<BBox> aabbs(body_count);
					me.m_gpu_integrator->Readback(me.m_gpu->m_job, bodies, aabbs);
					log.LogIntegrate(body_count, 0, bodies, aabbs);
				}
			}
			else
			{
				(void)body_count;
			}
		}
		void ReadbackSweep(D3DPtr<ID3D12Resource> counters)
		{
			if constexpr (PR_PHYSICS_DIAGNOSTICS)
			{
				auto& log = me.m_cache->m_log;
				if (log.IsActive())
				{
					std::vector<GpuCollisionPair> out_pairs(me.m_config.max_collision_pairs);
					auto pairs = me.m_gpu_sort_and_sweep->Readback(me.m_gpu->m_job, counters, out_pairs);
					assert(pairs.size() < me.m_config.max_collision_pairs && "Hit capacity on pairs");
					log.LogBroadphase(pairs);
				}
			}
			else
			{
				(void)counters;
			}
		}
		void ReadbackCollide(D3DPtr<ID3D12Resource> counters)
		{
			if constexpr (PR_PHYSICS_DIAGNOSTICS)
			{
				auto& log = me.m_cache->m_log;
				if (log.IsActive())
				{
					std::vector<GpuResolveContact> out_contacts(me.m_config.max_collision_pairs);
					auto contacts = me.m_gpu_collision_detector->Readback(me.m_gpu->m_job, counters, out_contacts);
					assert(contacts.size() < me.m_config.max_collision_pairs && "Hit capacity on contacts");
					log.LogNarrowPhase(contacts);
					log.EndFrame();
				}
			}
			else
			{
				(void)counters;
			}
		}
		void ReadbackResolve(int body_count, D3DPtr<ID3D12Resource> r_bodies)
		{
			if constexpr (PR_PHYSICS_DIAGNOSTICS)
			{
				static bool capture = false;
				if (capture)
				{
					std::vector<GpuRigidBody> bodies(body_count);
					me.m_gpu_resolver->Readback(me.m_gpu->m_job, r_bodies, bodies);
				}
			}
			else
			{
				(void)body_count, r_bodies;
			}
		}
	};
}
