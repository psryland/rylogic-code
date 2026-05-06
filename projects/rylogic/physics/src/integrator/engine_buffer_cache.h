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

		// Staging buffer for sleeping islands and the frame-local dense-id mapping.
		std::vector<GpuSleepIsland> m_sleep_islands;
		std::vector<int> m_sleep_gpu_to_cpu_island_id;
		std::unordered_map<int, int> m_sleep_cpu_to_gpu_island_id;
		int m_next_sleep_island_id;

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
			, m_sleep_islands()
			, m_sleep_gpu_to_cpu_island_id()
			, m_sleep_cpu_to_gpu_island_id()
			, m_next_sleep_island_id()
			#if PR_DBG_PHYSICS
			, m_history()
			, m_log()
			#endif
		{}

		// Prepare for a new Engine::Step()
		void NewFrame(std::span<RigidBody*> rigid_bodies, int max_contacts)
		{
			// Purge unused shapes from the cache
			m_shape_cache.BeginFrame();
			
			// Reset the GPU staging buffer for body dynamics.
			m_rb_dynamics.resize(0);

			// Reset the GPU staging buffer for contacts.
			if (std::ssize(m_contacts) < max_contacts)
				m_contacts.resize(max_contacts);

			// Reset per-frame sleeping-island staging.
			m_sleep_islands.resize(0);
			m_sleep_gpu_to_cpu_island_id.resize(0);
			m_sleep_cpu_to_gpu_island_id.clear();

			#if PR_DBG_PHYSICS
			m_history.BeginFrame(rigid_bodies);
			#else
			(void)rigid_bodies;
			#endif
		}

		// Drop all cached state.
		void Reset()
		{
			// Called when the previously-seen shapes/bodies may no longer be valid (e.g. between independent unit-test scenarios that share a single engine).
			m_shape_cache.Reset();
			m_rb_dynamics.clear();
			m_contacts.clear();
			m_contacts_cpu.clear();
			m_sleep_islands.clear();
			m_sleep_gpu_to_cpu_island_id.clear();
			m_sleep_cpu_to_gpu_island_id.clear();
			m_next_sleep_island_id = 0;

			#if PR_DBG_PHYSICS
			m_history = BodyHistory{};
			#endif
		}

		// Number of cached rigid bodies staged for GPU upload this frame.
		int RigidBodyCount() const
		{
			return static_cast<int>(m_rb_dynamics.size());
		}

		// Maximum number of contacts that can be read back from the GPU this frame.
		int MaxContactsCount() const
		{
			return static_cast<int>(m_contacts.size());
		}

		// Number of dense frame-local GPU sleep islands staged this frame.
		int SleepIslandCount() const
		{
			return static_cast<int>(m_sleep_islands.size());
		}

		// Map a sleeping dynamic body onto a dense GPU island and grow that island's bounds.
		void PackSleepIsland(RigidBody& body, GpuRigidBody& dyn)
		{
			dyn.sleep.island_id = -1;
			if (!body.Sleeping() || body.NeverSleep() || body.InvMass() <= 0.0f || AllSet(body.m_state_flags, ERigidBodyStateFlags::Static))
				return;

			if (body.m_sleep.m_island_id < 0)
			{
				body.m_sleep.m_island_id = m_next_sleep_island_id++;
			}
			else
			{
				m_next_sleep_island_id = std::max(m_next_sleep_island_id, body.m_sleep.m_island_id + 1);
			}

			auto const cpu_id = body.m_sleep.m_island_id;
			auto const [iter, inserted] = m_sleep_cpu_to_gpu_island_id.try_emplace(cpu_id, static_cast<int>(m_sleep_islands.size()));
			auto const gpu_id = iter->second;
			if (inserted)
			{
				m_sleep_gpu_to_cpu_island_id.push_back(cpu_id);
				m_sleep_islands.push_back(GpuSleepIsland{
					.bbox_ws = body.BBoxWS(),
					.flags = GpuSleepIslandFlags_Valid | GpuSleepIslandFlags_Sleeping,
					.body_count = 1,
					.generation = body.m_sleep.m_generation,
					.pad0 = 0,
				});
			}
			else
			{
				auto& island = m_sleep_islands[gpu_id];
				island.bbox_ws = Union(island.bbox_ws, body.BBoxWS());
				island.body_count += 1;
				island.generation = std::max(island.generation, body.m_sleep.m_generation);
			}

			dyn.sleep.island_id = gpu_id;
		}

		// Convert frame-local GPU island ids back to persistent CPU ids before unpacking.
		void UnpackSleepIsland(GpuRigidBody& dyn) const
		{
			auto const gpu_id = dyn.sleep.island_id;
			if (gpu_id < 0)
				return;

			assert(gpu_id < std::ssize(m_sleep_gpu_to_cpu_island_id));
			if (gpu_id >= std::ssize(m_sleep_gpu_to_cpu_island_id))
			{
				dyn.sleep.island_id = -1;
				return;
			}

			dyn.sleep.island_id = m_sleep_gpu_to_cpu_island_id[gpu_id];
		}
	};
}
