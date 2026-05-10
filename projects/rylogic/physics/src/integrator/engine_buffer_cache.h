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
	struct NullBodyHistory
	{
		template <typename... Args> void BeginFrame(Args&&...)
		{
		}
		template <typename... Args> void EndFrame(Args&&...)
		{
		}
		void Reset()
		{
		}
	};
	struct NullPhysicsLog
	{
		int m_frame = 0;

		template <typename... Args> void Open(Args&&...)
		{
		}
		void Close()
		{
		}
		bool IsActive() const
		{
			return false;
		}
		template <typename... Args> void LogIntegrate(Args&&...)
		{
		}
		template <typename... Args> void LogBroadphase(Args&&...)
		{
		}
		template <typename... Args> void LogNarrowPhase(Args&&...)
		{
		}
		void EndFrame()
		{
		}
	};
	using BodyHistoryT = std::conditional_t<PR_PHYSICS_DIAGNOSTICS != 0, BodyHistory, NullBodyHistory>;
	using PhysicsLogT = std::conditional_t<PR_PHYSICS_DIAGNOSTICS != 0, PhysicsLog, NullPhysicsLog>;

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
		int m_awake_dynamic_count;
		double m_broadphase_axis_sum[3] = {};
		double m_broadphase_axis_sum_sq[3] = {};
		int m_broadphase_axis_sample_count;
		int m_broadphase_sort_axis;

		// Diagnostics
		BodyHistoryT m_history;
		PhysicsLogT m_log;

		EngineBufferCache()
			: m_shape_cache()
			, m_rb_dynamics()
			, m_contacts()
			, m_contacts_cpu()
			, m_sleep_islands()
			, m_sleep_gpu_to_cpu_island_id()
			, m_sleep_cpu_to_gpu_island_id()
			, m_next_sleep_island_id()
			, m_awake_dynamic_count()
			, m_broadphase_axis_sample_count()
			, m_broadphase_sort_axis()
			, m_history()
			, m_log()
		{
		}

		// Prepare for a new Engine::Step()
		void NewFrame(std::span<RigidBody*> rigid_bodies, int max_contacts)
		{
			// Purge unused shapes from the cache
			m_shape_cache.BeginFrame();
			
			// Reset the GPU staging buffer for body dynamics.
			m_rb_dynamics.resize(0);
			m_awake_dynamic_count = 0;
			m_broadphase_axis_sum[0] = 0.0;
			m_broadphase_axis_sum[1] = 0.0;
			m_broadphase_axis_sum[2] = 0.0;
			m_broadphase_axis_sum_sq[0] = 0.0;
			m_broadphase_axis_sum_sq[1] = 0.0;
			m_broadphase_axis_sum_sq[2] = 0.0;
			m_broadphase_axis_sample_count = 0;
			m_broadphase_sort_axis = 0;

			// Reset the GPU staging buffer for contacts.
			if (std::ssize(m_contacts) < max_contacts)
				m_contacts.resize(max_contacts);

			// Reset per-frame sleeping-island staging.
			m_sleep_islands.resize(0);
			m_sleep_gpu_to_cpu_island_id.resize(0);
			m_sleep_cpu_to_gpu_island_id.clear();

			m_history.BeginFrame(rigid_bodies);
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
			m_awake_dynamic_count = 0;
			m_broadphase_axis_sum[0] = 0.0;
			m_broadphase_axis_sum[1] = 0.0;
			m_broadphase_axis_sum[2] = 0.0;
			m_broadphase_axis_sum_sq[0] = 0.0;
			m_broadphase_axis_sum_sq[1] = 0.0;
			m_broadphase_axis_sum_sq[2] = 0.0;
			m_broadphase_axis_sample_count = 0;
			m_broadphase_sort_axis = 0;
			m_history.Reset();
			m_log.Close();
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

		// Number of dynamic bodies that can move this frame.
		int AwakeDynamicCount() const
		{
			return m_awake_dynamic_count;
		}

		// Accumulate AABB-centre variance so broadphase can sort along the longest spread.
		void AddBroadphaseSortSample(GpuRigidBody const& dyn)
		{
			auto const centre = dyn.o2w * dyn.os_bbox.m_centre;
			m_broadphase_axis_sum[0] += centre.x;
			m_broadphase_axis_sum[1] += centre.y;
			m_broadphase_axis_sum[2] += centre.z;
			m_broadphase_axis_sum_sq[0] += centre.x * centre.x;
			m_broadphase_axis_sum_sq[1] += centre.y * centre.y;
			m_broadphase_axis_sum_sq[2] += centre.z * centre.z;
			++m_broadphase_axis_sample_count;
		}

		void FinaliseBroadphaseSortAxis()
		{
			if (m_broadphase_axis_sample_count < 2)
			{
				m_broadphase_sort_axis = 0;
				return;
			}

			auto const inv_count = 1.0 / m_broadphase_axis_sample_count;
			auto best_axis = 0;
			auto best_variance = -1.0;
			for (int axis = 0; axis != 3; ++axis)
			{
				auto const mean = m_broadphase_axis_sum[axis] * inv_count;
				auto const variance = m_broadphase_axis_sum_sq[axis] * inv_count - mean * mean;
				if (variance > best_variance)
				{
					best_variance = variance;
					best_axis = axis;
				}
			}

			m_broadphase_sort_axis = best_axis;
		}

		int BroadphaseSortAxis() const
		{
			return m_broadphase_sort_axis;
		}

		// Track whether the body requires GPU work this frame.
		void CountAwakeDynamic(RigidBody const& body)
		{
			if (body.InvMass() > 0.0f && !body.Sleeping() && !AllSet(body.m_state_flags, ERigidBodyStateFlags::Static))
				++m_awake_dynamic_count;
		}

		// Map a sleeping dynamic body onto a dense GPU island and grow that island's bounds.
		void PackSleepIsland(RigidBody& body, GpuRigidBody& dyn)
		{
			dyn.sleep.island_id = -1;
			if (!body.Sleeping() || body.NeverSleep() || body.InvMass() <= 0.0f || AllSet(body.m_state_flags, ERigidBodyStateFlags::Static))
				return;

			if (body.m_sleep.m_island_id < 0)
				return;

			m_next_sleep_island_id = std::max(m_next_sleep_island_id, body.m_sleep.m_island_id + 1);

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
		void UnpackSleepIsland(GpuRigidBody& dyn)
		{
			auto const gpu_id = dyn.sleep.island_id;
			if (gpu_id < 0)
				return;

			if (gpu_id >= std::ssize(m_sleep_gpu_to_cpu_island_id))
			{
				m_sleep_gpu_to_cpu_island_id.resize(gpu_id + 1, -1);
			}
			if (m_sleep_gpu_to_cpu_island_id[gpu_id] < 0)
				m_sleep_gpu_to_cpu_island_id[gpu_id] = m_next_sleep_island_id++;

			dyn.sleep.island_id = m_sleep_gpu_to_cpu_island_id[gpu_id];
		}
	};
}
