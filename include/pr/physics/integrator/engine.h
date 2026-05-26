//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine_config.h"

namespace pr::physics
{
	struct Engine
	{
		// Notes:
		//  - The engine does not own the bodies. The caller is responsible for managing their
		//    lifetime and ensuring they remain valid while being used by the engine.
		//  - The engine does not have a universal gravity setting, Gravity should be applied
		//    as a force to bodies each frame before calling Step().
		//  - Collision resolution and 'sleeping objects' require a concept of "down" however,
		//    even if it is a zero vector.
		//  - Only supporting step on the GPU, if you want a CPU step, use HLSL interop.

		struct StepProfile
		{
			double m_new_frame_ms = 0;
			double m_pack_ms = 0;
			double m_upload_ms = 0;
			double m_external_forces_ms = 0;
			double m_integrate_ms = 0;
			double m_sleepwake_ms = 0;
			double m_broadphase_ms = 0;
			double m_collide_ms = 0;
			double m_resolve_ms = 0;
			double m_selective_ms = 0;
			double m_sleepupdate_ms = 0;
			double m_readback_ms = 0;
			double m_gpu_run_ms = 0;
			double m_gpu_prepare_ms = 0;
			double m_gpu_execute_ms = 0;
			double m_gpu_wait_ms = 0;
			double m_gpu_reset_ms = 0;
			double m_unpack_ms = 0;
			double m_readback_access_ms = 0;
			double m_body_readback_copy_ms = 0;
			double m_contact_readback_copy_ms = 0;
			double m_collision_events_ms = 0;
			double m_sleep_island_unpack_ms = 0;
			double m_body_unpack_ms = 0;
			double m_unpack_diagnostics_ms = 0;
		};
		struct CollisionStats
		{
			int m_pair_count = 0;
			int m_contact_count = 0;
			int m_max_pairs = 0;
			int m_max_contacts = 0;

			// Number of contacts stored during the most recent call to Step().
			int LastContactCount() const
			{
				return std::min(m_contact_count, m_max_contacts);
			}
			bool PairLimitReached() const
			{
				return m_max_pairs != 0 && m_pair_count >= m_max_pairs;
			}
			bool PairLimitExceeded() const
			{
				return m_max_pairs != 0 && m_pair_count > m_max_pairs;
			}
			bool ContactLimitReached() const
			{
				return m_max_contacts != 0 && m_contact_count >= m_max_contacts;
			}
			bool ContactLimitExceeded() const
			{
				return m_max_contacts != 0 && m_contact_count > m_max_contacts;
			}
		};
		struct ExternalForceArgs
		{
			// Context supplied to pre-integrate GPU force callbacks.
			GpuJob& m_job;
			ID3D12Resource* m_bodies;
			int m_body_count;
			float m_dt;
			double m_time_s;
			int m_substep_index;
			int m_substep_count;
		};

	private:

		// Engine configuration parameters.
		EngineConfig m_config;

		// GPU device and command queue wrapper, shared by the integrator and collision detector.
		GpuPtr m_gpu;

		// GPU integrator
		GpuIntegratorPtr m_gpu_integrator;

		// GPU sleep/wake management
		GpuSleepManagerPtr m_gpu_sleep_manager;

		// GPU broadphase
		GpuSortAndSweepPtr m_gpu_sort_and_sweep;

		// GPU collision detector
		GpuCollisionDetectorPtr m_gpu_collision_detector;

		// GPU collision resolver
		GpuResolverPtr m_gpu_resolver;

		// GPU resolver for compact selective-refresh work sets
		GpuResolverPtr m_gpu_selective_resolver;

		// GPU selective contact refresher
		GpuSelectiveRefresherPtr m_gpu_selective_refresher;

		// Material map for looking up combined material properties during collision resolution.
		MaterialMapPtr m_materials;

		// Buffers for preparing GPU data
		CachePtr m_cache;

		// Storage for body pointers
		std::vector<RigidBody*> m_body_ptrs;

		// Diagnostics
		StepProfile m_last_step_profile;
		CollisionStats m_last_collision_stats;
		
		friend struct DbgPhysics;

	public:

		explicit Engine(EngineConfig const& config = {}, IShaderCache* shader_cache = nullptr, ID3D12Device4* existing_device = nullptr);

		// Engine configuration in use by this instance.
		EngineConfig const& Config() const;
		void Config(EngineConfig const& config);

		// Evolve the physics objects forward in time and resolve any collisions.
		void Step(float dt, std::span<RigidBody*> bodies, double time_s = 0.0);
		void Step(float dt, RigidBodyRange auto&& bodies, double time_s = 0.0)
		{
			m_body_ptrs.resize(0);
			for (auto& body : bodies)
				m_body_ptrs.push_back(&body);

			Step(dt, m_body_ptrs, time_s);
		}

		// Build missing sleep-island ids for bodies created directly in the sleeping state.
		// Call this after loading/creating sleeping bodies; Step() assumes sleep islands have already been established when needed.
		void UpdateSleepIslands(std::span<RigidBody*> bodies);
		void UpdateSleepIslands(RigidBodyRange auto&& bodies)
		{
			m_body_ptrs.resize(0);
			for (auto& body : bodies)
				m_body_ptrs.push_back(&body);

			UpdateSleepIslands(m_body_ptrs);
		}

		// Raised at the end of step, just before object dynamics are updated
		EventHandler<Engine&, std::span<RbContact const>> Collisions;
		
		// Raised after body upload and before integration so subscribers can add GPU-resident forces.
		EventHandler<Engine&, ExternalForceArgs const&> ExternalForces;

		// Get/set the physics material properties for a given material ID.
		physics::Material Material(int id) const;
		void Material(physics::Material mat);

		// Drop all internally-cached references to caller-supplied data.
		// Specifically: the shape cache (which keys entries by Shape pointer) is cleared
		// so that any pointers it still holds to caller-owned shapes are forgotten, and
		// the per-frame staging buffers (rb dynamics, contacts, contacts_cpu) are cleared.
		// GPU resources owned by the engine are *not* recreated.
		// Call this whenever the set of Shape objects the engine has previously seen may
		// have been destroyed, e.g. between independent unit-test scenarios that share a
		// single engine via SharedEngine().
		void ResetCaches();

		// Timing from the most recent call to Step().
		StepProfile const& LastStepProfile() const
		{
			return m_last_step_profile;
		}

		// Raw collision counts from the most recent call to Step().
		CollisionStats const& LastCollisionStats() const
		{
			return m_last_collision_stats;
		}

	private:

		// Pack the body data into GPU buffers for the current frame.
		void Pack(std::span<RigidBody*> rigid_bodies);

		// Upload staged body data into GPU buffers for the current frame.
		void Upload();

		// Apply forces, evolve body dynamics forward in time, and generate AABBs for broadphase.
		void Integrate(float dt);
		
		// Apply user-supplied GPU forces before integration.
		void ApplyExternalForces(float dt, double time_s);

		// Mark sleeping islands disturbed by awake bodies before broadphase filtering.
		void SleepWake();

		// Broadphase collision detection to generate potential collision pairs.
		void BroadPhase(bool sleeping_enabled);

		// Narrow phase collision detection to generate contact points.
		void Collide();

		// Apply impulses to resolve collisions and update body dynamics.
		void Resolve(float dt);

		// Extra narrowphase/resolve passes over problematic contacts.
		void SelectiveRefresh(float dt);

		// Persist wake-ups and update sleep state after collision resolution.
		void SleepUpdate(float dt);

		// Read buffers back to CPU memory
		void Readback(GpuBuffers& buffers);

		// Update rigid bodies with results from the step
		void Unpack(GpuBuffers const& buffers, std::span<RigidBody*> rigid_bodies);

		// Narrow phase collision detection.
		// Tests whether the two bodies in 'c' are geometrically in contact using GJK/SAT.
		bool NarrowPhaseCollision(float dt, RbContact& c);

		// Calculate and apply the restitution impulse to resolve a collision.
		void ResolveCollision(RbContact& c);
	};
}
