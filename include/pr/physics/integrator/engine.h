//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/collision/contact.h"
#include "pr/physics/integrator/engine_config.h"

namespace pr::physics
{
	struct Engine
	{
		// Notes:
		//  - The engine does not own bodies or articulations. The caller must keep them alive and
		//    unchanged until each BeginStep call is paired with CompleteStep or AbandonStep.
		//  - The engine does not have a universal gravity setting, Gravity should be applied
		//    as a force to bodies each frame before calling Step().
		//  - Collision resolution and 'sleeping objects' require a concept of "down" however,
		//    even if it is a zero vector.
		//  - Only supporting step on the GPU, if you want a CPU step, use HLSL interop.

		// Timing and GPU-boundary counts collected for the most recent submitted frame.
		struct StepProfile
		{
			double m_new_frame_ms = 0;
			double m_pack_ms = 0;
			double m_constraint_pack_ms = 0;
			double m_articulation_pack_ms = 0;
			double m_upload_ms = 0;
			double m_constraint_upload_ms = 0;
			double m_articulation_upload_ms = 0;
			double m_external_forces_ms = 0;
			double m_integrate_ms = 0;
			double m_articulation_integrate_ms = 0;
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
			double m_articulation_unpack_ms = 0;
			double m_unpack_diagnostics_ms = 0;
			int m_substep_count = 0;
			int m_submission_count = 0;
			int m_wait_count = 0;
			int m_readback_copy_count = 0;
		};
		// Peak collision demand and bounded event-queue status across one submitted frame.
		struct CollisionStats
		{
			int m_pair_count = 0;
			int m_contact_count = 0;
			int m_max_pairs = 0;
			int m_max_contacts = 0;
			int m_event_count = 0;
			int m_event_capacity = 0;
			int m_pair_limit_substep = -1;
			int m_contact_limit_substep = -1;
			int m_event_overflow_substep = -1;

			// Peak number of solver contacts retained by any internal substep.
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
			bool EventLimitReached() const
			{
				return m_event_overflow_substep >= 0;
			}
		};
		// GPU command-recording context for one pre-integration internal substep.
		struct ExternalForceArgs
		{
			GpuJob& m_job;
			ID3D12Resource* m_bodies;
			int m_body_count;
			int m_rigid_body_count;
			float m_dt;
			double m_time_s;
			int m_substep_index;
			int m_substep_count;
		};

		// Complete input for one submitted frame. Intermediate substeps remain GPU-resident and do not permit CPU state inspection.
		struct StepInput
		{
			std::span<RigidBody*> m_bodies;
			std::span<Articulation*> m_articulations;
			ConstraintSet const* m_constraints = nullptr;
			float m_elapsed_seconds = 0.0f;
			int m_substep_count = 1;
			double m_time_s = 0.0;
		};

	private:

		// Caller-owned simulation objects and GPU output that remain live between BeginStep and CompleteStep.
		struct PendingStep
		{
			// Expected packed ranges bind each compact readback record to one stable caller-owned articulation.
			struct ArticulationOutputRange
			{
				uint64_t m_identity = 0;
				int m_position_offset = 0;
				int m_position_count = 0;
				int m_velocity_offset = 0;
				int m_velocity_count = 0;
				int m_proxy_body_offset = 0;
				int m_link_count = 0;
			};

			std::vector<RigidBody*> m_bodies;
			std::vector<Articulation*> m_articulations;
			std::vector<ArticulationOutputRange> m_articulation_ranges;
			std::unordered_map<uint64_t, int> m_articulation_range_lookup;
			std::unique_ptr<GpuBuffers, Deleter<GpuBuffers>> m_buffers;
			GpuJob::RunHandle m_run;
			float m_substep_seconds = 0.0f;
			bool m_active = false;
			bool m_submitted = false;

			PendingStep();

			// Replace the staged body list with pointers to a range of caller-owned rigid bodies.
			void AssignBodies(RigidBodyRange auto&& bodies)
			{
				m_bodies.clear();
				m_bodies.reserve(static_cast<std::size_t>(std::ranges::distance(bodies)));
				for (auto& body : bodies)
					m_bodies.push_back(&body);
			}

			// Start tracking a begin/complete step pair using stable copies of every caller-owned object pointer.
			void Begin(std::span<RigidBody*> bodies, std::span<Articulation*> articulations, float substep_seconds);

			// Clear all per-step state once the GPU result has been consumed.
			void Clear();
		};

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

		// Lazily created GPU persistent-constraint solver.
		GpuConstraintSolverPtr m_gpu_constraint_solver;

		// Lazily created shared articulation dynamics resources.
		GpuArticulationForceAbaPtr m_gpu_articulation_force_aba;

		// Lazily created hidden link-proxy force and kinematics lane.
		GpuArticulationLinkProxiesPtr m_gpu_articulation_link_proxies;

		// Lazily created fused pure-tree midpoint integration lane.
		GpuArticulationMidpointPtr m_gpu_articulation_midpoint;

		// Gathered frame output and bounded substep event queue.
		GpuFrameOutputPtr m_gpu_frame_output;

		// GPU resolver for compact selective-refresh work sets
		GpuResolverPtr m_gpu_selective_resolver;

		// GPU selective contact refresher
		GpuSelectiveRefresherPtr m_gpu_selective_refresher;

		// Material map for looking up combined material properties during collision resolution.
		MaterialMapPtr m_materials;

		// Buffers for preparing GPU data
		CachePtr m_cache;

		// State for a BeginStep/CompleteStep pair.
		PendingStep m_pending_step;
		bool m_constraints_active;

		// Diagnostics
		StepProfile m_last_step_profile;
		CollisionStats m_last_collision_stats;
		
		friend struct DbgPhysics;

		// Submit one frame containing optional rigid bodies, pure-tree articulations, and persistent constraints.
		void BeginStepInternal(StepInput const& input);

	public:

		explicit Engine(EngineConfig const& config = {}, IShaderCache* shader_cache = nullptr, ID3D12Device4* existing_device = nullptr);

		// Engine configuration in use by this instance.
		EngineConfig const& Config() const;
		void Config(EngineConfig const& config);

		// Return the D3D12 device used by the physics compute engine.
		ID3D12Device4* Device() const;

		// Evolve the physics objects forward in time and resolve any collisions.
		void Step(float dt, std::span<RigidBody*> bodies, double time_s = 0.0);

		// Evolve bodies with persistent constraints; every enabled rigid endpoint must occur in 'bodies'.
		void Step(float dt, std::span<RigidBody*> bodies, ConstraintSet const& constraints, double time_s = 0.0);

		// Evolve a complete frame using one GPU submission regardless of the requested internal substep count.
		void Step(StepInput const& input);
		void Step(float dt, RigidBodyRange auto&& bodies, double time_s = 0.0)
		{
			BeginStep(dt, bodies, time_s);
			CompleteStep();
		}
		void Step(float dt, RigidBodyRange auto&& bodies, ConstraintSet const& constraints, double time_s = 0.0)
		{
			BeginStep(dt, bodies, constraints, time_s);
			CompleteStep();
		}

		// Supply a concrete body range for a StepInput while preserving the engine's stable pending-step pointer copy.
		void Step(StepInput input, RigidBodyRange auto&& bodies)
		{
			BeginStep(input, bodies);
			CompleteStep();
		}

		// Begin evolving the physics objects by submitting GPU work without waiting for it to finish.
		void BeginStep(float dt, std::span<RigidBody*> bodies, double time_s = 0.0);

		// Submit constrained work without waiting; every enabled rigid endpoint must occur in 'bodies' until completion.
		void BeginStep(float dt, std::span<RigidBody*> bodies, ConstraintSet const& constraints, double time_s = 0.0);

		// Submit a complete frame without waiting; all input topology and endpoints must remain valid until completion.
		void BeginStep(StepInput const& input);
		void BeginStep(float dt, RigidBodyRange auto&& bodies, double time_s = 0.0)
		{
			if (m_pending_step.m_active)
				throw std::runtime_error("Engine::BeginStep called while a previous step is pending");

			m_pending_step.AssignBodies(bodies);
			BeginStep(dt, std::span{ m_pending_step.m_bodies }, time_s);
		}
		void BeginStep(float dt, RigidBodyRange auto&& bodies, ConstraintSet const& constraints, double time_s = 0.0)
		{
			if (m_pending_step.m_active)
				throw std::runtime_error("Engine::BeginStep called while a previous step is pending");

			m_pending_step.AssignBodies(bodies);
			BeginStep(dt, std::span{ m_pending_step.m_bodies }, constraints, time_s);
		}

		// Supply a concrete body range for a StepInput without exposing a transient pointer container to the caller.
		void BeginStep(StepInput input, RigidBodyRange auto&& bodies)
		{
			if (m_pending_step.m_active)
				throw std::runtime_error("Engine::BeginStep called while a previous step is pending");

			m_pending_step.AssignBodies(bodies);
			input.m_bodies = std::span{ m_pending_step.m_bodies };
			BeginStep(input);
		}

		// Complete a previously-begun step and unpack the GPU results into the caller-owned bodies.
		void CompleteStep();

		// Wait for a pending step during terminal cleanup without updating bodies or reusing command state.
		void AbandonStep();

		// Build missing sleep-island ids for bodies created directly in the sleeping state.
		// Call this after loading/creating sleeping bodies; Step() assumes sleep islands have already been established when needed.
		void UpdateSleepIslands(std::span<RigidBody*> bodies);
		void UpdateSleepIslands(RigidBodyRange auto&& bodies)
		{
			auto body_ptrs = std::vector<RigidBody*>{};
			body_ptrs.reserve(static_cast<std::size_t>(std::ranges::distance(bodies)));
			for (auto& body : bodies)
				body_ptrs.push_back(&body);

			UpdateSleepIslands(body_ptrs);
		}

		// Raised at the end of step, just before object dynamics are updated
		EventHandler<Engine&, std::span<RbContact const>> Collisions;
		
		// Raised after body upload and before integration so subscribers can add GPU-resident forces.
		EventHandler<Engine&, ExternalForceArgs const&> ExternalForces;

		// Resolve a stable articulation link to its hidden body index during ExternalForces, or return -1 when its tree is absent.
		int ArticulationLinkStepIndex(ArticulationId articulation_id, LinkHandle link) const;

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
		void ApplyExternalForces(float dt, double time_s, int substep_index, int substep_count);

		// Mark sleeping islands disturbed by awake bodies before broadphase filtering.
		void SleepWake();

		// Broadphase collision detection with optional connected-body pair suppression.
		void BroadPhase(bool sleeping_enabled, std::span<GpuCollisionExclusion const> collision_exclusions = {});

		// Narrow phase collision detection to generate contact points.
		void Collide();

		// Apply impulses to resolve collisions and update body dynamics.
		void Resolve(float dt);

		// Extra narrowphase/resolve passes over problematic contacts.
		void SelectiveRefresh(float dt);

		// Persist wake-ups and update sleep state after collision resolution.
		void SleepUpdate(float dt);

		// Append aggregate counters and optional collision records for one completed GPU substep.
		void CaptureSubstepOutput(int substep_index, int substep_count, bool collect_events);

		// Record one packed readback using the same explicit articulation layout supplied at frame start.
		void Readback(GpuBuffers& buffers, GpuArticulationMidpointOutput const& articulations);

		// Validate the complete gathered frame before publishing rigid or articulation state.
		void Unpack(GpuBuffers const& buffers, std::span<RigidBody*> rigid_bodies, std::span<Articulation*> articulations, std::span<PendingStep::ArticulationOutputRange const> articulation_ranges, float articulation_substep_seconds);

		// Narrow phase collision detection.
		// Tests whether the two bodies in 'c' are geometrically in contact using GJK/SAT.
		bool NarrowPhaseCollision(float dt, RbContact& c);

		// Calculate and apply the restitution impulse to resolve a collision.
		void ResolveCollision(RbContact& c);
	};
}
