//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#pragma once
#include "src/forward.h"

namespace las
{
	// Owns the physics engine boundary for LAS and presents a stable object-level handle surface to game systems.
	struct PhysicsSystem
	{
		struct BodyHandle
		{
			int m_index;
			int m_generation;

			// Return an invalid body handle sentinel.
			static BodyHandle Invalid();

			// Return true if this handle could refer to a registered body.
			bool IsValid() const;
		};
		struct BoxBodyDesc
		{
			v4 m_size;
			m4x4 m_o2w;
			float m_mass_kg;
			bool m_never_sleep;
		};
		struct BodySnapshot
		{
			BodyHandle m_handle;
			m4x4 m_o2w;
			int m_step_index;
			bool m_valid;

			// Construct an invalid body snapshot.
			BodySnapshot();
		};

	private:

		struct BodySlot
		{
			std::unique_ptr<ShapeBox> m_shape;
			std::unique_ptr<RigidBody> m_body;
			int m_generation;
			int m_step_index;
		};

		std::thread::id m_owner_thread_id;
		physics::Engine m_engine;

		struct OceanSurfaceForce;
		std::unique_ptr<OceanSurfaceForce> m_ocean_surface_force;

		std::vector<BodySlot> m_body_slots;
		std::vector<int> m_free_slots;
		std::vector<RigidBody*> m_step_bodies;

		mutable std::mutex m_snapshot_mutex;
		std::vector<BodySnapshot> m_snapshots;

		mutable std::mutex m_config_mutex;
		v4 m_gravity_ws;

		bool m_step_pending;
		bool m_engine_step_pending;

	public:

		explicit PhysicsSystem(Renderer& rdr);
		PhysicsSystem(PhysicsSystem const&) = delete;
		PhysicsSystem& operator=(PhysicsSystem const&) = delete;
		~PhysicsSystem();

		// Create a physics-owned box body and return its stable game-level handle.
		BodyHandle CreateBoxBody(BoxBodyDesc const& desc);

		// Destroy a physics-owned body.
		void DestroyBody(BodyHandle handle);

		// Return true if 'handle' currently refers to a registered body.
		bool IsValid(BodyHandle handle) const;

		// Return the compact body index used by the most recent/current Engine::Step() call.
		int StepIndex(BodyHandle handle) const;

		// Return the latest published snapshot for a physics body.
		BodySnapshot Snapshot(BodyHandle handle) const;

		// Submit GPU physics work for all registered rigid bodies without waiting for completion.
		void BeginStep(float dt, double time_s);

		// Wait for pending GPU physics work and publish immutable snapshots, returning an exception on failure.
		[[nodiscard]] std::exception_ptr CompleteStep() noexcept;

		// Step all registered rigid bodies synchronously.
		void Step(float dt, double time_s);

		// Set the world-space gravity applied to registered bodies before each step.
		void GravityWS(v4 gravity_ws);

		// Return the world-space gravity applied before each step.
		v4 GravityWS() const;

	private:

		// Throw if mutable physics state is touched from a thread other than the owner thread.
		void CheckOwnerThread() const;

		// Throw if an operation would mutate state while a step is in flight.
		void CheckNoStepPending(char const* operation) const;

		// Return true if 'handle' refers to a live body slot on the owner thread.
		bool IsValidOnOwnerThread(BodyHandle handle) const;

		// Return the live body slot for 'handle' on the owner thread.
		BodySlot& Slot(BodyHandle handle);
		BodySlot const& Slot(BodyHandle handle) const;

		// Rebuild the compact body list submitted to the physics engine.
		void BuildStepBodyList();

		// Publish body state for render and camera readers.
		void PublishSnapshots();
	};
}
