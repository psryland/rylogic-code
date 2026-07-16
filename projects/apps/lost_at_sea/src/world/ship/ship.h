//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#pragma once
#include "src/forward.h"
#include "src/core/physics/physics_system.h"

namespace las
{
	struct Ship
	{
		// Notes:
		//  - A convex rudimentary ship hull whose render and collision surfaces share one canonical point definition.
		//    Buoyancy is applied by the physics system's external force passes rather than by Ship.

		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4     , m_i2w  , EInstComp::I2WTransform)\
			x(ModelPtr , m_model, EInstComp::ModelPtr)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		// Physics registration owned by the ship.
		PhysicsSystem& m_physics;
		PhysicsSystem::BodyHandle m_body_handle;
		PhysicsSystem::BuoyancyHullRegistration m_buoyancy_hull;
		float m_hull_volume_m3;
		std::atomic<float> m_requested_density_kg_m3;
		std::atomic<float> m_applied_density_kg_m3;

		// Graphics
		Instance m_inst;

		Ship(Renderer& rdr, PhysicsSystem& physics, v4 location);
		Ship(Ship const&) = delete;
		Ship& operator=(Ship const&) = delete;
		~Ship();

		// Return the latest published ship transform.
		m4x4 O2W() const;

		// Access the physics registration handle for this ship's body.
		PhysicsSystem::BodyHandle PhysicsHandle() const;

		// Request a new average density; the simulation owner applies it before the next physics step.
		void AverageDensity(float density_kg_m3);

		// Return the requested average ship density.
		float AverageDensity() const;

		// Return the density currently applied to the rigid body's mass properties.
		float AppliedAverageDensity() const;

		// Return the closed convex hull volume.
		float HullVolume() const;

		// Return the density-derived rigid-body mass.
		float Mass() const;

		// Apply pending tuning requests on the simulation owner thread.
		void ApplyTuning();

		// Return the latest read-back buoyancy diagnostics for the ship hull.
		physics::GpuBuoyancy::Diagnostics BuoyancyDiagnostics() const;

		// Prepare shader constant buffers for rendering (thread-safe).
		void PrepareRender(v4 camera_world_pos);

		// Add instance to the scene drawlist (NOT thread-safe).
		void AddToScene(Scene& scene);
	};
}
