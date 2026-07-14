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
		//  - A rigid body representing the first LAS physics object.
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

		// Prepare shader constant buffers for rendering (thread-safe).
		void PrepareRender(v4 camera_world_pos);

		// Add instance to the scene drawlist (NOT thread-safe).
		void AddToScene(Scene& scene);
	};
}
