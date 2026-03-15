//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#pragma once
#include "src/forward.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/inertia.h"
#include "pr/collision/shape_box.h"
#include "pr/container/byte_data.h"

namespace las
{
	struct Ocean;
	struct HeightField;

	struct Ship
	{
		// Notes:
		//  - A rigid body that floats on the ocean surface.
		//    The "ship" is a 1x1x1 cube with gravity and buoyancy forces applied.
		//    Buoyancy is approximated by the submersion depth of the centre of mass.

		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4     , m_i2w  , EInstComp::I2WTransform)\
			x(ModelPtr , m_model, EInstComp::ModelPtr)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		// Collision shape storage (value type, no heap allocation)
		ShapeBox m_col_shape;

		// Physics rigid body
		RigidBody m_body;

		// Graphics
		Instance m_inst;

		Ship(Renderer& rdr, HeightField const& height_field, v4 location);

		// Step the ship's physics: apply gravity, ocean buoyancy, terrain collision.
		void Step(float dt, Ocean const& ocean, HeightField const& height_field, float sim_time);

		// Prepare shader constant buffers for rendering (thread-safe).
		void PrepareRender(v4 camera_world_pos);

		// Add instance to the scene drawlist (NOT thread-safe).
		void AddToScene(Scene& scene);
	};
}
