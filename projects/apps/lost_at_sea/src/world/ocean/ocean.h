//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Water-field ocean rendering.
#pragma once
#include "src/forward.h"
#include "src/world/water/water_system.h"

namespace las
{
	struct OceanShader;

	// Ocean simulation and rendering
	struct Ocean
	{
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4     , m_i2w  , EInstComp::I2WTransform)\
			x(ModelPtr , m_model, EInstComp::ModelPtr)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		Instance m_inst;
		OceanShader* m_shader; // Owned by 'm_model'

		explicit Ocean(Renderer& rdr);

		// Prepare shader constant buffers for rendering (thread-safe, no scene interaction).
		void PrepareRender(water::Snapshot const& water_snapshot, v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour);

		// Add instance to the scene drawlist (NOT thread-safe, must be called serially).
		void AddToScene(Scene& scene);

		// Return whether the near-ocean mesh is rendered as wireframe.
		bool Wireframe() const;

		// Set the near-ocean mesh fill mode.
		void Wireframe(bool enabled);
	};
}