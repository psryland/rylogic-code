//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"
#include "src/utils/scene_loader.h"

namespace physics_sandbox
{
	// GPU-animated static grid whose vertex shader evaluates the same sine-wave surface used by buoyancy.
	struct WaterVisual
	{
		struct Instance
		{
			#define PR_RDR_INST(x)\
			x(m4x4           , m_i2w         , rdr12::EInstComp::I2WTransform)\
			x(rdr12::ModelPtr, m_model       , rdr12::EInstComp::ModelPtr)\
			x(float           , m_reflectivity, rdr12::EInstComp::EnvMapReflectivity)\
			x(float           , m_time_s       , rdr12::EInstComp::Float1)
			PR_RDR12_INSTANCE_MEMBERS(Instance, PR_RDR_INST);
			#undef PR_RDR_INST
		};

		Instance m_inst;

		// Create immutable grid geometry and its sandbox-local water shader.
		WaterVisual(rdr12::Renderer& rdr, scene_loader::WaterDesc const& water, v2 extent);

		// Add the visual to a render scene using the current simulation time.
		void AddToScene(rdr12::Scene& scene, float time_s);
	};
}
