//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/articulation/articulation.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Complete contiguous CPU upload image for optional GPU articulation processing.
	struct GpuArticulationUpload
	{
		std::vector<GpuArticulation> m_articulations;
		std::vector<GpuArticulationLink> m_links;
		std::vector<GpuArticulationDof> m_dofs;
		std::vector<float> m_positions;
		std::vector<float> m_velocities;
		std::vector<float> m_forces;
		std::vector<float> m_accelerations;
		std::vector<GpuFrameForce> m_external_forces;
		std::vector<uint32_t> m_children;
		std::vector<GpuArticulationLevel> m_levels;
		std::vector<uint32_t> m_level_links;
	};

	// Validate and flatten independent articulation trees into deterministic linear GPU ranges and traversal schedules.
	GpuArticulationUpload PackGpuArticulations(std::span<Articulation* const> articulations);
}
