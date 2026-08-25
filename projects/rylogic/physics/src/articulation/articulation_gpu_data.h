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

		// Number of scalar floats required for tightly packed active inverse joint-inertia blocks.
		int m_joint_matrix_scratch_count = 0;
	};

	// Validate and flatten independent articulation trees into deterministic linear GPU ranges and traversal schedules.
	GpuArticulationUpload PackGpuArticulations(std::span<Articulation* const> articulations);

	// Pack one hidden force/collision proxy per link and assign its contiguous body-buffer index.
	std::vector<GpuRigidBody> PackGpuArticulationProxies(GpuArticulationUpload& upload, std::span<Articulation* const> articulations, std::span<int const> shape_ids, int first_body_index);

	// Reject malformed packed ranges and topology before shared replay or GPU kernels can index them.
	void ValidateGpuArticulationUpload(GpuArticulationUpload const& upload);
}
