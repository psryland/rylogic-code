//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of compact participating-tree self-link mobility preparation.
	struct ArticulationMobilityInteropRunner
	{
		// Rebuild exact mobilities for the selected articulation indices.
		void Run(GpuArticulationUpload const& upload, std::span<int const> articulation_indices);

		// Return canonical participating ranges and their compact output offsets.
		std::span<GpuArticulationMobilityRange const> Ranges() const;

		// Return packed exact self-link mobilities in canonical range order.
		std::span<GpuArticulationSpatialMobility const> Mobilities() const;

		// Return shared ABA scratch after final-configuration factorization.
		std::span<GpuArticulationAbaScratch const> Scratch() const;

		// Return phase-reused generalized response scratch after factorization.
		std::span<float const> Accelerations() const;

		// Return retained per-DOF motion subspaces and articulated columns.
		std::span<GpuArticulationAbaDofScratch const> DofScratch() const;

		// Return retained packed inverse joint matrices.
		std::span<float const> InverseJointInertia() const;

	private:

		std::vector<GpuArticulationMobilityRange> m_ranges;
		std::vector<GpuArticulationSpatialMobility> m_mobilities;
		std::vector<float> m_accelerations;
		std::vector<GpuArticulationAbaScratch> m_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_dof_scratch;
		std::vector<float> m_joint_matrix_scratch;
	};
}
