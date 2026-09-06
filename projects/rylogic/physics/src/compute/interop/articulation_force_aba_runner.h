//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of the pure-tree force-ABA level pipeline.
	struct ArticulationForceAbaInteropRunner
	{
		// Execute one packed forest and replace its generalized acceleration output.
		void Run(GpuArticulationUpload& upload);

		// Return one solved link-frame spatial acceleration from the phase-reused core scratch.
		GpuArticulationSpatialVector const& LinkAcceleration(int link_index) const;

		// Return compact per-link state for focused phase-reuse and validity diagnostics.
		std::span<GpuArticulationAbaScratch const> Scratch() const;

		// Return the two spatial factors retained for every generalized joint DOF.
		std::span<GpuArticulationAbaDofScratch const> DofScratch() const;

		// Return tightly packed active inverse joint-inertia blocks.
		std::span<float const> JointMatrixScratch() const;

	private:

		std::vector<GpuArticulationAbaScratch> m_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_dof_scratch;
		std::vector<float> m_joint_matrix_scratch;
	};
}
