//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of fused pure-tree midpoint integration.
	struct ArticulationMidpointInteropRunner
	{
		// Execute the requested internal substeps and replace packed primary/output state.
		void Run(GpuArticulationUpload& upload, float dt, int substep_count);

		// Return writable articulation records containing the integrated root frames.
		std::span<GpuArticulation const> Articulations() const;

		// Return explicit sticky status and fixed-point diagnostics for every articulation.
		std::span<GpuArticulationIntegrationState const> States() const;

		// Return solved link-frame spatial acceleration from phase-reused ABA scratch.
		GpuArticulationSpatialVector const& LinkAcceleration(int link_index) const;

	private:

		std::vector<GpuArticulation> m_articulations;
		std::vector<GpuArticulationAbaScratch> m_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_dof_scratch;
		std::vector<float> m_joint_matrix_scratch;
		std::vector<float> m_position_start;
		std::vector<float> m_velocity_start;
		std::vector<float> m_midpoint_velocity;
		std::vector<GpuArticulationIntegrationState> m_states;
	};
}
