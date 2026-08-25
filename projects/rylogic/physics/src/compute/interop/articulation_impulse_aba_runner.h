//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of one simultaneous articulation impulse response.
	struct ArticulationImpulseAbaInteropRunner
	{
		// Rebuild participating-tree factors and apply one link-coordinate impulse per packed link.
		void Run(
			GpuArticulationUpload const& upload,
			std::span<int const> articulation_indices,
			std::span<GpuArticulationSpatialVector const> link_impulses);

		// Apply another simultaneous impulse through the retained fixed-configuration factors.
		void Apply(std::span<GpuArticulationSpatialVector const> link_impulses);

		// Return committed packed generalized velocities after impulse application.
		std::span<float const> Velocities() const;

		// Return retained ABA scratch containing committed cached link velocities.
		std::span<GpuArticulationAbaScratch const> Scratch() const;

	private:

		GpuArticulationUpload m_upload;
		std::vector<GpuArticulationMobilityRange> m_ranges;
		std::vector<GpuArticulationSpatialMobility> m_mobilities;
		std::vector<float> m_velocities;
		std::vector<float> m_accelerations;
		std::vector<GpuArticulationAbaScratch> m_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_dof_scratch;
		std::vector<float> m_inverse_joint_inertia;
		std::vector<GpuArticulationSpatialVector> m_work;
	};
}
