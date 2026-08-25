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

		// Evaluate selected impulses into detached response buffers without changing committed articulation state.
		std::span<uint32_t const> Evaluate(std::span<GpuArticulationSpatialVector const> link_impulses, std::span<uint32_t const> selection);

		// Commit the selected responses that succeeded during the preceding detached evaluation.
		void Commit(std::span<uint32_t const> selection);

		// Return committed packed generalized velocities after impulse application.
		std::span<float const> Velocities() const;

		// Return authoritative generalized accelerations preserved across detached impulse evaluation.
		std::span<float const> Accelerations() const;

		// Return retained ABA scratch containing committed cached link velocities.
		std::span<GpuArticulationAbaScratch const> Scratch() const;

		// Return detached per-link velocity deltas from the most recent evaluation.
		std::span<GpuArticulationSpatialVector const> Work() const;

	private:

		GpuArticulationUpload m_upload;
		std::vector<GpuArticulationMobilityRange> m_ranges;
		std::vector<GpuArticulationSpatialMobility> m_mobilities;
		std::vector<float> m_velocities;
		std::vector<float> m_accelerations;
		std::vector<float> m_velocity_deltas;
		std::vector<GpuArticulationAbaScratch> m_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_dof_scratch;
		std::vector<float> m_inverse_joint_inertia;
		std::vector<GpuArticulationSpatialVector> m_link_impulses;
		std::vector<GpuArticulationSpatialVector> m_work;
		std::vector<uint32_t> m_results;
	};
}
