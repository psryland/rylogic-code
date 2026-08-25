//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/compute/interop/articulation_impulse_aba_runner.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of one transactional articulation-coupled velocity sweep.
	struct CoupledConstraintVelocityInteropRunner
	{
	public:

		// Apply an optional once-per-substep warm start followed by one bounded-backtracking simultaneous sweep.
		void Run(
			float relaxation,
			GpuConstraintUpload const& constraint_upload,
			GpuArticulationUpload const& articulation_upload,
			std::span<GpuConstraintBlock const> blocks,
			std::span<GpuConstraintRow const> rows,
			std::span<GpuCoupledConstraintPreconditioner const> preconditioners,
			std::span<GpuRigidBody const> bodies,
			int backtrack_limit = 4,
			bool apply_warm_start = false);

		// Return committed rigid-body state after accepted island updates.
		std::span<GpuRigidBody const> Bodies() const;

		// Return persistent scalar rows containing accepted accumulated impulses.
		std::span<GpuConstraintRow const> Rows() const;

		// Return committed packed generalized articulation velocities.
		std::span<float const> ArticulationVelocities() const;

		// Return authoritative generalized articulation accelerations preserved by detached response evaluation.
		std::span<float const> ArticulationAccelerations() const;

		// Return committed cached articulation link velocities.
		std::span<GpuArticulationAbaScratch const> ArticulationScratch() const;

		// Return final per-island transaction states and failure diagnostics.
		std::span<GpuCoupledConstraintIslandState const> IslandStates() const;

	private:

		ArticulationImpulseAbaInteropRunner m_impulse_aba;
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintBlock> m_blocks;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuCoupledConstraintSolveScratch> m_scratch;
		std::vector<GpuArticulationSpatialVector> m_contributions;
		std::vector<GpuArticulationSpatialVector> m_target_impulses;
		std::vector<GpuCoupledConstraintIslandState> m_island_states;
		std::vector<uint32_t> m_island_failures;
		std::vector<GpuArticulationSpatialVector> m_link_impulses;
		std::vector<uint32_t> m_tree_selection;
		std::vector<uint32_t> m_tree_results;
	};
}
