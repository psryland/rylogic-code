//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/compute/interop/articulation_impulse_aba_runner.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics
{
	// Deterministic HLSL-as-C++ replay of detached generalized coupled position correction.
	struct CoupledConstraintPositionInteropRunner
	{
	public:

		// Solve hard coupled drift in pseudo state and integrate accepted coordinates exactly once.
		void Run(
			float timestep,
			float relaxation,
			float position_beta,
			float max_position_speed,
			int position_iterations,
			GpuConstraintUpload const& constraint_upload,
			GpuArticulationUpload const& articulation_upload,
			std::span<GpuConstraintBlock const> blocks,
			std::span<GpuConstraintRow const> rows,
			std::span<GpuCoupledConstraintPreconditioner const> preconditioners,
			std::span<GpuRigidBody const> bodies,
			int backtrack_limit = 4);

		// Return rigid state after coordinate-only pseudo correction.
		std::span<GpuRigidBody const> Bodies() const;

		// Return persistent scalar rows containing accumulated pseudo impulses in bounds.w.
		std::span<GpuConstraintRow const> Rows() const;

		// Return articulation headers after coordinate-only root correction.
		std::span<GpuArticulation const> Articulations() const;

		// Return packed child joint positions after coordinate-only correction.
		std::span<float const> Positions() const;

		// Return accumulated detached rigid pseudo twists.
		std::span<GpuConstraintPseudoVelocity const> RigidPseudoVelocities() const;

		// Return accumulated detached compact generalized pseudo velocities.
		std::span<float const> GeneralizedPseudoVelocities() const;

		// Return final per-island transaction states and failure diagnostics.
		std::span<GpuCoupledConstraintIslandState const> IslandStates() const;

		// Return physical generalized velocities preserved by detached position response.
		std::span<float const> PhysicalVelocities() const;

		// Return physical generalized accelerations preserved by detached position response.
		std::span<float const> PhysicalAccelerations() const;

		// Return physical cached link state preserved by detached position response.
		std::span<GpuArticulationAbaScratch const> PhysicalScratch() const;

	private:

		ArticulationImpulseAbaInteropRunner m_impulse_aba;
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintBlock> m_blocks;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuConstraintPseudoVelocity> m_rigid_pseudo;
		std::vector<GpuArticulationSpatialVector> m_link_pseudo;
		std::vector<float> m_generalized_pseudo;
		std::vector<GpuArticulation> m_articulations;
		std::vector<float> m_positions;
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
