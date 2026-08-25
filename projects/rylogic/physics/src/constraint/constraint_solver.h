//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/constraint/constraint_compiler.h"

namespace pr::physics
{
	// Fixed-work controls for the deterministic CPU rigid-constraint reference solver.
	struct CpuConstraintSolverConfig
	{
		int m_velocity_iterations = 12;
		int m_position_iterations = 4;
		float m_relaxation = 1.0f;
		float m_coupled_relaxation = 0.9f;
		float m_position_relaxation = 1.0f;
		float m_position_beta = 0.2f;
		float m_max_position_speed = 2.0f;
		float m_regularization = 1.0e-6f;
		float m_warm_start_factor = 0.85f;
		int m_coupled_backtrack_limit = 4;
	};

	// Measurements from one deterministic CPU constraint solve.
	struct CpuConstraintSolveMetrics
	{
		int m_active_velocity_rows = 0;
		int m_active_position_rows = 0;
		int m_regularized_blocks = 0;
		int m_singular_blocks = 0;
		int m_coupled_velocity_rows = 0;
		int m_coupled_sweeps = 0;
		int m_coupled_backtracks = 0;
		int m_rejected_coupled_sweeps = 0;
		float m_projected_velocity_residual = 0.0f;
		float m_initial_position_error = 0.0f;
		float m_max_impulse_bound_violation = 0.0f;
		float m_physical_kinetic_energy_change = 0.0f;
	};

	// Project a normal and two tangent impulses onto an exact circular Coulomb cone.
	std::array<float, 3> ProjectFrictionCone(std::array<float, 3> impulse, float friction);

	// Deterministic warm-started block PGS reference for ordinary rigid bodies.
	class CpuConstraintSolver
	{
	private:

		// Cached canonical D6 impulses remain associated with a generational source handle.
		struct WarmStartEntry
		{
			std::array<float, 6> m_impulses = {};
			float m_timestep = 0.0f;
		};

		std::unordered_map<uint64_t, WarmStartEntry> m_warm_start;
		ConstraintSet const* m_source;
		uint64_t m_topology_revision;
		uint64_t m_parameter_revision;

		// Solve a mixed rigid/articulation system with multiplicative rigid and simultaneous impulse-ABA sweeps.
		CpuConstraintSolveMetrics SolveHybrid(CompiledConstraintSet const& constraints, BodyRemap const& remap, float timestep, CpuConstraintSolverConfig const& config);

	public:

		// Construct an empty solver with no retained warm-start state.
		CpuConstraintSolver();

		// Discard every retained impulse.
		void ClearWarmStart();

		// Solve physical velocities and optional split position correction in deterministic block order.
		CpuConstraintSolveMetrics Solve(CompiledConstraintSet const& constraints, BodyRemap const& remap, float timestep, CpuConstraintSolverConfig const& config = {});
	};
}
