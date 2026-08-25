//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/constraint/constraint_solver.h"

namespace pr::physics::detail::constraint_solver
{
	inline constexpr int MaxBlockRows = 6;

	// Mutable state for one scalar row during a physical or pseudo-velocity solve.
	struct RuntimeRow
	{
		uint32_t m_compiled_index = 0;
		float m_impulse = 0.0f;
		float m_pending_delta = 0.0f;
		float m_residual_before = 0.0f;
		float m_lower = 0.0f;
		float m_upper = 0.0f;
		float m_target_velocity = 0.0f;
		float m_bias = 0.0f;
		float m_gamma = 0.0f;
		float m_position_error = 0.0f;
	};

	// Precomputed local response and mutable rows for one active solver block.
	struct RuntimeBlock
	{
		uint32_t m_compiled_index = 0;
		std::array<RuntimeRow, MaxBlockRows> m_rows = {};
		std::array<float, MaxBlockRows * MaxBlockRows> m_inverse_response = {};
		float m_response_scale = 0.0f;
		int m_row_count = 0;
		bool m_solvable = false;
	};

	// Current twist, inverse inertia, and accumulated physical impulse for one submitted rigid body.
	struct SolverBody
	{
		v8motion m_velocity = {};
		v8force m_momentum_delta = {};
		InertiaInv m_inertia_inv = {};
	};

	// Return whether every physically meaningful spatial-motion component is finite.
	bool IsFiniteMotion(v8motion const& value);

	// Return a stable map key for one generational constraint handle.
	uint64_t HandleKey(ConstraintHandle handle);

	// Map a canonical D6 row to its persistent linear-then-angular cache slot.
	int CacheSlot(CompiledConstraintRow const& row);

	// Return the current rigid-endpoint velocity represented by one compiled Jacobian row.
	float RigidRowVelocity(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, std::span<SolverBody const> bodies);

	// Apply one scalar row impulse to its ordinary rigid endpoints.
	void ApplyRigidImpulse(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, float impulse, std::span<SolverBody> bodies, bool accumulate_momentum);

	// Return one rigid-endpoint entry of J M^-1 J^T for two rows in the same block.
	float RigidResponse(CompiledConstraintRow const& lhs, CompiledConstraintRow const& rhs, CompiledConstraintBlock const& block, std::span<SolverBody const> bodies);

	// Factor one caller-assembled local response with shared regularization and singular-block diagnostics.
	void PrepareResponse(RuntimeBlock& runtime, std::array<float, MaxBlockRows * MaxBlockRows> response, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics);

	// Build and factor one local response using only ordinary rigid endpoints.
	void PrepareRigidResponse(RuntimeBlock& runtime, CompiledConstraintSet const& constraints, std::span<SolverBody const> bodies, CpuConstraintSolverConfig const& config, CpuConstraintSolveMetrics& metrics);

	// Compile one descriptor row into physical-velocity targets, regularization, and impulse bounds.
	bool PreparePhysicalRow(CompiledConstraintRow const& row, uint32_t compiled_index, float timestep, RuntimeRow& runtime);

	// Compile one hard passive row into a capped pseudo-velocity correction.
	bool PreparePositionRow(CompiledConstraintRow const& row, uint32_t compiled_index, float timestep, CpuConstraintSolverConfig const& config, RuntimeRow& runtime);

	// Apply the block's configured feasible-set projection to candidate impulses.
	void Project(std::array<float, MaxBlockRows>& candidate, RuntimeBlock const& runtime, CompiledConstraintBlock const& block);

	// Execute fixed-iteration block PGS over ordinary rigid endpoints.
	void SolveRigidBlocks(std::span<RuntimeBlock> runtime_blocks, CompiledConstraintSet const& constraints, std::span<SolverBody> bodies, int iterations, float relaxation, bool accumulate_momentum);

	// Measure the fixed-point projection residual and scalar-bound violations for ordinary rigid blocks.
	void MeasureRigidResidual(std::span<RuntimeBlock const> runtime_blocks, CompiledConstraintSet const& constraints, std::span<SolverBody const> bodies, CpuConstraintSolveMetrics& metrics);

	// Validate finite fixed-work settings before mutating any physical state.
	void ValidateSolverInputs(CpuConstraintSolverConfig const& config, float timestep);

	// Return the total kinetic energy of submitted ordinary rigid bodies.
	float RigidKineticEnergy(BodyRemap const& remap);
}
