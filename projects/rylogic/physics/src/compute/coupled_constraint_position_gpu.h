//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"

namespace pr::physics
{
	struct GpuCoupledConstraintSolver;

	// Observable resource and dispatch costs for the optional coupled position lane.
	struct GpuCoupledConstraintPositionStats
	{
		int m_link_pseudo_capacity;
		int m_generalized_pseudo_capacity;
		int m_dispatch_count;
		int m_active_count;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Focused one-submission output for detached coupled position parity tests.
	struct GpuCoupledConstraintPositionResult
	{
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuArticulation> m_articulations;
		std::vector<float> m_positions;
		std::vector<float> m_articulation_velocities;
		std::vector<float> m_articulation_accelerations;
		std::vector<GpuArticulationAbaScratch> m_articulation_scratch;
		std::vector<GpuCoupledConstraintIslandState> m_island_states;
	};

	// Lazily allocated GPU lane for detached generalized articulation-coupled position correction.
	struct GpuCoupledConstraintPosition
	{
	private:
		friend GpuCoupledConstraintSolver;

		Gpu& m_gpu;
		EngineConfig const& m_config;
		GpuCoupledConstraintVelocity& m_velocity;
		GpuCoupledConstraintPrepare& m_prepare;
		GpuArticulationImpulseAba& m_impulse_aba;
		ComputeStep m_cs_clear;
		ComputeStep m_cs_begin;
		ComputeStep m_cs_candidates;
		ComputeStep m_cs_gather;
		ComputeStep m_cs_select_trees;
		ComputeStep m_cs_validate_trees;
		ComputeStep m_cs_evaluate_merit;
		ComputeStep m_cs_commit_state;
		ComputeStep m_cs_commit_articulations;
		ComputeStep m_cs_finalize_islands;
		ComputeStep m_cs_apply;
		D3DPtr<ID3D12Resource> m_r_link_pseudo;
		D3DPtr<ID3D12Resource> m_r_generalized_pseudo;
		ConstraintSet const* m_source;
		float m_timestep;
		int m_body_count;
		int m_velocity_delta_count;
		GpuCoupledConstraintPositionStats m_stats;

	public:

		// Create fixed pipeline state without allocating optional pseudo-state resources.
		GpuCoupledConstraintPosition(GpuCoupledConstraintVelocity& velocity, EngineConfig const& config);

		// Prepare position-only exact-self inverses and clear detached pseudo state once per substep.
		bool Prepare(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, ID3D12Resource* link_to_world);

		// Execute one bounded-backtracking simultaneous pseudo-position sweep.
		void Run(GpuJob& job, ID3D12Resource* bodies);

		// Integrate converged rigid and articulation pseudo state exactly once into coordinates.
		void Apply(GpuJob& job, ID3D12Resource* bodies);

		// Prepare every dependency, solve configured position iterations, and read focused state through one submission.
		GpuCoupledConstraintPositionResult Solve(
			GpuJob& job,
			float timestep,
			GpuConstraintUpload const& constraint_upload,
			GpuArticulationUpload const& articulation_upload,
			std::span<GpuRigidBody const> bodies,
			std::span<GpuConstraintFrame const> link_to_world,
			std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override = {},
			std::span<GpuConstraintRow const> prepared_row_override = {});

		// Return current logical usage, retained capacities, and most recent dispatch count.
		GpuCoupledConstraintPositionStats const& Stats() const;

	private:

		// Release position-owned pseudo resources when no coupled push-out work remains.
		void ReleaseBuffers();

		// Create or geometrically grow compact pseudo link and generalized streams.
		void ResizeBuffers(CmdList& cmd_list);

		// Bind and dispatch one phase using the 64-DWORD common transaction layout.
		void DispatchCommon(GpuJob& job, ComputeStep& step, int phase, int attempt_index, int item_count, ID3D12Resource* bodies);

		// Bind and dispatch one complete-tree validation or commit phase.
		void DispatchArticulations(GpuJob& job, ComputeStep& step, int phase, int attempt_index, int item_count);

		// Bind and dispatch final rigid and articulation coordinate integration.
		void DispatchApply(GpuJob& job, ID3D12Resource* bodies);

		// Transition all resources used by the common transaction root layout.
		void PrepareCommonResources(GpuJob& job, ID3D12Resource* bodies);

		// Transition all resources used by complete-tree validation and pseudo commit.
		void PrepareArticulationResources(GpuJob& job);

		// Insert ordering across every mutable position and reused transaction resource.
		void CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies);
	};
}
