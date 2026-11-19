//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Common work and storage accounting for one optional GPU feature lane.
	struct FeatureResourceStats
	{
		int m_dispatch_count = 0;
		uint64_t m_logical_bytes = 0;
		uint64_t m_allocated_bytes = 0;
	};

	// Stable-slot, scratch, and break-latch costs for persistent constraints.
	struct ConstraintFeatureStats
	{
		int m_declared_count = 0;
		int m_active_count = 0;
		int m_breakable_count = 0;
		int m_slot_capacity = 0;
		int m_body_capacity = 0;
		int m_break_capacity = 0;
		FeatureResourceStats m_resources;
	};

	// Packed topology and optional GPU resource costs for reduced-coordinate articulations.
	struct ArticulationFeatureStats
	{
		int m_articulation_count = 0;
		int m_link_count = 0;
		int m_dof_count = 0;
		int m_position_count = 0;
		int m_velocity_count = 0;
		int m_articulation_capacity = 0;
		int m_link_capacity = 0;
		int m_dof_capacity = 0;
		int m_position_capacity = 0;
		int m_velocity_capacity = 0;
		FeatureResourceStats m_resources;
	};

	// Opt-in PR_PIX_DETAIL totals from every completed coupled velocity sweep, not a final-sweep snapshot or distinct-owner count.
	struct CoupledVelocityWorkStats
	{
		bool m_has_snapshots = false;
		uint64_t m_sweeps = 0;
		uint64_t m_recorded_dispatches = 0;
		uint64_t m_owner_sweeps = 0;
		uint64_t m_started_trials = 0; // Pending owners selected by begin-trial; later candidate/cache stages can still reject.
		uint64_t m_accepted_owner_sweeps = 0;
		uint64_t m_zero_attempt_accepted_owner_sweeps = 0;
		uint64_t m_admission_rejected_owner_sweeps = 0;
		uint64_t m_solve_rejected_owner_sweeps = 0;
		uint64_t m_numerical_failure_owner_sweeps = 0;
	};

	// Topology bounds and costs for articulation-coupled persistent constraints and contacts.
	struct CoupledFeatureStats
	{
		int m_constraint_count = 0;
		int m_constraint_slot_capacity = 0;
		int m_target_capacity = 0;
		int m_island_capacity = 0;
		int m_island_block_capacity = 0;
		int m_contact_capacity = 0;
		int m_contact_target_capacity = 0;
		int m_contact_participant_capacity = 0;
		int m_contact_tree_capacity = 0;
		FeatureResourceStats m_resources;
		CoupledVelocityWorkStats m_velocity_work;
	};

	// Packed output storage and work recorded for one frame. Readback bytes become known after completion when event copies are predicated.
	struct FrameOutputFeatureStats
	{
		int m_body_count = 0;
		int m_event_capacity = 0;
		int m_articulation_count = 0;
		int m_constraint_break_count = 0;
		int m_constraint_rejection_slot_count = 0;
		int m_dispatch_count = 0;
		int m_readback_count = 0;
		uint64_t m_logical_bytes = 0;
		uint64_t m_allocated_bytes = 0;
		uint64_t m_readback_bytes = 0;
	};

	// Bounded failure categories retained after a step rejects its gathered output.
	enum class EStepFailure
	{
		None = 0,
		CollisionPairCapacity = 1,
		CollisionContactCapacity = 2,
		ArticulationIntegration = 3,
		ConstraintNumerics = 5,
		ContactNumerics = 6,
	};

	// First terminal solver/capacity failure. Its absence does not establish that a frame was published.
	struct StepFailureStats
	{
		EStepFailure m_reason = EStepFailure::None;
		int m_substep_index = -1;
		int m_item_index = -1;
		int m_status = 0;
		int m_iteration_count = 0;
		uint64_t m_identity = 0;
		float m_residual = 0.0f;

		// True when a terminal solver/capacity failure has been recorded.
		bool HasTerminalFailure() const
		{
			return m_reason != EStepFailure::None;
		}
	};

	// Identity domain of a persistent constraint commit unit.
	enum class EConstraintRejectionOwner { OrdinaryBlock = 0, CoupledIsland = 1 };

	// Non-terminal owner outcomes; fatal numerical context uses StepFailureStats instead.
	enum class EConstraintRejectionOutcome { AdmissionRejected = 1, SolveRejected = 3 };

	// Physical velocity or detached position transaction.
	enum class EConstraintRejectionPhase { Velocity = 0, Position = 1 };

	// First rejection for one owner in a successfully published frame. Numerical details are zero when no numerical solve ran.
	struct ConstraintRejectionEvent
	{
		EConstraintRejectionOwner m_owner;
		uint32_t m_index = 0;
		uint32_t m_generation = 0;
		EConstraintRejectionOutcome m_outcome;
		EConstraintRejectionPhase m_phase;
		uint32_t m_failure_flags = 0;
		int m_substep_index = -1;
		// Actual numerical attempts for this owner, excluding later no-op dispatches.
		int m_iteration_count = 0;
		float m_relaxation = 0.0f;
		// Best-effort physical-unit work; zero also denotes unavailable or unrepresentable diagnostic output, never an acceptance certificate.
		float m_merit_change = 0.0f;
	};

	// Public bounded counts, retained capacities, storage costs, and failure state for one engine frame.
	struct EngineFeatureStats
	{
		ConstraintFeatureStats m_constraints;
		ArticulationFeatureStats m_articulations;
		CoupledFeatureStats m_coupled;
		FrameOutputFeatureStats m_frame_output;
		StepFailureStats m_failure;
	};
}
