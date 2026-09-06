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
	};

	// Packed output storage and work recorded before the frame's single readback.
	struct FrameOutputFeatureStats
	{
		int m_body_count = 0;
		int m_event_capacity = 0;
		int m_articulation_count = 0;
		int m_constraint_break_count = 0;
		int m_coupled_failure_count = 0;
		int m_dispatch_count = 0;
		int m_readback_count = 0;
		uint64_t m_logical_bytes = 0;
		uint64_t m_allocated_bytes = 0;
		uint64_t m_readback_bytes = 0;
	};

	// Bounded failure categories retained after a step rejects its gathered output.
	enum class EStepFailure
	{
		None,
		CollisionPairCapacity,
		CollisionContactCapacity,
		ArticulationIntegration,
		CoupledConstraintNonConvergence,
	};

	// First bounded failure from the most recently completed or rejected submitted frame.
	struct StepFailureStats
	{
		EStepFailure m_reason = EStepFailure::None;
		int m_substep_index = -1;
		int m_item_index = -1;
		int m_status = 0;
		int m_iteration_count = 0;
		uint64_t m_identity = 0;
		float m_residual = 0.0f;

		// True when the submitted frame did not report a terminal bounded failure.
		bool Succeeded() const
		{
			return m_reason == EStepFailure::None;
		}
	};

	// One coupled island whose bounded velocity or position transaction rejected every relaxation attempt.
	struct CoupledConstraintFailureEvent
	{
		uint32_t m_failure_flags = 0;
		int m_substep_index = -1;
		int m_phase = -1;
		int m_island_index = -1;
		int m_iteration_count = 0;
		float m_relaxation = 0.0f;
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
