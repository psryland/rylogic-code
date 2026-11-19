//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/coupled_constraint_work.hlsli"

namespace pr::physics
{
	struct GpuCoupledConstraintPosition;
	struct GpuCoupledConstraintSolver;

	// Observable resource and dispatch costs for the optional coupled velocity lane.
	struct GpuCoupledConstraintVelocityStats
	{
		int m_slot_capacity;
		int m_target_capacity;
		int m_adjacency_capacity;
		int m_island_capacity;
		int m_island_block_capacity;
		int m_articulation_range_capacity;
		int m_dispatch_count;
		int m_active_count;
		int m_pad0;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Focused one-submission output for transactional coupled velocity parity tests.
	struct GpuCoupledConstraintVelocityResult
	{
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintRow> m_rows; // Prepared/private trial coefficients, including a rejected final trial.
		std::vector<float> m_articulation_velocities;
		std::vector<float> m_articulation_accelerations;
		std::vector<GpuArticulationAbaScratch> m_articulation_scratch;
		std::vector<GpuArticulationAbaDofScratch> m_articulation_dof_scratch;
		std::vector<GpuCoupledConstraintIslandState> m_island_states;
		std::vector<GpuConstraintAcceptance> m_acceptance;
		std::vector<GpuConstraintAcceptedRows> m_accepted_rows;
		std::vector<uint32_t> m_results;
	};

	// Borrowed accepted owner storage. Result 3 preserves an existing numerical-solve rejection and its island failure flags.
	struct GpuCoupledConstraintTransactionOutput
	{
		ID3D12Resource* m_acceptance;
		ID3D12Resource* m_rows;
		ID3D12Resource* m_rigid_response;
		ID3D12Resource* m_velocity_response;
		ID3D12Resource* m_results;
		ID3D12Resource* m_island_states;
		ID3D12Resource* m_prepared_rows; // Current geometry/provisional coefficients, not the accepted cache.
		ID3D12Resource* m_blocks;
		int m_slot_count;
		int m_island_count;
	};

	// Lazily allocated GPU lane for deterministic simultaneous articulation-coupled velocity sweeps.
	struct GpuCoupledConstraintVelocity
	{
	private:
		friend GpuCoupledConstraintPosition;
		friend GpuCoupledConstraintSolver;

		Gpu& m_gpu;
		EngineConfig const& m_config;
		GpuCoupledConstraintPrepare& m_prepare;
		GpuArticulationImpulseAba& m_impulse_aba;
		ComputeStep m_cs_begin;
		ComputeStep m_cs_build_warm_start;
		ComputeStep m_cs_candidates;
		ComputeStep m_cs_gather;
		ComputeStep m_cs_select_trees;
		ComputeStep m_cs_validate_trees;
		ComputeStep m_cs_validate_warm_start;
		ComputeStep m_cs_evaluate_merit;
		ComputeStep m_cs_commit;
		ComputeStep m_cs_finalize_islands;
		D3DPtr<ID3D12Resource> m_r_block_topology;
		D3DPtr<ID3D12Resource> m_r_targets;
		D3DPtr<ID3D12Resource> m_r_adjacency;
		D3DPtr<ID3D12Resource> m_r_articulation_islands;
		D3DPtr<ID3D12Resource> m_r_islands;
		D3DPtr<ID3D12Resource> m_r_island_blocks;
		D3DPtr<ID3D12Resource> m_r_scratch;
		D3DPtr<ID3D12Resource> m_r_contributions;
		D3DPtr<ID3D12Resource> m_r_target_impulses;
		D3DPtr<ID3D12Resource> m_r_island_states;
		D3DPtr<ID3D12Resource> m_r_island_failures;
		D3DPtr<ID3D12Resource> m_r_tree_selection;
		D3DPtr<ID3D12Resource> m_r_tree_results;
		ConstraintSet const* m_source;
		int m_slot_count;
		int m_target_count;
		int m_adjacency_count;
		int m_island_count;
		int m_island_block_count;
		int m_articulation_range_count;
		int m_mobility_count;
		GpuCoupledConstraintVelocityStats m_stats;
		ComputeStep m_cs_transaction_recover;
		ComputeStep m_cs_transaction_select;
		ComputeStep m_cs_transaction_validate;
		ComputeStep m_cs_transaction_admit_seed;
		ComputeStep m_cs_transaction_seed;
		ComputeStep m_cs_transaction_collect;
		ComputeStep m_cs_transaction_stage;
		ComputeStep m_cs_transaction_publish;
		ComputeStep m_cs_capture_reference;
		ComputeStep m_cs_choose_reference;
		ComputeStep m_cs_admit_initialization;
		ComputeStep m_cs_restore_reference;
		ComputeStep m_cs_begin_trial;
		D3DPtr<ID3D12Resource> m_r_transaction_acceptance;
		D3DPtr<ID3D12Resource> m_r_transaction_rows;
		D3DPtr<ID3D12Resource> m_r_rigid_response;
		D3DPtr<ID3D12Resource> m_r_velocity_response;
		D3DPtr<ID3D12Resource> m_r_transaction_results;
		D3DPtr<ID3D12Resource> m_r_work_bodies;
		D3DPtr<ID3D12Resource> m_r_work_velocities;
		D3DPtr<ID3D12Resource> m_r_work_aba_scratch;
		D3DPtr<ID3D12Resource> m_r_reference_momenta;
		D3DPtr<ID3D12Resource> m_r_reference_velocities;
		D3DPtr<ID3D12Resource> m_r_initial_coefficients;
		D3DPtr<ID3D12Resource> m_r_work_spans;
		D3DPtr<ID3D12Resource> m_r_work_indices;
		D3DPtr<ID3D12Resource> m_r_reference_choices;
		D3DPtr<ID3D12Resource> m_r_work_motion;
		std::vector<GpuCoupledConstraintWorkSpan> m_work_spans;
		std::vector<uint32_t> m_work_indices;
		int m_rigid_work_count = 0;
		std::vector<std::byte> m_membership_key;
		std::vector<ConstraintEndpointIdentity> m_membership_identities;
		ConstraintSet const* m_membership_source = nullptr;
		uint64_t m_membership_revision = 0;
		uint64_t m_transaction_membership_revision = 0;
		GpuConstraintEpoch m_transaction_epoch = {};
		bool m_transaction_ready = false;
		int m_transaction_body_count = 0;
		int m_transaction_velocity_count = 0;
		int m_transaction_link_count = 0;

		// Copy each sweep before CSBeginCoupledVelocity resets its GPU counters. The owning job must complete before CPU access.
		struct SweepProfile
		{
			int m_island_count;
			int m_recorded_dispatches;
			ReadbackAlloc m_states;
			ReadbackAlloc m_results;
		};
		std::vector<SweepProfile> m_sweep_profiles;

	public:

		// Create all fixed pipeline state without allocating coupled-feature resources.
		GpuCoupledConstraintVelocity(GpuCoupledConstraintPrepare& prepare, GpuArticulationImpulseAba& impulse_aba, EngineConfig const& config);

		// Upload deterministic persistent topology after matching constraint and articulation participation streams.
		bool Upload(GpuJob& job, GpuConstraintUpload const& upload);

		// Return accepted coupled load ownership for the shared completed-substep detector.
		GpuCoupledConstraintLoadOutput LoadOutput();

		// Prepare detached storage with stable membership; supplied geometry must already have been compiled by coupled preparation.
		void PrepareVelocityTransactions(GpuJob& job, GpuConstraintEpoch epoch, int body_count);

		// Attempt one shared numerical sweep privately, then admit and publish each complete island or preserve its live state and record.
		void CommitVelocityTransactions(GpuJob& job, int body_count, ID3D12Resource* bodies, int substep_index = 0);

		// Return accepted storage and explicit attempt outcomes without recording work.
		GpuCoupledConstraintTransactionOutput TransactionOutput();

		// Prepare all dependent production lanes and read one coupled sweep through exactly one GPU submission.
		// Caller-supplied epoch distinguishes continuation from a fresh physical substep.
		GpuCoupledConstraintVelocityResult Solve(
			GpuJob& job,
			GpuConstraintEpoch epoch,
			GpuConstraintUpload const& constraint_upload,
			GpuArticulationUpload const& articulation_upload,
			std::span<GpuRigidBody const> bodies,
			std::span<GpuConstraintFrame const> link_to_world,
			std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override = {},
			std::span<GpuConstraintRow const> prepared_row_override = {});

		// Return current logical usage, retained capacities, and most recent coupled dispatch count.
		GpuCoupledConstraintVelocityStats Stats() const;

	private:

		// Release every optional coupled velocity resource when no coupled work remains.
		void ReleaseBuffers();

		// Create or geometrically grow topology and detached transaction buffers.
		void ResizeBuffers(GpuJob& job);

		// Bind one coupled phase against the complete stable resource layout and the caller's matching velocity cache.
		void Dispatch(GpuJob& job, ComputeStep& step, int phase, int attempt_index, int item_count, int body_count, ID3D12Resource* bodies, GpuArticulationVelocityBuffers state, int substep_index);

		// Transition the common coupled resources into the states required by a coupled shader phase.
		void PrepareResources(GpuJob& job, ID3D12Resource* bodies, GpuArticulationVelocityBuffers state);

		// Order every mutable coupled resource before its next dependent phase.
		void CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies);

		// Bind an accounting phase; the live and working states remain distinct throughout an attempt.
		void DispatchTransaction(GpuJob& job, ComputeStep& step, int body_count, ID3D12Resource* bodies);

		// Bind the private complete-work signature; one CBV plus thirty-one descriptors occupies exactly sixty-four DWORDs.
		void DispatchTrial(GpuJob& job, ComputeStep& step, int body_count, ID3D12Resource* bodies, int phase = 0, int attempt_index = 0);
	};
}
