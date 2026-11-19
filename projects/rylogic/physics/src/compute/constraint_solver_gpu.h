//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/constraint/constraint_gpu.h"
#include "src/utility/gpu.h"
#include "src/compute/constraint_transaction.hlsli"

namespace pr::physics
{
	struct GpuCoupledConstraintPrepare;
	struct GpuCoupledConstraintPosition;
	struct GpuCoupledConstraintVelocity;

	// Observable stable-slot storage and dispatch costs for the optional persistent-constraint lane.
	struct GpuConstraintSolverStats
	{
		int m_slot_count;
		int m_active_count;
		int m_breakable_count;
		int m_slot_capacity;
		int m_body_capacity;
		int m_break_capacity;
		int m_dispatch_count;
		int m_pad0;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Optional stable-slot overload stream copied into the frame's single packed readback.
	struct GpuConstraintBreakOutput
	{
		ID3D12Resource* m_states = nullptr;
		int m_slot_count = 0;
	};

	// Borrowed first-rejection-per-owner frame stream, including empty ordinary slots and coupled island entries.
	struct GpuConstraintRejectionOutput
	{
		ID3D12Resource* m_states = nullptr;
		int m_slot_count = 0;
	};

	// Borrowed coupled acceptance and slot mapping for completed-substep load sampling.
	struct GpuCoupledConstraintLoadOutput
	{
		ID3D12Resource* m_acceptance = nullptr;
		ID3D12Resource* m_rows = nullptr;
		ID3D12Resource* m_topology = nullptr;
		int m_island_count = 0;
	};

	// Borrowed stable-slot transaction resources for ordered GPU consumers and native readback.
	struct GpuConstraintTransactionOutput
	{
		ID3D12Resource* m_records;
		ID3D12Resource* m_results;
		int m_slot_count;
	};

	// Completed native diagnostic ownership state; prepared geometry does not contain accepted physical coefficients.
	struct GpuConstraintSolveResult
	{
		std::vector<GpuConstraintRigidRecord> m_records;
		std::vector<uint32_t> m_results;
	};

	// Lazily allocated GPU lane for typed persistent rigid constraints.
	struct GpuConstraintSolver
	{
	private:
		friend GpuCoupledConstraintPrepare;
		friend GpuCoupledConstraintPosition;
		friend GpuCoupledConstraintVelocity;

		Gpu& m_gpu;
		EngineConfig const& m_config;
		ComputeStep m_cs_compile;
		ComputeStep m_cs_reset_colours;
		ComputeStep m_cs_assign_colour_level;
		ComputeStep m_cs_finish_colours;
		ComputeStep m_cs_clear_pseudo_velocity;
		ComputeStep m_cs_prepare_position_solve;
		ComputeStep m_cs_solve_position;
		ComputeStep m_cs_apply_position;
		ComputeStep m_cs_prepare_velocity_solve;
		ComputeStep m_cs_commit_velocity_transaction;
		ComputeStep m_cs_detect_breakage;
		ComputeStep m_cs_solve_position_fused;
		D3DPtr<ID3D12Resource> m_r_endpoints;
		D3DPtr<ID3D12Resource> m_r_descriptors;
		D3DPtr<ID3D12Resource> m_r_blocks;
		D3DPtr<ID3D12Resource> m_r_rows;
		D3DPtr<ID3D12Resource> m_r_state;
		D3DPtr<ID3D12Resource> m_r_pseudo_velocities;
		D3DPtr<ID3D12Resource> m_r_break_states;
		D3DPtr<ID3D12Resource> m_r_colour_dispatch;
		D3DPtr<ID3D12Resource> m_r_solve_cache;
		D3DPtr<ID3D12Resource> m_r_colour_schedule;
		D3DPtr<ID3D12Resource> m_r_colour_masks;
		D3DPtr<ID3D12Resource> m_r_rigid_records;
		D3DPtr<ID3D12Resource> m_r_transaction_results;
		D3DPtr<ID3D12Resource> m_r_frame_rejections;
		int m_rejection_capacity = 0;
		int m_rejection_count = 0;
		GpuConstraintEpoch m_transaction_epoch;
		ConstraintSet const* m_transaction_source;
		int m_transaction_capacity;
		D3DPtr<ID3D12CommandSignature> m_cmd_sig;
		std::vector<GpuConstraintEndpoint> m_endpoint_shadow;
		std::vector<ConstraintEndpointIdentity> m_endpoint_identity_shadow;
		std::vector<GpuConstraintDesc> m_descriptor_shadow;
		// Per-slot input clocks outlive source changes and allocation/shadow resizing, so no retained receipt can alias a new event.
		std::vector<uint32_t> m_invalidation_revisions;
		std::vector<int> m_body_degrees;
		ConstraintColourSchedule m_colour_schedule;
		ConstraintSet const* m_source;
		int m_capacity;
		int m_body_capacity;
		int m_break_capacity;
		int m_slot_count;
		int m_active_count;
		int m_breakable_count;
		int m_colour_count;
		int m_body_count;
		int m_dispatch_count;
		int m_break_substep_index;

	public:

		// Create pipeline state without allocating feature-dependent buffers.
		explicit GpuConstraintSolver(Gpu& gpu, EngineConfig const& config);

		// Upload shared frame-local endpoints and changed persistent descriptors, returning whether independent rigid work is active.
		bool Upload(GpuJob& job, GpuConstraintUpload const& upload);

		// Compile geometry at the current pose without changing accepted ownership or position scratch.
		void CompileRows(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Initialize ordinary-owned position scratch before the first position writer.
		void ClearPositionState(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Build each block's split-position response once before the position iterations. Call after every position producer is initialized.
		void PreparePositionSolve(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Execute one coloured split-position sweep using already initialized shared pseudo velocity, without clearing it or changing physical momentum.
		void SolvePositionIteration(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Apply converged pseudo twists to body transforms after all split-position iterations.
		void ApplyPosition(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Return adequately sized rigid pseudo storage for a shared position solve; the caller initializes it before use.
		D3DPtr<ID3D12Resource> PseudoVelocityStorage(GpuJob& job, int body_count);

		// Build each block's physical response once before the velocity iterations. Call after the last body transform change in the substep.
		void PrepareVelocitySolve(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Prepare owner storage/context only, without geometry, pseudo clearing, or physical publication.
		void PrepareVelocityTransactions(GpuJob& job, GpuConstraintEpoch epoch);

		// Solve and publish complete ordinary blocks in dependency order. Zero-row owners still retire their accepted cache and response.
		void CommitVelocityTransactions(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> bodies, int substep_index = 0);

		// Return the prepared owner's accepted records and latest attempt results without recording work.
		GpuConstraintTransactionOutput TransactionOutput();

		// Direct native diagnostic of the production transaction phases, with committed records and per-owner admission results.
		void SolveTransactions(GpuJob& job, GpuConstraintEpoch epoch, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintRigidRecord> records, std::span<uint32_t> results);

		// Sample final accepted completed-substep loads and latch strict thresholds; no solve may follow in this epoch.
		void DetectBreakage(GpuJob& job, GpuConstraintEpoch epoch, int substep_index, GpuCoupledConstraintLoadOutput coupled = {});

		// Return the optional full stable-slot break stream for the final frame gather.
		GpuConstraintBreakOutput BreakOutput();

		// Return the only persistent-constraint rejection stream for the frame gather.
		GpuConstraintRejectionOutput RejectionOutput();

		// Return the current nonempty upload's frame-local solver state, or null when no slot stream was submitted.
		ID3D12Resource* FrameState();

		// Mirror completed breaks into the endpoint shadow so an immediate explicit repair invalidates stale GPU runtime state.
		void AcknowledgeBreaks(std::span<GpuConstraintBreakState const> states);

		// Force the next upload to clear retained blocks after an abandoned or failed frame.
		void InvalidateRuntimeState();

		// True when the most recently uploaded set contains enabled independent rigid constraints.
		bool Active() const;

		// Return current logical usage, retained capacities, and the most recent frame dispatch count.
		GpuConstraintSolverStats Stats() const;

		// Invalidate retained frame timing when a step does not submit this optional solver lane.
		void Deactivate();

		// Complete the production position/velocity phases and return accepted records/results; optional spans expose prepared geometry and break samples.
		GpuConstraintSolveResult Solve(GpuJob& job, GpuConstraintEpoch epoch, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintBlock> blocks = {}, std::span<GpuConstraintRow> rows = {}, std::span<GpuConstraintBreakState> break_states = {});

	private:

		// Create required upload buffers; Upload must finish initialization before publishing their capacity.
		bool ResizeBuffers(GpuJob& job, int capacity);

		// Create or grow shared rigid pseudo-twist storage for either independent or coupled position correction.
		void EnsurePseudoVelocityStorage(GpuJob& job, int body_count);

		// Create or geometrically grow the optional per-slot overload latch.
		void EnsureBreakStateStorage(GpuJob& job, int slot_count);

		// Build the colouring schedule for 'endpoints' and upload it. The schedule buffer has one entry per stable slot.
		void UploadColourSchedule(GpuJob& job, std::span<GpuConstraintEndpoint const> endpoints);

		// Record the colouring passes, which also write the per-colour indirect arguments that every later sweep in this frame reads.
		void RecordColouring(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Bind the common constraint root signature and resources for one compute step.
		// 'colour_dispatch' is written only by the colouring step. Other steps must pass a UAV-state placeholder, because the arguments buffer is
		// in the indirect-argument state while the colour sweeps read it.
		void Bind(GpuJob& job, ComputeStep& step, float timestep, int body_count, int colour, int position_iterations, D3DPtr<ID3D12Resource> bodies, ID3D12Resource* colour_dispatch);

		// Insert UAV ordering between dependent compiler, colouring, and solver passes.
		void CommitUavBarriers(GpuJob& job, D3DPtr<ID3D12Resource> bodies);

		// Return an upper bound on the number of colours that greedy colouring can use for 'endpoints'.
		int ColourBound(std::span<GpuConstraintEndpoint const> endpoints);

		// Record one colour sweep in the established colour order. Small slot sets use the single-group fused step; larger sets use one indirect pass for
		// each colour that the colour bound allows.
		void RecordColourSweep(GpuJob& job, ComputeStep& per_colour_step, ComputeStep& fused_step, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);
	};
}
