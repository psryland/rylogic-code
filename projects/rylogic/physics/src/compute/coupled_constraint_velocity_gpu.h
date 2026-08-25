//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"

namespace pr::physics
{
	// Observable resource and dispatch costs for the optional coupled velocity lane.
	struct GpuCoupledConstraintVelocityStats
	{
		int m_slot_capacity;
		int m_target_capacity;
		int m_adjacency_capacity;
		int m_island_capacity;
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
		std::vector<GpuConstraintRow> m_rows;
		std::vector<float> m_articulation_velocities;
		std::vector<float> m_articulation_accelerations;
		std::vector<GpuArticulationAbaScratch> m_articulation_scratch;
		std::vector<GpuCoupledConstraintIslandState> m_island_states;
	};

	// Lazily allocated GPU lane for deterministic simultaneous articulation-coupled velocity sweeps.
	struct GpuCoupledConstraintVelocity
	{
	private:

		Gpu& m_gpu;
		EngineConfig const& m_config;
		GpuCoupledConstraintPrepare& m_prepare;
		GpuArticulationImpulseAba& m_impulse_aba;
		ComputeStep m_cs_begin;
		ComputeStep m_cs_candidates;
		ComputeStep m_cs_gather;
		ComputeStep m_cs_select_trees;
		ComputeStep m_cs_validate_trees;
		ComputeStep m_cs_accept_islands;
		ComputeStep m_cs_commit;
		ComputeStep m_cs_finalize_islands;
		D3DPtr<ID3D12Resource> m_r_block_topology;
		D3DPtr<ID3D12Resource> m_r_targets;
		D3DPtr<ID3D12Resource> m_r_adjacency;
		D3DPtr<ID3D12Resource> m_r_articulation_islands;
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
		int m_articulation_range_count;
		int m_mobility_count;
		GpuCoupledConstraintVelocityStats m_stats;

	public:

		// Create all fixed pipeline state without allocating coupled-feature resources.
		GpuCoupledConstraintVelocity(GpuCoupledConstraintPrepare& prepare, GpuArticulationImpulseAba& impulse_aba, EngineConfig const& config);

		// Upload deterministic persistent topology after matching constraint and articulation participation streams.
		bool Upload(GpuJob& job, GpuConstraintUpload const& upload);

		// Execute one fixed-relaxation transactional simultaneous coupled velocity sweep.
		void Run(GpuJob& job, int body_count, ID3D12Resource* bodies);

		// Prepare all dependent production lanes and read one coupled sweep through exactly one GPU submission.
		GpuCoupledConstraintVelocityResult Solve(
			GpuJob& job,
			float timestep,
			GpuConstraintUpload const& constraint_upload,
			GpuArticulationUpload const& articulation_upload,
			std::span<GpuRigidBody const> bodies,
			std::span<GpuConstraintFrame const> link_to_world,
			std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override = {});

		// Return current logical usage, retained capacities, and most recent coupled dispatch count.
		GpuCoupledConstraintVelocityStats const& Stats() const;

	private:

		// Release every optional coupled velocity resource when no coupled work remains.
		void ReleaseBuffers();

		// Create or geometrically grow topology and detached transaction buffers.
		void ResizeBuffers(CmdList& cmd_list);

		// Bind one coupled phase against the complete stable resource layout.
		void Dispatch(GpuJob& job, ComputeStep& step, int selection_mode, int item_count, int body_count, ID3D12Resource* bodies);

		// Transition the common coupled resources into the states required by a coupled shader phase.
		void PrepareResources(GpuJob& job, ID3D12Resource* bodies);

		// Order every mutable coupled resource before its next dependent phase.
		void CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies);
	};
}
