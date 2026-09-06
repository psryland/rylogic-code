//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/articulation_link_proxies_gpu.h"

namespace pr::physics
{
	// Observable linear storage and dispatch costs for the optional transient articulation-contact lane.
	struct GpuCoupledContactStats
	{
		int m_contact_capacity;
		int m_target_capacity;
		int m_participant_capacity;
		int m_tree_capacity;
		int m_rigid_pseudo_capacity;
		int m_link_pseudo_capacity;
		int m_generalized_pseudo_capacity;
		int m_dispatch_count;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Lazily allocated solver for GPU-generated contacts involving one or more articulation-link proxies.
	struct GpuCoupledContactSolver
	{
	private:
		Gpu& m_gpu;
		EngineConfig const& m_config;
		GpuArticulationForceAba& m_aba;
		GpuArticulationLinkProxies& m_proxies;
		GpuArticulationMobility m_mobility;
		GpuArticulationImpulseAba m_impulse_aba;
		ContactEndpointSorter m_endpoint_sorter;
		ComputeStep m_cs_clear;
		ComputeStep m_cs_prepare;
		ComputeStep m_cs_prepare_position;
		ComputeStep m_cs_begin;
		ComputeStep m_cs_build_warm_start;
		ComputeStep m_cs_build_candidates;
		ComputeStep m_cs_build_position_candidates;
		ComputeStep m_cs_gather;
		ComputeStep m_cs_validate_trees;
		ComputeStep m_cs_validate_position_trees;
		ComputeStep m_cs_select_trees;
		ComputeStep m_cs_commit;
		ComputeStep m_cs_commit_position_articulations;
		ComputeStep m_cs_apply_position;
		D3DPtr<ID3D12Resource> m_r_blocks;
		D3DPtr<ID3D12Resource> m_r_scratch;
		D3DPtr<ID3D12Resource> m_r_contributions;
		D3DPtr<ID3D12Resource> m_r_endpoint_keys;
		D3DPtr<ID3D12Resource> m_r_endpoint_order;
		D3DPtr<ID3D12Resource> m_r_target_impulses;
		D3DPtr<ID3D12Resource> m_r_participant_degrees;
		D3DPtr<ID3D12Resource> m_r_tree_selection;
		D3DPtr<ID3D12Resource> m_r_tree_results;
		D3DPtr<ID3D12Resource> m_r_state;
		D3DPtr<ID3D12Resource> m_r_rigid_pseudo;
		D3DPtr<ID3D12Resource> m_r_link_pseudo;
		D3DPtr<ID3D12Resource> m_r_generalized_pseudo;
		D3DPtr<ID3D12Resource> m_r_uav_sentinel;
		D3DPtr<ID3D12Resource> m_r_counters;
		D3DPtr<ID3D12Resource> m_r_contacts;
		D3DPtr<ID3D12Resource> m_r_bodies;
		D3DPtr<ID3D12Resource> m_r_materials;
		int m_max_contacts;
		int m_body_count;
		int m_rigid_body_count;
		int m_articulation_count;
		int m_link_count;
		int m_target_count;
		int m_participant_count;
		int m_mobility_count;
		int m_velocity_delta_count;
		int m_work_count;
		float m_dt;
		float m_restitution_scale;
		bool m_active;
		bool m_position_active;
		GpuCoupledContactStats m_stats;

	public:

		// Create fixed pipeline state while leaving every contact-dependent resource unallocated.
		GpuCoupledContactSolver(Gpu& gpu, GpuArticulationForceAba& aba, GpuArticulationLinkProxies& proxies, EngineConfig const& config);

		// Select every packed articulation for exact whole-tree contact response in canonical forest order.
		bool Upload(GpuJob& job, GpuArticulationUpload const& upload);

		// Release retained optional resources when the frame contains no shaped articulation links.
		void Deactivate();

		// Build exact-self contact blocks and deterministic transient endpoint topology at the integrated configuration.
		void PrepareVelocity(
			GpuJob& job,
			float dt,
			int body_count,
			int rigid_body_count,
			int max_contacts,
			D3DPtr<ID3D12Resource> counters,
			D3DPtr<ID3D12Resource> contacts,
			D3DPtr<ID3D12Resource> bodies,
			D3DPtr<ID3D12Resource> materials,
			float restitution_scale);

		// Apply cached normal impulses through rigid endpoints and complete articulation trees as one finite transaction.
		void ApplyWarmStart(GpuJob& job);

		// Execute one degree-damped block-Jacobi velocity iteration through a complete-tree ABA response.
		void SolveVelocityIteration(GpuJob& job);

		// Clear contact-owned pseudo state before detached penetration correction.
		bool PreparePosition(GpuJob& job, int iteration_count);

		// Execute one zero-based monotone detached penetration-correction iteration with acceleration confined to the initial sweep.
		void SolvePositionIteration(GpuJob& job, int position_iteration_index);

		// Apply converged pseudo state once to rigid transforms and articulation coordinates.
		void ApplyPosition(GpuJob& job);

		// Return aggregate logical usage, retained storage, and dispatch cost for the complete current frame.
		GpuCoupledContactStats Stats() const;

	private:

		// Create or geometrically grow all contact, endpoint, target, participant, and tree transaction buffers.
		void ResizeBuffers(CmdList& cmd_list);

		// Refresh logical and retained byte counts after topology or optional position storage changes.
		void UpdateMemoryStats();

		// Run one warm-start, velocity, or position candidate through deterministic gather and detached ABA evaluation.
		void RunTransaction(GpuJob& job, ComputeStep& build_step, int phase, int position_iteration_index);

		// Bind and dispatch one common contact phase over a non-empty logical work range with an optional zero-based position-sweep index.
		void DispatchCommon(GpuJob& job, ComputeStep& step, int phase, int item_count, int position_iteration_index = -1);

		// Bind and dispatch one complete-tree position validation or pseudo-state commit phase.
		void DispatchPositionArticulations(GpuJob& job, ComputeStep& step, int phase);

		// Bind and dispatch final rigid and articulation coordinate integration.
		void DispatchApplyPosition(GpuJob& job);

		// Transition the common contact and articulation resources into the states required by contact shaders.
		void PrepareCommonResources(GpuJob& job);

		// Order every mutable resource before the next dependent contact or ABA phase.
		void CommitUavBarriers(GpuJob& job);
	};
}
