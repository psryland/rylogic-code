//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/articulation/articulation_gpu_data.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuArticulationMidpoint;
	struct GpuArticulationMobility;
	struct GpuArticulationImpulseAba;
	struct GpuArticulationLinkProxies;
	struct GpuCoupledConstraintPrepare;

	// Observable allocation and dispatch state for validating the optional articulation lane.
	struct GpuArticulationForceAbaStats
	{
		int m_articulation_capacity;
		int m_link_capacity;
		int m_dof_capacity;
		int m_position_capacity;
		int m_velocity_capacity;
		int m_force_capacity;
		int m_external_force_capacity;
		int m_child_capacity;
		int m_level_link_capacity;
		int m_acceleration_capacity;
		int m_scratch_capacity;
		int m_dof_scratch_capacity;
		int m_joint_matrix_capacity;
		int m_dispatch_count;

		// Active scratch follows 336L + 64D + 4*sum(d_j^2), independent of retained high-water capacity.
		size_t m_logical_scratch_bytes;

		// Sum of typed D3D buffer widths, including any active 64-byte sentinels but excluding heap-alignment overhead.
		size_t m_allocated_feature_bytes;
	};

	// Test-only force-ABA output recovered by one submission after all four production passes.
	struct GpuArticulationForceAbaResult
	{
		std::vector<float> m_accelerations;
		std::vector<GpuArticulationSpatialVector> m_link_accelerations;
		std::vector<int> m_solve_valid;

		// True when every packed link completed a non-singular joint and root solve.
		bool AllValid() const;
	};

	// Lazily allocated D3D12 lane for deterministic pure-tree articulation forward dynamics.
	struct GpuArticulationForceAba
	{
	private:
		friend GpuArticulationMidpoint;
		friend GpuArticulationMobility;
		friend GpuArticulationImpulseAba;
		friend GpuArticulationLinkProxies;
		friend GpuCoupledConstraintPrepare;

		Gpu& m_gpu;
		ComputeStep m_cs_prepare;
		ComputeStep m_cs_inward;
		ComputeStep m_cs_root;
		ComputeStep m_cs_outward;
		D3DPtr<ID3D12Resource> m_r_articulations;
		D3DPtr<ID3D12Resource> m_r_links;
		D3DPtr<ID3D12Resource> m_r_dofs;
		D3DPtr<ID3D12Resource> m_r_positions;
		D3DPtr<ID3D12Resource> m_r_velocities;
		D3DPtr<ID3D12Resource> m_r_forces;
		D3DPtr<ID3D12Resource> m_r_external_forces;
		D3DPtr<ID3D12Resource> m_r_children;
		D3DPtr<ID3D12Resource> m_r_level_links;
		D3DPtr<ID3D12Resource> m_r_accelerations;
		D3DPtr<ID3D12Resource> m_r_scratch;
		D3DPtr<ID3D12Resource> m_r_dof_scratch;
		D3DPtr<ID3D12Resource> m_r_joint_matrix_scratch;
		D3DPtr<ID3D12Resource> m_r_srv_sentinel;
		D3DPtr<ID3D12Resource> m_r_uav_sentinel;
		std::vector<GpuArticulationLevel> m_levels;
		int m_articulation_count;
		int m_link_count;
		int m_dof_count;
		int m_position_count;
		int m_velocity_count;
		int m_force_count;
		int m_child_count;
		int m_acceleration_count;
		int m_joint_matrix_count;
		GpuArticulationForceAbaStats m_stats;

	public:

		// Create the common root signature and four pipeline states without allocating feature buffers.
		explicit GpuArticulationForceAba(Gpu& gpu);

		// Upload one validated packed forest without submitting or waiting, returning whether work is active.
		bool Upload(GpuJob& job, GpuArticulationUpload const& upload);

		// Record prepare, inward, root, and outward passes in deterministic breadth-level order.
		void Run(GpuJob& job);

		// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
		GpuArticulationForceAbaResult Solve(GpuJob& job, GpuArticulationUpload const& upload);

		// Return current logical usage, retained capacities, and the most recent dispatch count.
		GpuArticulationForceAbaStats const& Stats() const;

	private:

		// Release every lazily allocated feature resource when the optional lane becomes empty.
		void ReleaseBuffers();

		// Create or grow all typed buffers required by the current packed forest.
		void ResizeBuffers(CmdList& cmd_list, GpuArticulationUpload const& upload);

		// Bind the shared root layout and dispatch one non-empty level or root range.
		void Dispatch(GpuJob& job, ComputeStep& step, GpuArticulationLevel const& level, int item_count);

		// Order all writable ranges that can feed the next dependent force-ABA pass.
		void CommitUavBarriers(GpuJob& job);
	};
}
