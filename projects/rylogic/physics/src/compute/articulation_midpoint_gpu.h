//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/articulation_force_aba_gpu.h"

namespace pr::physics
{
	// Observable allocation, logical-memory, and dispatch state for fused articulation integration.
	struct GpuArticulationMidpointStats
	{
		int m_position_start_capacity;
		int m_velocity_start_capacity;
		int m_midpoint_velocity_capacity;
		int m_integration_state_capacity;
		int m_dispatch_count;
		int m_pad0;
		int m_pad1;
		int m_pad2;

		// Active extra scratch is 4P + 8V + 48A bytes independently of retained high-water capacity.
		size_t m_logical_scratch_bytes;

		// Sum of integration-only typed buffer widths, excluding heap-alignment overhead and shared ABA resources.
		size_t m_allocated_feature_bytes;
	};

	// Detached test-only state recovered after one submission containing every requested internal substep.
	// Callers must validate the complete forest before publishing any result to its source objects.
	struct GpuArticulationMidpointResult
	{
		std::vector<GpuArticulation> m_articulations;
		std::vector<float> m_positions;
		std::vector<float> m_velocities;
		std::vector<float> m_accelerations;
		std::vector<GpuArticulationSpatialVector> m_link_accelerations;
		std::vector<GpuArticulationIntegrationState> m_states;
		size_t m_logical_scratch_bytes;
		int m_dispatch_count;

		// Report the side-effect-free forest status gate before caller-specific output validation and commit.
		bool AllSucceeded() const;
	};

	// Fused one-lane-per-articulation midpoint integrator composed over shared force-ABA resources.
	struct GpuArticulationMidpoint
	{
	private:

		GpuArticulationForceAba& m_aba;
		ComputeStep m_cs_midpoint;
		D3DPtr<ID3D12Resource> m_r_position_start;
		D3DPtr<ID3D12Resource> m_r_velocity_start;
		D3DPtr<ID3D12Resource> m_r_midpoint_velocity;
		D3DPtr<ID3D12Resource> m_r_integration_state;
		GpuArticulationMidpointStats m_stats;

	public:

		// Create the fused integration pipeline without allocating articulation-dependent buffers.
		explicit GpuArticulationMidpoint(GpuArticulationForceAba& aba);

		// Upload one validated forest, initialize diagnostics, and prepare shared state without submitting or waiting.
		bool Upload(GpuJob& job, GpuArticulationUpload const& upload);

		// Record exactly one fused dispatch per requested internal substep with explicit inter-substep ordering.
		void Run(GpuJob& job, float dt, int substep_count);

		// Upload, record all substeps, submit once, and read back focused integration diagnostics.
		GpuArticulationMidpointResult Solve(GpuJob& job, GpuArticulationUpload const& upload, float dt, int substep_count);

		// Return current integration-only capacities, logical usage, and most recent dispatch count.
		GpuArticulationMidpointStats const& Stats() const;

	private:

		// Release every integration-only feature resource when the optional forest becomes empty.
		void ReleaseBuffers();

		// Create or geometrically grow the exact P, V, V, and A integration scratch ranges.
		void ResizeBuffers(CmdList& cmd_list);

		// Bind shared ABA resources and integration-only scratch for one fused substep.
		void Dispatch(GpuJob& job, float dt);

		// Order all primary, ABA, and integration scratch writes before another substep or readback.
		void CommitUavBarriers(GpuJob& job);
	};
}
