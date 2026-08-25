//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuCoupledConstraintPosition;
	struct GpuCoupledConstraintSolver;
	struct GpuCoupledConstraintVelocity;
	struct GpuCoupledContactSolver;

	// Observable resource and dispatch state for the optional articulation impulse-response lane.
	struct GpuArticulationImpulseAbaStats
	{
		int m_link_impulse_capacity;
		int m_work_capacity;
		int m_velocity_delta_capacity;
		int m_dispatch_count;
		size_t m_logical_buffer_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Focused one-submission output for CPU/GPU impulse-response parity tests.
	struct GpuArticulationImpulseAbaResult
	{
		std::vector<GpuArticulationMobilityRange> m_ranges;
		std::vector<float> m_velocities;
		std::vector<GpuArticulationSpatialVector> m_link_velocities;
		std::vector<int> m_solve_valid;

		// True when every packed link retained a valid fixed-configuration factorization.
		bool AllValid() const;
	};

	// Lazily allocated D3D12 lane for exact simultaneous articulation impulse response.
	struct GpuArticulationImpulseAba
	{
	private:
		friend GpuCoupledConstraintPosition;
		friend GpuCoupledConstraintSolver;
		friend GpuCoupledConstraintVelocity;
		friend GpuCoupledContactSolver;

		Gpu& m_gpu;
		GpuArticulationForceAba& m_aba;
		GpuArticulationMobility& m_mobility;
		ComputeStep m_cs_apply_impulses;
		ComputeStep m_cs_evaluate_impulses;
		ComputeStep m_cs_commit_impulses;
		D3DPtr<ID3D12Resource> m_r_link_impulses;
		D3DPtr<ID3D12Resource> m_r_work;
		D3DPtr<ID3D12Resource> m_r_velocity_deltas;
		int m_impulse_count;
		int m_work_count;
		int m_velocity_delta_count;
		GpuArticulationImpulseAbaStats m_stats;

	public:

		// Create the impulse-response pipeline without allocating optional feature buffers.
		GpuArticulationImpulseAba(Gpu& gpu, GpuArticulationForceAba& aba, GpuArticulationMobility& mobility);

		// Allocate compact participating-link impulse and work buffers without initializing the impulse stream.
		bool Prepare(GpuJob& job);

		// Upload one link-coordinate spatial impulse per participating link without submitting or waiting.
		bool Upload(GpuJob& job, std::span<GpuArticulationSpatialVector const> link_impulses);

		// Apply every participating tree's gathered impulses through one fixed-configuration ABA response.
		void Run(GpuJob& job);

		// Evaluate selected tree responses into detached work buffers and one caller-owned validity result per range.
		void Evaluate(GpuJob& job, ID3D12Resource* selection, ID3D12Resource* results);

		// Commit previously evaluated responses for the caller-selected valid ranges.
		void Commit(GpuJob& job, ID3D12Resource* selection, ID3D12Resource* results);

		// Prepare factors, apply impulses, and read focused output through exactly one GPU submission.
		GpuArticulationImpulseAbaResult Apply(
			GpuJob& job,
			GpuArticulationUpload const& upload,
			std::span<int const> articulation_indices,
			std::span<GpuArticulationSpatialVector const> link_impulses);

		// Return the GPU impulse stream so deterministic gather passes can populate it directly.
		ID3D12Resource* LinkImpulses();

		// Return detached per-link velocity deltas from the most recent impulse evaluation.
		ID3D12Resource* Work();

		// Return current logical usage, retained capacities, and the most recent dispatch count.
		GpuArticulationImpulseAbaStats const& Stats() const;

	private:

		// Release every lazily allocated impulse-response resource when no tree participates.
		void ReleaseBuffers();

		// Create or grow typed impulse, link-response, and generalized-response buffers for the active packed forest.
		void ResizeBuffers(CmdList& cmd_list, int link_count, int work_count, int velocity_count);

		// Bind and dispatch one transactional evaluate or commit pass using caller-owned selection and result streams.
		void DispatchTransactional(GpuJob& job, ComputeStep& step, ID3D12Resource* selection, ID3D12Resource* results);
	};
}
