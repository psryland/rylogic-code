//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/articulation_force_aba_gpu.h"

namespace pr::physics
{
	struct GpuArticulationImpulseAba;
struct GpuCoupledConstraintPrepare;

	// Observable resource and dispatch costs for the optional articulation self-link mobility lane.
	struct GpuArticulationMobilityStats
	{
		int m_range_capacity;
		int m_mobility_capacity;
		int m_dispatch_count;
		int m_pad0;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Focused readback from one hardware mobility run.
	struct GpuArticulationMobilityResult
	{
		std::vector<GpuArticulationMobilityRange> m_ranges;
		std::vector<GpuArticulationSpatialMobility> m_mobilities;
		std::vector<int> m_solve_valid;

		// Return true when every participating link completed a non-singular factorization.
		bool AllValid() const;
	};

	// Return canonical compact output ranges for the selected articulation indices.
	std::vector<GpuArticulationMobilityRange> BuildGpuArticulationMobilityRanges(GpuArticulationUpload const& upload, std::span<int const> articulation_indices);

	// Lazily allocated GPU pass for exact self-link mobility at committed generalized coordinates.
	struct GpuArticulationMobility
	{
	private:
		friend GpuArticulationImpulseAba;
		friend GpuCoupledConstraintPrepare;

		GpuArticulationForceAba& m_aba;
		ComputeStep m_cs_prepare;
		D3DPtr<ID3D12Resource> m_r_ranges;
		D3DPtr<ID3D12Resource> m_r_mobilities;
		std::vector<GpuArticulationMobilityRange> m_ranges;
		int m_mobility_count;
		int m_velocity_delta_count;
		GpuArticulationMobilityStats m_stats;

	public:

		// Create the mobility pipeline without allocating articulation-dependent resources.
		explicit GpuArticulationMobility(GpuArticulationForceAba& aba);

		// Upload compact participating-tree ranges after the matching forest has been uploaded to the shared ABA owner.
		bool Upload(GpuJob& job, GpuArticulationUpload const& upload, std::span<int const> articulation_indices);

		// Rebuild final-configuration ABA factors and exact self-link mobilities for all participating trees.
		// This replaces shared ABA factors and accelerations, so later consumers must treat these mobility factors as the current scratch contents.
		void Run(GpuJob& job);

		// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
		GpuArticulationMobilityResult Solve(GpuJob& job, GpuArticulationUpload const& upload, std::span<int const> articulation_indices);

		// Return compact link-body-frame mobility matrices for downstream coupled-block preparation.
		ID3D12Resource* Mobilities();

		// Return canonical participating ranges and their compact mobility offsets.
		std::span<GpuArticulationMobilityRange const> Ranges() const;

		// Return current logical usage, retained capacities, and the most recent dispatch count.
		GpuArticulationMobilityStats const& Stats() const;

	private:

		// Release every mobility-owned resource when no articulation participates.
		void ReleaseBuffers();

		// Create or geometrically grow the range and compact mobility resources.
		void ResizeBuffers(CmdList& cmd_list);
	};
}
