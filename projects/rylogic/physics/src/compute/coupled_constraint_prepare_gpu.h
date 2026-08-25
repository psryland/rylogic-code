//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/physics_types.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuArticulationMobility;
	struct GpuCoupledConstraintVelocity;

	// Observable storage and dispatch costs for the optional coupled-row preparation lane.
	struct GpuCoupledConstraintPrepareStats
	{
		int m_slot_capacity;
		int m_active_count;
		int m_dispatch_count;
		int m_pad0;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Focused readback from one hardware coupled-row preparation pass.
	struct GpuCoupledConstraintPrepareResult
	{
		std::vector<GpuConstraintBlock> m_blocks;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuCoupledConstraintPreconditioner> m_preconditioners;
	};

	// Lazily allocated GPU lane that compiles articulation-coupled D6 rows and exact-self block preconditioners.
	struct GpuCoupledConstraintPrepare
	{
	private:
		friend GpuCoupledConstraintVelocity;

		Gpu& m_gpu;
		EngineConfig const& m_config;
		GpuConstraintSolver& m_constraints;
		ComputeStep m_cs_prepare;
		D3DPtr<ID3D12Resource> m_r_coupled_endpoints;
		D3DPtr<ID3D12Resource> m_r_preconditioners;
		ConstraintSet const* m_source;
		int m_slot_count;
		int m_active_count;
		GpuCoupledConstraintPrepareStats m_stats;

	public:

		// Create pipeline state without allocating any coupled-feature buffers.
		GpuCoupledConstraintPrepare(GpuConstraintSolver& constraints, EngineConfig const& config);

		// Upload stable-slot link ownership after the shared constraint streams have been uploaded.
		bool Upload(GpuJob& job, GpuConstraintUpload const& upload);

		// Compile link-coordinate rows and exact-self preconditioners from retained final-configuration mobility factors.
		void Run(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, ID3D12Resource* link_to_world, GpuArticulationMobility& mobility);

		// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
		GpuCoupledConstraintPrepareResult Solve(
			GpuJob& job,
			float timestep,
			GpuConstraintUpload const& upload,
			std::span<GpuRigidBody const> bodies,
			std::span<GpuConstraintFrame const> link_to_world,
			std::span<GpuArticulationSpatialMobility const> mobilities,
			std::span<GpuArticulationAbaScratch const> aba_scratch);

		// Return current logical usage, retained capacity, and dispatch count.
		GpuCoupledConstraintPrepareStats const& Stats() const;

	private:

		// Release every coupled-owned resource when no coupled slot is active.
		void ReleaseBuffers();

		// Create or geometrically grow stable-slot metadata and preconditioner buffers.
		void ResizeBuffers(CmdList& cmd_list);

		// Record the preparation pass against explicit production or diagnostic resources.
		void Run(
			GpuJob& job,
			float timestep,
			int body_count,
			int link_count,
			int mobility_count,
			ID3D12Resource* bodies,
			ID3D12Resource* link_to_world,
			ID3D12Resource* mobilities,
			ID3D12Resource* aba_scratch);
	};
}
