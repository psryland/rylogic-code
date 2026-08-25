//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/constraint/constraint_gpu.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuCoupledConstraintPrepare;
	struct GpuCoupledConstraintVelocity;

	// Lazily allocated GPU lane for persistent rigid D6 constraints.
	struct GpuConstraintSolver
	{
	private:
		friend GpuCoupledConstraintPrepare;
		friend GpuCoupledConstraintVelocity;

		Gpu& m_gpu;
		EngineConfig const& m_config;
		ComputeStep m_cs_compile;
		ComputeStep m_cs_assign_colours;
		ComputeStep m_cs_apply_warm_start;
		ComputeStep m_cs_clear_pseudo_velocity;
		ComputeStep m_cs_solve_position;
		ComputeStep m_cs_apply_position;
		ComputeStep m_cs_solve_velocity;
		D3DPtr<ID3D12Resource> m_r_endpoints;
		D3DPtr<ID3D12Resource> m_r_descriptors;
		D3DPtr<ID3D12Resource> m_r_blocks;
		D3DPtr<ID3D12Resource> m_r_rows;
		D3DPtr<ID3D12Resource> m_r_overflow;
		D3DPtr<ID3D12Resource> m_r_pseudo_velocities;
		std::vector<GpuConstraintEndpoint> m_endpoint_shadow;
		std::vector<ConstraintEndpointIdentity> m_endpoint_identity_shadow;
		std::vector<GpuD6ConstraintDesc> m_descriptor_shadow;
		ConstraintSet const* m_source;
		int m_capacity;
		int m_body_capacity;
		int m_slot_count;
		int m_active_count;
		float m_previous_timestep;
		float m_frame_warm_start_scale;

	public:

		// Create pipeline state without allocating feature-dependent buffers.
		explicit GpuConstraintSolver(Gpu& gpu, EngineConfig const& config);

		// Upload shared frame-local endpoints and changed persistent descriptors, returning whether independent rigid work is active.
		bool Upload(GpuJob& job, GpuConstraintUpload const& upload);

		// Compile current world-space rows and graph-colour active blocks after body integration.
		void Prepare(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Apply retained physical impulses before the new velocity solve.
		void ApplyWarmStart(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// Execute one complete coloured split-position sweep without changing physical momentum.
		void SolvePositionIteration(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Apply converged pseudo twists to body transforms after all split-position iterations.
		void ApplyPosition(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Execute one complete coloured physical block-PGS sweep.
		void SolveVelocityIteration(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies);

		// True when the most recently uploaded set contains enabled independent rigid constraints.
		bool Active() const;

		// Invalidate retained frame timing when a step does not submit this optional solver lane.
		void Deactivate();

		// CPU-side testing: upload, solve, and read back bodies and runtime state in one GPU job.
		void Solve(GpuJob& job, float timestep, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintBlock> blocks = {}, std::span<GpuConstraintRow> rows = {});

	private:

		// Create or grow feature-dependent buffers while preserving stable-slot capacity.
		bool ResizeBuffers(CmdList& cmd_list, int capacity);

		// Bind the common constraint root signature and resources for one compute step.
		void Bind(GpuJob& job, ComputeStep& step, float timestep, int body_count, int colour, int position_iterations, D3DPtr<ID3D12Resource> bodies);

		// Insert UAV ordering between dependent compiler, colouring, and solver passes.
		void CommitUavBarriers(GpuJob& job, D3DPtr<ID3D12Resource> bodies);
	};
}
