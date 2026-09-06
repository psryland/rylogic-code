//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/coupled_constraint_position_gpu.h"

namespace pr::physics
{
	// Aggregate costs for the optional articulation-coupled solver and all of its lazily retained resources.
	struct GpuCoupledConstraintSolverStats
	{
		int m_dispatch_count;
		int m_active_count;
		int m_slot_capacity;
		int m_target_capacity;
		int m_island_capacity;
		int m_island_block_capacity;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Coordinates fixed-configuration articulation mobility, transactional velocity solving, and detached position correction.
	struct GpuCoupledConstraintSolver
	{
	private:

		GpuArticulationLinkProxies& m_link_proxies;
		GpuArticulationMobility m_mobility;
		GpuArticulationImpulseAba m_impulse_aba;
		GpuCoupledConstraintPrepare m_prepare;
		GpuCoupledConstraintVelocity m_velocity;
		GpuCoupledConstraintPosition m_position;
		bool m_active;

	public:

		// Create fixed pipeline state while leaving all topology-dependent resources unallocated.
		GpuCoupledConstraintSolver(Gpu& gpu, GpuConstraintSolver& constraints, GpuArticulationForceAba& articulation_solver, GpuArticulationLinkProxies& link_proxies, EngineConfig const& config);

		// Upload frame-local coupled topology after the matching shared constraint and articulation streams.
		bool Upload(GpuJob& job, GpuConstraintUpload const& constraint_upload, GpuArticulationUpload const& articulation_upload);

		// Rebuild mobility and physical rows, optionally retaining impulses already applied earlier in the current substep.
		void PrepareVelocity(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, bool retain_current_impulses = false);

		// Apply retained coupled impulses through atomic rigid and complete-tree updates.
		void ApplyWarmStart(GpuJob& job, int body_count, ID3D12Resource* bodies);

		// Execute one transactional coupled velocity sweep.
		void SolveVelocityIteration(GpuJob& job, int body_count, ID3D12Resource* bodies, int substep_index);

		// Prepare exact position preconditioners and clear detached pseudo state once for the substep.
		bool PreparePosition(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies);

		// Execute one transactional detached coupled position sweep.
		void SolvePositionIteration(GpuJob& job, ID3D12Resource* bodies, int substep_index);

		// Apply all shared rigid and articulation pseudo state exactly once.
		void ApplyPosition(GpuJob& job, ID3D12Resource* bodies);

		// True when the latest upload contains at least one enabled articulation-coupled block.
		bool Active() const;

		// Return the frame-sticky rejection stream for inclusion in the sole owning-frame readback.
		GpuCoupledConstraintFailureOutput FailureOutput();

		// Release all topology-dependent coupled resources and invalidate retained warm-start timing.
		void Deactivate();

		// Return aggregate logical, retained, and dispatch costs for the complete current frame.
		GpuCoupledConstraintSolverStats Stats() const;
	};
}
