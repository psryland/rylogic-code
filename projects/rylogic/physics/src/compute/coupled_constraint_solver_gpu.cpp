//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/coupled_constraint_solver_gpu.h"
#include "src/compute/articulation_link_proxies_gpu.h"

namespace pr::physics
{
	// Create fixed pipeline state while leaving all topology-dependent resources unallocated.
	GpuCoupledConstraintSolver::GpuCoupledConstraintSolver(Gpu& gpu, GpuConstraintSolver& constraints, GpuArticulationForceAba& articulation_solver, GpuArticulationLinkProxies& link_proxies, EngineConfig const& config)
		: m_link_proxies(link_proxies)
		, m_mobility(articulation_solver)
		, m_impulse_aba(gpu, articulation_solver, m_mobility)
		, m_prepare(constraints, config)
		, m_velocity(m_prepare, m_impulse_aba, config)
		, m_position(m_velocity, config)
		, m_active(false)
	{}

	// Upload frame-local coupled topology after the matching shared constraint and articulation streams.
	bool GpuCoupledConstraintSolver::Upload(GpuJob& job, GpuConstraintUpload const& constraint_upload, GpuArticulationUpload const& articulation_upload)
	{
		m_impulse_aba.ResetDispatchCount();
		m_position.ResetDispatchCount();
		if (constraint_upload.m_coupled_active_count == 0)
		{
			Deactivate();
			return false;
		}

		// Every coupled block needs a compact complete-tree range and matching metadata in each dependent lane.
		if (!m_mobility.Upload(job, articulation_upload, constraint_upload.m_coupled_articulation_indices))
			throw std::logic_error("Coupled constraint topology requires at least one participating articulation");
		if (!m_prepare.Upload(job, constraint_upload))
			throw std::logic_error("Coupled constraint preparation rejected active coupled topology");
		if (!m_velocity.Upload(job, constraint_upload))
			throw std::logic_error("Coupled constraint velocity solver rejected active coupled topology");

		m_active = true;
		return true;
	}

	// Rebuild mobility and physical rows, optionally retaining impulses already applied earlier in the current substep.
	void GpuCoupledConstraintSolver::PrepareVelocity(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, bool retain_current_impulses)
	{
		if (!m_active)
			return;
		if (body_count < 0 || bodies == nullptr || m_link_proxies.LinkToWorld() == nullptr)
			throw std::invalid_argument("Coupled velocity preparation requires valid rigid and articulation frame streams");

		// Constraint impulses use a final-configuration factorization that remains fixed for the complete velocity and position transaction.
		m_mobility.Run(job);
		m_prepare.Run(job, timestep, body_count, bodies, m_link_proxies.LinkToWorld(), m_mobility, retain_current_impulses);
	}

	// Apply retained coupled impulses through atomic rigid and complete-tree updates.
	void GpuCoupledConstraintSolver::ApplyWarmStart(GpuJob& job, int body_count, ID3D12Resource* bodies)
	{
		if (m_active)
			m_velocity.ApplyWarmStart(job, body_count, bodies);
	}

	// Execute one transactional coupled velocity sweep.
	void GpuCoupledConstraintSolver::SolveVelocityIteration(GpuJob& job, int body_count, ID3D12Resource* bodies, int substep_index)
	{
		if (m_active)
			m_velocity.Run(job, body_count, bodies, substep_index);
	}

	// Prepare exact position preconditioners and clear detached pseudo state once for the substep.
	bool GpuCoupledConstraintSolver::PreparePosition(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies)
	{
		return m_active && m_position.Prepare(job, timestep, body_count, bodies, m_link_proxies.LinkToWorld());
	}

	// Execute one transactional detached coupled position sweep.
	void GpuCoupledConstraintSolver::SolvePositionIteration(GpuJob& job, ID3D12Resource* bodies, int substep_index)
	{
		if (m_active)
			m_position.Run(job, bodies, substep_index);
	}

	// Apply all shared rigid and articulation pseudo state exactly once.
	void GpuCoupledConstraintSolver::ApplyPosition(GpuJob& job, ID3D12Resource* bodies)
	{
		if (m_active)
			m_position.Apply(job, bodies);
	}

	// True when the latest upload contains at least one enabled articulation-coupled block.
	bool GpuCoupledConstraintSolver::Active() const
	{
		return m_active;
	}

	// Forward the velocity-owned stable stream because it spans velocity and position rejection phases.
	GpuCoupledConstraintFailureOutput GpuCoupledConstraintSolver::FailureOutput()
	{
		return m_velocity.FailureOutput();
	}

	// Release all topology-dependent coupled resources and invalidate retained warm-start timing.
	void GpuCoupledConstraintSolver::Deactivate()
	{
		m_position.ReleaseBuffers();
		m_velocity.ReleaseBuffers();
		m_prepare.ReleaseBuffers();
		m_impulse_aba.ReleaseBuffers();
		m_mobility.ReleaseBuffers();
		m_active = false;
	}

	// Return aggregate logical, retained, and dispatch costs for the complete current frame.
	GpuCoupledConstraintSolverStats GpuCoupledConstraintSolver::Stats() const
	{
		auto const mobility = m_mobility.Stats();
		auto const impulse = m_impulse_aba.Stats();
		auto const prepare = m_prepare.Stats();
		auto const velocity = m_velocity.Stats();
		auto const position = m_position.Stats();
		return GpuCoupledConstraintSolverStats{
			.m_dispatch_count =
				mobility.m_dispatch_count +
				impulse.m_dispatch_count +
				prepare.m_dispatch_count +
				velocity.m_dispatch_count +
				position.m_dispatch_count,
			.m_active_count = m_active ? velocity.m_active_count : 0,
			.m_slot_capacity = velocity.m_slot_capacity,
			.m_target_capacity = velocity.m_target_capacity,
			.m_island_capacity = velocity.m_island_capacity,
			.m_island_block_capacity = velocity.m_island_block_capacity,
			.m_logical_bytes =
				mobility.m_logical_bytes +
				impulse.m_logical_buffer_bytes +
				prepare.m_logical_bytes +
				velocity.m_logical_bytes +
				position.m_logical_bytes,
			.m_allocated_feature_bytes =
				mobility.m_allocated_feature_bytes +
				impulse.m_allocated_feature_bytes +
				prepare.m_allocated_feature_bytes +
				velocity.m_allocated_feature_bytes +
				position.m_allocated_feature_bytes,
		};
	}

	// Custom deleter implementation keeps the incomplete type out of the public Engine header.
	void Deleter<GpuCoupledConstraintSolver>::operator()(GpuCoupledConstraintSolver* solver) const
	{
		delete solver;
	}
}
