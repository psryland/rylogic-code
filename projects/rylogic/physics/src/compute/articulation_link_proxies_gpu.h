//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/integrate_gpu.h"

namespace pr::physics
{
	struct GpuCoupledContactSolver;
	// Observable resource and dispatch costs for the optional articulation link-proxy lane.
	struct GpuArticulationLinkProxyStats
	{
		int m_external_force_capacity;
		int m_link_frame_capacity;
		int m_gather_dispatch_count;
		int m_refresh_dispatch_count;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
	};

	// Routes ordinary GPU force modules through hidden link proxies and refreshes proxy kinematics after ABA integration.
	struct GpuArticulationLinkProxies
	{
	private:
		friend GpuCoupledContactSolver;

		GpuArticulationForceAba& m_aba;
		EngineConfig const& m_config;
		ComputeStep m_cs_gather_forces;
		ComputeStep m_cs_refresh;
		D3DPtr<ID3D12Resource> m_r_external_forces;
		D3DPtr<ID3D12Resource> m_r_link_to_world;
		GpuArticulationLinkProxyStats m_stats;

	public:

		// Create proxy pipelines without allocating any link-dependent resources.
		GpuArticulationLinkProxies(GpuArticulationForceAba& aba, EngineConfig const& config);

		// Prepare the per-substep link-wrench buffer for the currently uploaded forest.
		void Upload(GpuJob& job);

		// Convert world-space proxy force accumulators into link-frame ABA wrenches.
		ID3D12Resource* GatherForces(GpuJob& job, ID3D12Resource* bodies);

		// Refresh proxy transforms, momenta, and broadphase bounds from committed generalized state.
		void Refresh(GpuJob& job, GpuIntegrator& integrator, int broadphase_sort_axis);

		// Return persistent link-to-world frames indexed by the packed forest link index.
		ID3D12Resource* LinkToWorld();

		// Return current logical usage, retained capacity, and dispatch counts.
		GpuArticulationLinkProxyStats const& Stats() const;

	private:

		// Release every link-dependent resource when no forest is active.
		void ReleaseBuffers();
	};
}
