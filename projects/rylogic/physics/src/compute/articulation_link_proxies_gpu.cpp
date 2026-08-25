//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/articulation_link_proxies_gpu.h"
#include "src/compute/shader_code.h"
#include "pr/physics/integrator/engine_config.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Active forest bounds and broadphase settings shared by proxy force and refresh passes.
		struct alignas(16) cbArticulationLinkProxies
		{
			int articulation_count;
			int link_count;
			int broadphase_sort_axis;
			float broadphase_aabb_margin;
		};
		static_assert(sizeof(cbArticulationLinkProxies) == 16);

		// Root-register assignments are shared so both proxy passes use one immutable signature.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Links = ESRVReg::t0;
			inline static constexpr auto Dofs = ESRVReg::t1;
			inline static constexpr auto ExternalForces = ESRVReg::t2;
			inline static constexpr auto Articulations = EUAVReg::u0;
			inline static constexpr auto Positions = EUAVReg::u1;
			inline static constexpr auto Velocities = EUAVReg::u2;
			inline static constexpr auto Scratch = EUAVReg::u3;
			inline static constexpr auto DofScratch = EUAVReg::u4;
			inline static constexpr auto Bodies = EUAVReg::u5;
			inline static constexpr auto AABB_Idx = EUAVReg::u6;
			inline static constexpr auto AABB_Sort = EUAVReg::u7;
			inline static constexpr auto AABB_Box = EUAVReg::u8;
			inline static constexpr auto WorkingExternalForces = EUAVReg::u9;
			inline static constexpr auto LinkToWorld = EUAVReg::u12;
		};

		// Return the number of dispatch groups needed for a non-empty linear item range.
		int ThreadGroupCount(int item_count)
		{
			return (item_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}

		// Create or geometrically grow one typed proxy-owned resource.
		template <typename T>
		void EnsureProxyBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int count, int& capacity, EUsage usage, char const* name)
		{
			if (resource != nullptr && capacity >= count)
				return;

			auto const doubled_capacity = capacity <= std::numeric_limits<int>::max() / 2
				? 2 * capacity
				: std::numeric_limits<int>::max();
			capacity = std::max(count, std::max(1, doubled_capacity));
			resource = gpu.CreateResource(ResDesc::Buf<T>(capacity, {}).usage(usage), cmd_list, name);
		}
	}

	// Create proxy pipelines without allocating any link-dependent resources.
	GpuArticulationLinkProxies::GpuArticulationLinkProxies(GpuArticulationForceAba& aba, EngineConfig const& config)
		:m_aba(aba)
		,m_config(config)
		,m_cs_gather_forces()
		,m_cs_refresh()
		,m_r_external_forces()
		,m_r_link_to_world()
		,m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationLinkProxies>(EReg::Params)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::ExternalForces)
			.UAV(EReg::Articulations)
			.UAV(EReg::Positions)
			.UAV(EReg::Velocities)
			.UAV(EReg::Scratch)
			.UAV(EReg::DofScratch)
			.UAV(EReg::Bodies)
			.UAV(EReg::AABB_Idx)
			.UAV(EReg::AABB_Sort)
			.UAV(EReg::AABB_Box)
			.UAV(EReg::WorkingExternalForces)
			.UAV(EReg::LinkToWorld)
			.Create(m_aba.m_gpu, "Physics:ArticulationLinkProxiesSig");

		m_cs_gather_forces.m_sig = root_sig;
		m_cs_gather_forces.m_pso = ComputePSO(root_sig.get(), shader_code::articulation_gather_proxy_forces).Create(m_aba.m_gpu, "Physics:ArticulationGatherProxyForcesPSO");
		m_cs_refresh.m_sig = root_sig;
		m_cs_refresh.m_pso = ComputePSO(root_sig.get(), shader_code::articulation_refresh_proxies).Create(m_aba.m_gpu, "Physics:ArticulationRefreshProxiesPSO");
	}

	// Release every link-dependent resource when no forest is active.
	void GpuArticulationLinkProxies::ReleaseBuffers()
	{
		m_r_external_forces = nullptr;
		m_r_link_to_world = nullptr;
		m_stats = {};
	}

	// Prepare the per-substep link-wrench buffer for the currently uploaded forest.
	void GpuArticulationLinkProxies::Upload(GpuJob& job)
	{
		m_stats.m_gather_dispatch_count = 0;
		m_stats.m_refresh_dispatch_count = 0;
		if (m_aba.m_link_count == 0)
		{
			ReleaseBuffers();
			return;
		}

		// Geometric growth avoids reallocating per-link working and persistent frame streams when forest sizes fluctuate modestly.
		EnsureProxyBuffer<GpuFrameForce>(
			m_aba.m_gpu,
			job.m_cmd_list,
			m_r_external_forces,
			m_aba.m_link_count,
			m_stats.m_external_force_capacity,
			EUsage::UnorderedAccess,
			"Physics:ArticulationLinkWorkingForces");
		EnsureProxyBuffer<GpuConstraintFrame>(
			m_aba.m_gpu,
			job.m_cmd_list,
			m_r_link_to_world,
			m_aba.m_link_count,
			m_stats.m_link_frame_capacity,
			EUsage::UnorderedAccess,
			"Physics:ArticulationLinkToWorld");

		m_stats.m_logical_bytes = static_cast<size_t>(m_aba.m_link_count) * (sizeof(GpuFrameForce) + sizeof(GpuConstraintFrame));
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_external_force_capacity) * sizeof(GpuFrameForce) +
			static_cast<size_t>(m_stats.m_link_frame_capacity) * sizeof(GpuConstraintFrame);
	}

	// Convert world-space proxy force accumulators into link-frame ABA wrenches.
	ID3D12Resource* GpuArticulationLinkProxies::GatherForces(GpuJob& job, ID3D12Resource* bodies)
	{
		if (m_aba.m_link_count == 0)
			return nullptr;
		if (bodies == nullptr || m_r_external_forces == nullptr)
			throw std::logic_error("Articulation proxy force gathering requires active body and wrench buffers");

		auto const constants = cbArticulationLinkProxies{
			.articulation_count = m_aba.m_articulation_count,
			.link_count = m_aba.m_link_count,
			.broadphase_sort_axis = 0,
			.broadphase_aabb_margin = 0.0f,
		};

		// Bind every root slot deterministically; unused refresh resources point at valid shared sentinels.
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_external_forces.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_to_world.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		job.m_cmd_list.SetPipelineState(m_cs_gather_forces.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_gather_forces.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_to_world->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(ThreadGroupCount(m_aba.m_link_count), 1, 1);
		++m_stats.m_gather_dispatch_count;

		// Publish the gathered wrenches as an immutable SRV before the fused midpoint pass consumes them.
		job.m_barriers.UAV(m_r_external_forces.get()).Commit();
		job.m_barriers.Transition(m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		return m_r_external_forces.get();
	}

	// Refresh proxy transforms, momenta, and broadphase bounds from committed generalized state.
	void GpuArticulationLinkProxies::Refresh(GpuJob& job, GpuIntegrator& integrator, int broadphase_sort_axis)
	{
		if (m_aba.m_articulation_count == 0)
			return;

		auto const constants = cbArticulationLinkProxies{
			.articulation_count = m_aba.m_articulation_count,
			.link_count = m_aba.m_link_count,
			.broadphase_sort_axis = broadphase_sort_axis,
			.broadphase_aabb_margin = m_config.broadphase_aabb_margin,
		};
		auto bodies_resource = integrator.Bodies();
		auto aabb_idx_resource = integrator.AABBBodyIndices();
		auto aabb_sort_resource = integrator.AABBSortAxis();
		auto aabb_box_resource = integrator.AABBBoxes();
		auto* bodies = bodies_resource.get();
		auto* aabb_idx = aabb_idx_resource.get();
		auto* aabb_sort = aabb_sort_resource.get();
		auto* aabb_box = aabb_box_resource.get();

		// Kinematic refresh reads committed generalized state and rewrites only hidden proxy body records and their bounds.
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(aabb_idx, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(aabb_sort, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(aabb_box, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_to_world.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		job.m_cmd_list.SetPipelineState(m_cs_refresh.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_refresh.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(aabb_idx->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(aabb_sort->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(aabb_box->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_to_world->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(ThreadGroupCount(m_aba.m_articulation_count), 1, 1);
		++m_stats.m_refresh_dispatch_count;

		job.m_barriers.UAV(m_aba.m_r_scratch.get());
		if (m_aba.m_dof_count != 0)
			job.m_barriers.UAV(m_aba.m_r_dof_scratch.get());
		job.m_barriers.UAV(bodies);
		job.m_barriers.UAV(aabb_idx);
		job.m_barriers.UAV(aabb_sort);
		job.m_barriers.UAV(aabb_box);
		job.m_barriers.UAV(m_r_link_to_world.get());
		job.m_barriers.Commit();
		job.m_barriers.Transition(m_r_link_to_world.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
	}

	// Return persistent link-to-world frames indexed by the packed forest link index.
	ID3D12Resource* GpuArticulationLinkProxies::LinkToWorld()
	{
		return m_r_link_to_world.get();
	}

	// Return current logical usage, retained capacity, and dispatch counts.
	GpuArticulationLinkProxyStats const& GpuArticulationLinkProxies::Stats() const
	{
		return m_stats;
	}

	// Destroy a link-proxy manager where its complete type is visible.
	void Deleter<GpuArticulationLinkProxies>::operator()(GpuArticulationLinkProxies* proxies) const
	{
		delete proxies;
	}
}
