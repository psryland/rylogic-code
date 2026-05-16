//*********************************************
// Physics Engine — GPU Integration Implementation
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/integrate_gpu.h"
#include "src/compute/physics_types.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	// Integrate constants
	struct alignas(16) cbIntegrate
	{
		float g_dt;
		int g_body_count;
		int g_sleeping_enabled;
		float g_broadphase_aabb_margin;
		float g_sleep_velocity_threshold_lin;
		float g_sleep_velocity_threshold_ang;
		int g_broadphase_sort_axis;
		float pad2;
	};
	static_assert(sizeof(cbIntegrate) == 32);

	// Register assignments for the root signature
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Counters = EUAVReg::u0;
		inline static constexpr auto Bodies = EUAVReg::u1;
		inline static constexpr auto AABB_Idx = EUAVReg::u2;
		inline static constexpr auto AABB_Sort = EUAVReg::u3;
		inline static constexpr auto AABB_Box = EUAVReg::u4;
	};

	GpuIntegrator::GpuIntegrator(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_integrate()
		, m_r_counters()
		, m_r_bodies()
		, m_r_aabb_sort()
		, m_r_aabb_idx()
		, m_r_aabb_box()
		, m_capacity()
	{
		// m_cs_integrate
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbIntegrate>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::AABB_Idx)
				.UAV(EReg::AABB_Sort)
				.UAV(EReg::AABB_Box)
				;

			m_cs_integrate.m_sig = sig.Create(m_gpu, "Physics:IntegrateSig");
			m_cs_integrate.m_pso = ComputePSO(m_cs_integrate.m_sig.get(), shader_code::integrate).Create(m_gpu, "Physics:IntegratePSO");
		}
	}

	// Resize the buffers to hold 'capacity' bodies.
	void GpuIntegrator::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_counters == nullptr)
		{
			m_r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionCounters");
		}
		if (m_r_bodies == nullptr || m_capacity < capacity)
		{
			m_r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:BodyDynamics");
			m_r_aabb_sort = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Sort");
			m_r_aabb_idx = m_gpu.CreateResource(ResDesc::Buf<int>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Idx");
			m_r_aabb_box = m_gpu.CreateResource(ResDesc::Buf<BBox>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Box");
			m_capacity = capacity;
		}
	}

	// Reset the collision counters without changing the body buffer.
	void GpuIntegrator::ResetCounters(GpuJob& job)
	{
		if (m_r_counters == nullptr)
			return;

		{
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload_counters = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload_counters.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.pair_count = 0,
				.contact_count = 0,
			};
			job.m_cmd_list.CopyBufferRegion(m_r_counters.get(), 0, upload_counters);

			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}
	}

	// Upload staged body dynamics and reset collision counters.
	void GpuIntegrator::Upload(GpuJob& job, std::span<GpuRigidBody> bodies)
	{
		auto body_count = static_cast<int>(bodies.size());
		if (body_count == 0)
			return;

		pix::BeginEvent(job.m_cmd_list.get(), 0xFF9a6ce7, "Physics::Upload");

		// Ensure the buffers are large enough to hold the body count.
		ResizeBuffers(job.m_cmd_list, body_count);

		ResetCounters(job);

		// Upload bodies to the GPU
		{
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.template Alloc<GpuRigidBody>(body_count);
			memcpy(upload.template ptr<GpuRigidBody>(), bodies.data(), body_count * sizeof(GpuRigidBody));
			job.m_cmd_list.CopyBufferRegion(m_r_bodies.get(), 0, upload);

			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Integrate bodies on GPU
	void GpuIntegrator::Integrate(GpuJob& job, int body_count, float dt, int broadphase_sort_axis)
	{
		if (body_count == 0)
			return;

		assert(m_r_counters != nullptr);
		assert(m_r_bodies != nullptr && m_capacity >= body_count);

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFbc45f2, "Physics::Integrate");

		cbIntegrate cb_integrate = {
			.g_dt = dt,
			.g_body_count = body_count,
			.g_sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
			.g_broadphase_aabb_margin = m_config.broadphase_aabb_margin,
			.g_sleep_velocity_threshold_lin = m_config.sleep_velocity_threshold_lin,
			.g_sleep_velocity_threshold_ang = m_config.sleep_velocity_threshold_ang,
			.g_broadphase_sort_axis = broadphase_sort_axis,
			.pad2 = 0,
		};

		// Switch states for resources
		{
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_idx.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_sort.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_box.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Dispatch the compute shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_integrate.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_integrate.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_integrate);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_idx->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_sort->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_box->GetGPUVirtualAddress());

			auto dispatch = (body_count + IntegrateThreadCount - 1) / IntegrateThreadCount;
			job.m_cmd_list.Dispatch(dispatch, 1, 1);

			job.m_barriers.UAV(m_r_counters.get());
			job.m_barriers.UAV(m_r_bodies.get());
			job.m_barriers.UAV(m_r_aabb_idx.get());
			job.m_barriers.UAV(m_r_aabb_sort.get());
			job.m_barriers.UAV(m_r_aabb_box.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// CPU-side testing: upload bodies, integrate on GPU, readback results. Calls job.Run() internally.
	void GpuIntegrator::Integrate(GpuJob& job, std::span<GpuRigidBody> bodies, float dt, std::span<BBox> aabbs)
	{
		Upload(job, bodies);
		Integrate(job, static_cast<int>(bodies.size()), dt, 0);
		Readback(job, bodies, aabbs);
	}

	// Readback data into the provided buffers. 0-length means "don't readback".
	void GpuIntegrator::Readback(GpuJob& job, std::span<GpuRigidBody> bodies, std::span<BBox> aabbs)
	{
		GpuReadbackBuffer::Allocation readback_bodies;
		GpuReadbackBuffer::Allocation readback_aabb_box;

		// Readback bodies and aabbs (if requested)
		{
			if (!bodies.empty())
			{
				job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			}
			if (!aabbs.empty())
			{
				job.m_barriers.Transition(m_r_aabb_box.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			}
			job.m_barriers.Commit();

			if (!bodies.empty())
			{
				readback_bodies = job.m_readback.template Alloc<GpuRigidBody>(static_cast<int>(bodies.size()));
				job.m_cmd_list.CopyBufferRegion(readback_bodies, m_r_bodies.get(), 0);
			}
			if (!aabbs.empty())
			{
				readback_aabb_box = job.m_readback.template Alloc<BBox>(static_cast<int>(aabbs.size()));
				job.m_cmd_list.CopyBufferRegion(readback_aabb_box, m_r_aabb_box.get(), 0);
			}
			if (!bodies.empty())
			{
				job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			}
			if (!aabbs.empty())
			{
				job.m_barriers.Transition(m_r_aabb_box.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			}
			job.m_barriers.Commit();
		}

		job.Run();

		if (!bodies.empty())
		{
			memcpy(bodies.data(), readback_bodies.template ptr<GpuRigidBody>(), bodies.size() * sizeof(GpuRigidBody));
		}
		if (!aabbs.empty())
		{
			memcpy(aabbs.data(), readback_aabb_box.template ptr<BBox>(), aabbs.size() * sizeof(BBox));
		}
	}

	// Custom deleter implementation (GpuIntegrator is complete here)
	void Deleter<GpuIntegrator>::operator()(GpuIntegrator* p) const
	{
		delete p;
	}
}
