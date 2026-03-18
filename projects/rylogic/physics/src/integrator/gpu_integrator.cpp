//*********************************************
// Physics Engine — GPU Integration Implementation
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/rigid_body/rigid_body_dynamics.h"
#include "src/integrator/gpu_integrator.h"
#include "src/collision/gpu_collision_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Integrate constants
	struct alignas(16) cbIntegrate
	{
		float g_dt;
		int g_pad0;
		int g_pad1;
		int g_pad2;
	};
	static_assert(sizeof(cbIntegrate) == 16);

	// Register assignments for the root signature
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Counters = EUAVReg::u0;
		inline static constexpr auto Bodies = EUAVReg::u1;
		inline static constexpr auto AABB_X = EUAVReg::u2;
		inline static constexpr auto AABB_Y = EUAVReg::u3;
		inline static constexpr auto AABB_Z = EUAVReg::u4;
		inline static constexpr auto AABB_Idx = EUAVReg::u5;
		#if PR_COLLISION_DIAGNOSTICS
		inline static constexpr auto Diag = EUAVReg::u6;
		#endif
	};

	GpuIntegrator::GpuIntegrator(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_integrate()
		, m_r_bodies()
		, m_r_aabb_x()
		, m_r_aabb_y()
		, m_r_aabb_z()
		, m_r_aabb_idx()
		#if PR_COLLISION_DIAGNOSTICS
		, m_r_diag()
		#endif
		, m_capacity()
	{
		CompileShaders();
	}

	// Compile the integration compute shader
	void GpuIntegrator::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/integrate.hlsl", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.Define(L"PR_COLLISION_DIAGNOSTICS", L"" PR_STRINGISE(PR_COLLISION_DIAGNOSTICS))
			.ShaderModel(L"cs_6_0")
			.Optimise();

		// m_cs_integrate
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbIntegrate>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::AABB_X)
				.UAV(EReg::AABB_Y)
				.UAV(EReg::AABB_Z)
				.UAV(EReg::AABB_Idx)
				#if PR_COLLISION_DIAGNOSTICS
				.UAV(EReg::Diag)
				#endif
				;

			auto bytecode = compiler.EntryPoint(L"CSIntegrate").Compile();

			m_cs_integrate.m_sig = sig.Create(m_gpu, "Physics:IntegrateSig");
			m_cs_integrate.m_pso = ComputePSO(m_cs_integrate.m_sig.get(), bytecode).Create(m_gpu, "Physics:IntegratePSO");
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
			m_r_bodies = m_gpu.CreateResource(ResDesc::Buf<RigidBodyDynamics>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:BodyDynamics");
			m_r_aabb_x = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_X");
			m_r_aabb_y = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Y");
			m_r_aabb_z = m_gpu.CreateResource(ResDesc::Buf<float>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Z");
			m_r_aabb_idx = m_gpu.CreateResource(ResDesc::Buf<int>(2 * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:IntegrateAABB_Idx");
			#if PR_COLLISION_DIAGNOSTICS
			m_r_diag = m_gpu.CreateResource(ResDesc::Buf<GpuIntegrateDiag>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:GpuIntegrateDiag");
			#endif
			m_capacity = capacity;
		}
	}

	// Integrate bodies on GPU
	void GpuIntegrator::Integrate(GpuJob& job, std::span<RigidBodyDynamics> bodies, float dt)
	{
		auto body_count = static_cast<int>(bodies.size());
		if (body_count == 0)
			return;

		// Ensure the buffers are large enough to hold the body count.
		ResizeBuffers(job.m_cmd_list, body_count);
		
		// Initialise and upload the counters
		{
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload_counters = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload_counters.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.body_count = body_count,
				.pair_count = 0,
				.contact_count = 0,
			};
			job.m_cmd_list.CopyBufferRegion(m_r_counters.get(), 0, upload_counters);

			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Upload bodies to the GPU
		{
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.template Alloc<RigidBodyDynamics>(body_count);
			memcpy(upload.template ptr<RigidBodyDynamics>(), bodies.data(), body_count * sizeof(RigidBodyDynamics));
			job.m_cmd_list.CopyBufferRegion(m_r_bodies.get(), 0, upload);

			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Switch states for resources
		{
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_x.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_y.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_z.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_aabb_idx.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#endif
			job.m_barriers.Commit();
		}

		// Dispatch the compute shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_integrate.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_integrate.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbIntegrate{ .g_dt = dt });
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_x->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_y->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_z->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_aabb_idx->GetGPUVirtualAddress());
			#if PR_COLLISION_DIAGNOSTICS
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diag->GetGPUVirtualAddress());
			#endif

			auto dispatch_count = (body_count + IntegrateThreadCount - 1) / IntegrateThreadCount;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(m_r_counters.get());
			job.m_barriers.UAV(m_r_bodies.get());
			job.m_barriers.UAV(m_r_aabb_x.get());
			job.m_barriers.UAV(m_r_aabb_y.get());
			job.m_barriers.UAV(m_r_aabb_z.get());
			job.m_barriers.UAV(m_r_aabb_idx.get());
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.UAV(m_r_diag.get());
			#endif
			job.m_barriers.Commit();
		}
	}

	// Readback data into the provided buffers. 0-length means "don't readback".
	void GpuIntegrator::Readback(GpuJob& job, std::span<RigidBodyDynamics> bodies, std::span<BBox> aabbs, std::span<GpuIntegrateDiag> diag)
	{
		GpuReadbackBuffer::Allocation readback_bodies;
		GpuReadbackBuffer::Allocation readback_aabb_x;
		GpuReadbackBuffer::Allocation readback_aabb_y;
		GpuReadbackBuffer::Allocation readback_aabb_z;
		#if PR_COLLISION_DIAGNOSTICS
		GpuReadbackBuffer::Allocation readback_diag;
		#endif

		// Readback bodies, aabbs, and diagnostics (if requested)
		{
			if (!bodies.empty())
			{
				job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			}
			if (!aabbs.empty())
			{
				job.m_barriers.Transition(m_r_aabb_x.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(m_r_aabb_y.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Transition(m_r_aabb_z.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			}
			if (!diag.empty())
			{
				#if PR_COLLISION_DIAGNOSTICS
				job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				#endif
			}
			job.m_barriers.Commit();

			if (!bodies.empty())
			{
				readback_bodies = job.m_readback.template Alloc<RigidBodyDynamics>(static_cast<int>(bodies.size()));
				job.m_cmd_list.CopyBufferRegion(readback_bodies, m_r_bodies.get(), 0);
			}
			if (!aabbs.empty())
			{
				auto aabb_count = static_cast<int>(2 * aabbs.size());
				readback_aabb_x = job.m_readback.template Alloc<float>(aabb_count);
				readback_aabb_y = job.m_readback.template Alloc<float>(aabb_count);
				readback_aabb_z = job.m_readback.template Alloc<float>(aabb_count);
				job.m_cmd_list.CopyBufferRegion(readback_aabb_x, m_r_aabb_x.get(), 0);
				job.m_cmd_list.CopyBufferRegion(readback_aabb_y, m_r_aabb_y.get(), 0);
				job.m_cmd_list.CopyBufferRegion(readback_aabb_z, m_r_aabb_z.get(), 0);
			}
			if (!diag.empty())
			{
				#if PR_COLLISION_DIAGNOSTICS
				readback_diag = job.m_readback.template Alloc<GpuIntegrateDiag>(int(diag.size()));
				job.m_cmd_list.CopyBufferRegion(readback_diag, m_r_diag.get(), 0);
				#endif
			}

			if (!bodies.empty())
			{
				job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			}
			if (!aabbs.empty())
			{
				job.m_barriers.Transition(m_r_aabb_x.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				job.m_barriers.Transition(m_r_aabb_y.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				job.m_barriers.Transition(m_r_aabb_z.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			}
			if (!diag.empty())
			{
				#if PR_COLLISION_DIAGNOSTICS
				job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				#endif
			}
			job.m_barriers.Commit();
		}

		job.Run();

		if (!bodies.empty())
		{
			memcpy(bodies.data(), readback_bodies.template ptr<RigidBodyDynamics>(), bodies.size() * sizeof(RigidBodyDynamics));
		}
		if (!aabbs.empty())
		{
			auto aabb_count = 2 * aabbs.size();
			auto aabb_x = std::span{readback_aabb_x.template ptr<float>(), aabb_count};
			auto aabb_y = std::span{readback_aabb_y.template ptr<float>(), aabb_count};
			auto aabb_z = std::span{readback_aabb_z.template ptr<float>(), aabb_count};

			for (int i = 0; i != aabbs.size(); ++i)
			{
				auto lower = v4{ aabb_x[i * 2 + 0], aabb_y[i * 2 + 0], aabb_z[i * 2 + 0], 1 };
				auto upper = v4{ aabb_x[i * 2 + 1], aabb_y[i * 2 + 1], aabb_z[i * 2 + 1], 1 };
				aabbs[i] = BBox::Make(lower, upper);
			}
		}
		if (!diag.empty())
		{
			#if PR_COLLISION_DIAGNOSTICS
			memcpy(diag.data(), readback_diag.template ptr<GpuIntegrateDiag>(), diag.size() * sizeof(GpuIntegrateDiag));
			#endif
		}
	}

	// CPU-side testing: upload bodies, integrate on GPU, readback results. Calls job.Run() internally.
	void GpuIntegrator::Integrate(GpuJob& job, std::span<RigidBodyDynamics> bodies, float dt, std::span<BBox> aabbs)
	{
		Integrate(job, bodies, dt);
		Readback(job, bodies, aabbs, {});
	}

	// Custom deleter implementation (GpuIntegrator is complete here)
	void Deleter<GpuIntegrator>::operator()(GpuIntegrator* p) const
	{
		delete p;
	}
}
