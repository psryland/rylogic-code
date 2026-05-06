//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/sleep_gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace rdr12;

	struct alignas(16) cbSleep
	{
		int g_body_count;
		int g_island_count;
		int g_sleeping_enabled;
		int g_pad0;
	};
	static_assert(sizeof(cbSleep) == 16);

	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto SleepIslands = EUAVReg::u0;
		inline static constexpr auto BodiesRW = EUAVReg::u1;
		inline static constexpr auto Bodies = ESRVReg::t0;
	};

	GpuSleepManager::GpuSleepManager(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_disturb_islands()
		, m_cs_wake_collided()
		, m_r_sleep_islands()
		, m_capacity()
	{
		CompileShaders();
	}

	// Compile the sleep/wake compute shaders.
	void GpuSleepManager::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/sleep.hlsl", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.ShaderModel(L"cs_6_0")
			.Optimise();

		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSleep>(EReg::Params)
				.UAV(EReg::SleepIslands)
				.SRV(EReg::Bodies)
				;

			auto bytecode = compiler.EntryPoint(L"CSDisturbIslands").Compile();

			m_cs_disturb_islands.m_sig = sig.Create(m_gpu, "Physics:SleepWakeSig");
			m_cs_disturb_islands.m_pso = ComputePSO(m_cs_disturb_islands.m_sig.get(), bytecode).Create(m_gpu, "Physics:SleepWakePSO");
		}
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSleep>(EReg::Params)
				.UAV(EReg::BodiesRW)
				;

			auto bytecode = compiler.EntryPoint(L"CSWakeCollidedBodies").Compile();

			m_cs_wake_collided.m_sig = sig.Create(m_gpu, "Physics:SleepUpdateSig");
			m_cs_wake_collided.m_pso = ComputePSO(m_cs_wake_collided.m_sig.get(), bytecode).Create(m_gpu, "Physics:SleepUpdatePSO");
		}
	}

	// Resize the buffers to support 'capacity' sleep islands.
	void GpuSleepManager::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_sleep_islands == nullptr || m_capacity < capacity)
		{
			m_r_sleep_islands = m_gpu.CreateResource(ResDesc::Buf<GpuSleepIsland>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SleepIslands");
			m_capacity = capacity;
		}
	}

	// Upload staged sleep islands to the GPU.
	void GpuSleepManager::Upload(GpuJob& job, std::span<GpuSleepIsland const> islands)
	{
		ResizeBuffers(job.m_cmd_list, static_cast<int>(islands.size()));
		if (islands.empty())
			return;

		job.m_barriers.Transition(m_r_sleep_islands.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();

		auto upload = job.m_upload.template Alloc<GpuSleepIsland>(static_cast<int>(islands.size()));
		memcpy(upload.template ptr<GpuSleepIsland>(), islands.data(), islands.size_bytes());
		job.m_cmd_list.CopyBufferRegion(m_r_sleep_islands.get(), 0, upload);

		job.m_barriers.Transition(m_r_sleep_islands.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Mark sleeping islands whose world-space bounds are overlapped by actually-awake bodies.
	void GpuSleepManager::SleepWake(GpuJob& job, int body_count, int island_count, D3DPtr<ID3D12Resource> bodies)
	{
		if (body_count == 0 || island_count == 0 || !m_config.sleeping_enabled)
			return;

		assert(m_r_sleep_islands != nullptr && m_capacity >= island_count);
		assert(bodies != nullptr);

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf2bc45, "Physics::SleepWake");

		cbSleep cb_sleep = {
			.g_body_count = body_count,
			.g_island_count = island_count,
			.g_sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
			.g_pad0 = 0,
		};

		{
			job.m_barriers.Transition(m_r_sleep_islands.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}
		{
			job.m_cmd_list.SetPipelineState(m_cs_disturb_islands.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_disturb_islands.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sleep);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_sleep_islands->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(bodies->GetGPUVirtualAddress());

			auto pair_count = body_count * island_count;
			auto dispatch_count = (pair_count + SleepThreadCount - 1) / SleepThreadCount;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(m_r_sleep_islands.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Persist wake-ups for sleeping bodies that received resolver impulses.
	void GpuSleepManager::SleepUpdate(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		if (body_count == 0 || !m_config.sleeping_enabled)
			return;

		assert(bodies != nullptr);

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf2a545, "Physics::SleepUpdate");

		cbSleep cb_sleep = {
			.g_body_count = body_count,
			.g_island_count = 0,
			.g_sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
			.g_pad0 = 0,
		};

		{
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}
		{
			job.m_cmd_list.SetPipelineState(m_cs_wake_collided.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_wake_collided.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sleep);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());

			auto dispatch_count = (body_count + SleepThreadCount - 1) / SleepThreadCount;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	void Deleter<GpuSleepManager>::operator()(GpuSleepManager* p) const
	{
		delete p;
	}
}
