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
		float dt;
		float sleep_velocity_threshold_lin;
		float sleep_velocity_threshold_ang;
		float sleep_delay_s;
		int body_count;
		int island_count;
		int max_contacts;
		int sleeping_enabled;
	};
	static_assert(sizeof(cbSleep) == 32);

	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto SleepIslands = EUAVReg::u0;
		inline static constexpr auto BodiesRW = EUAVReg::u1;
		inline static constexpr auto SleepParents = EUAVReg::u2;
		inline static constexpr auto SleepStats = EUAVReg::u3;
		inline static constexpr auto Bodies = ESRVReg::t0;
		inline static constexpr auto Counters = ESRVReg::t1;
		inline static constexpr auto Contacts = ESRVReg::t2;
	};

	GpuSleepManager::GpuSleepManager(Gpu& gpu, EngineConfig const& config, IShaderCache* shader_cache)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_disturb_islands()
		, m_cs_init_sleep_state()
		, m_cs_union_sleep_contacts()
		, m_cs_canonicalise_roots()
		, m_cs_reduce_sleep_stats()
		, m_cs_apply_sleep_state()
		, m_cmd_sig()
		, m_r_sleep_islands()
		, m_r_sleep_parents()
		, m_r_sleep_stats()
		, m_island_capacity()
		, m_body_capacity()
	{
		// Compile the sleep/wake compute shaders.
		auto resolver = shader_cache::ResourceSourceResolver{};
		auto compiler = ShaderCompiler{}
			.Cache(shader_cache)
			.Source("src/compute/sleep.hlsl", resolver)
			.HlslVersion(EHlslVersion::DxcDefault)
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
		auto make_update_sig = [&]()
		{
			return RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSleep>(EReg::Params)
				.UAV(EReg::SleepIslands)
				.UAV(EReg::BodiesRW)
				.UAV(EReg::SleepParents)
				.UAV(EReg::SleepStats)
				.SRV(EReg::Counters)
				.SRV(EReg::Contacts)
				;
		};
		auto compile_update = [&](ComputeStep& step, wchar_t const* entry_point, char const* sig_name, char const* pso_name)
		{
			auto sig = make_update_sig();
			auto bytecode = compiler.EntryPoint(entry_point).Compile();

			step.m_sig = sig.Create(m_gpu, sig_name);
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};

		compile_update(m_cs_init_sleep_state, L"CSInitSleepState", "Physics:InitSleepStateSig", "Physics:InitSleepStatePSO");
		compile_update(m_cs_union_sleep_contacts, L"CSUnionSleepContacts", "Physics:UnionSleepContactsSig", "Physics:UnionSleepContactsPSO");
		compile_update(m_cs_canonicalise_roots, L"CSCanonicaliseSleepRoots", "Physics:CanonicaliseSleepRootsSig", "Physics:CanonicaliseSleepRootsPSO");
		compile_update(m_cs_reduce_sleep_stats, L"CSReduceSleepStats", "Physics:ReduceSleepStatsSig", "Physics:ReduceSleepStatsPSO");
		compile_update(m_cs_apply_sleep_state, L"CSApplySleepState", "Physics:ApplySleepStateSig", "Physics:ApplySleepStatePSO");

		D3D12_INDIRECT_ARGUMENT_DESC arg = {
			.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH
		};
		D3D12_COMMAND_SIGNATURE_DESC desc = {
			.ByteStride = sizeof(D3D12_DISPATCH_ARGUMENTS),
			.NumArgumentDescs = 1,
			.pArgumentDescs = &arg,
		};
		Check(m_gpu->CreateCommandSignature(&desc, nullptr, __uuidof(ID3D12CommandSignature), (void**)m_cmd_sig.address_of()));
	}

	// Resize the buffers to support 'capacity' sleep islands.
	void GpuSleepManager::ResizeIslandBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_sleep_islands == nullptr || m_island_capacity < capacity)
		{
			m_r_sleep_islands = m_gpu.CreateResource(ResDesc::Buf<GpuSleepIsland>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SleepIslands");
			m_island_capacity = capacity;
		}
	}
	void GpuSleepManager::ResizeBodyBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_sleep_parents == nullptr || m_body_capacity < capacity)
		{
			m_r_sleep_parents = m_gpu.CreateResource(ResDesc::Buf<int>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SleepParents");
			m_r_sleep_stats = m_gpu.CreateResource(ResDesc::Buf<GpuSleepIslandStats>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SleepStats");
			m_body_capacity = capacity;
		}
	}

	// Upload staged sleep islands to the GPU.
	void GpuSleepManager::Upload(GpuJob& job, std::span<GpuSleepIsland const> islands)
	{
		ResizeIslandBuffers(job.m_cmd_list, static_cast<int>(islands.size()));
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

		assert(m_r_sleep_islands != nullptr && m_island_capacity >= island_count);
		assert(bodies != nullptr);

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf2bc45, "Physics::SleepWake");

		cbSleep cb_sleep = {
			.dt = 0.0f,
			.sleep_velocity_threshold_lin = m_config.sleep_velocity_threshold_lin,
			.sleep_velocity_threshold_ang = m_config.sleep_velocity_threshold_ang,
			.sleep_delay_s = m_config.sleep_delay_s,
			.body_count = body_count,
			.island_count = island_count,
			.max_contacts = 0,
			.sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
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

	// Persist wake-ups and update automatic sleeping from the resolved contact graph.
	void GpuSleepManager::SleepUpdate(GpuJob& job, float dt, int body_count, int island_count, int max_contacts, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> contact_dispatch, D3DPtr<ID3D12Resource> bodies)
	{
		if (body_count == 0 || !m_config.sleeping_enabled)
			return;

		ResizeBodyBuffers(job.m_cmd_list, body_count);
		assert(bodies != nullptr);
		assert(counters != nullptr);
		assert(contacts != nullptr);
		assert(contact_dispatch != nullptr);
		assert(m_r_sleep_islands != nullptr && m_island_capacity >= island_count);

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf2a545, "Physics::SleepUpdate");

		cbSleep cb_sleep = {
			.dt = dt,
			.sleep_velocity_threshold_lin = m_config.sleep_velocity_threshold_lin,
			.sleep_velocity_threshold_ang = m_config.sleep_velocity_threshold_ang,
			.sleep_delay_s = m_config.sleep_delay_s,
			.body_count = body_count,
			.island_count = island_count,
			.max_contacts = max_contacts,
			.sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
		};

		{
			job.m_barriers.Transition(m_r_sleep_islands.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_sleep_parents.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_sleep_stats.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(contact_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Commit();
		}
		auto bind_update = [&](ComputeStep& step)
		{
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sleep);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_sleep_islands->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_sleep_parents->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_sleep_stats->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(contacts->GetGPUVirtualAddress());
		};
		auto commit_scratch_barriers = [&]()
		{
			job.m_barriers.UAV(m_r_sleep_parents.get());
			job.m_barriers.UAV(m_r_sleep_stats.get());
			job.m_barriers.Commit();
		};

		auto body_dispatch_count = (body_count + SleepThreadCount - 1) / SleepThreadCount;

		{
			bind_update(m_cs_init_sleep_state);
			job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);
			commit_scratch_barriers();
		}
		{
			// Parallel union can miss a transitive edge if two contacts relink a shared endpoint in the same dispatch. Repeating hook/compress sweeps makes
			// the connected components converge without serialising contact processing.
			auto union_iteration_count = 1;
			for (auto span = body_count; span > 2 && union_iteration_count != 8; span = (span + 1) / 2)
				++union_iteration_count;

			for (int iteration = 0; iteration != union_iteration_count; ++iteration)
			{
				bind_update(m_cs_union_sleep_contacts);
				job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, contact_dispatch.get());

				job.m_barriers.UAV(m_r_sleep_parents.get());
				job.m_barriers.Commit();

				bind_update(m_cs_canonicalise_roots);
				job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);

				job.m_barriers.UAV(m_r_sleep_parents.get());
				job.m_barriers.Commit();
			}
		}
		{
			bind_update(m_cs_reduce_sleep_stats);
			job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);
			commit_scratch_barriers();
		}
		{
			bind_update(m_cs_apply_sleep_state);
			job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);

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
