//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/selective_gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	struct alignas(16) cbSelectiveRefresh
	{
		int g_max_pairs;
		int g_max_contacts;
		int g_body_count;
		int g_sleeping_enabled;
		int g_full_max_pairs;
		int g_support_only;
		int pad_i1;
		int pad_i2;

		float g_depth_slop;
		float g_support_depth_slop;
		float g_closing_speed_slop;
		float g_support_alignment;

		float g_aabb_margin;
		float pad0;
		float pad1;
		float pad2;
	};
	static_assert((sizeof(cbSelectiveRefresh) & 0xf) == 0);

	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto DstCounters = EUAVReg::u0;
		inline static constexpr auto DstPairs = EUAVReg::u1;
		inline static constexpr auto DstDispatchArgs = EUAVReg::u2;
		inline static constexpr auto ProblemBodies = EUAVReg::u3;
		inline static constexpr auto Metrics = EUAVReg::u4;
		inline static constexpr auto Bodies = EUAVReg::u5;
		inline static constexpr auto SourceCounters = ESRVReg::t0;
		inline static constexpr auto SourceContacts = ESRVReg::t1;
		inline static constexpr auto FullCounters = ESRVReg::t2;
		inline static constexpr auto FullPairs = ESRVReg::t3;
	};

	GpuSelectiveRefresher::GpuSelectiveRefresher(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_prepare()
		, m_cs_score_contacts()
		, m_cs_compact_pairs()
		, m_cs_build_dispatch()
		, m_cmd_sig()
		, m_r_problem_bodies()
		, m_r_metrics()
		, m_work_sets()
		, m_body_capacity()
		, m_max_pairs()
		, m_max_contacts()
	{
		CompileShaders();

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

	void GpuSelectiveRefresher::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/selective.hlsl", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.ShaderModel(L"cs_6_0")
			.Optimise();

		auto compile = [&](ComputeStep& step, wchar_t const* entry_point, char const* sig_name, char const* pso_name, RootSig& root_sig)
		{
			auto bytecode = compiler.EntryPoint(entry_point).Compile();

			step.m_sig = root_sig.Create(m_gpu, sig_name);
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};

		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSelectiveRefresh>(EReg::Params)
				.UAV(EReg::DstCounters)
				.UAV(EReg::DstDispatchArgs)
				.UAV(EReg::ProblemBodies)
				.UAV(EReg::Metrics);
			compile(
				m_cs_prepare,
				L"CSPrepareSelectiveRefresh",
				"Physics:SelectivePrepareSig",
				"Physics:SelectivePreparePSO",
				sig);
		}

		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSelectiveRefresh>(EReg::Params)
				.UAV(EReg::ProblemBodies)
				.UAV(EReg::Metrics)
				.UAV(EReg::Bodies)
				.SRV(EReg::SourceCounters)
				.SRV(EReg::SourceContacts);
			compile(
				m_cs_score_contacts,
				L"CSScoreSelectiveContacts",
				"Physics:SelectiveScoreSig",
				"Physics:SelectiveScorePSO",
				sig);
		}

		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSelectiveRefresh>(EReg::Params)
				.UAV(EReg::DstCounters)
				.UAV(EReg::DstPairs)
				.UAV(EReg::ProblemBodies)
				.UAV(EReg::Metrics)
				.UAV(EReg::Bodies)
				.SRV(EReg::FullCounters)
				.SRV(EReg::FullPairs);
			compile(
				m_cs_compact_pairs,
				L"CSCompactSelectivePairs",
				"Physics:SelectiveCompactSig",
				"Physics:SelectiveCompactPSO",
				sig);
		}

		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSelectiveRefresh>(EReg::Params)
				.UAV(EReg::DstCounters)
				.UAV(EReg::DstDispatchArgs);
			compile(
				m_cs_build_dispatch,
				L"CSBuildSelectiveDispatch",
				"Physics:SelectiveBuildDispatchSig",
				"Physics:SelectiveBuildDispatchPSO",
				sig);
		}
	}

	void GpuSelectiveRefresher::ResizeBuffers(CmdList& cmd_list, int body_count, int max_pairs, int max_contacts)
	{
		body_count = std::max(1, body_count);
		max_pairs = std::max(1, max_pairs);
		max_contacts = std::max(1, max_contacts);

		if (m_r_problem_bodies == nullptr || m_body_capacity < body_count)
		{
			m_r_problem_bodies = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(body_count, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveProblemBodies");
			m_body_capacity = body_count;
		}
		if (m_r_metrics == nullptr)
		{
			m_r_metrics = m_gpu.CreateResource(ResDesc::Buf<GpuSelectiveRefreshMetrics>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveMetrics");
		}
		if (m_max_pairs < max_pairs || m_max_contacts < max_contacts)
		{
			for (auto& work_set : m_work_sets)
			{
				if (work_set.m_counters == nullptr || work_set.m_max_pairs < max_pairs)
				{
					work_set.m_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveCounters");
					work_set.m_pairs = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionPair>(max_pairs, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectivePairs");
					work_set.m_cd_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveCDDispatch");
					work_set.m_max_pairs = max_pairs;
				}
				if (work_set.m_contacts == nullptr || work_set.m_max_contacts < max_contacts)
				{
					work_set.m_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveContacts");
					work_set.m_resolve_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SelectiveResolveDispatch");
					work_set.m_max_contacts = max_contacts;
				}
			}
			m_max_pairs = std::max(m_max_pairs, max_pairs);
			m_max_contacts = std::max(m_max_contacts, max_contacts);
		}
	}

	GpuSelectiveWorkSet& GpuSelectiveRefresher::BuildWorkSet(
		GpuJob& job,
		int pass_index,
		int body_count,
		int max_pairs,
		int max_contacts,
		int source_max_contacts,
		int full_max_pairs,
		D3DPtr<ID3D12Resource> source_counters,
		D3DPtr<ID3D12Resource> source_contacts,
		D3DPtr<ID3D12Resource> source_contact_dispatch,
		D3DPtr<ID3D12Resource> full_pair_dispatch,
		D3DPtr<ID3D12Resource> full_counters,
		D3DPtr<ID3D12Resource> full_pairs,
		D3DPtr<ID3D12Resource> bodies)
	{
		auto& work_set = m_work_sets[pass_index & 1];
		ResizeBuffers(job.m_cmd_list, body_count, max_pairs, max_contacts);

		cbSelectiveRefresh cb_selective = {
			.g_max_pairs = max_pairs,
			.g_max_contacts = source_max_contacts,
			.g_body_count = body_count,
			.g_sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
			.g_full_max_pairs = full_max_pairs,
			.g_support_only = m_config.selective_refresh_support_only ? 1 : 0,
			.pad_i1 = 0,
			.pad_i2 = 0,
			.g_depth_slop = m_config.selective_refresh_depth_slop,
			.g_support_depth_slop = m_config.selective_refresh_support_depth_slop,
			.g_closing_speed_slop = m_config.selective_refresh_closing_speed_slop,
			.g_support_alignment = m_config.selective_refresh_support_alignment,
			.g_aabb_margin = m_config.selective_refresh_aabb_margin,
			.pad0 = 0,
			.pad1 = 0,
			.pad2 = 0,
		};

		pix::BeginEvent(job.m_cmd_list.get(), 0xFF45f2bc, "Physics::SelectiveRefresh");

		{
			job.m_barriers.Transition(work_set.m_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(work_set.m_pairs.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(work_set.m_cd_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_problem_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_metrics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		{
			job.m_cmd_list.SetPipelineState(m_cs_prepare.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_prepare.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_selective);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_cd_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_problem_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_metrics->GetGPUVirtualAddress());

			auto dispatch_count = (std::max(1, body_count) + SelectiveRefreshThreadCount - 1) / SelectiveRefreshThreadCount;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(work_set.m_counters.get());
			job.m_barriers.UAV(work_set.m_cd_dispatch.get());
			job.m_barriers.UAV(m_r_problem_bodies.get());
			job.m_barriers.UAV(m_r_metrics.get());
			job.m_barriers.Commit();
		}

		{
			job.m_barriers.Transition(source_contact_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(source_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(source_contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_score_contacts.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_score_contacts.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_selective);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_problem_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_metrics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(source_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(source_contacts->GetGPUVirtualAddress());
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, source_contact_dispatch.get());

			job.m_barriers.UAV(m_r_problem_bodies.get());
			job.m_barriers.UAV(m_r_metrics.get());
			job.m_barriers.Commit();
		}

		{
			// Compact only the pair groups emitted by broadphase. Dispatching over the
			// whole pair capacity makes global defaults scale with buffer size rather
			// than with scene complexity.
			job.m_barriers.Transition(full_pair_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(full_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(full_pairs.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_compact_pairs.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_compact_pairs.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_selective);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_problem_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_metrics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(full_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(full_pairs->GetGPUVirtualAddress());

			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, full_pair_dispatch.get());

			job.m_barriers.UAV(work_set.m_counters.get());
			job.m_barriers.UAV(work_set.m_pairs.get());
			job.m_barriers.UAV(m_r_metrics.get());
			job.m_barriers.Commit();
		}

		{
			job.m_cmd_list.SetPipelineState(m_cs_build_dispatch.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_build_dispatch.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_selective);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(work_set.m_cd_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(work_set.m_cd_dispatch.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
		return work_set;
	}

	void Deleter<GpuSelectiveRefresher>::operator()(GpuSelectiveRefresher* p) const
	{
		delete p;
	}
}
