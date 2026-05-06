//*********************************************
// Physics Sandbox — GPU Sort-and-Sweep Broadphase
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/sweep_gpu.h"
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Integrate constants
	struct alignas(16) cbSweep
	{
		int g_max_pair_count; // The maximum length of the g_collision_pairs buffer
		int g_body_count;
		int g_sleeping_enabled;
		int g_sleep_island_count;
	};
	static_assert(sizeof(cbSweep) == 16);

	// Register assignments for the root signature
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Counters = EUAVReg::u0;
		inline static constexpr auto ColPairs = EUAVReg::u1;
		inline static constexpr auto DispatchArgs = EUAVReg::u2;
		inline static constexpr auto Bodies = ESRVReg::t0;
		inline static constexpr auto AABB_Idx = ESRVReg::t1;
		inline static constexpr auto SleepIslands = ESRVReg::t2;
	};

	GpuSortAndSweep::GpuSortAndSweep(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_sorter(gpu.m_gpu)
		, m_cs_sweep()
		, m_cs_calc_dispatch()
		, m_r_col_pairs()
		, m_r_cd_dispatch()
		, m_max_col_pairs()
	{
		CompileShaders();
	}

	// Compile the compute shaders
	void GpuSortAndSweep::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/sweep.hlsl", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.ShaderModel(L"cs_6_0")
			.Optimise();

		// m_cs_sweep: Root signature: constants + 3 SRVs (Bodies, AABB_Idx, SleepIslands) + 3 UAVs (Counters, ColPairs, DispatchArgs)
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSweep>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::ColPairs)
				.UAV(EReg::DispatchArgs)
				.SRV(EReg::Bodies)
				.SRV(EReg::AABB_Idx)
				.SRV(EReg::SleepIslands)
				;

			auto bytecode = compiler.EntryPoint(L"CSSweep").Compile();

			m_cs_sweep.m_sig = sig.Create(m_gpu, "Physics:SweepSig");
			m_cs_sweep.m_pso = ComputePSO(m_cs_sweep.m_sig.get(), bytecode).Create(m_gpu, "Physics:SweepPSO");
		}

		// m_cs_calc_dispatch: Root signature: constants + 1 SRV (Counters) + 1 UAV (DispatchArgs)
		{
			auto sig= RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSweep>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::DispatchArgs);

			auto bytecode = compiler.EntryPoint(L"CSCalcCDDispatch").Compile();

			m_cs_calc_dispatch.m_sig = sig.Create(m_gpu, "Physics:CalcCDDispatchSig");
			m_cs_calc_dispatch.m_pso = ComputePSO(m_cs_calc_dispatch.m_sig.get(), bytecode).Create(m_gpu, "Physics:CalcCDDispatchPSO");
		}
	}

	// Resize the buffers to support
	void GpuSortAndSweep::ResizeBuffers(CmdList& cmd_list, int max_col_pairs)
	{
		max_col_pairs = std::max(1, max_col_pairs);
		
		if (m_r_col_pairs == nullptr || m_max_col_pairs < max_col_pairs)
		{
			m_r_col_pairs = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionPair>(max_col_pairs, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionPairs");
			m_max_col_pairs = max_col_pairs;
		}
		if (m_r_cd_dispatch == nullptr)
		{
			m_r_cd_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CDDispatchArgs");
		}
	}
	
	// Sort the body index list based on the bounding box bounds
	void GpuSortAndSweep::Sort(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> aabb, D3DPtr<ID3D12Resource> aabb_idx)
	{
		// Defer sorting to the radix sorter. This wills sort the "payload" in 'aabb_idx' based on the "keys" in 'aabb'
		m_sorter.Bind(job.m_cmd_list, 2*body_count, aabb, aabb_idx);
		m_sorter.Sort(job.m_cmd_list);
	}

	// Enumerate overlapping pairs using pre-computed world-space AABBs from the GPU integrate step.
	void GpuSortAndSweep::Sweep(GpuJob& job, int body_count, int max_col_pairs, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> aabb_idx, D3DPtr<ID3D12Resource> bodies, int sleep_island_count, D3DPtr<ID3D12Resource> sleep_islands)
	{
		pix::BeginEvent(job.m_cmd_list.get(), 0xFF45bcf2, "Physics::Sweep");

		ResizeBuffers(job.m_cmd_list, max_col_pairs);

		cbSweep cb_sweep = {
			.g_max_pair_count = max_col_pairs,
			.g_body_count = body_count,
			.g_sleeping_enabled = m_config.sleeping_enabled ? 1 : 0,
			.g_sleep_island_count = sleep_island_count,
		};

		// Switch states for resources
		{
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_col_pairs.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_cd_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(aabb_idx.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(sleep_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Dispatch the compute shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_sweep.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_sweep.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sweep);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_col_pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_cd_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(aabb_idx->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(sleep_islands->GetGPUVirtualAddress());

			// One thread for each array element in the AABB index buffer
			auto dispatch_count = (2*body_count + SweepThreadCount - 1) / SweepThreadCount;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(counters.get());
			job.m_barriers.UAV(m_r_col_pairs.get());
			job.m_barriers.Commit();
		}

		// Dispatch the calculate dispatch size shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_calc_dispatch.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_calc_dispatch.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sweep);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_cd_dispatch->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_cd_dispatch.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Read back the results of the Sort and Sweep steps.
	std::span<GpuCollisionPair> GpuSortAndSweep::Readback(GpuJob& job, D3DPtr<ID3D12Resource> r_counters, std::span<GpuCollisionPair> out_pairs)
	{
		// This is a debug synchronisation point — it flushes all queued GPU work,
		// then makes the broadphase output available on the CPU for inspection.
		GpuReadbackBuffer::Allocation readback_pairs;
		GpuReadbackBuffer::Allocation readback_counters;

		// Readback pairs and counters
		{
			job.m_barriers.Transition(m_r_col_pairs.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_pairs = job.m_readback.Alloc<GpuCollisionPair>(static_cast<int>(out_pairs.size()));
			job.m_cmd_list.CopyBufferRegion(readback_pairs, m_r_col_pairs.get(), 0);

			readback_counters = job.m_readback.Alloc<GpuCollisionCounters>(1);
			job.m_cmd_list.CopyBufferRegion(readback_counters, r_counters.get(), 0);

			job.m_barriers.Transition(m_r_col_pairs.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		job.Run();

		auto& counters = *readback_counters.ptr<GpuCollisionCounters>();
		auto max_pairs = static_cast<int>(out_pairs.size());
		if (counters.pair_count > max_pairs)
		{
			throw std::runtime_error(std::format(
				"GPU broadphase pair readback overflow: {} potential pairs generated for {} output slots.",
				counters.pair_count,
				max_pairs));
		}
		auto pair_count = std::min(counters.pair_count, max_pairs);
		std::memcpy(out_pairs.data(), readback_pairs.ptr<GpuCollisionPair>(), pair_count * sizeof(GpuCollisionPair));
		return out_pairs.subspan(0, pair_count);
	}

	// CPU-side testing: upload bodies, sort + sweep, readback pairs. Calls job.Run() internally.
	std::span<GpuCollisionPair> GpuSortAndSweep::SortAndSweep(GpuJob& job, std::span<GpuRigidBody const> bodies, int sort_axis, std::span<GpuCollisionPair> out_pairs)
	{
		auto body_count = static_cast<int>(bodies.size());
		auto pair_count = static_cast<int>(out_pairs.size());
		if (body_count < 2)
			return {};

		// Create temporary GPU resources
		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempCounters");
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(body_count, {}), job.m_cmd_list, "Physics:TempBodies");
		auto r_aabb = m_gpu.CreateResource(ResDesc::Buf<float>(2 * body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABB");
		auto r_aabb_idx = m_gpu.CreateResource(ResDesc::Buf<int>(2 * body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABBIdx");
		auto r_sleep_islands = m_gpu.CreateResource(ResDesc::Buf<GpuSleepIsland>(1, {}), job.m_cmd_list, "Physics:TempSleepIslands");

		// Upload counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.pair_count = 0,
				.contact_count = 0,
			};
			job.m_cmd_list.CopyBufferRegion(r_counters.get(), 0, upload);

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Upload bodies
		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuRigidBody>(body_count);
			memcpy(upload.ptr<GpuRigidBody>(), bodies.data(), body_count * sizeof(GpuRigidBody));
			job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, upload);

			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload AABBs and AABB_Idx
		{
			job.m_barriers.Transition(r_aabb.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_aabb_idx.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload_aabb = job.m_upload.Alloc<float>(2 * body_count);
			auto upload_idx = job.m_upload.Alloc<int>(2 * body_count);
			auto* bounds = upload_aabb.ptr<float>();
			auto* idx = upload_idx.ptr<int>();
			for (int i = 0; i != body_count; ++i)
			{
				auto ws_bbox = bodies[i].o2w * bodies[i].os_bbox;
				bounds[i * 2 + 0] = ws_bbox.Lower()[sort_axis];
				bounds[i * 2 + 1] = ws_bbox.Upper()[sort_axis];
				idx[i * 2 + 0] = (i << 1) | 0; // start
				idx[i * 2 + 1] = (i << 1) | 1; // end
			}
			job.m_cmd_list.CopyBufferRegion(r_aabb.get(), 0, upload_aabb);
			job.m_cmd_list.CopyBufferRegion(r_aabb_idx.get(), 0, upload_idx);

			job.m_barriers.Transition(r_aabb.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(r_aabb_idx.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Run the sort step
		Sort(job, body_count, r_aabb, r_aabb_idx);

		// Run the sweep step (skip sort for CPU-side testing)
		Sweep(job, body_count, pair_count, r_counters, r_aabb_idx, r_bodies, 0, r_sleep_islands);

		// Read back data from the GPU
		return Readback(job, r_counters, out_pairs);
	}

	// Custom deleter implementation
	void Deleter<GpuSortAndSweep>::operator()(GpuSortAndSweep* p) const
	{
		delete p;
	}
}
