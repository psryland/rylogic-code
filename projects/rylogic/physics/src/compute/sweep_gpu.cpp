//*********************************************
// Physics Sandbox — GPU Sort-and-Sweep Broadphase
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/sweep_gpu.h"
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	// Integrate constants
	struct alignas(16) cbSweep
	{
		int max_pair_count; // The maximum length of the g_collision_pairs buffer
		int body_count;
		int sleeping_enabled;
		int sleep_island_count;
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
		inline static constexpr auto AABB_Box = ESRVReg::t2;
		inline static constexpr auto SleepIslands = ESRVReg::t3;
	};

	GpuSortAndSweep::GpuSortAndSweep(Gpu& gpu, EngineConfig const& config, IShaderCache* shader_cache)
		: m_gpu(gpu)
		, m_config(config)
		, m_sorter(gpu.m_gpu, BoundsSorter::TuningParams{}, shader_cache)
		, m_cs_sweep()
		, m_cs_calc_dispatch()
		, m_r_col_pairs()
		, m_r_cd_dispatch()
		, m_max_col_pairs()
	{
		// Compile the compute shaders
		auto resolver = shader_cache::ResourceSourceResolver{};
		auto compiler = ShaderCompiler{}
			.Cache(shader_cache)
			.Source("src/compute/sweep.hlsl", resolver)
			.HlslVersion(EHlslVersion::DxcDefault)
			.ShaderModel(L"cs_6_0")
			.Optimise();

		auto make_sweep_sig = [&]()
		{
			return RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSweep>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::ColPairs)
				.UAV(EReg::DispatchArgs)
				.SRV(EReg::Bodies)
				.SRV(EReg::AABB_Idx)
				.SRV(EReg::AABB_Box)
				.SRV(EReg::SleepIslands)
				;
		};

		// m_cs_sweep
		{
			auto sig = make_sweep_sig();
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
	void GpuSortAndSweep::Sweep(
		GpuJob& job,
		int body_count,
		int max_col_pairs,
		D3DPtr<ID3D12Resource> counters,
		D3DPtr<ID3D12Resource> aabb_idx,
		D3DPtr<ID3D12Resource> aabb_box,
		D3DPtr<ID3D12Resource> bodies,
		int sleep_island_count,
		D3DPtr<ID3D12Resource> sleep_islands,
		bool sleeping_enabled)
	{
		pix::BeginEvent(job.m_cmd_list.get(), 0xFF45bcf2, "Physics::Sweep");

		ResizeBuffers(job.m_cmd_list, max_col_pairs);

		cbSweep cb_sweep = {
			.max_pair_count = max_col_pairs,
			.body_count = body_count,
			.sleeping_enabled = sleeping_enabled ? 1 : 0,
			.sleep_island_count = sleep_island_count,
		};

		// Switch states for resources
		{
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_col_pairs.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_cd_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(aabb_idx.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(aabb_box.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
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
			job.m_cmd_list.AddComputeRootShaderResourceView(aabb_box->GetGPUVirtualAddress());
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
		auto r_aabb_sort = m_gpu.CreateResource(ResDesc::Buf<float>(2 * body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABB_Sort");
		auto r_aabb_box = m_gpu.CreateResource(ResDesc::Buf<BBox>(body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABB_Box");
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
			job.m_barriers.Transition(r_aabb_idx.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_aabb_sort.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_aabb_box.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload_aabb_sort = job.m_upload.Alloc<float>(2 * body_count);
			auto upload_aabb_box = job.m_upload.Alloc<BBox>(body_count);
			auto upload_idx = job.m_upload.Alloc<int>(2 * body_count);
			auto* bounds_sort = upload_aabb_sort.ptr<float>();
			auto* bounds_box = upload_aabb_box.ptr<BBox>();
			auto* idx = upload_idx.ptr<int>();
			auto const margin = std::max(m_config.broadphase_aabb_margin, 0.0f);
			for (int i = 0; i != body_count; ++i)
			{
				auto ws_bbox = bodies[i].o2w * bodies[i].os_bbox;
				auto lower = ws_bbox.Lower();
				auto upper = ws_bbox.Upper();
				bounds_box[i] = ws_bbox;
				switch (sort_axis)
				{
				case 0:
				{
					bounds_sort[i * 2 + 0] = lower.x - margin;
					bounds_sort[i * 2 + 1] = upper.x + margin;
					break;
				}
				case 1:
				{
					bounds_sort[i * 2 + 0] = lower.y - margin;
					bounds_sort[i * 2 + 1] = upper.y + margin;
					break;
				}
				default:
				{
					bounds_sort[i * 2 + 0] = lower.z - margin;
					bounds_sort[i * 2 + 1] = upper.z + margin;
					break;
				}
				}
				idx[i * 2 + 0] = (i << 1) | 0; // start
				idx[i * 2 + 1] = (i << 1) | 1; // end
			}
			job.m_cmd_list.CopyBufferRegion(r_aabb_sort.get(), 0, upload_aabb_sort);
			job.m_cmd_list.CopyBufferRegion(r_aabb_box.get(), 0, upload_aabb_box);
			job.m_cmd_list.CopyBufferRegion(r_aabb_idx.get(), 0, upload_idx);

			job.m_barriers.Transition(r_aabb_idx.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(r_aabb_sort.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(r_aabb_box.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Run the sort step
		Sort(job, body_count, r_aabb_sort, r_aabb_idx);

		// Run the sweep step
		Sweep(job, body_count, pair_count, r_counters, r_aabb_idx, r_aabb_box, r_bodies, 0, r_sleep_islands, m_config.sleeping_enabled);

		// Read back data from the GPU
		return Readback(job, r_counters, out_pairs);
	}

	// Custom deleter implementation
	void Deleter<GpuSortAndSweep>::operator()(GpuSortAndSweep* p) const
	{
		delete p;
	}
}
