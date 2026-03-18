//*********************************************
// Physics Sandbox — GPU Sort-and-Sweep Broadphase
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/sweep_gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Integrate constants
	struct alignas(16) cbSweep
	{
		int g_max_pair_count; // The maximum length of the g_collision_pairs buffer
		int g_pad0;
		int g_pad1;
		int g_pad2;
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
	};

	GpuSortAndSweep::GpuSortAndSweep(Gpu& gpu)
		: m_gpu(gpu)
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
			.Define(L"PR_COLLISION_DIAGNOSTICS", L"" PR_STRINGISE(PR_COLLISION_DIAGNOSTICS))
			.ShaderModel(L"cs_6_0")
			.Optimise();

		// m_cs_sweep: Root signature: constants + 2 SRVs (AABB, AABB_Idx) + 3 UAVs (ColPairs, Counters, DispatchArgs)
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbSweep>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::ColPairs)
				.UAV(EReg::DispatchArgs)
				.SRV(EReg::Bodies)
				.SRV(EReg::AABB_Idx)
				;

			auto bytecode = compiler.EntryPoint(L"CSSweep").Compile();

			m_cs_sweep.m_sig = sig.Create(m_gpu, "Physics:SweepSig");
			m_cs_sweep.m_pso = ComputePSO(m_cs_sweep.m_sig.get(), bytecode).Create(m_gpu, "Physics:SweepPSO");
		}

		// m_cs_calc_dispatch: Root signature: constants + 1 SRV (Counters) + 1 UAV (DispatchArgs)
		{
			auto sig= RootSig(ERootSigFlags::ComputeOnly)
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
	void GpuSortAndSweep::Sweep(GpuJob& job, int body_count, int max_col_pairs, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> aabb_idx, D3DPtr<ID3D12Resource> bodies)
	{
		ResizeBuffers(job.m_cmd_list, max_col_pairs);

		// Switch states for resources
		{
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_col_pairs.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_cd_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(aabb_idx.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Dispatch the compute shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_sweep.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_sweep.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbSweep{ .g_max_pair_count = max_col_pairs });
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_col_pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_cd_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(aabb_idx->GetGPUVirtualAddress());

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
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_cd_dispatch->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_cd_dispatch.get());
			job.m_barriers.Commit();
		}
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
		auto pair_count = std::min(counters.pair_count, static_cast<int>(out_pairs.size()));
		std::memcpy(out_pairs.data(), readback_pairs.ptr<GpuCollisionPair>(), pair_count * sizeof(GpuCollisionPair));
		return out_pairs.subspan(0, pair_count);
	}

	// CPU-side testing: upload bodies, sort + sweep, readback pairs. Calls job.Run() internally.
	std::span<GpuCollisionPair> GpuSortAndSweep::SortAndSweep(GpuJob& job, std::span<GpuRigidBody const> bodies, int sort_axis, int max_col_pairs, std::span<GpuCollisionPair> out_pairs)
	{
		auto body_count = static_cast<int>(bodies.size());
		if (body_count < 2)
			return {};

		// Create temporary GPU resources
		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempCounters");
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(body_count, {}), job.m_cmd_list, "Physics:TempBodies");
		auto r_aabb = m_gpu.CreateResource(ResDesc::Buf<float>(2 * body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABB");
		auto r_aabb_idx = m_gpu.CreateResource(ResDesc::Buf<int>(2 * body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempAABBIdx");

		// Upload counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload.ptr<GpuCollisionCounters>() = GpuCollisionCounters{ .body_count = body_count };
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
		Sweep(job, body_count, max_col_pairs, r_counters, r_aabb_idx, r_bodies);

		// Read back data from the GPU
		return Readback(job, r_counters, out_pairs);
	}

	// Custom deleter implementation
	void Deleter<GpuSortAndSweep>::operator()(GpuSortAndSweep* p) const
	{
		delete p;
	}
}



#if 0


// Enumerate overlapping pairs using pre-computed world-space AABBs from the GPU integrate step.
void GpuSortAndSweep::EnumOverlappingPairs(GpuJob& job, std::span<IntegrateAABB const> aabbs, std::function<void(RigidBody const&, RigidBody const&)> cb) const
{
	// TODO: 'aabbs' is not matched up with 'm_entity' in any way.
	// Need to think about how this might work when broadphase is entirely on the gpu... not now though.
	// I think ultimately, the best option is to not expose the Broadphase in the API. All physical objects need collision detection
	// otherwise, they're not physics objects... although neighbouring links in a multi-body might be a special case.  That can be handled
	// by a collision group bit-mask at some point

	assert(m_entity.size() == aabbs.size() && "There is an assumption that all bodies are registered with the boardphase and the order is the same");

	//auto n = static_cast<int>(m_entity.size());
	auto n = static_cast<int>(aabbs.size());
	if (n < 2)
		return;

	// Measure the variance of body positions along each axis to choose the best primary sort axis.
	auto var_sq = v4::Zero();
	auto var_sum = v4::Zero();
	for (auto const& aabb : aabbs)
	{
		auto c = aabb.ws_bbox.m_centre;
		var_sum += c;
		var_sq += c * c;
	}

	// Variance = E[x²] - E[x]² per axis
	auto inv_n = 1.0f / n;
	auto mean = var_sum * inv_n;
	auto variance = var_sq * inv_n - mean * mean;

	// Choose axis with maximum variance
	int axis = 0; // primary axis
	if (variance.y > variance.x) axis = 1;
	if (variance.z > variance[axis]) axis = 2;

	// Pack AABB endpoints for the chosen axis
	// 'keys' is the value to sort on, 'payloads' is the buffer that is actually sorted
	auto endpoint_count = n * 2;
	m_keys.resize(endpoint_count);
	m_payloads.resize(endpoint_count);
	for (auto const& [aabb, i] : with_index(aabbs))
	{
		m_keys[i * 2 + 0] = aabb.ws_bbox.m_centre[axis] - aabb.ws_bbox.m_radius[axis];
		m_keys[i * 2 + 1] = aabb.ws_bbox.m_centre[axis] + aabb.ws_bbox.m_radius[axis];

		// Add the bbox index with the bound type marker (begin/end)
		m_payloads[i * 2 + 0] = static_cast<uint32_t>(i << 1) | 0; // begin
		m_payloads[i * 2 + 1] = static_cast<uint32_t>(i << 1) | 1; // end
	}

	// GPU radix sort by primary axis coordinate
	{
		m_sorter.Resize(job.m_cmd_list, endpoint_count);
		m_sorter.Sort(m_keys, m_payloads, job);
	}


	m_sweep.reserve(n);
	m_sweep.resize(0);

	// CPU sweep with full 3-axis AABB filtering
	for (int i = 0; i != endpoint_count; ++i)
	{
		auto payload = m_payloads[i];
		auto body_idx = static_cast<int>(payload >> 1);
		auto is_end = (payload & 1) != 0;

		// "end" means the body is leaving the active set,
		if (is_end)
		{
			// Unstable erase
			auto at = std::find(begin(m_sweep), end(m_sweep), body_idx);
			if (at != end(m_sweep))
			{
				*at = m_sweep.back();
				m_sweep.pop_back();
			}
		}

		// Compare this aabb with those in the active sweep set.
		else
		{
			auto const& aabb = aabbs[body_idx];
			for (auto other_idx : m_sweep)
			{
				auto const& other = aabbs[other_idx];
				
				// This actually retests the primary axis, but it's a very cheap test compared to the GPU sort and it keeps the code simpler.
				if (!IsIntersection(aabb.ws_bbox, other.ws_bbox))
					continue;

				// This is a weak assumption, that index in 'aabbbs' corresponds to index in 'm_entity'!
				cb(*m_entity[body_idx], *m_entity[other_idx]);
			}

			m_sweep.push_back(body_idx);
		}
	}
}





// Enumerate all pairs of bodies whose bounding boxes overlap.
// Uses GPU radix sort for the primary axis, then CPU sweep with Y/Z filtering.
void GpuSortAndSweep::EnumOverlappingPairs(GpuJob& job, std::function<void(RigidBody const&, RigidBody const&)> cb) const
{
	auto n = static_cast<int>(m_entity.size());
	if (n < 2) return;

	// Step 1: Compute world-space AABBs and choose the primary sort axis.
	// The axis with the largest position variance gives the best separation.
	m_bboxes.resize(n);
	auto var_sum = v4::Zero();
	auto var_sq = v4::Zero();
	for (int i = 0; i != n; ++i)
	{
		if (!m_entity[i]->HasShape())
		{
			// Body has no shape — give it a degenerate bbox at its position
			m_bboxes[i] = BBox(m_entity[i]->O2W().pos, v4::Zero());
			continue;
		}
		m_bboxes[i] = m_entity[i]->BBoxWS();
		auto c = m_bboxes[i].Centre();
		var_sum += c;
		var_sq += v4(c.x * c.x, c.y * c.y, c.z * c.z, 0);
	}

	// Variance = E[x²] - E[x]² per axis
	auto inv_n = 1.0f / n;
	auto mean = var_sum * inv_n;
	auto variance = var_sq * inv_n - v4(mean.x * mean.x, mean.y * mean.y, mean.z * mean.z, 0);

	// Choose axis with maximum variance
	int axis = 0;
	if (variance.y > variance.x) axis = 1;
	if (variance.z > variance[axis]) axis = 2;

	// Step 2: Pack AABB endpoints for the chosen axis.
	// Each body produces two entries: a "begin" (min) and "end" (max).
	// Also precompute the secondary axis bounds for the CPU sweep inner loop.
	auto endpoint_count = n * 2;
	m_keys.resize(endpoint_count);
	m_payloads.resize(endpoint_count);
	m_axis_bounds.resize(n);

	int axis_y = (axis + 1) % 3;
	int axis_z = (axis + 2) % 3;

	for (int i = 0; i != n; ++i)
	{
		auto& bb = m_bboxes[i];
		auto lo = bb.Centre()[axis] - bb.Radius()[axis];
		auto hi = bb.Centre()[axis] + bb.Radius()[axis];

		m_keys[i * 2 + 0] = lo;
		m_keys[i * 2 + 1] = hi;
		m_payloads[i * 2 + 0] = static_cast<uint32_t>(i << 1) | 0; // begin
		m_payloads[i * 2 + 1] = static_cast<uint32_t>(i << 1) | 1; // end

		// Precompute secondary axis bounds to avoid repeated Centre/Radius lookups in sweep
		m_axis_bounds[i] = {
			bb.Centre()[axis_y] - bb.Radius()[axis_y],
			bb.Centre()[axis_y] + bb.Radius()[axis_y],
			bb.Centre()[axis_z] - bb.Radius()[axis_z],
			bb.Centre()[axis_z] + bb.Radius()[axis_z],
		};
	}

	// Step 3: GPU radix sort by primary axis coordinate.
	{
		m_sorter.Resize(job.m_cmd_list, endpoint_count);
		m_sorter.Sort(m_keys, m_payloads, job);
	}

	// Step 4: CPU sweep with full 3-axis AABB filtering.
	// Walk the sorted endpoints. "Begin" markers add to the active set,
	// "end" markers remove. For each new "begin", test Y and Z overlap
	// against all currently active bodies.
	auto active = std::vector<int>();
	active.reserve(n);

	for (int i = 0; i != endpoint_count; ++i)
	{
		auto payload = m_payloads[i];
		auto body_idx = static_cast<int>(payload >> 1);
		auto is_end = (payload & 1) != 0;

		if (is_end)
		{
			// Remove from active set (swap-with-last for O(1))
			auto at = std::find(active.begin(), active.end(), body_idx);
			if (at != active.end())
			{
				*at = active.back();
				active.pop_back();
			}
		}
		else
		{
			// "Begin" — test against all active bodies on the remaining two axes
			auto const& new_ab = m_axis_bounds[body_idx];

			for (auto active_idx : active)
			{
				auto const& act_ab = m_axis_bounds[active_idx];

				// Y-axis overlap test
				if (new_ab.lo_y > act_ab.hi_y || new_ab.hi_y < act_ab.lo_y)
					continue;

				// Z-axis overlap test
				if (new_ab.lo_z > act_ab.hi_z || new_ab.hi_z < act_ab.lo_z)
					continue;

				// Full 3-axis AABB overlap — emit pair
				cb(*m_entity[active_idx], *m_entity[body_idx]);
			}

			active.push_back(body_idx);
		}
	}
}
#endif
