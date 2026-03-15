//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/collision/gpu_collision_detector.h"
#include "src/collision/gpu_collision_types.h"
#include <chrono>

namespace pr::physics
{
	using namespace pr::rdr12;

	// Thread group size matching the HLSL [numthreads(64, 1, 1)] declaration.
	static constexpr int ThreadGroupSize = 32;

	// Constant buffer layout matching the HLSL cbCollision declaration.
	struct alignas(16) cbCollision
	{
		uint32_t pair_count;
		uint32_t pad0;
		uint32_t pad1;
		uint32_t pad2;
	};
	static_assert(sizeof(cbCollision) == 16);

	// Register assignments for the collision root signature.
	// b0 = constants, t0 = shapes (SRV), t1 = pairs (SRV), t2 = verts (SRV),
	// u0 = contacts (UAV), u1 = counters (UAV)
	struct EReg
	{
		inline static constexpr ECBufReg Params = ECBufReg::b0;
		inline static constexpr ESRVReg  Shapes = ESRVReg::t0;
		inline static constexpr ESRVReg  Pairs = ESRVReg::t1;
		inline static constexpr ESRVReg  Verts = ESRVReg::t2;
		inline static constexpr EUAVReg  Contacts = EUAVReg::u0;
		inline static constexpr EUAVReg  Counters = EUAVReg::u1;
		inline static constexpr EUAVReg  Diag = EUAVReg::u2;
	};

	GpuCollisionDetector::GpuCollisionDetector(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_gjk()
		, m_r_shapes()
		, m_r_pairs()
		, m_r_verts()
		, m_r_contacts()
		, m_r_counters()
		, m_r_diag()
		, m_max_shapes()
		, m_max_verts()
		, m_max_pairs()
	{
		CompileShader();
	}

	// Run collision detection on the GPU.
	// Uploads shapes, pairs, and vertices → dispatches GJK shader → reads back contacts.
	int GpuCollisionDetector::DetectCollisions(
		GpuJob& job,
		std::span<GpuCollisionPair const> pairs,
		std::span<GpuShape const> shapes,
		std::span<v4 const> verts,
		std::vector<GpuContact>& out_contacts,
		bool shapes_dirty)
	{
		auto shape_count = static_cast<int>(shapes.size());
		auto vert_count = static_cast<int>(verts.size());
		auto pair_count = static_cast<int>(pairs.size());
		if (pair_count == 0)
			return 0;

		// If ResizeBuffers reallocated the shape/vert buffers, we must re-upload
		// regardless of the caller's dirty flag (the new buffers contain garbage).
		if (ResizeBuffers(job.m_cmd_list, shape_count, vert_count, pair_count))
			shapes_dirty = true;

		// Only upload shapes and verts when they've changed. When the shape cache
		// reports clean (no new shapes, no evictions), the previous frame's GPU
		// buffers are still valid and we skip the upload entirely.
		if (shapes_dirty)
		{
			// Upload shapes buffer
			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto shape_upload = job.m_upload.Alloc<GpuShape>(shape_count);
			memcpy(shape_upload.ptr<GpuShape>(), shapes.data(), shape_count * sizeof(GpuShape));
			job.m_cmd_list.CopyBufferRegion(m_r_shapes.get(), 0, shape_upload);

			// Upload vertices buffer (may be empty if no polytopes/triangles)
			if (vert_count > 0)
			{
				job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				job.m_barriers.Commit();

				auto vert_upload = job.m_upload.Alloc<v4>(vert_count);
				memcpy(vert_upload.ptr<v4>(), verts.data(), vert_count * sizeof(v4));
				job.m_cmd_list.CopyBufferRegion(m_r_verts.get(), 0, vert_upload);
			}

			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			if (vert_count > 0)
				job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Pairs change every frame — always upload
		{
			job.m_barriers.Transition(m_r_pairs.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto pair_upload = job.m_upload.Alloc<GpuCollisionPair>(pair_count);
			memcpy(pair_upload.ptr<GpuCollisionPair>(), pairs.data(), pair_count * sizeof(GpuCollisionPair));
			job.m_cmd_list.CopyBufferRegion(m_r_pairs.get(), 0, pair_upload);

			job.m_barriers.Transition(m_r_pairs.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Zero the counter buffer before dispatch
		{
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			auto* counters = upload.ptr<GpuCollisionCounters>();
			*counters = {};
			job.m_cmd_list.CopyBufferRegion(m_r_counters.get(), 0, upload);

			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Dispatch the collision compute shader
		{
			auto cb = cbCollision{ .pair_count = static_cast<uint32_t>(pair_count) };

			job.m_cmd_list.SetPipelineState(m_cs_gjk.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_gjk.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_shapes->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_verts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_counters->GetGPUVirtualAddress());
			#if PR_DBG
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diag->GetGPUVirtualAddress());
			#endif

			auto dispatch_count = (pair_count + ThreadGroupSize - 1) / ThreadGroupSize;
			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			job.m_barriers.UAV(m_r_contacts.get());
			job.m_barriers.UAV(m_r_counters.get());
			#if PR_DBG
			job.m_barriers.UAV(m_r_diag.get());
			#endif
		}

		// Read back contacts, counter, and diagnostics
		GpuReadbackBuffer::Allocation readback_contacts;
		GpuReadbackBuffer::Allocation readback_counters;
		{
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_contacts = job.m_readback.Alloc<GpuContact>(pair_count);
			job.m_cmd_list.CopyBufferRegion(readback_contacts, m_r_contacts.get(), 0);

			readback_counters = job.m_readback.Alloc<GpuCollisionCounters>(1);
			job.m_cmd_list.CopyBufferRegion(readback_counters, m_r_counters.get(), 0);

			// Transition back for next frame
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}
		#if PR_DBG
		GpuReadbackBuffer::Allocation readback_diag;
		{
			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_diag = job.m_readback.Alloc<GpuPairDiag>(pair_count);
			job.m_cmd_list.CopyBufferRegion(readback_diag, m_r_diag.get(), 0);

			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}
		#endif

		// Execute and wait for GPU completion
		job.Run();

		// Read the contact count
		auto* counters = readback_counters.ptr<GpuCollisionCounters>();
		auto contact_count = static_cast<int>(counters->contact_count);
		contact_count = std::min(contact_count, pair_count); // safety clamp

		// Log per-pair diagnostics
		#if PR_DBG
		if (pair_count > 10)
		{
			auto t_end = std::chrono::high_resolution_clock::now();
			static auto t_start = t_end;
			auto gpu_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
			t_start = t_end;

			static FILE* f_timing = nullptr;
			if (!f_timing) f_timing = fopen("dump\\gjk_timing.log", "w");
			if (f_timing)
			{
				static char const* shape_names[] = {"sphere", "box", "line", "tri", "poly"};
				fprintf(f_timing, "--- pairs=%d contacts=%d gpu=%.2fms ---\n", pair_count, contact_count, gpu_ms);

				auto* diag = readback_diag.ptr<GpuPairDiag>();
				int max_gjk = 0, max_epa = 0;
				int total_gjk = 0, total_epa = 0;
				int gjk_pairs = 0;
				for (int i = 0; i != pair_count; ++i)
				{
					auto& d = diag[i];
					if (d.gjk_iters > 0) ++gjk_pairs;
					total_gjk += d.gjk_iters;
					total_epa += d.epa_iters;
					if (d.gjk_iters > max_gjk) max_gjk = d.gjk_iters;
					if (d.epa_iters > max_epa) max_epa = d.epa_iters;

					if (d.gjk_iters + d.epa_iters > 20)
					{
						auto sa = (d.shape_type_a >= 0 && d.shape_type_a <= 4) ? shape_names[d.shape_type_a] : "?";
						auto sb = (d.shape_type_b >= 0 && d.shape_type_b <= 4) ? shape_names[d.shape_type_b] : "?";
						fprintf(f_timing, "  pair[%d] %s vs %s: gjk=%d epa=%d hit=%d\n",
							d.pair_index, sa, sb, d.gjk_iters, d.epa_iters, d.hit);
					}
				}
				fprintf(f_timing, "  summary: gjk_pairs=%d max_gjk=%d max_epa=%d avg_gjk=%.1f avg_epa=%.1f\n",
					gjk_pairs, max_gjk, max_epa,
					gjk_pairs > 0 ? (float)total_gjk / gjk_pairs : 0.0f,
					gjk_pairs > 0 ? (float)total_epa / gjk_pairs : 0.0f);
				fflush(f_timing);
			}
		}
		#endif

		// Copy contacts to output
		out_contacts.resize(contact_count);
		if (contact_count > 0)
			memcpy(out_contacts.data(), readback_contacts.ptr<GpuContact>(), contact_count * sizeof(GpuContact));

		return contact_count;
	}

	// Compile the collision compute shader from embedded resources.
	void GpuCollisionDetector::CompileShader()
	{
		auto device = static_cast<ID3D12Device4*>(m_gpu);
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"COLLIDE_HLSL", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.EntryPoint(L"CSCollisionDetect")
			.ShaderModel(L"cs_6_0")
			#if PR_DBG
			.Define(L"COLLISION_DIAGNOSTICS", L"1")
			#endif
			.Optimise();

		auto bytecode = compiler.Compile();

		// Root signature: constants + 3 SRVs + UAVs (contacts, counters, diag)
		auto sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCollision>(EReg::Params)
			.SRV(EReg::Shapes)
			.SRV(EReg::Pairs)
			.SRV(EReg::Verts)
			.UAV(EReg::Contacts)
			.UAV(EReg::Counters);

		#if PR_DBG
		sig.UAV(EReg::Diag);
		#endif

		m_cs_gjk.m_sig = sig.Create(device, "Physics:CollideSig");
		m_cs_gjk.m_pso = ComputePSO(m_cs_gjk.m_sig.get(), bytecode)
			.Create(device, "Physics:CollidePSO");
	}

	// Create GPU buffers for collision pipeline.
	// Returns true if shape or vertex buffers were reallocated (requiring re-upload).
	bool GpuCollisionDetector::ResizeBuffers(CmdList& cmd_list, int max_shapes, int max_verts, int max_pairs)
	{
		max_shapes = std::max(1, max_shapes);
		max_verts = std::max(1, max_verts);
		max_pairs = std::max(1, max_pairs);

		bool shapes_resized = false;

		// SRV buffers (created in COMMON state, transitioned to SRV on first use)
		if (m_r_shapes == nullptr || max_shapes > m_max_shapes)
		{
			m_r_shapes = m_gpu.CreateResource(ResDesc::Buf<GpuShape>(max_shapes, {}), cmd_list, "Physics:Shapes");
			m_max_shapes = max_shapes;
			shapes_resized = true;
		}
		if (m_r_verts == nullptr || max_verts > m_max_verts)
		{
			m_r_verts = m_gpu.CreateResource(ResDesc::Buf<v4>(max_verts, {}), cmd_list, "Physics:CollisionVerts");
			m_max_verts = max_verts;
			shapes_resized = true;
		}
		if (m_r_pairs == nullptr || max_pairs > m_max_pairs)
		{
			m_r_pairs = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionPair>(max_pairs, {}), cmd_list, "Physics:CollisionPairs");
			m_r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuContact>(max_pairs, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:Contacts");
			#if PR_DBG
			m_r_diag = m_gpu.CreateResource(ResDesc::Buf<GpuPairDiag>(max_pairs, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:PairDiag");
			#endif
			m_max_pairs = max_pairs;
		}
		if (m_r_counters == nullptr)
		{
			m_r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionCounters");
		}
		return shapes_resized;
	}

	// Custom deleter implementation (GpuCollisionDetector is complete here)
	void Deleter<GpuCollisionDetector>::operator()(GpuCollisionDetector* p) const
	{
		delete p;
	}
}
