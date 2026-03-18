//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/collision/gpu_collision_detector.h"
#include "src/collision/gpu_collision_types.h"
#include "src/collision/shape_cache.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Constant buffer layout matching the HLSL cbCollision declaration.
	struct alignas(16) cbCollision
	{
		int g_max_contacts;
		int pad0;
		int pad1;
		int pad2;
	};
	static_assert(sizeof(cbCollision) == 16);

	// Register assignments for the collision root signature.
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Counters = EUAVReg::u0;
		inline static constexpr auto Contacts = EUAVReg::u1;
		inline static constexpr auto DispatchArgs = EUAVReg::u2;
		inline static constexpr auto Pairs = ESRVReg::t0;
		inline static constexpr auto Shapes = ESRVReg::t1;
		inline static constexpr auto Verts = ESRVReg::t2;
		inline static constexpr auto Diag = EUAVReg::u3;
	};

	GpuCollisionDetector::GpuCollisionDetector(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_collide()
		, m_cmd_sig()
		, m_r_shapes()
		, m_r_verts()
		, m_r_contacts()
		, m_r_resolve_dispatch()
		#if PR_COLLISION_DIAGNOSTICS
		, m_r_diag()
		#endif
		, m_max_contacts()
		, m_max_shapes()
		, m_max_verts()
	{
		CompileShaders();

		// Create a command signature for indirect dispatch
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

	// Compile the collision compute shader from embedded resources.
	void GpuCollisionDetector::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/collide.hlsl", L"TEXT"))
			.Includes({ new ResourceIncludeHandler, true })
			.ShaderModel(L"cs_6_0")
			.Define(L"PR_COLLISION_DIAGNOSTICS", L"" PR_STRINGISE(PR_COLLISION_DIAGNOSTICS))
			.Optimise();

		// m_cs_collide
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Contacts)
				.UAV(EReg::DispatchArgs)
				.SRV(EReg::Pairs)
				.SRV(EReg::Shapes)
				.SRV(EReg::Verts)
				#if PR_DBG
				.UAV(EReg::Diag)
				#endif
				;

			auto bytecode = compiler.EntryPoint(L"CSCollide").Compile();

			m_cs_collide.m_sig = sig.Create(m_gpu, "Physics:CollideSig");
			m_cs_collide.m_pso = ComputePSO(m_cs_collide.m_sig.get(), bytecode).Create(m_gpu, "Physics:CollidePSO");
		}

		// m_cs_calc_dispatch
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.UAV(EReg::Counters)
				.UAV(EReg::DispatchArgs);

			auto bytecode = compiler.EntryPoint(L"CSCalcResolveDispatch").Compile();

			m_cs_calc_dispatch.m_sig = sig.Create(m_gpu, "Physics:CalcResolveDispatchSig");
			m_cs_calc_dispatch.m_pso = ComputePSO(m_cs_calc_dispatch.m_sig.get(), bytecode).Create(m_gpu, "Physics:CalcResolveDispatchPSO");
		}
	}

	// Create GPU buffers for collision pipeline.
	// Returns true if shape or vertex buffers were reallocated (requiring re-upload).
	bool GpuCollisionDetector::ResizeBuffers(CmdList& cmd_list, int max_contacts, int max_shapes, int max_verts)
	{
		max_contacts = std::max(1, max_contacts);
		max_shapes = std::max(1, max_shapes);
		max_verts = std::max(1, max_verts);

		bool shapes_resized = false;

		if (m_r_shapes == nullptr || max_shapes > m_max_shapes)
		{
			m_r_shapes = m_gpu.CreateResource(ResDesc::Buf<GpuShape>(max_shapes, {}), cmd_list, "Physics:Shapes");
			m_max_shapes = max_shapes;
			shapes_resized = true;
		}
		if (m_r_verts == nullptr || max_verts > m_max_verts)
		{
			m_r_verts = m_gpu.CreateResource(ResDesc::Buf<v4>(max_verts, {}), cmd_list, "Physics:ShapeVerts");
			m_max_verts = max_verts;
			shapes_resized = true;
		}
		if (m_r_contacts == nullptr || max_contacts > m_max_contacts)
		{
			m_r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:Contacts");
			#if PR_COLLISION_DIAGNOSTICS
			m_r_diag = m_gpu.CreateResource(ResDesc::Buf<GpuPairDiag>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactDiag");
			#endif
			m_max_contacts = max_contacts;
		}
		if (m_r_resolve_dispatch == nullptr)
		{
			m_r_resolve_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ResolveDispatchArgs");
		}
		return shapes_resized;
	}

	// Run collision detection on the GPU.
	void GpuCollisionDetector::DetectCollisions(GpuJob& job, int max_contacts, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> pairs, D3DPtr<ID3D12Resource> counters, ShapeCache const& shape_cache)
	{
		// Notes:
		//  - Assumes that the counters.contact_count has been zeroed already by the broad phase shader.
		auto shape_count = static_cast<int>(shape_cache.m_shapes.size());
		auto vert_count = static_cast<int>(shape_cache.m_verts.size());
		auto shapes_upload_needed = shape_cache.m_changed;

		// If ResizeBuffers reallocated the shape/vert buffers, we must re-upload
		// regardless of the caller's dirty flag (the new buffers contain garbage).
		shapes_upload_needed |= ResizeBuffers(job.m_cmd_list, max_contacts, shape_count, vert_count);
		if (shapes_upload_needed)
		{
			// Upload shapes and vertex buffers
			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto shape_upload = job.m_upload.Alloc<GpuShape>(shape_count);
			memcpy(shape_upload.ptr<GpuShape>(), shape_cache.m_shapes.data(), shape_count * sizeof(GpuShape));
			job.m_cmd_list.CopyBufferRegion(m_r_shapes.get(), 0, shape_upload);

			// Upload vertices buffer (may be empty if no polytopes/triangles), it can be uninitialised in this case.
			auto vert_upload = job.m_upload.Alloc<v4>(std::max(1, vert_count));
			memcpy(vert_upload.ptr<v4>(), shape_cache.m_verts.data(), vert_count * sizeof(v4));
			job.m_cmd_list.CopyBufferRegion(m_r_verts.get(), 0, vert_upload);

			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Switch states for resources
		{
			job.m_barriers.Transition(dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(pairs.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_resolve_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#endif
			job.m_barriers.Commit();
		}

		// Dispatch the collision compute shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_collide.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_collide.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbCollision{ .g_max_contacts = max_contacts });
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_shapes->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_verts->GetGPUVirtualAddress());
			#if PR_COLLISION_DIAGNOSTICS
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diag->GetGPUVirtualAddress());
			#endif

			// Dispatch with the count provided by the broad phase shader
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			job.m_barriers.UAV(counters.get());
			job.m_barriers.UAV(m_r_contacts.get());
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.UAV(m_r_diag.get());
			#endif
			job.m_barriers.Commit();
		}

		// Dispatch the calculate dispatch size shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_calc_dispatch.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_calc_dispatch.m_sig.get());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_resolve_dispatch->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_resolve_dispatch.get());
			job.m_barriers.Commit();
		}
	}

	// Read back the results of the collision detection.
	// This is a debug synchronisation point — it flushes all queued GPU work,
	// then makes the narrow-phase output available on the CPU for inspection.
	std::tuple<std::span<GpuResolveContact>, std::span<GpuPairDiag>> GpuCollisionDetector::Readback(GpuJob& job, D3DPtr<ID3D12Resource> r_counters, std::span<GpuResolveContact> out_contacts, std::span<GpuPairDiag> out_diag)
	{
		// This is a debug synchronisation point — it flushes all queued GPU work,
		// then makes the broadphase output available on the CPU for inspection.
		GpuReadbackBuffer::Allocation readback_counters;
		GpuReadbackBuffer::Allocation readback_contacts;
		#if PR_COLLISION_DIAGNOSTICS
		GpuReadbackBuffer::Allocation readback_diag;
		#endif

		// Readback contacts and counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			#endif
			job.m_barriers.Commit();

			readback_contacts = job.m_readback.Alloc<GpuResolveContact>(static_cast<int>(out_contacts.size()));
			job.m_cmd_list.CopyBufferRegion(readback_contacts, m_r_contacts.get(), 0);

			readback_counters = job.m_readback.Alloc<GpuCollisionCounters>(1);
			job.m_cmd_list.CopyBufferRegion(readback_counters, r_counters.get(), 0);

			#if PR_COLLISION_DIAGNOSTICS
			readback_diag = job.m_readback.Alloc<GpuPairDiag>(static_cast<int>(out_diag.size()));
			job.m_cmd_list.CopyBufferRegion(readback_diag, m_r_diag.get(), 0);
			#endif

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#if PR_COLLISION_DIAGNOSTICS
			job.m_barriers.Transition(m_r_diag.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#endif
			job.m_barriers.Commit();
		}

		// Execute and wait for GPU completion
		job.Run();

		// Read back results
		auto& counters = *readback_counters.ptr<GpuCollisionCounters>();
		auto contact_count = std::min(static_cast<int>(counters.contact_count), static_cast<int>(out_contacts.size()));
		std::memcpy(out_contacts.data(), readback_contacts.ptr<GpuResolveContact>(), contact_count * sizeof(GpuResolveContact));

		#if PR_COLLISION_DIAGNOSTICS
		auto diag_count = std::min(static_cast<int>(counters.contact_count), static_cast<int>(out_diag.size()));
		std::memcpy(out_diag.data(), readback_diag.ptr<GpuPairDiag>(), diag_count * sizeof(GpuPairDiag));
		#endif

		return {
			out_contacts.subspan(0, contact_count),
			#if PR_COLLISION_DIAGNOSTICS
			out_diag.subspan(0, diag_count),
			#else
			out_diag.subspan(0, 0),
			#endif
		};
	}

	// Run collision detection on the GPU with CPU-side data.
	// This overload uploads pairs from CPU, runs the GPU collision detection, reads back contacts.
	// Used by unit tests and CPU-fallback paths. Returns the number of contacts found.
	std::tuple<std::span<GpuResolveContact>, std::span<GpuPairDiag>> GpuCollisionDetector::DetectCollisions(GpuJob& job, std::span<GpuCollisionPair const> pairs, ShapeCache const& shape_cache, std::span<GpuResolveContact> out_contacts, std::span<GpuPairDiag> out_diag)
	{
		auto pair_count = static_cast<int>(pairs.size());
		if (pair_count == 0)
			return {};

		// Create temporary GPU resources for the CPU-provided data
		auto r_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempDispatchArgs");
		auto r_pairs = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionPair>(pair_count, {}), job.m_cmd_list, "Physics:TempPairs");
		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempCounters");

		// Upload pairs to GPU
		{
			job.m_barriers.Transition(r_pairs.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionPair>(pair_count);
			memcpy(upload.ptr<GpuCollisionPair>(), pairs.data(), pair_count * sizeof(GpuCollisionPair));
			job.m_cmd_list.CopyBufferRegion(r_pairs.get(), 0, upload);

			job.m_barriers.Transition(r_pairs.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload dispatch args (pair_count threads, grouped into thread groups)
		{
			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<D3D12_DISPATCH_ARGUMENTS>(1);
			auto* args = upload.ptr<D3D12_DISPATCH_ARGUMENTS>();
			args->ThreadGroupCountX = static_cast<UINT>((pair_count + CollideThreadCount - 1) / CollideThreadCount);
			args->ThreadGroupCountY = 1;
			args->ThreadGroupCountZ = 1;
			job.m_cmd_list.CopyBufferRegion(r_dispatch.get(), 0, upload);

			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Zero the counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			auto* counters = upload.ptr<GpuCollisionCounters>();
			*counters = {};
			counters->pair_count = pair_count;
			job.m_cmd_list.CopyBufferRegion(r_counters.get(), 0, upload);

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Run the GPU collision detection
		DetectCollisions(job, static_cast<int>(out_contacts.size()), r_dispatch, r_pairs, r_counters, shape_cache);

		// Readback the data from the GPU
		return Readback(job, r_counters, out_contacts, out_diag);
	}

	// Custom deleter implementation (GpuCollisionDetector is complete here)
	void Deleter<GpuCollisionDetector>::operator()(GpuCollisionDetector* p) const
	{
		delete p;
	}
}

#if 0 

	void Readback()
	{
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
#endif
