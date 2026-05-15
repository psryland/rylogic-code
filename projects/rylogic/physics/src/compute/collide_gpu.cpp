//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/physics_types.h"
#include "src/collision/shape_cache.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	// Constant buffer layout matching the HLSL cbCollision declaration.
	struct alignas(16) cbCollision
	{
		int g_max_contacts;
		int g_max_pair_count;
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
		inline static constexpr auto BinCounts = EUAVReg::u3;
		inline static constexpr auto BinPairIndices = EUAVReg::u4;
		inline static constexpr auto BinDispatchArgs = EUAVReg::u5;
		inline static constexpr auto Pairs = ESRVReg::t0;
		inline static constexpr auto Shapes = ESRVReg::t1;
		inline static constexpr auto Verts = ESRVReg::t2;
		inline static constexpr auto Faces = ESRVReg::t3;
		inline static constexpr auto Edges = ESRVReg::t4;
	};

	GpuCollisionDetector::GpuCollisionDetector(Gpu& gpu, EngineConfig const& config, IShaderCache* shader_cache)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_clear_bins()
		, m_cs_bin_pairs()
		, m_cs_build_bin_dispatch()
		, m_cs_collide_bins()
		, m_cs_calc_dispatch()
		, m_cmd_sig()
		, m_r_shapes()
		, m_r_verts()
		, m_r_faces()
		, m_r_edges()
		, m_r_contacts()
		, m_r_resolve_dispatch()
		, m_r_bin_counts()
		, m_r_bin_pair_indices()
		, m_r_bin_dispatch()
		, m_max_contacts()
		, m_max_pairs()
		, m_max_shapes()
		, m_max_verts()
		, m_max_faces()
		, m_max_edges()
	{
		// Compile the collision compute shader from embedded resources.
		auto resolver = shader_cache::ResourceSourceResolver{};
		auto compiler = ShaderCompiler{}
			.Cache(shader_cache)
			.Source("src/compute/collide.hlsl", resolver)
			.HlslVersion(EHlslVersion::DxcDefault)
			.ShaderModel(L"cs_6_0")
			.Optimise();

		auto make_setup_sig = [&]()
		{
			return RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Contacts)
				.UAV(EReg::DispatchArgs)
				.UAV(EReg::BinCounts)
				.UAV(EReg::BinPairIndices)
				.UAV(EReg::BinDispatchArgs)
				.SRV(EReg::Pairs)
				.SRV(EReg::Shapes)
				.SRV(EReg::Verts)
				.SRV(EReg::Faces)
				.SRV(EReg::Edges)
				;
		};
		auto make_collide_sig = [&]()
		{
			return RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Contacts)
				.UAV(EReg::BinCounts)
				.UAV(EReg::BinPairIndices)
				.SRV(EReg::Pairs)
				.SRV(EReg::Shapes)
				.SRV(EReg::Verts)
				.SRV(EReg::Faces)
				.SRV(EReg::Edges)
				;
		};

		auto compile_setup = [&](ComputeStep& step, wchar_t const* entry_point, char const* sig_name, char const* pso_name)
		{
			auto sig = make_setup_sig();
			auto bytecode = compiler.Optimise().EntryPoint(entry_point).Compile();

			step.m_sig = sig.Create(m_gpu, sig_name);
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};
		auto compile_collide = [&](ComputeStep& step, wchar_t const* entry_point, char const* sig_name, char const* pso_name)
		{
			auto sig = make_collide_sig();
			auto bytecode = compiler.Optimise().EntryPoint(entry_point).Compile();

			step.m_sig = sig.Create(m_gpu, sig_name);
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};

		compile_setup(m_cs_clear_bins, L"CSClearCollisionBins", "Physics:ClearCollisionBinsSig", "Physics:ClearCollisionBinsPSO");
		compile_setup(m_cs_bin_pairs, L"CSBinCollisionPairs", "Physics:BinCollisionPairsSig", "Physics:BinCollisionPairsPSO");
		compile_setup(m_cs_build_bin_dispatch, L"CSBuildCollisionBinDispatch", "Physics:BuildCollisionBinDispatchSig", "Physics:BuildCollisionBinDispatchPSO");

		struct BinShader
		{
			ComputeStep* step;
			wchar_t const* entry_point;
			char const* sig_name;
			char const* pso_name;
		};
		std::array<BinShader, COLLISION_BIN_COUNT> bin_shaders = {{
			{&m_cs_collide_bins[COLLISION_BIN_SPHERE_VS_SPHERE],     L"CSCollideSphereVsSphere",     "Physics:CollideSphereVsSphereSig",     "Physics:CollideSphereVsSpherePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_BOX_VS_SPHERE],        L"CSCollideBoxVsSphere",        "Physics:CollideBoxVsSphereSig",        "Physics:CollideBoxVsSpherePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_BOX_VS_BOX],           L"CSCollideBoxVsBox",           "Physics:CollideBoxVsBoxSig",           "Physics:CollideBoxVsBoxPSO"},
			{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_SPHERE],       L"CSCollideLineVsSphere",       "Physics:CollideLineVsSphereSig",       "Physics:CollideLineVsSpherePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_BOX],          L"CSCollideLineVsBox",          "Physics:CollideLineVsBoxSig",          "Physics:CollideLineVsBoxPSO"},
			{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_LINE],         L"CSCollideLineVsLine",         "Physics:CollideLineVsLineSig",         "Physics:CollideLineVsLinePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_SPHERE],   L"CSCollideTriangleVsSphere",   "Physics:CollideTriangleVsSphereSig",   "Physics:CollideTriangleVsSpherePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_BOX],      L"CSCollideTriangleVsBox",      "Physics:CollideTriangleVsBoxSig",      "Physics:CollideTriangleVsBoxPSO"},
			{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_LINE],     L"CSCollideTriangleVsLine",     "Physics:CollideTriangleVsLineSig",     "Physics:CollideTriangleVsLinePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_TRIANGLE], L"CSCollideTriangleVsTriangle", "Physics:CollideTriangleVsTriangleSig", "Physics:CollideTriangleVsTrianglePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_SPHERE],   L"CSCollidePolytopeVsSphere",   "Physics:CollidePolytopeVsSphereSig",   "Physics:CollidePolytopeVsSpherePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_BOX],      L"CSCollidePolytopeVsBox",      "Physics:CollidePolytopeVsBoxSig",      "Physics:CollidePolytopeVsBoxPSO"},
			{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_LINE],     L"CSCollidePolytopeVsLine",     "Physics:CollidePolytopeVsLineSig",     "Physics:CollidePolytopeVsLinePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_TRIANGLE], L"CSCollidePolytopeVsTriangle", "Physics:CollidePolytopeVsTriangleSig", "Physics:CollidePolytopeVsTrianglePSO"},
			{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_POLYTOPE], L"CSCollidePolytopeVsPolytope", "Physics:CollidePolytopeVsPolytopeSig", "Physics:CollidePolytopeVsPolytopePSO"},
		}};
		for (auto const& bin_shader : bin_shaders)
			compile_collide(*bin_shader.step, bin_shader.entry_point, bin_shader.sig_name, bin_shader.pso_name);

		// m_cs_calc_dispatch
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::DispatchArgs);

			auto bytecode = compiler.Optimise().EntryPoint(L"CSCalcResolveDispatch").Compile();

			m_cs_calc_dispatch.m_sig = sig.Create(m_gpu, "Physics:CalcResolveDispatchSig");
			m_cs_calc_dispatch.m_pso = ComputePSO(m_cs_calc_dispatch.m_sig.get(), bytecode).Create(m_gpu, "Physics:CalcResolveDispatchPSO");
		}

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

	// Create GPU buffers for collision pipeline.
	// Returns true if shape or vertex buffers were reallocated (requiring re-upload).
	bool GpuCollisionDetector::ResizeBuffers(CmdList& cmd_list, int max_contacts, int max_pairs, int max_shapes, int max_verts, int max_faces, int max_edges)
	{
		max_contacts = std::max(1, max_contacts);
		max_pairs = std::max(1, max_pairs);
		max_shapes = std::max(1, max_shapes);
		max_verts = std::max(1, max_verts);
		max_faces = std::max(1, max_faces);
		max_edges = std::max(1, max_edges);

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
		if (m_r_faces == nullptr || max_faces > m_max_faces)
		{
			m_r_faces = m_gpu.CreateResource(ResDesc::Buf<GpuPolytopeFace>(max_faces, {}), cmd_list, "Physics:ShapeFaces");
			m_max_faces = max_faces;
			shapes_resized = true;
		}
		if (m_r_edges == nullptr || max_edges > m_max_edges)
		{
			m_r_edges = m_gpu.CreateResource(ResDesc::Buf<GpuPolytopeEdge>(max_edges, {}), cmd_list, "Physics:ShapeEdges");
			m_max_edges = max_edges;
			shapes_resized = true;
		}
		if (m_r_contacts == nullptr || max_contacts > m_max_contacts)
		{
			m_r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:Contacts");
			m_max_contacts = max_contacts;
		}
		if (m_r_bin_pair_indices == nullptr || max_pairs > m_max_pairs)
		{
			m_r_bin_pair_indices = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(COLLISION_BIN_COUNT * max_pairs, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionBinPairIndices");
			m_max_pairs = max_pairs;
		}
		if (m_r_resolve_dispatch == nullptr)
		{
			m_r_resolve_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ResolveDispatchArgs");
		}
		if (m_r_bin_counts == nullptr)
		{
			m_r_bin_counts = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(COLLISION_BIN_COUNT, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionBinCounts");
		}
		if (m_r_bin_dispatch == nullptr)
		{
			m_r_bin_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(COLLISION_BIN_COUNT, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CollisionBinDispatchArgs");
		}
		return shapes_resized;
	}

	// Run collision detection on the GPU.
	void GpuCollisionDetector::DetectCollisions(GpuJob& job, int max_contacts, int max_pairs, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> pairs, D3DPtr<ID3D12Resource> counters, ShapeCache const& shape_cache)
	{
		DetectCollisions(job, max_contacts, max_pairs, dispatch, pairs, counters, nullptr, nullptr, shape_cache);
	}
	void GpuCollisionDetector::DetectCollisions(GpuJob& job, int max_contacts, int max_pairs, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> pairs, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> resolve_dispatch, ShapeCache const& shape_cache)
	{
		// Notes:
		//  - Assumes that the counters.contact_count has been zeroed already by the broad phase shader.
		auto shape_count = static_cast<int>(shape_cache.m_shapes.size());
		auto vert_count = static_cast<int>(shape_cache.m_verts.size());
		auto face_count = static_cast<int>(shape_cache.m_faces.size());
		auto edge_count = static_cast<int>(shape_cache.m_edges.size());
		auto shapes_upload_needed = shape_cache.m_changed;

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf245bc, "Physics::Collide");

		// If ResizeBuffers reallocated the shape/vert buffers, we must re-upload
		// regardless of the caller's dirty flag (the new buffers contain garbage).
		shapes_upload_needed |= ResizeBuffers(job.m_cmd_list, max_contacts, max_pairs, shape_count, vert_count, face_count, edge_count);
		if (contacts == nullptr)
			contacts = m_r_contacts;
		if (resolve_dispatch == nullptr)
			resolve_dispatch = m_r_resolve_dispatch;

		if (shapes_upload_needed)
		{
			// Upload shapes and vertex buffers
			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(m_r_faces.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(m_r_edges.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto shape_upload = job.m_upload.Alloc<GpuShape>(shape_count);
			memcpy(shape_upload.ptr<GpuShape>(), shape_cache.m_shapes.data(), shape_count * sizeof(GpuShape));
			job.m_cmd_list.CopyBufferRegion(m_r_shapes.get(), 0, shape_upload);

			// Upload vertices buffer (may be empty if no polytopes/triangles), it can be uninitialised in this case.
			auto vert_upload = job.m_upload.Alloc<v4>(std::max(1, vert_count));
			memcpy(vert_upload.ptr<v4>(), shape_cache.m_verts.data(), vert_count * sizeof(v4));
			job.m_cmd_list.CopyBufferRegion(m_r_verts.get(), 0, vert_upload);

			auto face_upload = job.m_upload.Alloc<GpuPolytopeFace>(std::max(1, face_count));
			memcpy(face_upload.ptr<GpuPolytopeFace>(), shape_cache.m_faces.data(), face_count * sizeof(GpuPolytopeFace));
			job.m_cmd_list.CopyBufferRegion(m_r_faces.get(), 0, face_upload);

			auto edge_upload = job.m_upload.Alloc<GpuPolytopeEdge>(std::max(1, edge_count));
			memcpy(edge_upload.ptr<GpuPolytopeEdge>(), shape_cache.m_edges.data(), edge_count * sizeof(GpuPolytopeEdge));
			job.m_cmd_list.CopyBufferRegion(m_r_edges.get(), 0, edge_upload);

			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_faces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_edges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Switch states for resources
		{
			job.m_barriers.Transition(dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(pairs.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_faces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_edges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(resolve_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_bin_counts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_bin_pair_indices.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_bin_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		auto cb_collision = cbCollision{
			.g_max_contacts = max_contacts,
			.g_max_pair_count = max_pairs,
		};
		auto bind_setup = [&](ComputeStep& step)
		{
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_collision);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resolve_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bin_counts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bin_pair_indices->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bin_dispatch->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_shapes->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_verts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_faces->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_edges->GetGPUVirtualAddress());
		};
		auto bind_collide = [&](ComputeStep& step)
		{
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_collision);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bin_counts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bin_pair_indices->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(pairs->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_shapes->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_verts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_faces->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_edges->GetGPUVirtualAddress());
		};

		// Clear all exact pair bins.
		{
			bind_setup(m_cs_clear_bins);
			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_bin_counts.get());
			job.m_barriers.UAV(m_r_bin_dispatch.get());
			job.m_barriers.Commit();
		}

		// Classify broadphase pairs into exact shape-pair bins.
		{
			bind_setup(m_cs_bin_pairs);
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			job.m_barriers.UAV(m_r_bin_counts.get());
			job.m_barriers.UAV(m_r_bin_pair_indices.get());
			job.m_barriers.Commit();
		}

		// Build one indirect dispatch record per exact pair bin.
		{
			bind_setup(m_cs_build_bin_dispatch);
			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_bin_dispatch.get());
			job.m_barriers.Transition(m_r_bin_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Commit();
		}

		// Dispatch the exact pair-type collision kernels. Empty bins have a zero-sized dispatch.
		for (int bin = 0; bin != COLLISION_BIN_COUNT; ++bin)
		{
			bind_collide(m_cs_collide_bins[bin]);
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, m_r_bin_dispatch.get(), static_cast<UINT64>(bin) * sizeof(D3D12_DISPATCH_ARGUMENTS));

			job.m_barriers.UAV(counters.get());
			job.m_barriers.UAV(contacts.get());
			job.m_barriers.Commit();
		}

		// Dispatch the calculate dispatch size shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_calc_dispatch.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_calc_dispatch.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_collision);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resolve_dispatch->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(resolve_dispatch.get());
			job.m_barriers.Commit();
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Run collision detection on the GPU with CPU-side data.
	// This overload uploads pairs from CPU, runs the GPU collision detection, reads back contacts.
	// Used by unit tests and CPU-fallback paths. Returns the contacts found.
	std::span<GpuResolveContact> GpuCollisionDetector::DetectCollisions(GpuJob& job, std::span<GpuCollisionPair const> pairs, ShapeCache const& shape_cache, std::span<GpuResolveContact> out_contacts)
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
		DetectCollisions(job, static_cast<int>(out_contacts.size()), pair_count, r_dispatch, r_pairs, r_counters, shape_cache);

		// Readback the data from the GPU
		return Readback(job, r_counters, out_contacts);
	}

	// Read back the results of the collision detection.
	// This is a debug synchronisation point — it flushes all queued GPU work,
	// then makes the narrow-phase output available on the CPU for inspection.
	std::span<GpuResolveContact> GpuCollisionDetector::Readback(GpuJob& job, D3DPtr<ID3D12Resource> r_counters, std::span<GpuResolveContact> out_contacts)
	{
		// This is a debug synchronisation point — it flushes all queued GPU work,
		// then makes the broadphase output available on the CPU for inspection.
		GpuReadbackBuffer::Allocation readback_counters;
		GpuReadbackBuffer::Allocation readback_contacts;

		// Readback contacts and counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_contacts = job.m_readback.Alloc<GpuResolveContact>(static_cast<int>(out_contacts.size()));
			job.m_cmd_list.CopyBufferRegion(readback_contacts, m_r_contacts.get(), 0);

			readback_counters = job.m_readback.Alloc<GpuCollisionCounters>(1);
			job.m_cmd_list.CopyBufferRegion(readback_counters, r_counters.get(), 0);

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Execute and wait for GPU completion
		job.Run();

		// Read back results
		auto& counters = *readback_counters.ptr<GpuCollisionCounters>();
		auto max_contacts = static_cast<int>(out_contacts.size());
		if (counters.contact_count > max_contacts)
		{
			throw std::runtime_error(std::format(
				"GPU collision contact readback overflow: {} contacts generated for {} output slots.",
				counters.contact_count,
				max_contacts));
		}
		auto contact_count = std::min(counters.contact_count, max_contacts);
		std::memcpy(out_contacts.data(), readback_contacts.ptr<GpuResolveContact>(), contact_count * sizeof(GpuResolveContact));

		return out_contacts.subspan(0, contact_count);
	}

	// Custom deleter implementation (GpuCollisionDetector is complete here)
	void Deleter<GpuCollisionDetector>::operator()(GpuCollisionDetector* p) const
	{
		delete p;
	}
}
