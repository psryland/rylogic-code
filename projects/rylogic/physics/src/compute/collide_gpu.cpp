//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/physics_types.h"
#include "src/compute/shader_code.h"
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

	GpuCollisionDetector::GpuCollisionDetector(Gpu& gpu, EngineConfig const& config)
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
		auto create_step = [&](ComputeStep& step, RootSig& sig, shader_code::ByteCode const& bytecode, char const* sig_name, char const* pso_name)
		{
			step.m_sig = sig.Create(m_gpu, sig_name);
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};

		// Setup steps
		{
			auto setup_sig = RootSig(ERootSigFlags::ComputeOnly)
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
				.SRV(EReg::Edges);

			create_step(m_cs_clear_bins, setup_sig, shader_code::clear_collision_bins, "Physics:ClearCollisionBinsSig", "Physics:ClearCollisionBinsPSO");
			create_step(m_cs_bin_pairs, setup_sig, shader_code::bin_collision_pairs, "Physics:BinCollisionPairsSig", "Physics:BinCollisionPairsPSO");
			create_step(m_cs_build_bin_dispatch, setup_sig, shader_code::build_collision_bin_dispatch, "Physics:BuildCollisionBinDispatchSig", "Physics:BuildCollisionBinDispatchPSO");
		}

		// Collision detection steps
		{
			struct CollideStepDesc
			{
				ComputeStep* step;
				shader_code::ByteCode const* bytecode;
				char const* sig_name;
				char const* pso_name;
			};
			std::array<CollideStepDesc, COLLISION_BIN_COUNT> collide_steps = {
				{
					{&m_cs_collide_bins[COLLISION_BIN_SPHERE_VS_SPHERE],     &shader_code::collide_sphere_vs_sphere,     "Physics:CollideSphereVsSphereSig",     "Physics:CollideSphereVsSpherePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_BOX_VS_SPHERE],        &shader_code::collide_box_vs_sphere,        "Physics:CollideBoxVsSphereSig",        "Physics:CollideBoxVsSpherePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_BOX_VS_BOX],           &shader_code::collide_box_vs_box,           "Physics:CollideBoxVsBoxSig",           "Physics:CollideBoxVsBoxPSO"},
					{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_SPHERE],       &shader_code::collide_line_vs_sphere,       "Physics:CollideLineVsSphereSig",       "Physics:CollideLineVsSpherePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_BOX],          &shader_code::collide_line_vs_box,          "Physics:CollideLineVsBoxSig",          "Physics:CollideLineVsBoxPSO"},
					{&m_cs_collide_bins[COLLISION_BIN_LINE_VS_LINE],         &shader_code::collide_line_vs_line,         "Physics:CollideLineVsLineSig",         "Physics:CollideLineVsLinePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_SPHERE],   &shader_code::collide_triangle_vs_sphere,   "Physics:CollideTriangleVsSphereSig",   "Physics:CollideTriangleVsSpherePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_BOX],      &shader_code::collide_triangle_vs_box,      "Physics:CollideTriangleVsBoxSig",      "Physics:CollideTriangleVsBoxPSO"},
					{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_LINE],     &shader_code::collide_triangle_vs_line,     "Physics:CollideTriangleVsLineSig",     "Physics:CollideTriangleVsLinePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_TRIANGLE_VS_TRIANGLE], &shader_code::collide_triangle_vs_triangle, "Physics:CollideTriangleVsTriangleSig", "Physics:CollideTriangleVsTrianglePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_SPHERE],   &shader_code::collide_polytope_vs_sphere,   "Physics:CollidePolytopeVsSphereSig",   "Physics:CollidePolytopeVsSpherePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_BOX],      &shader_code::collide_polytope_vs_box,      "Physics:CollidePolytopeVsBoxSig",      "Physics:CollidePolytopeVsBoxPSO"},
					{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_LINE],     &shader_code::collide_polytope_vs_line,     "Physics:CollidePolytopeVsLineSig",     "Physics:CollidePolytopeVsLinePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_TRIANGLE], &shader_code::collide_polytope_vs_triangle, "Physics:CollidePolytopeVsTriangleSig", "Physics:CollidePolytopeVsTrianglePSO"},
					{&m_cs_collide_bins[COLLISION_BIN_POLYTOPE_VS_POLYTOPE], &shader_code::collide_polytope_vs_polytope, "Physics:CollidePolytopeVsPolytopeSig", "Physics:CollidePolytopeVsPolytopePSO"},
				}
			};
			auto collide_sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Contacts)
				.UAV(EReg::BinCounts)
				.UAV(EReg::BinPairIndices)
				.SRV(EReg::Pairs)
				.SRV(EReg::Shapes)
				.SRV(EReg::Verts)
				.SRV(EReg::Faces)
				.SRV(EReg::Edges);

			for (auto const& step_desc : collide_steps)
				create_step(*step_desc.step, collide_sig, *step_desc.bytecode, step_desc.sig_name, step_desc.pso_name);
		}

		// m_cs_calc_dispatch
		{
			auto dispatch_sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbCollision>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::DispatchArgs);

			m_cs_calc_dispatch.m_sig = dispatch_sig.Create(m_gpu, "Physics:CalcResolveDispatchSig");
			m_cs_calc_dispatch.m_pso = ComputePSO(m_cs_calc_dispatch.m_sig.get(), shader_code::calc_resolve_dispatch).Create(m_gpu, "Physics:CalcResolveDispatchPSO");
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

	// Make the cached shape data resident on the GPU.
	// The broadphase and the narrowphase both read this buffer, so it is uploaded once per frame before
	// either runs and only when the cache has actually changed. Clearing the cache's dirty flag here is
	// what keeps steady-state frames free of shape traffic.
	void GpuCollisionDetector::UploadShapes(GpuJob& job, ShapeCache& shape_cache)
	{
		auto shape_count = static_cast<int>(shape_cache.m_shapes.size());
		auto vert_count = static_cast<int>(shape_cache.m_verts.size());
		auto face_count = static_cast<int>(shape_cache.m_faces.size());
		auto edge_count = static_cast<int>(shape_cache.m_edges.size());

		// A reallocated buffer contains garbage, so it must be re-uploaded regardless of the dirty flag.
		auto upload_needed = shape_cache.m_changed;
		upload_needed |= ResizeBuffers(job.m_cmd_list, m_max_contacts, m_max_pairs, shape_count, vert_count, face_count, edge_count);
		if (!upload_needed)
			return;

		// Upload shapes and vertex buffers
		job.m_barriers.Transition(m_r_shapes.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_verts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_faces.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Transition(m_r_edges.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();

		auto shape_upload = job.m_upload.Alloc<GpuShape>(std::max(1, shape_count));
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

		shape_cache.m_changed = false;
	}

	// Run collision detection on the GPU.
	void GpuCollisionDetector::DetectCollisions(GpuJob& job, int max_contacts, int max_pairs, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> pairs, D3DPtr<ID3D12Resource> counters, ShapeCache& shape_cache)
	{
		DetectCollisions(job, max_contacts, max_pairs, dispatch, pairs, counters, nullptr, nullptr, shape_cache);
	}
	void GpuCollisionDetector::DetectCollisions(GpuJob& job, int max_contacts, int max_pairs, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> pairs, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> resolve_dispatch, ShapeCache& shape_cache)
	{
		// Notes:
		//  - Assumes that the counters.contact_count has been zeroed already by the broad phase shader.
		auto shape_count = static_cast<int>(shape_cache.m_shapes.size());
		auto vert_count = static_cast<int>(shape_cache.m_verts.size());
		auto face_count = static_cast<int>(shape_cache.m_faces.size());
		auto edge_count = static_cast<int>(shape_cache.m_edges.size());

		pix::BeginEvent(job.m_cmd_list.get(), 0xFFf245bc, "Physics::Collide");

		// Make the shape data resident. The broadphase reads the same buffer to expand compound bodies,
		// so this is typically already done for the frame and costs nothing here.
		UploadShapes(job, shape_cache);

		// Size the contact and bin buffers. The shape buffers cannot move here because UploadShapes has
		// already sized them for the same shape counts.
		ResizeBuffers(job.m_cmd_list, max_contacts, max_pairs, shape_count, vert_count, face_count, edge_count);
		if (contacts == nullptr)
			contacts = m_r_contacts;
		if (resolve_dispatch == nullptr)
			resolve_dispatch = m_r_resolve_dispatch;

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
	std::span<GpuResolveContact> GpuCollisionDetector::DetectCollisions(GpuJob& job, std::span<GpuCollisionPair const> pairs, ShapeCache& shape_cache, std::span<GpuResolveContact> out_contacts)
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
