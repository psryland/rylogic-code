//*********************************************
// Physics Engine — GPU Collision Resolution
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "src/compute/resolve_gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Constant buffer layout matching the HLSL cbResolve declaration.
	struct alignas(16) cbResolve
	{
		int g_colour;       // current colour batch being processed (for CSResolve)
		int g_max_contacts; // The capacity of the contacts buffer
		float g_dt;         // timestep in seconds (for CSGraphColouring collision time estimation)
		int pad0;
	};
	static_assert(sizeof(cbResolve) == 16);

	// Register assignments for the resolve root signature
	struct EReg
	{
		inline static constexpr auto Params         = ECBufReg::b0;
		inline static constexpr auto Counters       = ESRVReg::t0;
		inline static constexpr auto Materials      = ESRVReg::t1;
		inline static constexpr auto Bodies         = EUAVReg::u0;
		inline static constexpr auto Colours        = EUAVReg::u1;
		inline static constexpr auto Contacts       = EUAVReg::u2;
		inline static constexpr auto ContactTimes   = EUAVReg::u3;
		inline static constexpr auto ContactOrder   = EUAVReg::u4;
	};

	GpuResolver::GpuResolver(Gpu& gpu)
		: m_gpu(gpu)
		, m_contact_sorter(gpu.m_gpu)
		, m_cs_compute_times()
		, m_cs_assign_colours()
		, m_cs_resolve()
		, m_cmd_sig()
		, m_r_materials()
		, m_r_colours()
		, m_r_contact_times()
		, m_r_contact_order()
		, m_max_materials()
		, m_max_contacts()
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

	// Compile the resolve compute shader from embedded resources.
	void GpuResolver::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/resolve.hlsl", L"TEXT"))
			.Includes({new ResourceIncludeHandler, true})
			.Define(L"PR_COLLISION_DIAGNOSTICS", L"" PR_STRINGISE(PR_COLLISION_DIAGNOSTICS))
			.ShaderModel(L"cs_6_0")
			.Optimise();

		// m_cs_compute_times: parallel, one thread per contact — writes collision_time + zeroes body colour_used
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactTimes)
				.UAV(EReg::ContactOrder)
				;

			auto bytecode = compiler.EntryPoint(L"CSComputeCollisionTimes").Compile();

			m_cs_compute_times.m_sig = sig.Create(m_gpu, "Physics:ComputeTimesSig");
			m_cs_compute_times.m_pso = ComputePSO(m_cs_compute_times.m_sig.get(), bytecode).Create(m_gpu, "Physics:ComputeTimesPSO");
		}

		// m_cs_assign_colours: serial, walks sorted contacts + assigns colours
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::Colours)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder);

			auto bytecode = compiler.EntryPoint(L"CSAssignColours").Compile();

			m_cs_assign_colours.m_sig = sig.Create(m_gpu, "Physics:AssignColoursSig");
			m_cs_assign_colours.m_pso = ComputePSO(m_cs_assign_colours.m_sig.get(), bytecode).Create(m_gpu, "Physics:AssignColoursPSO");
		}

		// m_cs_resolve
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.SRV(EReg::Materials)
				.UAV(EReg::Bodies)
				.UAV(EReg::Colours)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder);

			auto bytecode = compiler.EntryPoint(L"CSResolve").Compile();

			m_cs_resolve.m_sig = sig.Create(m_gpu, "Physics:ResolveSig");
			m_cs_resolve.m_pso = ComputePSO(m_cs_resolve.m_sig.get(), bytecode).Create(m_gpu, "Physics:ResolvePSO");
		}
	}

	// Create or grow GPU buffers for contacts and colour assignments.
	void GpuResolver::ResizeBuffers(CmdList& cmd_list, int max_contacts, int max_materials)
	{
		max_contacts = std::max(1, max_contacts);
		max_materials = std::max(1, max_materials);

		if (m_r_materials == nullptr || max_materials > m_max_materials)
		{
			m_r_materials = m_gpu.CreateResource(ResDesc::Buf<GpuMaterial>(max_materials, {}), cmd_list, "Physics:Materials");
			m_max_materials = max_materials;
		}
		if (m_r_colours == nullptr || m_max_contacts < max_contacts)
		{
			m_r_colours = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ResolveColours");
			m_r_contact_times = m_gpu.CreateResource(ResDesc::Buf<float>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactTimes");
			m_r_contact_order = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactOrder");
			m_max_contacts = max_contacts;
		}
	}

	// Resolve collisions on the GPU using graph-coloured batches.
	void GpuResolver::Resolve(GpuJob& job, float dt, int max_contacts, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies, std::span<GpuMaterial const> materials)
	{
		auto material_count = static_cast<int>(materials.size());

		ResizeBuffers(job.m_cmd_list, max_contacts, material_count);

		// Upload materials (small buffer, upload every frame for simplicity)
		{
			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto mat_upload = job.m_upload.Alloc<GpuMaterial>(std::max(1, material_count));
			memcpy(mat_upload.ptr<GpuMaterial>(), materials.data(), material_count * sizeof(GpuMaterial));
			job.m_cmd_list.CopyBufferRegion(m_r_materials.get(), 0, mat_upload);

			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Switch states for resources
		{
			job.m_barriers.Transition(dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_times.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Calculate the contact times and zero the body colour_used bitmasks.
		{
			job.m_cmd_list.SetPipelineState(m_cs_compute_times.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_compute_times.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbResolve{ .g_colour = 0, .g_max_contacts = max_contacts, .g_dt = dt });
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_times->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

			// The collide shader's 'CSCalcResolveDispatch' function ensures that there will always be at least
			// one thread group dispatched, even if contact_count is 0. This ensures the 'colour_used' and
			// contact times are always initialised.
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(contacts.get());
			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		// Create the sorted contact order based on contact time
		{
			m_contact_sorter.Bind(job.m_cmd_list, max_contacts, m_r_contact_times, m_r_contact_order);
			m_contact_sorter.Sort(job.m_cmd_list);

			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		// Greedy graph colouring on sorted contacts.
		{
			job.m_cmd_list.SetPipelineState(m_cs_assign_colours.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_assign_colours.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbResolve{});
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(m_r_colours.get());
			job.m_barriers.Commit();
		}

		// Resolve each colour batch.
		{
			job.m_cmd_list.SetPipelineState(m_cs_resolve.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_resolve.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cbResolve{});
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_materials->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

			for (int colour = 0; colour != MaxColours; ++colour)
			{
				job.m_cmd_list.SetComputeRoot32BitConstants(0, cbResolve{ .g_colour = colour });
				job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

				job.m_barriers.UAV(bodies.get());
				job.m_barriers.Commit();
			}
		}
	}

	// CPU-side testing: upload contacts and bodies, run graph colouring + resolve on GPU, readback bodies.
	void GpuResolver::Resolve(GpuJob& job, float dt, std::span<GpuResolveContact const> contacts, std::span<GpuRigidBody> bodies, std::span<GpuMaterial const> materials)
	{
		auto contact_count = static_cast<int>(contacts.size());
		auto body_count = static_cast<int>(bodies.size());
		if (contact_count == 0 || body_count == 0)
			return;

		// Create temporary GPU resources
		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job.m_cmd_list, "Physics:TempCounters");
		auto r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(contact_count, {}), job.m_cmd_list, "Physics:TempContacts");
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempBodies");
		auto r_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempDispatch");

		// Upload counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.body_count = body_count,
				.pair_count = 0,
				.contact_count = contact_count,
			};
			job.m_cmd_list.CopyBufferRegion(r_counters.get(), 0, upload);

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload contacts
		{
			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuResolveContact>(contact_count);
			memcpy(upload.ptr<GpuResolveContact>(), contacts.data(), contact_count * sizeof(GpuResolveContact));
			job.m_cmd_list.CopyBufferRegion(r_contacts.get(), 0, upload);

			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload bodies
		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuRigidBody>(body_count);
			memcpy(upload.ptr<GpuRigidBody>(), bodies.data(), body_count * sizeof(GpuRigidBody));
			job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, upload);

			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Upload dispatch args
		{
			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<D3D12_DISPATCH_ARGUMENTS>(1);
			auto* args = upload.ptr<D3D12_DISPATCH_ARGUMENTS>();
			args->ThreadGroupCountX = static_cast<UINT>((contact_count + ResolveThreadCount - 1) / ResolveThreadCount);
			args->ThreadGroupCountY = 1;
			args->ThreadGroupCountZ = 1;
			job.m_cmd_list.CopyBufferRegion(r_dispatch.get(), 0, upload);

			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Run the GPU resolve pipeline
		Resolve(job, dt, contact_count, r_dispatch, r_counters, r_contacts, r_bodies, materials);

		// Readback bodies
		GpuReadbackBuffer::Allocation readback_bodies;
		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_bodies = job.m_readback.Alloc<GpuRigidBody>(body_count);
			job.m_cmd_list.CopyBufferRegion(readback_bodies, r_bodies.get(), 0);

			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}

		job.Run();

		memcpy(bodies.data(), readback_bodies.ptr<GpuRigidBody>(), body_count * sizeof(GpuRigidBody));
	}

	// Custom deleter implementation (GpuResolver is complete here)
	void Deleter<GpuResolver>::operator()(GpuResolver* p) const
	{
		delete p;
	}
}


#if 0

	// Greedy graph colouring: assign colours so no two contacts sharing a body have the same colour.
	// Uses per-body colour tracking to avoid the O(n²) scan of all earlier contacts.
	// Complexity: O(n × k) where k is the max colour count (typically small, ~4-8).
	std::pair<std::vector<int>, int> GraphColourContacts(std::span<GpuResolveContact const> contacts)
	{
		auto n = static_cast<int>(contacts.size());
		std::vector<int> colours(n, -1);
		int max_colour = 0;

		// Map from body_index → bitset of colours already used by that body's contacts.
		// This lets us find conflicting colours in O(k) instead of scanning all earlier contacts.
		std::unordered_map<int, std::vector<bool>> body_used;

		for (int i = 0; i != n; ++i)
		{
			auto a = contacts[i].body_idx_a;
			auto b = contacts[i].body_idx_b;

			auto& ua = body_used[a];
			auto& ub = body_used[b];

			// Find the lowest colour not used by either body
			auto limit = static_cast<int>(std::max(ua.size(), ub.size()));
			int c = 0;
			for (; c < limit; ++c)
			{
				auto used_a = c < static_cast<int>(ua.size()) && ua[c];
				auto used_b = c < static_cast<int>(ub.size()) && ub[c];
				if (!used_a && !used_b)
					break;
			}

			colours[i] = c;
			max_colour = std::max(max_colour, c + 1);

			// Mark this colour as used for both bodies
			if (c >= static_cast<int>(ua.size())) ua.resize(c + 1, false);
			if (c >= static_cast<int>(ub.size())) ub.resize(c + 1, false);
			ua[c] = true;
			ub[c] = true;
		}

		return {colours, max_colour};
	}


		// Upload contacts
		{
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuResolveContact>(contact_count);
			memcpy(upload.ptr<GpuResolveContact>(), contacts.data(), contact_count * sizeof(GpuResolveContact));
			job.m_cmd_list.CopyBufferRegion(m_r_contacts.get(), 0, upload);

			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload colours (convert int→uint32_t for GPU)
		{
			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<uint32_t>(contact_count);
			auto* dst = upload.ptr<uint32_t>();
			for (int i = 0; i != contact_count; ++i)
				dst[i] = static_cast<uint32_t>(colours[i]);
			job.m_cmd_list.CopyBufferRegion(m_r_colours.get(), 0, upload);

			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}
#endif