//*********************************************
// Physics Engine — GPU Collision Resolution
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "src/collision/gpu_resolver.h"
#include "pr/physics/rigid_body/rigid_body_dynamics.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Constant buffer layout matching the HLSL cbResolve declaration.
	struct alignas(16) cbResolve
	{
		int g_colour;       // current colour batch being processed
		int pad0;
		int pad1;
		int pad2;
	};
	static_assert(sizeof(cbResolve) == 16);

	// Register assignments for the resolve root signature
	struct EResolveReg
	{
		inline static constexpr auto Params   = ECBufReg::b0;
		inline static constexpr auto Counters = ESRVReg::t0;
		inline static constexpr auto Contacts = ESRVReg::t1;
		inline static constexpr auto Bodies   = EUAVReg::u0;
		inline static constexpr auto Colours  = EUAVReg::u1;
	};

	GpuResolver::GpuResolver(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_colouring()
		, m_cs_resolve()
		, m_cmd_sig()
		, m_r_colours()
		, m_capacity()
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
			.Define(L"COLLISION_DIAGNOSTICS", L"" PR_STRINGISE(COLLISION_DIAGNOSTICS))
			.ShaderModel(L"cs_6_0")
			.Optimise();

		// m_cs_colouring
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EResolveReg::Params)
				.SRV(EResolveReg::Counters)
				.SRV(EResolveReg::Contacts)
				.UAV(EResolveReg::Colours);

			auto bytecode = compiler.EntryPoint(L"CSGraphColouring").Compile();

			m_cs_colouring.m_sig = sig.Create(m_gpu, "Physics:GraphColouringSig");
			m_cs_colouring.m_pso = ComputePSO(m_cs_colouring.m_sig.get(), bytecode).Create(m_gpu, "Physics:GraphColouringPSO");
		}

		// m_cs_resolve
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EResolveReg::Params)
				.SRV(EResolveReg::Counters)
				.SRV(EResolveReg::Contacts)
				.UAV(EResolveReg::Bodies)
				.UAV(EResolveReg::Colours);

			auto bytecode = compiler.EntryPoint(L"CSResolve").Compile();

			m_cs_resolve.m_sig = sig.Create(m_gpu, "Physics:ResolveSig");
			m_cs_resolve.m_pso = ComputePSO(m_cs_resolve.m_sig.get(), bytecode).Create(m_gpu, "Physics:ResolvePSO");
		}
	}

	// Create or grow GPU buffers for contacts and colour assignments.
	void GpuResolver::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_colours == nullptr || m_capacity < capacity)
		{
			m_r_colours = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}), cmd_list, "Physics:ResolveColours");
			m_capacity = capacity;
		}
	}

	// Resolve collisions on the GPU using graph-coloured batches.
	void GpuResolver::Resolve(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies)
	{
		ResizeBuffers(job.m_cmd_list, body_count);

		// Dispatch the graph colouring shader
		{
			job.m_cmd_list.SetPipelineState(m_cs_colouring.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_colouring.m_sig.get());
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());

			// TODO: this will depend on how the colouring algorithm works on the GPU
			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(m_r_colours.get());
			job.m_barriers.Commit();
		}

		// Dispatch the resolve shader
		{
			auto cb = cbResolve{ .g_colour = {} };

			job.m_cmd_list.SetPipelineState(m_cs_resolve.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_resolve.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());

			// Dispatch with the count provided by the collision detection shader
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			// UAV barrier on bodies ensures writes from this colour batch are visible
			// before the next batch reads them
			job.m_barriers.UAV(bodies.get());
			job.m_barriers.Commit();
		}
	}

	// Custom deleter implementation (GpuResolver is complete here)
	void Deleter<GpuResolver>::operator()(GpuResolver* p) const
	{
		delete p;
	}
}


#if 0

	// Greedy graph colouring: assign colours so no two contacts sharing a body have the same colour.
	std::pair<std::vector<int>, int> GraphColourContacts(std::span<GpuResolveContact const> contacts)
	{
		auto n = static_cast<int>(contacts.size());
		std::vector<int> colours(n, -1);
		int max_colour = 0;

		// @Copilot, this is an O(n^2) algorithm, is there a way to colour more efficiently?

		// For each contact (in time-sorted order), find conflicting colours
		for (int i = 0; i != n; ++i)
		{
			auto a = contacts[i].body_idx_a;
			auto b = contacts[i].body_idx_b;

			// Collect colours used by earlier contacts that share body A or B
			std::vector<bool> used(max_colour + 2, false);
			for (int j = 0; j != i; ++j)
			{
				if (contacts[j].body_idx_a == a || contacts[j].body_idx_b == a ||
					contacts[j].body_idx_a == b || contacts[j].body_idx_b == b)
				{
					if (colours[j] >= 0)
						used[colours[j]] = true;
				}
			}

			// Assign lowest available colour
			int c = 0;
			while (c < static_cast<int>(used.size()) && used[c]) ++c;
			colours[i] = c;
			max_colour = std::max(max_colour, c + 1);
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