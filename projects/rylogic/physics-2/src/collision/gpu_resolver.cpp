//*********************************************
// Physics Engine — GPU Collision Resolution
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "src/collision/gpu_resolver.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Thread group size matching the HLSL [numthreads(64, 1, 1)] declaration.
	static constexpr int ResolveThreadGroupSize = 64;

	// Constant buffer layout matching the HLSL cbResolve declaration.
	struct alignas(16) cbResolve
	{
		uint32_t contact_count;
		uint32_t colour;       // current colour batch being processed
		uint32_t pad0;
		uint32_t pad1;
	};
	static_assert(sizeof(cbResolve) == 16);

	// Register assignments for the resolve root signature
	struct EResolveReg
	{
		inline static constexpr ECBufReg Params   = ECBufReg::b0;
		inline static constexpr EUAVReg  Bodies   = EUAVReg::u0;
		inline static constexpr ESRVReg  Contacts = ESRVReg::t0;
		inline static constexpr ESRVReg  Colours  = ESRVReg::t1;
	};

	GpuResolver::GpuResolver(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_resolve()
		, m_r_contacts()
		, m_r_colours()
		, m_capacity()
	{
		CompileShader();
	}

	// Compile the resolve compute shader from embedded resources.
	void GpuResolver::CompileShader()
	{
		auto shader_source = resource::Read<char>(L"PHYSICS_RESOLVE_HLSL", L"TEXT");
		auto compiler = ShaderCompiler{}
			.Source(shader_source)
			.Includes({new ResourceIncludeHandler, true})
			.EntryPoint(L"CSResolve")
			.ShaderModel(L"cs_6_0")
			.Optimise();

		auto bytecode = compiler.Compile();

		// Root signature: root constants (cbResolve) + UAV (bodies) + 2 SRVs (contacts, colours)
		m_cs_resolve.m_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbResolve>(EResolveReg::Params)
			.UAV(EResolveReg::Bodies)
			.SRV(EResolveReg::Contacts)
			.SRV(EResolveReg::Colours)
			.Create(m_gpu, "Physics:ResolveSig");

		m_cs_resolve.m_pso = ComputePSO(m_cs_resolve.m_sig.get(), bytecode)
			.Create(m_gpu, "Physics:ResolvePSO");
	}

	// Create or grow GPU buffers for contacts and colour assignments.
	void GpuResolver::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);

		if (m_r_contacts == nullptr || m_capacity < capacity)
		{
			m_r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(capacity, {}), cmd_list, "Physics:ResolveContacts");
			m_r_colours = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}), cmd_list, "Physics:ResolveColours");
			m_capacity = capacity;
		}
	}

	// Resolve collisions on the GPU using graph-coloured batches.
	void GpuResolver::Resolve(
		GpuJob& job,
		std::span<GpuResolveContact const> contacts,
		std::span<int const> colours,
		int max_colour,
		ID3D12Resource* bodies_resource)
	{
		auto contact_count = static_cast<int>(contacts.size());
		if (contact_count == 0 || max_colour == 0)
			return;

		ResizeBuffers(job.m_cmd_list, contact_count);

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

		// Dispatch one compute pass per colour batch.
		// Bodies buffer is assumed to be in UNORDERED_ACCESS state from the integrator.
		auto dispatch_count = (contact_count + ResolveThreadGroupSize - 1) / ResolveThreadGroupSize;
		for (int colour = 0; colour != max_colour; ++colour)
		{
			auto cb = cbResolve{
				.contact_count = static_cast<uint32_t>(contact_count),
				.colour = static_cast<uint32_t>(colour),
			};

			job.m_cmd_list.SetPipelineState(m_cs_resolve.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_resolve.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies_resource->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_r_colours->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(dispatch_count, 1, 1);

			// UAV barrier on bodies ensures writes from this colour batch are visible
			// before the next batch reads them
			job.m_barriers.UAV(bodies_resource);
			job.m_barriers.Commit();
		}
	}

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

	// Custom deleter implementation (GpuResolver is complete here)
	void Deleter<GpuResolver>::operator()(GpuResolver* p) const
	{
		delete p;
	}
}
