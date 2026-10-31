//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/shaders/shader_procedural.h"
#include "pr/view3d-12/shaders/shader_ray_cast.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	Shader::Shader(Renderer& rdr)
		: RefCounted<Shader>()
		, m_rdr(&rdr)
		, m_code()
		, m_signature()
	{
	}

	// Renderer access
	Renderer const& Shader::rdr() const
	{
		return *m_rdr;
	}
	Renderer& Shader::rdr()
	{
		return *m_rdr;
	}

	// Sort id for the shader
	SortKeyId Shader::SortId() const
	{
		// Hash all of the ByteCode pointers together for the sort id.
		return SortKeyId(hash::HashBytes32(&m_code, &m_code + 1) % SortKey::MaxShaderId);
	}

	// Ref counting clean up function
	void Shader::RefCountZero(RefCounted<Shader>* doomed)
	{
		auto shdr = static_cast<Shader*>(doomed);
		shdr->rdr().DeferRelease(shdr->m_signature);
		shdr->Delete();
	}
	void Shader::Delete()
	{
		::pr::compute::Delete<Shader>(this);
	}

	// Create a procedural shader by copying all caller-owned data.
	ProceduralShader::ProceduralShader(Renderer& rdr, ERenderStep rdr_step, std::span<BYTE const> vs_bytecode, std::span<std::span<BYTE const> const> pixel_family, ForwardPixelFamily const& replaces, std::span<std::byte const> constants, D3DPtr<ID3D12Resource> buffer, std::string_view name)
		:Shader(rdr)
		,m_rdr_step(rdr_step)
		,m_vs_bytecode(vs_bytecode.begin(), vs_bytecode.end())
		,m_ps_bytecode()
		,m_pixel_family()
		,m_constants()
		,m_buffers{buffer}
		,m_writer()
		,m_dispatch_cs()
		,m_dispatch_sig()
		,m_dispatch_pso()
		,m_name(name)
	{
		// A pixel family is all or nothing, and only the forward step has pixel shaders to replace.
		if (!pixel_family.empty() && pixel_family.size() != ForwardPixelFamily::SlotCount)
			throw std::invalid_argument("A procedural pixel family must contain one pixel shader per forward pixel slot");
		if (!pixel_family.empty() && rdr_step != ERenderStep::RenderForward)
			throw std::invalid_argument("Procedural pixel shaders are only supported for the forward render step");
		if (vs_bytecode.empty() && pixel_family.empty())
			throw std::invalid_argument("A procedural shader must replace the vertex shader, the forward pixel shaders, or both");
		if (&replaces != &shader_code::forward_family && &replaces != &shader_code::forward_pbr_family && &replaces != &shader_code::forward_detail_family)
			throw std::invalid_argument("A procedural pixel family must replace a stock forward pixel family");

		// Retain stable storage for the bytecode referenced by the pipeline state. An empty vertex stage leaves the stock vertex shader in place.
		std::copy(constants.begin(), constants.end(), m_constants.begin());
		if (!m_vs_bytecode.empty())
			m_code.VS = ShaderCode::ByteCode(std::span<BYTE const>(m_vs_bytecode));
		if (!pixel_family.empty())
			m_pixel_family.m_replaces = &replaces;

		for (size_t i = 0; i != pixel_family.size(); ++i)
		{
			// The family is selected per sub-pass when the pipeline is built, so it is not stored in 'm_code.PS'.
			m_ps_bytecode[i].assign(pixel_family[i].begin(), pixel_family[i].end());
			m_pixel_family.m_code[i] = ShaderCode::ByteCode(std::span<BYTE const>(m_ps_bytecode[i]));
		}
	}

	// Release the writer here, where the factory type is complete. Its destructor waits for any queued write to finish.
	ProceduralShader::~ProceduralShader() = default;

	// True if this shader replaces the stock vertex shader.
	bool ProceduralShader::HasVertexShader() const
	{
		return !m_vs_bytecode.empty();
	}

	// True if this shader replaces the stock forward pixel shaders.
	bool ProceduralShader::HasPixelFamily() const
	{
		return !m_ps_bytecode[0].empty();
	}

	// True if this shader's forward pixel family replaces the stock PBR pixel shaders.
	bool ProceduralShader::HasPbrPixelFamily() const
	{
		return m_pixel_family.m_replaces == &shader_code::forward_pbr_family;
	}

	// Replace the copied constants used by later draws.
	void ProceduralShader::Constants(std::span<std::byte const> constants)
	{
		// The fixed-size block is the procedural binding contract, so partial updates are not supported.
		if (constants.size() != ConstantsSize)
			throw std::invalid_argument("Procedural shader constants must be exactly 1024 bytes");

		std::copy(constants.begin(), constants.end(), m_constants.begin());
	}

	// Replace the buffer bound at 'slot' by later draws.
	void ProceduralShader::Buffer(int slot, D3DPtr<ID3D12Resource> buffer)
	{
		// Callers may rebind the same buffer every frame, which needs no change.
		if (slot < 0 || slot >= BufferSlotCount)
			throw std::invalid_argument("Procedural buffer slot is out of range");
		if (m_buffers[slot] == buffer)
			return;

		// Frames already recorded may still read the old buffer, so its release waits for them.
		rdr().DeferRelease(m_buffers[slot]);
		m_buffers[slot] = buffer;

		// Record the resting state so that graphics-queue updates transition from, and back to, the state that draws expect.
		if (buffer != nullptr)
			::pr::compute::DefaultResState(buffer.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
	}

	// Return the buffer bound at 'slot'.
	D3DPtr<ID3D12Resource> const& ProceduralShader::Buffer(int slot) const
	{
		// Slots outside the fixed range do not exist.
		if (slot < 0 || slot >= BufferSlotCount)
			throw std::invalid_argument("Procedural buffer slot is out of range");

		return m_buffers[slot];
	}

	// Overwrite part of the slot 0 buffer for later frames.
	void ProceduralShader::BufferWrite(size_t offset, std::span<std::byte const> data)
	{
		// Shaders read the buffer in 4-byte words, and a write must stay inside it.
		auto* buffer = m_buffers[0].get();
		if (buffer == nullptr)
			throw std::invalid_argument("Procedural shader has no buffer to write");
		auto const buffer_size = s_cast<size_t>(buffer->GetDesc().Width);
		if (offset % 4 != 0 || data.size() % 4 != 0 || data.empty() || offset > buffer_size || data.size() > buffer_size - offset)
			throw std::invalid_argument("Procedural shader buffer writes must be non-empty 4-byte aligned ranges within the buffer");

		// Queue on the graphics queue, so the copy is ordered after the frames that read the old values and before the frames that read the new ones.
		if (m_writer == nullptr)
			m_writer = std::make_unique<ResourceFactory>(rdr());

		::pr::compute::UpdateSubresourceScope map(m_writer->CmdList(), m_writer->UploadBuffer(), buffer, 4, s_cast<int>(offset), s_cast<int>(data.size()));
		std::memcpy(map.ptr<std::byte>(), data.data(), data.size());
		map.Commit(::pr::compute::EFinalState::Restore);
		m_writer->FlushToGpu(EGpuFlush::Async);
	}

	// Run caller compute over the slot 0 buffer for later frames.
	void ProceduralShader::BufferDispatch(std::span<BYTE const> cs_bytecode, std::span<std::byte const> constants, iv3 groups)
	{
		// Validate the dispatch contract before recording GPU work.
		auto* buffer = m_buffers[0].get();
		if (buffer == nullptr)
			throw std::invalid_argument("Procedural shader has no buffer to dispatch over");
		if (!AllSet(buffer->GetDesc().Flags, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS))
			throw std::invalid_argument("Procedural buffer dispatch requires a buffer that allows unordered access");
		if (cs_bytecode.empty())
			throw std::invalid_argument("Procedural buffer dispatch requires compute bytecode");
		if (constants.empty() || constants.size() > D3D12_REQ_CONSTANT_BUFFER_ELEMENT_COUNT * 16ULL)
			throw std::invalid_argument("Procedural buffer dispatch constants must contain 1 to 65536 bytes");
		for (int i = 0; i != 3; ++i)
		{
			// Each dimension must be a positive count within the D3D12 limit.
			if (groups[i] <= 0 || groups[i] > D3D12_CS_DISPATCH_MAX_THREAD_GROUPS_PER_DIMENSION)
				throw std::invalid_argument("Procedural buffer dispatch thread-group counts are invalid");
		}

		// Rebuild the pipeline only when the bytecode changes. Queued dispatches may still reference the old pipeline, so its release is deferred.
		if (!std::ranges::equal(cs_bytecode, m_dispatch_cs))
		{
			rdr().DeferRelease(m_dispatch_pso);
			rdr().DeferRelease(m_dispatch_sig);
			m_dispatch_cs.assign(cs_bytecode.begin(), cs_bytecode.end());
			m_dispatch_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.CBuf(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.Create(rdr().d3d(), "Procedural buffer dispatch root signature");
			m_dispatch_pso = ::pr::compute::ComputePSO(m_dispatch_sig.get(), std::span<BYTE const>(m_dispatch_cs)).Create(rdr().d3d(), "Procedural buffer dispatch pipeline");
		}

		// Queue on the graphics queue, so the dispatch is ordered after the frames that read the old values and before the frames that read the new ones.
		if (m_writer == nullptr)
			m_writer = std::make_unique<ResourceFactory>(rdr());

		auto cbuf = m_writer->UploadBuffer().Alloc(s_cast<int>(constants.size()), D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
		std::memcpy(cbuf.m_mem + cbuf.m_ofs, constants.data(), constants.size());

		// Expose the buffer for writing, dispatch, then return it to the state that draws read it in. The transition back also orders the writes before later reads.
		auto& cmd_list = m_writer->CmdList();
		::pr::compute::BarrierBatch barriers(cmd_list);
		barriers.Transition(buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		barriers.Commit();
		cmd_list.SetPipelineState(m_dispatch_pso.get());
		cmd_list.SetComputeRootSignature(m_dispatch_sig.get());
		cmd_list.SetComputeRootConstantBufferView(0, cbuf);
		cmd_list.SetComputeRootUnorderedAccessView(1, buffer);
		cmd_list.Dispatch(groups);
		barriers.Transition(buffer, D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		barriers.Commit();
		m_writer->FlushToGpu(EGpuFlush::Async);
	}

	// Bind the copied constants and optional buffers through the render-step-specific reserved root slots.
	void ProceduralShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const&, CameraTransforms const&, DrawListElement const*)
	{
		// Upload the current copy each draw; identical content is shared within the frame.
		auto gpu_address = upload.Add(m_constants, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true).VirtualAddress();
		auto Bind = [&](UINT cbuf_slot, std::array<UINT, BufferSlotCount> const& buffer_slots)
		{
			// Empty buffer slots stay unbound, because shaders must not declare a slot without a buffer.
			cmd_list->SetGraphicsRootConstantBufferView(cbuf_slot, gpu_address);
			for (int i = 0; i != BufferSlotCount; ++i)
			{
				// Bind each populated slot to its reserved root parameter.
				if (m_buffers[i] != nullptr)
					cmd_list->SetGraphicsRootShaderResourceView(buffer_slots[i], m_buffers[i]->GetGPUVirtualAddress());
			}
		};
		switch (m_rdr_step)
		{
			case ERenderStep::RenderForward:
			{
				using shaders::fwd::ERootParam;
				Bind(static_cast<UINT>(ERootParam::CBufProcedural), { static_cast<UINT>(ERootParam::ProceduralBuffer0), static_cast<UINT>(ERootParam::ProceduralBuffer1) });
				return;
			}
			case ERenderStep::RayCast:
			{
				using shaders::ray_cast::ERootParam;
				Bind(static_cast<UINT>(ERootParam::CBufProcedural), { static_cast<UINT>(ERootParam::ProceduralBuffer0), static_cast<UINT>(ERootParam::ProceduralBuffer1) });
				return;
			}
			case ERenderStep::ShadowMap:
			{
				using shaders::smap::ERootParam;
				Bind(static_cast<UINT>(ERootParam::CBufProcedural), { static_cast<UINT>(ERootParam::ProceduralBuffer0), static_cast<UINT>(ERootParam::ProceduralBuffer1) });
				return;
			}
			default:
			{
				throw std::runtime_error("Unsupported procedural shader render step");
			}
		}
	}

	// Destroy the concrete shader rather than the base subobject.
	void ProceduralShader::Delete()
	{
		// Release copied bytecode and constants with the shader handle. The GPU may still be reading the buffers, or running a queued dispatch, from in-flight work.
		for (auto const& buffer : m_buffers)
			rdr().DeferRelease(buffer);


		rdr().DeferRelease(m_dispatch_pso);
		rdr().DeferRelease(m_dispatch_sig);
		::pr::compute::Delete<ProceduralShader>(this);
	}

	// Compiled shader byte code
	namespace shader_code
	{
		// Not a shader
		ByteCode const none;

		// Forward rendering shaders
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(forward_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_reflection_attrs_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_reflection_attrs_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_alpha_collect_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_alpha_collect_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_texn_pbr_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_population_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_texn_population_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_texn_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_reflection_attrs_texn_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_alpha_collect_texn_pbr_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(forward_radial_fade_ps.h)
	        #include PR_RDR_SHADER_COMPILED_DIR(forward_detail_ps.h)
	        #include PR_RDR_SHADER_COMPILED_DIR(forward_reflection_attrs_detail_ps.h)
	        #include PR_RDR_SHADER_COMPILED_DIR(forward_alpha_collect_detail_ps.h)
			#include PR_RDR_SHADER_COMPILED_DIR(kbuffer_resolve_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(kbuffer_alpha_resolve_ps.h)
		}
		ByteCode const forward_vs(compiled::forward_vs);
		ByteCode const forward_ps(compiled::forward_ps);
		ByteCode const forward_pbr_ps(compiled::forward_pbr_ps);
		ByteCode const forward_reflection_attrs_ps(compiled::forward_reflection_attrs_ps);
		ByteCode const forward_reflection_attrs_pbr_ps(compiled::forward_reflection_attrs_pbr_ps);
		ByteCode const forward_alpha_collect_ps(compiled::forward_alpha_collect_ps);
		ByteCode const forward_alpha_collect_pbr_ps(compiled::forward_alpha_collect_pbr_ps);
		ByteCode const forward_texn_pbr_vs(compiled::forward_texn_pbr_vs);
		ByteCode const forward_population_vs(compiled::forward_population_vs);
		ByteCode const forward_texn_population_vs(compiled::forward_texn_population_vs);
		ByteCode const forward_texn_pbr_ps(compiled::forward_texn_pbr_ps);
		ByteCode const forward_reflection_attrs_texn_pbr_ps(compiled::forward_reflection_attrs_texn_pbr_ps);
		ByteCode const forward_alpha_collect_texn_pbr_ps(compiled::forward_alpha_collect_texn_pbr_ps);
		ByteCode const forward_radial_fade_ps(compiled::forward_radial_fade_ps);
	    ByteCode const forward_detail_ps(compiled::forward_detail_ps);
	    ByteCode const forward_reflection_attrs_detail_ps(compiled::forward_reflection_attrs_detail_ps);
	    ByteCode const forward_alpha_collect_detail_ps(compiled::forward_alpha_collect_detail_ps);
		ByteCode const kbuffer_resolve_vs(compiled::kbuffer_resolve_vs);
		ByteCode const kbuffer_alpha_resolve_ps(compiled::kbuffer_alpha_resolve_ps);

		// Stock families, defined after their members in this translation unit so the members are initialised first. The detail family replaces simple-material shading.
		ForwardPixelFamily const forward_family = {{ forward_ps, forward_reflection_attrs_ps, forward_alpha_collect_ps }, nullptr};
		ForwardPixelFamily const forward_pbr_family = {{ forward_pbr_ps, forward_reflection_attrs_pbr_ps, forward_alpha_collect_pbr_ps }, nullptr};
		ForwardPixelFamily const forward_texn_pbr_family = {{ forward_texn_pbr_ps, forward_reflection_attrs_texn_pbr_ps, forward_alpha_collect_texn_pbr_ps }, nullptr};
		ForwardPixelFamily const forward_detail_family = {{ forward_detail_ps, forward_reflection_attrs_detail_ps, forward_alpha_collect_detail_ps }, &forward_family};

		// Post-processing
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(post_effect_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(underwater_ps.h)
		}
		ByteCode const post_effect_vs(compiled::post_effect_vs);
		ByteCode const underwater_ps(compiled::underwater_ps);

		// Shadows
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(shadow_map_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(shadow_map_population_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(shadow_map_ps.h)
		}
		ByteCode const shadow_map_vs(compiled::shadow_map_vs);
		ByteCode const shadow_map_population_vs(compiled::shadow_map_population_vs);
		ByteCode const shadow_map_ps(compiled::shadow_map_ps);

		// Screen Space
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(point_sprites_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(thick_line_list_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(thick_line_strip_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(arrow_head_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(show_normals_gs.h)
		}
		ByteCode const point_sprites_gs(compiled::point_sprites_gs);
		ByteCode const thick_line_list_gs(compiled::thick_line_list_gs);
		ByteCode const thick_line_strip_gs(compiled::thick_line_strip_gs);
		ByteCode const arrow_head_gs(compiled::arrow_head_gs);
		ByteCode const show_normals_gs(compiled::show_normals_gs);

		// Ray cast
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_vert_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_edge_gs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_cast_face_gs.h)
		}
		ByteCode const ray_cast_vs(compiled::ray_cast_vs);
		ByteCode const ray_cast_vert_gs(compiled::ray_cast_vert_gs);
		ByteCode const ray_cast_edge_gs(compiled::ray_cast_edge_gs);
		ByteCode const ray_cast_face_gs(compiled::ray_cast_face_gs);

		// Procedural atmosphere
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(procedural_sky_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(procedural_sky_ps.h)
		}
		ByteCode const procedural_sky_vs(compiled::procedural_sky_vs);
		ByteCode const procedural_sky_ps(compiled::procedural_sky_ps);

		// Ray tracing
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(ray_trace_lib.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_trace_present_vs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(ray_trace_present_ps.h)
		}
		ByteCode const ray_trace_lib(compiled::ray_trace_lib);
		ByteCode const ray_trace_present_vs(compiled::ray_trace_present_vs);
		ByteCode const ray_trace_present_ps(compiled::ray_trace_present_ps);

		// MipMap generation
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(mipmap_generator_cs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(skinning_cs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(env_map_distance_cs.h)
		}
		ByteCode const mipmap_generator_cs(compiled::mipmap_generator_cs);
		ByteCode const env_map_distance_cs(compiled::env_map_distance_cs);
		ByteCode const skinning_cs(compiled::skinning_cs);

		// Population visibility, LOD classification, and draw argument generation
		namespace compiled
		{
			#include PR_RDR_SHADER_COMPILED_DIR(population_clear_cs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(population_count_cs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(population_prefix_cs.h)
			#include PR_RDR_SHADER_COMPILED_DIR(population_scatter_cs.h)
		}
		ByteCode const population_clear_cs(compiled::population_clear_cs);
		ByteCode const population_count_cs(compiled::population_count_cs);
		ByteCode const population_prefix_cs(compiled::population_prefix_cs);
		ByteCode const population_scatter_cs(compiled::population_scatter_cs);
	}
}
