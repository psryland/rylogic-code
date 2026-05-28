//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/render/alpha_kbuffer.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	struct EReg
	{
		inline static constexpr auto OpaqueColour = ESRVReg::t0;
		inline static constexpr auto AlphaColour = ESRVReg::t1;
		inline static constexpr auto AlphaDepth = ESRVReg::t2;
	};
	enum class EResolveRootParam
	{
		Textures = 0,
	};

	AlphaKBuffer::AlphaKBuffer()
		: m_opaque_colour_1x()
		, m_alpha_colour()
		, m_alpha_depth()
		, m_alpha_rt_attrs()
		, m_signature_alpha_resolve()
		, m_pso_alpha_resolve()
	{}

	// Release D3D resources
	void AlphaKBuffer::Release()
	{
		m_opaque_colour_1x = nullptr;
		m_alpha_colour = nullptr;
		m_alpha_depth = nullptr;
		m_alpha_rt_attrs = nullptr;
		m_signature_alpha_resolve = nullptr;
		m_pso_alpha_resolve = nullptr;
	}

	// Resize the buffers to 'size'
	void AlphaKBuffer::Resize(Renderer& rdr, iv2 size, ClearValue rt_clear)
	{
		if (size.x == 0 || size.y == 0)
		{
			Release();
			return;
		}

		ResourceFactory factory(rdr);
		auto colour_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, rt_clear.Format }, 1U, EUsage::RenderTarget)
			.clear(rt_clear)
			.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		auto kbuffer_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, DXGI_FORMAT_R32G32B32A32_UINT }, 1U, EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);

		m_opaque_colour_1x = factory.CreateTexture2D(TextureDesc(AutoId, colour_desc).name("KBuffer-OpaqueColour1x").srv_format(::pr::compute::ToSRGB(rt_clear.Format)));
		m_alpha_colour = factory.CreateTexture2D(TextureDesc(AutoId, kbuffer_desc).name("KBuffer-AlphaColour"));
		m_alpha_depth = factory.CreateTexture2D(TextureDesc(AutoId, kbuffer_desc).name("KBuffer-AlphaDepth"));
		m_alpha_rt_attrs = factory.CreateTexture2D(TextureDesc(AutoId, kbuffer_desc).name("KBuffer-AlphaRtAttrs"));

		// Create the root signature for the alpha resolve pass
		m_signature_alpha_resolve = RootSig(ERootSigFlags::GraphicsOnly)
			.SRV(EReg::OpaqueColour, 3, D3D12_SHADER_VISIBILITY_PIXEL)
			.Create(rdr.d3d(), "KBufferAlphaResolveSig");

		// Create the pipeline state object for the alpha resolve pass
		auto desc = D3D12_GRAPHICS_PIPELINE_STATE_DESC{
			.pRootSignature = m_signature_alpha_resolve.get(),
			.VS = shader_code::kbuffer_resolve_vs,
			.PS = shader_code::kbuffer_alpha_resolve_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.StreamOutput = StreamOutputDesc{},
			.BlendState = BlendStateDesc{},
			.SampleMask = UINT_MAX,
			.RasterizerState = RasterStateDesc{},
			.DepthStencilState = DepthStateDesc{}.Enabled(false),
			.InputLayout = {},
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 1U,
			// Match the sRGB RTV cast applied in Window::CreateSwapChain so the PSO writes through an sRGB RTV.
			.RTVFormats = { ::pr::compute::ToSRGB(rt_clear.Format) },
			.DSVFormat = DXGI_FORMAT_UNKNOWN,
			.SampleDesc = MultiSamp(1, 0),
			.NodeMask = 0U,
			.CachedPSO = {},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};
		D3DPtr<ID3D12PipelineState> pso_alpha_resolve;
		Check(rdr.d3d()->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), (void**)pso_alpha_resolve.address_of()));
		DebugName(pso_alpha_resolve, "KBufferAlphaResolvePSO");
		m_pso_alpha_resolve = std::move(pso_alpha_resolve);
	}

	// Create the buffers for the next frame
	void AlphaKBuffer::Clear(GfxCmdList& cmd_list, GpuViewHeap& heap_view)
	{
		if (m_alpha_colour == nullptr || m_alpha_depth == nullptr || m_alpha_rt_attrs == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(m_alpha_colour->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(m_alpha_depth->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(m_alpha_rt_attrs->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Commit();

		auto alpha_colour_uav = heap_view.Add(m_alpha_colour->m_uav);
		auto alpha_depth_uav = heap_view.Add(m_alpha_depth->m_uav);
		auto alpha_rt_attrs_uav = heap_view.Add(m_alpha_rt_attrs->m_uav);
		UINT const clear_colour[4] = {};
		UINT const clear_depth[4] = {0x00FFFFFFu, 0x00FFFFFFu, 0x00FFFFFFu, 0x00FFFFFFu};
		cmd_list.ClearUnorderedAccessViewUint(alpha_colour_uav, m_alpha_colour->m_uav.m_cpu, m_alpha_colour->m_res.get(), clear_colour);
		cmd_list.ClearUnorderedAccessViewUint(alpha_depth_uav, m_alpha_depth->m_uav.m_cpu, m_alpha_depth->m_res.get(), clear_depth);
		cmd_list.ClearUnorderedAccessViewUint(alpha_rt_attrs_uav, m_alpha_rt_attrs->m_uav.m_cpu, m_alpha_rt_attrs->m_res.get(), clear_colour);
	}

	// Copy the opaque buffer 'source' into the 1x resolved buffer using resolve or direct copy
	void AlphaKBuffer::CopyOpaqueBuffer(GfxCmdList& cmd_list, ID3D12Resource* source, DXGI_FORMAT format, bool resolve)
	{
		if (m_opaque_colour_1x == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(m_opaque_colour_1x->m_res.get(), resolve ? D3D12_RESOURCE_STATE_RESOLVE_DEST : D3D12_RESOURCE_STATE_COPY_DEST);
		bb.Commit();

		if (resolve)
			cmd_list.ResolveSubresource(m_opaque_colour_1x->m_res.get(), source, format);
		else
			cmd_list.CopyResource(m_opaque_colour_1x->m_res.get(), source);

		bb.Transition(m_opaque_colour_1x->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Commit();
	}

	// Composite the collected alpha buffer over the resolved opaque colour
	void AlphaKBuffer::ResolveAlpha(GfxCmdList& cmd_list, GpuViewHeap& heap_view, BackBuffer const& bb_post, Viewport const& viewport, D3D12_RECT const& scissor)
	{
		if (m_opaque_colour_1x == nullptr || m_alpha_colour == nullptr || m_alpha_depth == nullptr || m_alpha_rt_attrs == nullptr || bb_post.m_render_target == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(bb_post.m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
		bb.Transition(m_opaque_colour_1x->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Transition(m_alpha_colour->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Transition(m_alpha_depth->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Transition(m_alpha_rt_attrs->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Commit();

		Descriptor descriptors[] = { m_opaque_colour_1x->m_srv, m_alpha_colour->m_srv, m_alpha_depth->m_srv };
		auto textures = heap_view.Add(descriptors);

		cmd_list.SetPipelineState(m_pso_alpha_resolve.get());
		cmd_list.SetGraphicsRootSignature(m_signature_alpha_resolve.get());
		cmd_list.SetGraphicsRootDescriptorTable(EResolveRootParam::Textures, textures);
		cmd_list.OMSetRenderTargets({ &bb_post.m_rtv, 1 }, FALSE, nullptr);
		cmd_list.RSSetViewports({ &viewport, 1U });
		cmd_list.RSSetScissorRects({ &scissor, 1U });
		cmd_list.IASetPrimitiveTopology(ETopo::TriList);
		cmd_list.DrawInstanced(3, 1, 0, 0);
	}

	// True if this AlphaKBuffer is valid and ready for use
	AlphaKBuffer::operator bool() const
	{
		// It should be all non-null or all null
		return m_alpha_colour != nullptr && m_alpha_depth != nullptr && m_alpha_rt_attrs != nullptr;
	}
}

