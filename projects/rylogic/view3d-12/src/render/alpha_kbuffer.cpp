//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/render/alpha_kbuffer.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/resource/descriptor.h"
#include "pr/view3d-12/resource/descriptor_store.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/shaders/shader.h"
#include "view3d-12/src/shaders/common.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/root_signature.h"

namespace pr::rdr12
{
	struct EReg
	{
		inline static constexpr auto CBufKBufferResolve = ECBufReg::b0;
		inline static constexpr auto OpaqueColour = ESRVReg::t0;
		inline static constexpr auto AlphaColour = ESRVReg::t1;
		inline static constexpr auto AlphaDepth = ESRVReg::t2;
	};
	enum class EResolveRootParam
	{
		CBufResolve = 0,
		Textures,
	};

	// Resize the buffers to 'size'
	void AlphaKBuffer::Resize(Renderer& rdr, iv2 size, ClearValue rt_clear, ClearValue ds_clear)
	{
		if (size.x == 0 || size.y == 0)
		{
			m_opaque_colour_1x = nullptr;
			m_opaque_depth_1x = nullptr;
			m_alpha_colour = nullptr;
			m_alpha_depth = nullptr;
			m_signature_depth_resolve = nullptr;
			m_signature_alpha_resolve = nullptr;
			m_pso_depth_resolve = nullptr;
			m_pso_alpha_resolve = nullptr;
			return;
		}

		ResourceFactory factory(rdr);
		auto colour_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, rt_clear.Format }, 1U, EUsage::RenderTarget)
			.clear(rt_clear)
			.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		auto depth_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, ds_clear.Format }, 1U, EUsage::DepthStencil | EUsage::DenyShaderResource)
			.clear(ds_clear)
			.def_state(D3D12_RESOURCE_STATE_DEPTH_WRITE);
		auto kbuffer_desc = ResDesc::Tex2D(Image{ size.x, size.y, nullptr, DXGI_FORMAT_R32G32B32A32_UINT }, 1U, EUsage::UnorderedAccess)
			.def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);

		m_opaque_colour_1x = factory.CreateTexture2D(TextureDesc(AutoId, colour_desc).name("KBuffer-OpaqueColour1x"));
		m_opaque_depth_1x = factory.CreateTexture2D(TextureDesc(AutoId, depth_desc).name("KBuffer-OpaqueDepth1x"));
		m_alpha_colour = factory.CreateTexture2D(TextureDesc(AutoId, kbuffer_desc).name("KBuffer-AlphaColour"));
		m_alpha_depth = factory.CreateTexture2D(TextureDesc(AutoId, kbuffer_desc).name("KBuffer-AlphaDepth"));

		// Create the root signatures for resolve passes
		m_signature_depth_resolve = RootSig(ERootSigFlags::GraphicsOnly)
			.CBuf(EReg::CBufKBufferResolve, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::OpaqueColour, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.Create(rdr.d3d(), "KBufferDepthResolveSig");

		m_signature_alpha_resolve = RootSig(ERootSigFlags::GraphicsOnly)
			.CBuf(EReg::CBufKBufferResolve, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::OpaqueColour, 3, D3D12_SHADER_VISIBILITY_PIXEL)
			.Create(rdr.d3d(), "KBufferAlphaResolveSig");

		// Create the pipeline state objects for the resolve passes
		auto desc = D3D12_GRAPHICS_PIPELINE_STATE_DESC{
			.pRootSignature = m_signature_depth_resolve.get(),
			.VS = shader_code::kbuffer_resolve_vs,
			.PS = shader_code::kbuffer_depth_resolve_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.StreamOutput = StreamOutputDesc{},
			.BlendState = BlendStateDesc{},
			.SampleMask = UINT_MAX,
			.RasterizerState = RasterStateDesc{},
			.DepthStencilState = DepthStateDesc{},
			.InputLayout = {},
			.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
			.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE,
			.NumRenderTargets = 0U,
			.RTVFormats = {},
			.DSVFormat = ds_clear.Format,
			.SampleDesc = MultiSamp(1, 0),
			.NodeMask = 0U,
			.CachedPSO = {},
			.Flags = D3D12_PIPELINE_STATE_FLAG_NONE,
		};
		Check(rdr.d3d()->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), (void**)m_pso_depth_resolve.address_of()));
		DebugName(m_pso_depth_resolve, "KBufferDepthResolvePSO");

		desc.PS = shader_code::kbuffer_alpha_resolve_ps;
		desc.pRootSignature = m_signature_alpha_resolve.get();
		desc.DepthStencilState = DepthStateDesc{}.Enabled(false);
		desc.NumRenderTargets = 1U;
		desc.RTVFormats[0] = rt_clear.Format;
		desc.DSVFormat = DXGI_FORMAT_UNKNOWN;
		Check(rdr.d3d()->CreateGraphicsPipelineState(&desc, __uuidof(ID3D12PipelineState), (void**)m_pso_alpha_resolve.address_of()));
		DebugName(m_pso_alpha_resolve, "KBufferAlphaResolvePSO");
	}

	// Create the buffers for the next frame
	void AlphaKBuffer::Clear(GfxCmdList& cmd_list, GpuViewHeap& heap_view)
	{
		if (m_alpha_colour == nullptr || m_alpha_depth == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(m_alpha_colour->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Transition(m_alpha_depth->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		bb.Commit();

		auto alpha_colour_uav = heap_view.Add(m_alpha_colour->m_uav);
		auto alpha_depth_uav = heap_view.Add(m_alpha_depth->m_uav);
		UINT const clear_colour[4] = {};
		UINT const clear_depth[4] = {0x00FFFFFFu, 0x00FFFFFFu, 0x00FFFFFFu, 0x00FFFFFFu};
		cmd_list.ClearUnorderedAccessViewUint(alpha_colour_uav, m_alpha_colour->m_uav.m_cpu, m_alpha_colour->m_res.get(), clear_colour);
		cmd_list.ClearUnorderedAccessViewUint(alpha_depth_uav, m_alpha_depth->m_uav.m_cpu, m_alpha_depth->m_res.get(), clear_depth);
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

	// Resolve the MSAA depth buffer into the 1x opaque depth buffer
	void AlphaKBuffer::ResolveDepth(GfxCmdList& cmd_list, GpuViewHeap& heap_view, GpuUploadBuffer& upload, ID3D12Resource* msaa_depth, Descriptor const& msaa_depth_srv, int sample_count, Viewport const& viewport, D3D12_RECT const& scissor)
	{
		if (m_opaque_depth_1x == nullptr || msaa_depth == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(msaa_depth, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
		bb.Transition(m_opaque_depth_1x->m_res.get(), D3D12_RESOURCE_STATE_DEPTH_WRITE);
		bb.Commit();

		cmd_list.ClearDepthStencilView(m_opaque_depth_1x->m_dsv.m_cpu, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0);

		shaders::fwd::CBufKBufferResolve cb = {};
		cb.screen_dim = iv2(s_cast<int>(viewport.Width), s_cast<int>(viewport.Height));
		cb.sample_count = sample_count;
		auto gpu_address = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		auto msaa_depth_handle = heap_view.Add(msaa_depth_srv);

		cmd_list.SetPipelineState(m_pso_depth_resolve.get());
		cmd_list.SetGraphicsRootSignature(m_signature_depth_resolve.get());
		cmd_list.SetGraphicsRootConstantBufferView(EResolveRootParam::CBufResolve, gpu_address);
		cmd_list.SetGraphicsRootDescriptorTable(EResolveRootParam::Textures, msaa_depth_handle);
		cmd_list.OMSetRenderTargets({}, FALSE, &m_opaque_depth_1x->m_dsv.m_cpu);
		cmd_list.RSSetViewports({ &viewport, 1U });
		cmd_list.RSSetScissorRects({ &scissor, 1U });
		cmd_list.IASetPrimitiveTopology(ETopo::TriList);
		cmd_list.DrawInstanced(3, 1, 0, 0);
	}

	// Composite the collected alpha buffer over the resolved opaque colour
	void AlphaKBuffer::ResolveAlpha(GfxCmdList& cmd_list, GpuViewHeap& heap_view, GpuUploadBuffer& upload, BackBuffer const& bb_post, Viewport const& viewport, D3D12_RECT const& scissor)
	{
		if (m_opaque_colour_1x == nullptr || m_alpha_colour == nullptr || m_alpha_depth == nullptr || bb_post.m_render_target == nullptr)
			return;

		BarrierBatch bb(cmd_list);
		bb.Transition(bb_post.m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
		bb.Transition(m_opaque_colour_1x->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Transition(m_alpha_colour->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Transition(m_alpha_depth->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
		bb.Commit();

		shaders::fwd::CBufKBufferResolve cb = {};
		cb.screen_dim = iv2(s_cast<int>(viewport.Width), s_cast<int>(viewport.Height));
		cb.sample_count = 1;
		auto gpu_address = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);

		Descriptor descriptors[] = { m_opaque_colour_1x->m_srv, m_alpha_colour->m_srv, m_alpha_depth->m_srv };
		auto textures = heap_view.Add(descriptors);

		cmd_list.SetPipelineState(m_pso_alpha_resolve.get());
		cmd_list.SetGraphicsRootSignature(m_signature_alpha_resolve.get());
		cmd_list.SetGraphicsRootConstantBufferView(EResolveRootParam::CBufResolve, gpu_address);
		cmd_list.SetGraphicsRootDescriptorTable(EResolveRootParam::Textures, textures);
		cmd_list.OMSetRenderTargets({ &bb_post.m_rtv, 1 }, FALSE, nullptr);
		cmd_list.RSSetViewports({ &viewport, 1U });
		cmd_list.RSSetScissorRects({ &scissor, 1U });
		cmd_list.IASetPrimitiveTopology(ETopo::TriList);
		cmd_list.DrawInstanced(3, 1, 0, 0);
	}
}
