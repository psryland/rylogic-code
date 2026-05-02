//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/render/alpha_kbuffer.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/resource/descriptor_store.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/utility/barrier_batch.h"

namespace pr::rdr12
{
	// Resize the buffers to 'size'
	void AlphaKBuffer::Resize(Renderer& rdr, iv2 size, ClearValue rt_clear, ClearValue ds_clear)
	{
		if (size.x == 0 || size.y == 0)
		{
			m_opaque_colour_1x = nullptr;
			m_opaque_depth_1x = nullptr;
			m_alpha_colour = nullptr;
			m_alpha_depth = nullptr;
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
}
