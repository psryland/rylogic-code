//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/texture/texture_2D.h"

namespace pr::rdr12
{
	BackBuffer::BackBuffer()
		: m_wnd()
		, m_multisamp()
		, m_sync_point()
		, m_render_target()
		, m_depth_stencil()
		, m_d2d_target()
		, m_rtv()
		, m_dsv()
	{}
	BackBuffer::BackBuffer(Window& wnd, MultiSamp ms, Texture2D* render_target, Texture2D* depth_stencil)
		: m_wnd(&wnd)
		, m_multisamp(ms)
		, m_sync_point(wnd.m_gsync.CompletedSyncPoint())
		, m_render_target(render_target ? render_target->m_res : nullptr)
		, m_depth_stencil(depth_stencil ? depth_stencil->m_res : nullptr)
		, m_d2d_target()
		, m_rtv(render_target ? render_target->m_rtv.m_cpu : D3D12_CPU_DESCRIPTOR_HANDLE{})
		, m_dsv(depth_stencil ? depth_stencil->m_dsv.m_cpu : D3D12_CPU_DESCRIPTOR_HANDLE{})
	{}

	// An empty back buffer
	BackBuffer& BackBuffer::Null()
	{
		static BackBuffer s_null;
		return s_null;
	}

	// Accessors
	Renderer& BackBuffer::rdr() const
	{
		return wnd().rdr();
	}
	Window& BackBuffer::wnd() const
	{
		return *m_wnd;
	}

	iv2 BackBuffer::rt_size() const
	{
		if (m_render_target == nullptr)
			return iv2::Zero();

		auto desc = m_render_target->GetDesc();
		return iv2(s_cast<int>(desc.Width), s_cast<int>(desc.Height));
	}
	float4_t const& BackBuffer::rt_clear() const
	{
		return wnd().m_rt_props.Color;
	}
	float BackBuffer::ds_depth() const
	{
		return wnd().m_ds_props.DepthStencil.Depth;
	}
	uint8_t BackBuffer::ds_stencil() const
	{
		return wnd().m_ds_props.DepthStencil.Stencil;
	}
}
