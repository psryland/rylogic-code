//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"

namespace pr::rdr12
{
	Texture2D::Texture2D(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc)
		:TextureBase(rdr, res, desc)
		,m_t2s(m4x4::Identity())
	{}
	Texture2D::Texture2D(Renderer& rdr, HANDLE shared_handle, TextureDesc const& desc)
		:TextureBase(rdr, shared_handle, desc)
		,m_t2s(m4x4::Identity())
	{}
	Texture2D::Texture2D(Renderer& rdr, IUnknown* shared_resource, TextureDesc const& desc)
		:TextureBase(rdr, shared_resource, desc)
		,m_t2s(m4x4::Identity())
	{}

	// Get the GDI DC from the surface
	HDC Texture2D::GetDC(bool discard)
	{
		HDC dc;
		D3DPtr<IDXGISurface2> surf;
		pr::Check(m_res->QueryInterface(__uuidof(IDXGISurface2), (void**)surf.address_of()));
		pr::Check(surf->GetDC(discard, &dc), "GetDC can only be called for textures that are GDI compatible");
//		++m_mgr->m_gdi_dc_ref_count;
		return dc;
	}

	// Release the GDI DC from the surface
	void Texture2D::ReleaseDC()
	{
		D3DPtr<IDXGISurface2> surf;
		pr::Check(m_res->QueryInterface(__uuidof(IDXGISurface2), (void**)surf.address_of()));
		pr::Check(surf->ReleaseDC(nullptr));
//		--m_mgr->m_gdi_dc_ref_count;
		// Note: the main RT must be restored once all ReleaseDC's have been called
	}

	// Get a d2d render target for the DXGI surface within this texture
	D3DPtr<ID2D1RenderTarget> Texture2D::GetD2DRenderTarget(Window const* wnd)
	{
		auto surf = GetSurface();

		// Create render target properties
		auto props = D2D1::RenderTargetProperties(D2D1_RENDER_TARGET_TYPE_DEFAULT, D2D1::PixelFormat(DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_PREMULTIPLIED));
		auto dpi = wnd ? wnd->Dpi() : rdr().SystemDpi();
		props.dpiX = dpi.x;
		props.dpiY = dpi.y;

		// Create a D2D render target which can draw into our off screen D3D surface.
		D3DPtr<ID2D1RenderTarget> rt;
		auto d2dfactory = rdr().D2DFactory();
		Check(d2dfactory->CreateDxgiSurfaceRenderTarget(surf.m_ptr, props, rt.address_of()));
		return rt;
	}
}
