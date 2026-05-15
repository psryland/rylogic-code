//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/texture/d2d_context.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12
{
	using namespace ::pr::compute;

	// RAII Scope for a wrapped Dx12 resource
	D2D1Context::D2D1Context(Renderer& rdr, ID3D12Resource* res)
		:m_dc()
		,m_dx11_res()
		,m_dx11_dc(rdr.Dx11DeviceContext())
		,m_dx11(rdr.Dx11Device())
	{
		// Create a d2d device context to access the d2d drawing commands
		Check(rdr.D2DDevice()->CreateDeviceContext(D2D1_DEVICE_CONTEXT_OPTIONS_NONE, m_dc.address_of()));

		// Need to use blend source over to use ClearType fonts
		m_dc->SetPrimitiveBlend(D2D1_PRIMITIVE_BLEND_SOURCE_OVER);

		// Set the properties of the bitmap surface we will get 
		v2 dpi; m_dc->GetDpi(&dpi.x, &dpi.y);

		// Create a Dx11 resource that wraps the Dx12 resource
		// If this fails, check that 'res' was created with 'D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET',
		// 'D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS', and 'D3D12_HEAP_FLAG_SHARED'.
		D3D11_RESOURCE_FLAGS flags = {
			.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE,
			.MiscFlags = D3D11_RESOURCE_MISC_SHARED,
			.CPUAccessFlags = 0U,
			.StructureByteStride = 0U,
		};
		auto default_state = DefaultResState(res);
		Check(m_dx11->CreateWrappedResource(res, &flags, default_state, D3D12_RESOURCE_STATE_RENDER_TARGET, __uuidof(ID3D11Resource), (void**)m_dx11_res.address_of()));

		// Get a DXGI surface from the wrapped resource
		D3DPtr<IDXGISurface> surf;
		Check(m_dx11_res->QueryInterface<IDXGISurface>(surf.address_of()));
		DebugName(surf, FmtS("%s-dx11", DebugName(res)));

		// Create a bitmap wrapper for 'surf'
		D3DPtr<ID2D1Bitmap1> target;
		D2D1_BITMAP_PROPERTIES1 bmp_props = {
			.pixelFormat = {
				.format = DXGI_FORMAT_B8G8R8A8_UNORM,
				.alphaMode = D2D1_ALPHA_MODE_PREMULTIPLIED,
			},
			.dpiX = dpi.x,
			.dpiY = dpi.y,
			.bitmapOptions = D2D1_BITMAP_OPTIONS_TARGET | D2D1_BITMAP_OPTIONS_CANNOT_DRAW,
			.colorContext = nullptr,
		};
		Check(m_dc->CreateBitmapFromDxgiSurface(surf.get(), &bmp_props, target.address_of()));

		// Acquire the texture as the current render target.
		// Transitions the texture from the 'InState' to a render target state
		ID3D11Resource* arr[] = {m_dx11_res.get()};
		m_dx11->AcquireWrappedResources(&arr[0], 1U);

		// Set 'target' as the d2d render target
		m_dc->SetTarget(target.get());
	}
	D2D1Context::~D2D1Context()
	{
		if (m_dx11_res == nullptr)
			return;
	
		// Return the texture to the "OutState"
		ID3D11Resource* arr[] = {m_dx11_res.get()};
		m_dx11->ReleaseWrappedResources(&arr[0], 1U);

		// Push the commands to the dx12 command queue
		m_dx11_dc->Flush();
	}
}
