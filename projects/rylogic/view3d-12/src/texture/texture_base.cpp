//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/texture/texture_base.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/resource/descriptor_store.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/render/sortkey.h"
#include "pr/view3d-12/utility/utility.h"

namespace pr::rdr12
{
	// Validate a texture pointer
	void Validate(TextureBase const* texture)
	{
		if (texture == nullptr)
			throw std::runtime_error("Texture pointer is null");
	}

	// Get the shared handle from a shared resource
	HANDLE SharedHandleFromSharedResource(IUnknown* shared_resource)
	{
		// Get the DXGI resource interface for the shared resource
		D3DPtr<IDXGIResource> dxgi_resource;
		Check(shared_resource->QueryInterface<IDXGIResource>(dxgi_resource.address_of()));

		// Get the handled of the shared resource so that we can open it with our d3d device
		HANDLE shared_handle;
		Check(dxgi_resource->GetSharedHandle(&shared_handle));
		return shared_handle;
	}

	// Constructors
	TextureBase::TextureBase(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc)
		: RefCounted<TextureBase>()
		, m_rdr(&rdr)
		, m_res(res, true)
		, m_srv()
		, m_uav()
		, m_rtv()
		, m_dsv()
		, m_id(desc.m_id == AutoId ? MakeId(this) : desc.m_id)
		, m_uri(desc.m_uri)
		, m_dim(s_cast<int>(desc.m_tdesc.Width), s_cast<int>(desc.m_tdesc.Height), s_cast<int>(desc.m_tdesc.DepthOrArraySize))
		, m_tflags(desc.m_has_alpha ? ETextureFlag::HasAlpha : ETextureFlag::None)
		, m_name(desc.m_name)
	{
		auto device = rdr.d3d();
		auto tdesc = desc.m_tdesc;

		// Create views for the texture
		if (AllSet(tdesc.Flags, D3D12_RESOURCE_FLAG_DENY_SHADER_RESOURCE) == false)
		{
			// Check the texture format is supported
			D3D12_FEATURE_DATA_FORMAT_SUPPORT support = {tdesc.Format};
			Check(device->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support)));
			if (!AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_SHADER_SAMPLE))
				throw std::runtime_error("Texture format is not supported as a shader resource view");

			// Create the SRV
			ResourceStore::Access store(rdr);
			D3D12_SHADER_RESOURCE_VIEW_DESC srv_desc = {
				.Format = tdesc.Format,
				.ViewDimension = desc.m_tdesc.SrvDimension(),
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.Texture2D = {
					.MostDetailedMip = 0,
					.MipLevels = s_cast<UINT>(-1),
					.PlaneSlice = 0,
					.ResourceMinLODClamp = 0.f,
				},
			};
			m_srv = store.Descriptors().Create(res, srv_desc);
		}
		if (AllSet(tdesc.Flags, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS))
		{
			// Check the texture format is supported
			D3D12_FEATURE_DATA_FORMAT_SUPPORT support = {tdesc.Format};
			Check(device->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support)));
			if (!AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_TYPED_UNORDERED_ACCESS_VIEW) ||
				!AllSet(support.Support2, D3D12_FORMAT_SUPPORT2_UAV_TYPED_LOAD) ||
				!AllSet(support.Support2, D3D12_FORMAT_SUPPORT2_UAV_TYPED_STORE))
				throw std::runtime_error("Texture format is not supported as a unordered access view");

			// Create the UAV
			ResourceStore::Access store(rdr);
			D3D12_UNORDERED_ACCESS_VIEW_DESC uav_desc = {
				.Format = tdesc.Format,
				.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D,
				.Texture2D = {
					.MipSlice = 0,
					.PlaneSlice = 0,
				},
			};
			m_uav = store.Descriptors().Create(res, uav_desc);
		}
		if (AllSet(tdesc.Flags, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET))
		{
			D3D12_FEATURE_DATA_FORMAT_SUPPORT support = {tdesc.Format};
			Check(device->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support)));
			if (!AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_RENDER_TARGET))
				throw std::runtime_error("Texture format is not supported as a render target view");

			// Create the RTV
			ResourceStore::Access store(rdr);
			D3D12_RENDER_TARGET_VIEW_DESC rtv_desc = {
				.Format = tdesc.Format,
				.ViewDimension = desc.m_tdesc.RtvDimension(),
			};
			m_rtv = store.Descriptors().Create(res, rtv_desc);
		}
		if (AllSet(tdesc.Flags, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL))
		{
			D3D12_FEATURE_DATA_FORMAT_SUPPORT support = {tdesc.Format};
			Check(device->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT, &support, sizeof(support)));
			if (!AllSet(support.Support1, D3D12_FORMAT_SUPPORT1_DEPTH_STENCIL))
				throw std::runtime_error("Texture format is not supported as a depth stencil view");

			// Create the DSV
			ResourceStore::Access store(rdr);
			D3D12_DEPTH_STENCIL_VIEW_DESC dsv_desc = {
				.Format = tdesc.Format,
				.ViewDimension = desc.m_tdesc.DsvDimension(),
				.Flags = D3D12_DSV_FLAGS::D3D12_DSV_FLAG_NONE,
			};
			m_dsv = store.Descriptors().Create(res, dsv_desc);
		}
	}
	TextureBase::TextureBase(Renderer& rdr, HANDLE shared_handle, TextureDesc const& desc)
		: TextureBase(rdr, static_cast<ID3D12Resource*>(nullptr), desc)
	{
		// Open the shared resource in our d3d device
		D3DPtr<IUnknown> resource;
		Check(rdr.d3d()->OpenSharedHandle(shared_handle, __uuidof(ID3D12Resource), (void**)resource.address_of()));

		// Query the resource interface from the resource
		Check(resource->QueryInterface(__uuidof(ID3D12Resource), (void**)m_res.address_of()));
	}
	TextureBase::TextureBase(Renderer& rdr, IUnknown* shared_resource, TextureDesc const& desc)
		: TextureBase(rdr, SharedHandleFromSharedResource(shared_resource), desc)
	{
	}
	TextureBase::~TextureBase()
	{
		OnDestruction(*this, EmptyArgs());

		// Release any views
		{
			ResourceStore::Access store(rdr());
			if (m_srv) store.Descriptors().Release(m_srv);
			if (m_uav) store.Descriptors().Release(m_uav);
			if (m_rtv) store.Descriptors().Release(m_rtv);
			if (m_dsv) store.Descriptors().Release(m_dsv);
		}
	}

	// Access the renderer
	Renderer& TextureBase::rdr() const
	{
		return *m_rdr;
	}

	// A sort key component for this texture
	SortKeyId TextureBase::SortId() const
	{
		return m_id % SortKey::MaxTextureId;
	}
	
	// Access the texture flags
	ETextureFlag TextureBase::Flags() const
	{
		return m_tflags;
	}

	// True if this texture contains alpha pixels
	bool TextureBase::Alpha() const
	{
		return AllSet(m_tflags, ETextureFlag::HasAlpha);
	}

	// Get the description of the texture resource
	ResDesc TextureBase::TexDesc() const
	{
		auto desc = m_res->GetDesc();
		return ResDesc{
			{desc},
		};
	}

	// Resize this texture to 'size'
	void TextureBase::Resize(uint64_t width, uint32_t height, uint16_t depth_or_array_len)
	{
		PR_ASSERT(PR_DBG_RDR, width*height != 0, "Do not resize textures to 0x0");
		
		auto tdesc = m_res->GetDesc();
		tdesc.Width = width;
		tdesc.Height = height;
		tdesc.DepthOrArraySize = depth_or_array_len;
		throw std::runtime_error("Not implemented");
	}

	// Return the shared handle associated with this texture
	HANDLE TextureBase::SharedHandle() const
	{
		HANDLE handle;
		D3DPtr<IDXGIResource> res;
		Check(m_res->QueryInterface(__uuidof(IDXGIResource), (void**)res.address_of()));
		Check(res->GetSharedHandle(&handle));
		return handle;
	}
	
	// Get the DXGI surface within this texture
	D3DPtr<IDXGISurface> TextureBase::GetSurface()
	{
		D3DPtr<IDXGISurface> surf;
		Check(m_res->QueryInterface(surf.address_of()));
		return surf;
	}

	// Ref counting clean up function
	void TextureBase::RefCountZero(RefCounted<TextureBase>* doomed)
	{
		auto tex = static_cast<TextureBase*>(doomed);
		tex->Delete();
	}
	void TextureBase::Delete()
	{
		rdr().DeferRelease(m_res);
		ResourceStore::Access store(rdr());
		store.Delete(this);
	}
}