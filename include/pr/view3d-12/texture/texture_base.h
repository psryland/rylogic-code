//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/descriptor.h"

namespace pr::rdr12
{
	// Notes:
	//   - A base class for all renderer texture instances.
	//   - TextureBase (and derived objects) are 'lightweight-ish', they are basically reference
	//     counted pointers+data to D3D resources. Think Instance (TextureBase) of a Model (D3D resource).
	//   - TextureBase does not have value semantics, but is clone-able.
	//   - Each time CreateTexture is called, a new texture instance is allocated, however, the resources
	//     associated with the texture may be shared with other textures.
	//
	// The structure looks like this:
	//                 TextureBase
	//                 +----------+
	//    +------------+-res, t2s | <------ TexturePtr
	//    V            | SRV, UAV |
	// +-----+         | RTV, Id  | <------ TexturePtr
	// | D3D |         | etc      |
	// | Res |         +----------+
	// +-----+                    
	//    ^            TextureBase
	//    |            +----------+
	//    +------------+-res, t2s | <------ TexturePtr
	//                 | SRV, UAV | <------ TexturePtr
	//                 | RTV, Id  |
	//                 | etc      | <------ TexturePtr
	//                 +----------+

	// Flags for Textures
	enum class ETextureFlag :int
	{
		None = 0,

		// The texture contains alpha pixels
		HasAlpha = 1 << 0,

		_flags_enum = 0,
	};

	// Texture base class
	struct TextureBase :RefCounted<TextureBase>
	{
		Renderer*              m_rdr;    // The renderer that owns this texture
		D3DPtr<ID3D12Resource> m_res;    // The texture resource (possibly shared with other Texture instances)
		Descriptor             m_srv;    // Shader resource view (if available)
		Descriptor             m_uav;    // Unordered access view (if available)
		Descriptor             m_rtv;    // Render target view (if available)
		Descriptor             m_dsv;    // Depth stencil view (if available)
		RdrId                  m_id;     // Id for this texture in the resource manager
		RdrId                  m_uri;    // An id identifying the source this texture was created from (needed when deleting the last ref to a DX tex)
		iv3                    m_dim;    // The dimensions of the texture
		ETextureFlag           m_tflags; // Flags for boolean properties of the texture
		string32               m_name;   // Human readable id for the texture

		TextureBase(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc);
		TextureBase(Renderer& rdr, HANDLE shared_handle, TextureDesc const& desc);
		TextureBase(Renderer& rdr, IUnknown* shared_resource, TextureDesc const& desc);
		TextureBase(TextureBase&&) = delete;
		TextureBase(TextureBase const&) = delete;
		TextureBase& operator =(TextureBase&&) = delete;
		TextureBase& operator =(TextureBase const&) = delete;
		virtual ~TextureBase();

		// Access the renderer
		Renderer& rdr() const;

		// A sort key component for this texture
		SortKeyId SortId() const;

		// Access the texture flags
		ETextureFlag Flags() const;

		// True if this texture contains alpha pixels
		bool Alpha() const;

		// Get the description of the texture resource
		ResDesc TexDesc() const;

		// Resize this texture to 'size'
		void Resize(uint64_t width, uint32_t height, uint16_t depth_or_array_len);

		// Return the shared handle associated with this texture
		HANDLE SharedHandle() const;

		// Get the DXGI surface within this texture
		D3DPtr<IDXGISurface> GetSurface();

		// Delegates to call when the texture is destructed
		// WARNING: Don't add lambdas that capture a ref counted pointer to the texture
		// or the texture will never get destructed, since the ref will never hit zero.
		EventHandler<TextureBase&, EmptyArgs const&, true> OnDestruction;

		// Ref counting clean up
		static void RefCountZero(RefCounted<TextureBase>* doomed);
		protected: virtual void Delete();
	};

	// Validate a texture pointer
	void Validate(TextureBase const* texture);
}
