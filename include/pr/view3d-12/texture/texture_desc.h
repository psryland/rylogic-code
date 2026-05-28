//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/stock_resources.h"

namespace pr::rdr12
{
	struct TextureDesc
	{
		using ResDesc = ::pr::compute::ResDesc;

		RdrId       m_id;          // The id to assign to the created texture instance. Use 'AutoId' to auto generate an id.
		ResDesc     m_rdesc;       // A description of the resource to be created.
		RdrId       m_uri;         // An id for the source of this texture. Replaced by a hash of the resource path name for loaded textures
		bool        m_has_alpha;   // True if the texture contains alpha pixels and should be rendered in the alpha group
		DXGI_FORMAT m_srv_format;  // Optional SRV format override. If UNKNOWN, the resource format is used. Use this to cast UNORM <-> UNORM_SRGB on the SRV.
		DXGI_FORMAT m_rtv_format;  // Optional RTV format override. If UNKNOWN, the resource format is used. Use this to cast UNORM <-> UNORM_SRGB on the RTV.
		string32    m_name;        // Debugging name for the texture. Replaced by the file name for loaded textures if empty

		TextureDesc()
			:m_id()
			,m_rdesc()
			,m_uri()
			,m_has_alpha()
			,m_srv_format(DXGI_FORMAT_UNKNOWN)
			,m_rtv_format(DXGI_FORMAT_UNKNOWN)
			,m_name()
		{}
		TextureDesc(RdrId id, ResDesc const& td)
			:m_id(id)
			,m_rdesc(td)
			,m_uri()
			,m_has_alpha()
			,m_srv_format(DXGI_FORMAT_UNKNOWN)
			,m_rtv_format(DXGI_FORMAT_UNKNOWN)
			,m_name()
		{}

		TextureDesc& name(std::string_view name)
		{
			m_name = name;
			return *this;
		}
		TextureDesc& uri(EStockTexture id)
		{
			return uri(s_cast<RdrId>(id));
		}
		TextureDesc& uri(RdrId id)
		{
			m_uri = id;
			return *this;
		}
		TextureDesc& has_alpha(bool has_alpha = true)
		{
			m_has_alpha = has_alpha;
			return *this;
		}

		// Override the SRV format. Useful when the SRV needs to cast between UNORM and UNORM_SRGB siblings of the resource format.
		TextureDesc& srv_format(DXGI_FORMAT format)
		{
			m_srv_format = format;
			return *this;
		}

		// Override the RTV format. Useful when the RTV needs to cast between UNORM and UNORM_SRGB siblings of the resource format so
		// the GPU performs linear -> sRGB encoding on write while the resource stays in a non-SRGB format.
		TextureDesc& rtv_format(DXGI_FORMAT format)
		{
			m_rtv_format = format;
			return *this;
		}
	};
}
