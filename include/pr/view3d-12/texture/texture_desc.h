//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/stock_resources.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	struct TextureDesc
	{
		RdrId    m_id;          // The id to assign to the created texture instance. Use 'AutoId' to auto generate an id.
		ResDesc  m_tdesc;       // A description of the texture to be created.
		RdrId    m_uri;         // An id for the source of this texture. Replaced by a hash of the resource path name for loaded textures
		bool     m_has_alpha;   // True if the texture contains alpha pixels and should be rendered in the alpha group
		string32 m_name;        // Debugging name for the texture. Replaced by the file name for loaded textures if empty

		TextureDesc()
			:m_id()
			,m_tdesc()
			,m_uri()
			,m_has_alpha()
			,m_name()
		{}
		TextureDesc(RdrId id, ResDesc const& td)
			:m_id(id)
			,m_tdesc(td)
			,m_uri()
			,m_has_alpha()
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
	};
}
