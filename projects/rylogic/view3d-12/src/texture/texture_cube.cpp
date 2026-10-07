//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/texture/texture_cube.h"

namespace pr::rdr12
{
	TextureCube::TextureCube(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc)
		:TextureBase(rdr, res, desc)
		,m_cube2w(m4x4::Identity())
		,m_centre(v4::Origin())
		,m_distance_scale()
		,m_distance()
	{}
}