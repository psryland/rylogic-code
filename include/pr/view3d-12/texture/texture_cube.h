//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/texture/texture_base.h"

namespace pr::rdr12
{
	struct TextureCube :TextureBase
	{
		// Notes:
		//  - A cube texture is basically just a special case 2d texture.
		//  - The cube texture should look like:
		//            Top
		//     Left  Front  Right  Back
		//           Bottom
		
		// Cube map to world orientation. Only directions are transformed, so this must be an orthonormal basis. It may be a rotation or a
		// mirror; a mirror maps right-handed world directions onto the left-handed DX cube face layout.
		m4x4 m_cube2w;

		TextureCube(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc);
	};
}
