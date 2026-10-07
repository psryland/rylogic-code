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

		// The world-space position the cube's contents were captured from. Reflections march from it to correct parallax when the
		// scene's environment map parallax bounds are valid (see Scene::m_global_envmap_parallax_bounds).
		v4 m_centre;

		// The scale 'S' of the distances stored in 'm_distance', or 0 if the cube has no distances. A texel's distance 'd' from 'm_centre' is
		// stored as 'd / (d + S)', so 1 means infinitely distant. Reflections compare these distances with points on the reflected ray.
		float m_distance_scale;

		// A single-channel 16-bit cube with the same size and mips as this cube, holding each texel's distance from 'm_centre'.
		// Null until a capture stores distances.
		TextureCubePtr m_distance;

		TextureCube(Renderer& rdr, ID3D12Resource* res, TextureDesc const& desc);
	};
}
