//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once

#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/lighting/light.h"

namespace pr::rdr12
{
	struct ProjectionParams
	{
		m4x4 m_l2w;  // The position of the light in world space
		m4x4 m_w2ls; // The transform from world space to (perspective skewed) light space
		m4x4 m_ls2s; // The projection from light space to the shadow map

		BBox m_bounds;
	};
	struct ShadowCaster
	{
		ProjectionParams m_params; // Projection parameters
		Light m_light;             // The shadow casting light (world space)
		Texture2DPtr m_smap;       // The texture containing the shadow map
		int m_size;                // The dimensions of the (square) shadow map

		ShadowCaster(Texture2DPtr smap, int size);

		// Update the projection parameters for the given scene
		void UpdateParams(Scene const& scene, BBox ws_bounds);
	};
}
