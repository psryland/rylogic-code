//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"

namespace pr::rdr12::materials
{
	// Diffuse/base-colour material state shared by fixed-function style render paths.
	struct BaseColour
	{
		static constexpr RdrId Id = hash::HashCT("materials::BaseColour");

		Texture2DPtr m_tex_diffuse = {}; // Diffuse texture.
		SamplerPtr m_sam_diffuse = {};   // Sampler to use with the diffuse texture.
		Colour32 m_tint = Colour32White; // Per-material tint.
	};
	static_assert(ComponentType<BaseColour>);
}

