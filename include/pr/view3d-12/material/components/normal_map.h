//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Tangent-space normal-map state for a physically-based material.
	struct NormalMap
	{
		static constexpr RdrId Id = hash::HashCT("materials::NormalMap");

		TextureSlot m_tex = {{}, {}, ETextureColourSpace::Linear}; // Tangent-space normal map, inactive until tangent support exists.
	};
	static_assert(ComponentType<NormalMap>);
}

