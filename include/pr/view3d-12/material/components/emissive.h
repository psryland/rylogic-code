//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Emissive state for a physically-based material.
	struct Emissive
	{
		static constexpr RdrId Id = hash::HashCT("materials::Emissive");

		Colour m_colour = ColourZero;                                          // Linear emissive factor.
		TextureSlot m_tex = {{}, {}, ETextureColourSpace::Srgb};        // Emissive texture, sampled as colour data.
	};
	static_assert(ComponentType<Emissive>);
}

