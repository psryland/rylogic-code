//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Base-colour state for a physically-based material.
	struct BaseColour
	{
		static constexpr RdrId Id = hash::HashCT("materials::BaseColour");

		Colour m_colour = ColourWhite;                                     // Linear base-colour factor.
		TextureSlot m_tex = { {}, {}, {}, ETextureColourSpace::Srgb, {} }; // Base-colour texture, sampled as colour data.
	};
	static_assert(ComponentType<BaseColour>);
}

