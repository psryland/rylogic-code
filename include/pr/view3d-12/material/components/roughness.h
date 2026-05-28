//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Roughness state for a physically-based material.
	struct Roughness
	{
		static constexpr RdrId Id = hash::HashCT("materials::PbrRoughness");

		float m_factor = 1.0f;      // Roughness factor.
		ScalarTextureSlot m_tex = { // Roughness scalar map.
			{{}, {}, {}, ETextureColourSpace::Linear, 0},
			ETextureChannel::Green
		};
	};
	static_assert(ComponentType<Roughness>);
}

