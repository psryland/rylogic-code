//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Metallic state for a physically-based material.
	struct Metallic
	{
		static constexpr RdrId Id = hash::HashCT("materials::Metallic");

		float m_factor = 0.0f;      // Metallic factor.
		ScalarTextureSlot m_tex = { // Metallic scalar map.
			{{}, {}, {}, ETextureColourSpace::Linear, 0},
			ETextureChannel::Blue
		};
	};
	static_assert(ComponentType<Metallic>);
}

