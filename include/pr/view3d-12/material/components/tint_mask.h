//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// Tint-mask state for a physically-based material.
	// The texture's R channel is a linear weight that limits where the appearance tint (the instance tint and the population instance tint) changes the base colour:
	// 1 = the tint colours the surface, 0 = the authored colour is kept. The tint alpha still applies everywhere. Without a texture, the tint applies everywhere.
	struct TintMask
	{
		static constexpr RdrId Id = hash::HashCT("materials::TintMask");
		TextureSlot m_tex = { {}, {}, {}, ETextureColourSpace::Linear, {} }; // Tint-mask texture.
	};
	static_assert(ComponentType<TintMask>);
}
