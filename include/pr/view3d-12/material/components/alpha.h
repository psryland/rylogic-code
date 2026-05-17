//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// The alpha interpretation for a PBR material.
	enum class EAlphaMode
	{
		Opaque,
		Mask,
		Blend,
	};

	// Alpha state for a physically-based material.
	struct Alpha
	{
		static constexpr RdrId Id = hash::HashCT("materials::PbrAlpha");

		EAlphaMode m_mode = EAlphaMode::Opaque; // How alpha from the material should be interpreted.
		float m_cutoff = 0.5f;                        // Cutoff for mask alpha mode.

		// Return true if this material needs the sorted alpha rendering path.
		bool RequiresAlpha() const
		{
			switch (m_mode)
			{
				case EAlphaMode::Opaque:
				case EAlphaMode::Mask:
				{
					return false;
				}
				case EAlphaMode::Blend:
				{
					return true;
				}
				default:
				{
					throw std::runtime_error("Unknown PBR alpha mode");
				}
			}
		}
	};
	static_assert(ComponentType<Alpha>);
}

