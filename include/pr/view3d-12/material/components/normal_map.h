//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"

namespace pr::rdr12::materials
{
	// How a normal map's samples are interpreted.
	enum class ENormalMapSpace
	{
		// X/Y perturb the interpolated vertex normal in a tangent frame derived from the map's UVs.
		Tangent,

		// X/Y are model-space normal components and Z is rebuilt as non-negative. The map replaces the vertex normal, so it suits height
		// fields whose normals all face model +Z. Geometry must still have vertex normals.
		Model,
	};

	// Normal-map state for a physically-based material.
	struct NormalMap
	{
		static constexpr RdrId Id = hash::HashCT("materials::NormalMap");
		TextureSlot m_tex = { {}, {}, {}, ETextureColourSpace::Linear, {} }; // Normal map texture.
		float m_scale = 1.0f;                                                // Multiplier for the X/Y components.
		ENormalMapSpace m_space = ENormalMapSpace::Tangent;                  // How the map's samples are interpreted.
	};
	static_assert(ComponentType<NormalMap>);
}
