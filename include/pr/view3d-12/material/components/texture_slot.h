//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/sampler/sampler.h"

namespace pr::rdr12::materials
{
	// The colour-space interpretation requested by a material texture slot.
	enum class ETextureColourSpace
	{
		Linear,
		Srgb,
	};

	// Texture channels used when scalar PBR values are packed into shared textures.
	enum class ETextureChannel
	{
		Red,
		Green,
		Blue,
		Alpha,
	};

	// A texture and sampler together with the material slot's colour-space contract.
	struct TextureSlot
	{
		Texture2DPtr m_texture = {};                                      // Texture used by this slot.
		SamplerPtr m_sampler = {};                                        // Sampler used by this slot.
		ETextureColourSpace m_colour_space = ETextureColourSpace::Linear; // How shader sampling should interpret texture values.
		explicit operator bool() const
		{
			return m_texture != nullptr && m_sampler != nullptr;
		}
	};

	// A scalar texture slot that can read one channel from a packed map.
	struct ScalarTextureSlot
	{
		TextureSlot m_slot = {};                          // Texture slot containing the scalar value.
		ETextureChannel m_channel = ETextureChannel::Red; // Channel containing the scalar value.
	};
}

