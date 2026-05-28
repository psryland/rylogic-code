//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/alpha.h"
#include "pr/view3d-12/material/components/base_colour.h"
#include "pr/view3d-12/material/components/emissive.h"
#include "pr/view3d-12/material/components/metallic.h"
#include "pr/view3d-12/material/components/normal_map.h"
#include "pr/view3d-12/material/components/roughness.h"
#include "pr/view3d-12/material/components/two_sided.h"
#include "pr/view3d-12/material/material.h"

namespace pr::rdr12
{
	// Built-in physically-based metallic-roughness material.
	struct MaterialPBR : Material
	{
		static constexpr RdrId MaterialTypeId = hash::HashCT("MaterialPBR");

		materials::BaseColour m_base_colour; // Base-colour properties.
		materials::Metallic m_metallic;      // Metallic properties.
		materials::Roughness m_roughness;    // Roughness properties.
		materials::Emissive m_emissive;      // Emissive properties.
		materials::NormalMap m_normal_map;   // Normal-map properties.
		materials::Alpha m_alpha;            // Alpha behaviour.
		materials::TwoSided m_two_sided;     // Two-sided lighting state.

		MaterialPBR();
		MaterialPBR(MaterialPBR const& rhs);
		MaterialPBR(MaterialPBR&&) = delete;
		MaterialPBR& operator =(MaterialPBR const&) = delete;
		MaterialPBR& operator =(MaterialPBR&&) = delete;
		using Material::Component;

		// Return the extensible type id for this material.
		RdrId TypeId() const override;

		// Return the PBR material pass for supported render steps.
		MaterialPass const* Pass(ERenderStep step) const override;

		// Create a mutable copy of this material instance.
		virtual RefPtr<Material> Clone() const override;

		// Return true if this material requires alpha rendering.
		virtual bool RequiresAlpha() const override;

		// Return the material colour that should be folded into the shared nugget tint constant.
		Colour TintColour() const override;

		// Set the linear base-colour factor.
		MaterialPBR& base_colour(Colour colour);
		MaterialPBR& base_colour(Colour32 colour);
		MaterialPBR& base_texture(Texture2DPtr tex, SamplerPtr sam);
		MaterialPBR& base_texture(materials::TextureSlot slot);

		// Set the metallic factor.
		MaterialPBR& metallic(float value);
		MaterialPBR& metallic_texture(materials::ScalarTextureSlot slot);

		// Set the roughness factor.
		MaterialPBR& roughness(float value);
		MaterialPBR& roughness_texture(materials::ScalarTextureSlot slot);

		// Set the linear emissive factor.
		MaterialPBR& emissive(Colour colour);
		MaterialPBR& emissive_texture(materials::TextureSlot slot);

		// Set the texture slot used for tangent-space normals.
		MaterialPBR& normal_texture(materials::TextureSlot slot, float scale = 1.0f);

		// Set the alpha interpretation for this material.
		MaterialPBR& alpha_mode(materials::EAlphaMode mode, float cutoff = 0.5f);

		// Get/Set whether back-facing pixels should flip their lit surface normal.
		bool two_sided() const;
		MaterialPBR& two_sided(bool enabled = true);

	protected:

		// Return a component block for 'component_id', or null if this material does not provide that block.
		void const* Component(RdrId component_id) const override;

		// Delete this PBR material instance.
		void Delete() override;
	};
	static_assert(MaterialType<MaterialPBR>);
}
