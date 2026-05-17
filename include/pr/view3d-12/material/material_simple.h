//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/base_colour.h"
#include "pr/view3d-12/material/components/optics.h"
#include "pr/view3d-12/material/components/reflectivity.h"
#include "pr/view3d-12/material/components/shader_overlays.h"
#include "pr/view3d-12/material/components/two_sided.h"
#include "pr/view3d-12/material/material.h"

namespace pr::rdr12
{
	// Built-in material that preserves the default NuggetDesc material behaviour.
	struct MaterialSimple : Material
	{
		static constexpr RdrId MaterialTypeId = hash::HashCT("MaterialSimple");

		materials::BaseColour m_base_colour;     // Diffuse/base-colour properties.
		materials::Reflectivity m_reflectivity;  // Reflection properties.
		materials::ShaderOverlays m_shaders;     // Shader overlays used by this material.
		materials::TwoSided m_two_sided;         // Two-sided lighting state.
		materials::Optics m_optics;              // Optical properties used by RT paths.

		MaterialSimple(Colour32 tint = Colour32White, Texture2DPtr tex_diffuse = {}, SamplerPtr sam_diffuse = {}, float rel_reflec = 1.0f);
		MaterialSimple(MaterialSimple const& rhs);
		MaterialSimple(MaterialSimple&&) = delete;
		MaterialSimple& operator =(MaterialSimple const&) = delete;
		MaterialSimple& operator =(MaterialSimple&&) = delete;
		using Material::Component;

		// Return the extensible type id for this material.
		RdrId TypeId() const override;

		// Return the simple material pass for supported render steps.
		MaterialPass const* Pass(ERenderStep step) const override;

		// Create a mutable copy of this material instance.
		virtual RefPtr<Material> Clone() const override;

		// Return true if this material requires alpha rendering
		virtual bool RequiresAlpha() const override;

		// Set the base colour.
		MaterialSimple& base_colour(Colour colour);
		MaterialSimple& base_colour(Colour32 colour);
		MaterialSimple& base_texture(Texture2DPtr tex, SamplerPtr sam);

		// Set the relative reflectivity.
		MaterialSimple& rel_reflec(float reflectivity);

		// Set whether back-facing pixels should flip their lit surface normal.
		MaterialSimple& two_sided(bool enabled = true);

		// Add a shader overlay to this material.
		MaterialSimple& use_shader_overlay(ERenderStep step, ShaderPtr overlay);

	protected:

		// Return a component block for 'component_id', or null if this material does not provide that block.
		void const* Component(RdrId component_id) const override;

		// Delete this simple material instance.
		void Delete() override;
	};
	static_assert(MaterialType<MaterialSimple>);
}
