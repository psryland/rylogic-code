//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// Selects the coordinate frame used to evaluate a procedural surface.
	enum class EProceduralCoordinateSpace
	{
		Object,
		World,
	};

	// GPU-evaluated surface parameters shared by all procedural material presets.
	struct ProceduralSurface
	{
		// Component state is fully caller-owned and contains no preset identity.

		static constexpr RdrId Id = hash::HashCT("ProceduralSurface");

		bool m_enabled = false; // Disabled components leave ordinary material channels unchanged.
		EProceduralCoordinateSpace m_coordinate_space = EProceduralCoordinateSpace::World;
		uint32_t m_seed = 0;
		float m_feature_scale = 1.0f;
		v4 m_coordinate_origin = v4::Origin();
		std::array<Colour, 4> m_palette = {ColourWhite, ColourWhite, ColourWhite, ColourWhite};
		v4 m_axis_scale = v4{1, 1, 1, 0};
		float m_normal_strength = 0.0f;
		float m_roughness_min = 0.5f;
		float m_roughness_max = 1.0f;
		float m_detail = 0.5f;
		float m_warp = 0.0f;

		// Validate the caller-owned coordinate and channel ranges.
		void Validate() const;
	};
	static_assert(ComponentType<ProceduralSurface>);
}
