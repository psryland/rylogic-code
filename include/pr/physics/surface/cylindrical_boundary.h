//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/surface/forward.h"
#include "pr/physics/materials/material.h"

namespace pr::physics
{
	// Inward-facing, infinite-height cylinder. Distances are metres; contacts depend on current geometry, not speed or step size.
	struct CylindricalBoundaryConfig
	{
		double m_centre_x = 0;
		double m_centre_y = 0;
		double m_radius = 4000;
		int m_material_id = 0;
		float m_surface_spacing = 0.05f;

		// Validate external geometry and sample representability before constructing the GPU source.
		void Validate() const
		{
			auto const coordinate = std::max(std::abs(m_centre_x), std::abs(m_centre_y)) + m_radius;
			if (!std::isfinite(m_centre_x) || !std::isfinite(m_centre_y) || !std::isfinite(m_radius) || m_radius <= 0 ||
				!std::isfinite(m_surface_spacing) || m_surface_spacing <= 0 ||
				m_material_id < 0 || m_material_id >= Material::MaxMaterialId ||
				m_surface_spacing < 16 * std::numeric_limits<float>::epsilon() * coordinate)
				throw std::runtime_error("Invalid cylindrical boundary geometry, material or unrepresentable sample spacing");
		}
	};
}
