//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/surface/forward.h"
#include "pr/physics/materials/material.h"

namespace pr::physics
{
	// Inward-facing, infinite-height cylinder. Distances are metres; motion limits apply only in XY.
	struct CylindricalBoundaryConfig
	{
		double m_centre_x = 0;
		double m_centre_y = 0;
		double m_radius = 4000;
		int m_material_id = 0;
		float m_surface_spacing = 0.05f;
		float m_max_substep_motion = 0.1f;

		// Gross rejection cutoff, not a startup/restore allowance or an acceptable resting overlap.
		float m_max_penetration = 0.25f;

		// Reject configurations that cannot leave a representable sampling and discrete-motion safety margin.
		void Validate() const
		{
			auto const coordinate = std::max(std::abs(m_centre_x), std::abs(m_centre_y)) + m_radius;
			if (!std::isfinite(m_centre_x) || !std::isfinite(m_centre_y) || !std::isfinite(m_radius) || m_radius <= 0 ||
				!std::isfinite(m_surface_spacing) || m_surface_spacing <= 0 ||
				!std::isfinite(m_max_substep_motion) || m_max_substep_motion <= 0 ||
				!std::isfinite(m_max_penetration) || m_max_penetration <= 0 ||
				m_material_id < 0 || m_material_id >= Material::MaxMaterialId ||
				coordinate > 1e6 || m_surface_spacing < 16 * std::numeric_limits<float>::epsilon() * coordinate ||
				2 * m_surface_spacing + m_max_substep_motion > m_max_penetration || m_max_penetration >= m_radius * 0.01)
				throw std::runtime_error("Invalid cylindrical boundary geometry, material, sampling or motion envelope");
		}
	};
}
