//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/terrain/water/water_field.h"

namespace pr::physics
{
	// A water environment that applies buoyancy and drag to every dynamic rigid body in the engine.
	// The water surface is a height field over world XY with +Z up. A point is wet when it is below the sampled water surface.
	// Rigid bodies cannot lie below the terrain surface, so a wet body point always has water above the terrain beneath it.
	// Each body uses one analytic volume proxy: spheres are exact, boxes are evaluated in cells against the local water plane,
	// and other shapes use their bounding box scaled to the true shape volume. Use either this or GpuBuoyancy for a body, not both.
	struct WaterConfig
	{
		// The sampled water surface.
		terrain::water::WaterField m_field;

		// Water density in kg/m³.
		float m_density = 1000.0f;

		// Exponential decay rate (1/s) of the body's linear velocity relative to the water when fully submerged.
		float m_linear_drag_rate = 0.5f;

		// Dimensionless drag coefficient for the force 0.5*rho*Cd*A*|v|*v, where A is the submerged volume^(2/3).
		float m_quadratic_drag_coefficient = 0.5f;

		// Exponential decay rate (1/s) of the body's angular velocity when fully submerged.
		float m_angular_drag_rate = 0.5f;

		// Throw invalid_argument unless every parameter is finite, the density is positive, and the drag values are non-negative.
		void Validate() const;
	};
}
