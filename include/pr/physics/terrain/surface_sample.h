//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"

namespace pr::physics::terrain
{
	// One deterministic sample of a double-precision terrain surface in XY metres.
	struct SurfaceSample
	{
		double m_height = 0.0;
		v2d m_gradient_xy = v2d::Zero();
		MaterialId m_material_id = 0;

		// Return the +Z-up unit normal implied by the sampled height derivatives.
		v4d Normal() const
		{
			return Normalise(v4d{-m_gradient_xy.x, -m_gradient_xy.y, 1.0, 0.0}, v4d::ZAxis());
		}
	};
}
