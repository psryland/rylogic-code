//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	struct Material
	{
		// Notes:
		//  The interaction between elasticity and friction is described in the impulse restitution function.

		static int constexpr NoID = -1;
		static int constexpr DefaultID = 0;
		static int constexpr MaxMaterialId = 32; // Can easily be increased if needed

		int m_id = DefaultID;           // Unique id for this material
		float m_friction_static = 1.0f; // Static friction: 0 = no friction, 1 = infinite friction
		float m_elasticity_norm = 1.0f; // Elasticity in the collision normal direction: [0,+1]
		float m_elasticity_tang = 1.0f; // Elasticity in the collision tangential direction: [-1,+1]
		float m_elasticity_tors = 0.0f; // Angular elasticity in the collision normal direction: 1 = elastic, 0 = inelastic
		float m_density = 1000.0f;      // Material density in kg/m^3

		// Rolling resistance: dimensionless, >= 0. Limits the angular impulse that opposes relative rolling (rotation about axes perpendicular
		// to the contact normal) to 'm_rolling_resistance * normal_impulse * r', where 'r' is the contact-to-centre-of-mass distance along the normal.
		// It acts like a rolling friction coefficient: a round body rests on a slope whose tangent is less than this value. 0 = no rolling resistance.
		float m_rolling_resistance = 0.0f;

		// Spin resistance: dimensionless, >= 0. Limits the angular impulse that opposes relative spin about the contact normal, using the same
		// 'normal_impulse * r' scale as rolling resistance. 0 = no spin resistance.
		float m_spin_resistance = 0.0f;

		// Merge the properties of two contacting materials
		static Material Merge(Material const& mat0, Material const& mat1)
		{
			return Material{
				.m_id = Material::NoID,
				.m_friction_static = Sqrt(mat0.m_friction_static * mat1.m_friction_static),
				.m_elasticity_norm = (mat0.m_elasticity_norm + mat1.m_elasticity_norm) * 0.5f,
				.m_elasticity_tang = (mat0.m_elasticity_tang + mat1.m_elasticity_tang) * 0.5f,
				.m_elasticity_tors = (mat0.m_elasticity_tors + mat1.m_elasticity_tors) * 0.5f,
				.m_density = (mat0.m_density + mat1.m_density) * 0.5f,
				.m_rolling_resistance = std::max(mat0.m_rolling_resistance, mat1.m_rolling_resistance),
				.m_spin_resistance = std::max(mat0.m_spin_resistance, mat1.m_spin_resistance),
			};
		}
	};
}
