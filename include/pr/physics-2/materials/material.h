//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"

namespace pr::physics
{
	struct Material
	{
		// Notes:
		//  The interaction between elasticity and friction is described in the impulse restitution function.

		static int const NoID = -1;
		static int const DefaultID = 0;

		int m_id;                // Unique id for this material
		float m_density;         // Material density in kg/m^3
		float m_friction_static; // Static friction: 0 = no friction, 1 = infinite friction
		float m_elasticity_norm; // Elasticity in the collision normal direction: [0,+1]
		float m_elasticity_tang; // Elasticity in the collision tangential direction: [-1,+1]
		float m_elasticity_tors; // Angular elasticity in the collision normal direction: 1 = elastic, 0 = inelastic

		Material(
			int id                = DefaultID,
			float friction_static = 1.0f,
			float elasticity_norm = 1.0f,
			float elasticity_tang = 1.0f,
			float elasticity_tors = 0.0f,
			float density         = 1000.0f)
			:m_id(id)
			,m_density(density)
			,m_friction_static(friction_static)
			,m_elasticity_norm(elasticity_norm)
			,m_elasticity_tang(elasticity_tang)
			,m_elasticity_tors(elasticity_tors)
		{}

		// Merge the properties of two contacting materials
		static Material Merge(Material const& mat0, Material const& mat1)
		{
			return Material(
				Material::NoID,
				Sqrt(mat0.m_friction_static * mat1.m_friction_static),
				(mat0.m_elasticity_norm + mat1.m_elasticity_norm) * 0.5f,
				(mat0.m_elasticity_tang + mat1.m_elasticity_tang) * 0.5f,
				(mat0.m_elasticity_tors + mat1.m_elasticity_tors) * 0.5f,
				(mat0.m_density + mat1.m_density) * 0.5f);
		}
	};
}
