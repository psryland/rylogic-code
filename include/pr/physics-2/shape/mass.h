//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"

namespace pr::physics
{
	// Mass properties of a collision shape
	struct MassProperties
	{
		// Notes:
		//  This class is used as a return type when calculating inertia
		//  for Shapes. It's not used in the RigidBody object.
		//  All values are in model space.

		// Normalised object space inertia, expressed at the object centre of mass.
		m3x4 m_os_unit_inertia;

		// Offset to the centre of mass from the model space origin (note: w = 0)
		v4 m_centre_of_mass;

		// Mass
		float m_mass;

		MassProperties()
			:m_os_unit_inertia(v4{limits<float>::infinity()}, v4{limits<float>::infinity()}, v4{limits<float>::infinity()})
			,m_centre_of_mass(v4{})
			,m_mass(limits<float>::infinity())
		{}
		MassProperties(m3x4 const& os_unit_inertia, v4 centre_of_mass, float mass)
			:m_os_unit_inertia(os_unit_inertia)
			,m_centre_of_mass(centre_of_mass)
			,m_mass(mass)
		{}
	};
}
