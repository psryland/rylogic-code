//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "physics/utility/Assert.h"
#include "physics/utility/Debug.h"
#include "pr/Physics/Engine/IGravity.h"

using namespace pr;
using namespace pr::ph;

namespace pr
{
	namespace ph
	{
		// A global gravity field interface pointer
		NoGravity g_default_gravity_interface;
		ph::IGravity const* g_gravity_interface = &g_default_gravity_interface;

	}//namespace ph
}//namespace pr

// Assign the gravity field interface to use.
// This must remain in scope for the lifetime of the physics engine
void pr::ph::RegisterGravityField(ph::IGravity const* gravity_interface)
{
	g_gravity_interface = gravity_interface;
}

// Return the gravity experienced at 'position'
v4 pr::ph::GetGravitationalAcceleration(v4 position)
{
	return g_gravity_interface->GravityField(position);
}

// Returns the potential energy of a position in the gravity field
float pr::ph::GetGravitationalPotential(v4 position)
{
	return g_gravity_interface->GravityPotential(position);
}
