//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/materials/material.h"

namespace pr::physics
{
	// Abstract interface for looking up combined material properties.
	// When two bodies collide, the engine queries the material map with both
	// material IDs to get the combined properties (elasticity, friction) that
	// govern the collision response.
	struct IMaterials
	{
		virtual ~IMaterials() = default;

		// Return the combined material for two bodies in contact.
		// The result merges the individual material properties (e.g. averaging
		// coefficients of restitution, taking the minimum friction, etc.)
		virtual Material operator()(int id0, int id1) const = 0;
	};
}
