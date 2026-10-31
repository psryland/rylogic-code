//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Properties of the air that surrounds simulated bodies. Force modules that act on the parts of a body exposed to the air
	// read these values, so all of them see the same air. The air is currently still (no wind).
	struct Air
	{
		// Density of the air (kg/m^3). The default is sea-level air at about 15 C. Zero removes all air forces.
		float m_density = 1.2f;
	};
}
