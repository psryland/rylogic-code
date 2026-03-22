//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once

namespace pr::physics
{
	struct EngineConfig
	{
		// Maximum number of collision pairs that the engine can handle per frame.
		int max_collision_pairs = 65536;
		
		// Sleep thresholds: if a body has linear velocity below 'sleep_velocity_threshold_lin' and
		// angular velocity below 'sleep_velocity_threshold_ang' for some time, it can be put to sleep.
		// These thresholds are in world units (e.g., m/s for linear, rad/s for angular).
		float sleep_velocity_threshold_lin = 0.1f;
		float sleep_velocity_threshold_ang = 0.05f;
	};
}
