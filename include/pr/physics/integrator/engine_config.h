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
		bool sleeping_enabled = true;
		float sleep_velocity_threshold_lin = 0.25f;
		float sleep_velocity_threshold_ang = 1.50f;
		float sleep_delay_s = 1.0f;

		// Number of coloured Gauss-Seidel contact solver iterations.
		int solver_iterations = 8;

		// Number of coloured split-position solver iterations.
		int position_iterations = 4;

		// Number of Temporal Gauss-Seidel resolver substeps. A value of 1 gives the classic
		// single-timestep PGS-style solve; larger values advance and solve contacts in smaller temporal slices.
		int tgs_steps = 1;

		// Velocity-level Baumgarte bias for shallow penetrations.
		float penetration_slop = 0.005f;
		float velocity_baumgarte = 0.2f;
		float tgs_velocity_bias_max = 2.0f;

		// Position-level split correction. This moves bodies only; momenta are unchanged.
		float position_slop = 0.005f;
		float position_baumgarte = 0.2f;

		// Existing deep-penetration positional correction parameters.
		float deep_penetration_threshold = 0.3f;
		float deep_penetration_range = 0.4f;
		float deep_penetration_baumgarte_min = 0.2f;
		float deep_penetration_baumgarte_max = 0.8f;
	};
}
