//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"

namespace las::water
{
	// Connects LaS field snapshots to the physics engine's water configuration.
	struct BuoyancyAdapter
	{
		// Copy one immutable LaS field snapshot into the water configuration for the matching simulation time.
		static void SetField(physics::WaterConfig& water, Snapshot const& snapshot, double simulation_time_s);
	};
}
