//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"

namespace las::water
{
	// Connects LaS field snapshots to the generic physics GPU-buoyancy water field.
	struct BuoyancyAdapter
	{
		// Copy one immutable LaS field snapshot into GPU buoyancy for the matching simulation time.
		static void SetField(physics::GpuBuoyancy& buoyancy, Snapshot const& snapshot, double simulation_time_s);
	};
}
