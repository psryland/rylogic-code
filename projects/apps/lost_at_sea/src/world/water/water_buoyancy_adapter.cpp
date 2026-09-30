//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/forward.h"
#include "src/world/water/water_system.h"
#include "src/world/water/water_buoyancy_adapter.h"

namespace las::water
{
	// Copy one immutable LaS field snapshot into GPU buoyancy for the matching simulation time.
	void BuoyancyAdapter::SetField(physics::GpuBuoyancy& buoyancy, Snapshot const& snapshot, double simulation_time_s)
	{
		// The physics step must consume the same immutable field value that rendering received for this simulation time.
		if (!std::isfinite(simulation_time_s) || snapshot.m_time_s != static_cast<float>(simulation_time_s))
			throw std::invalid_argument("Water snapshot time must match the physics simulation time");
		if (std::ssize(snapshot.Elements()) > MaxWaterFieldElementCount)
			throw std::invalid_argument("Water snapshot element count is outside the fixed field capacity");
		if (!std::isfinite(snapshot.WaterLevel()))
			throw std::invalid_argument("Water snapshot level must be finite");

		buoyancy.SetWaterField(snapshot.m_field);
	}
}
