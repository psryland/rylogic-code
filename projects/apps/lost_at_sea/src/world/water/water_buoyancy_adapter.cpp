//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/forward.h"
#include "src/world/water/water_system.h"
#include "src/world/water/water_buoyancy_adapter.h"

namespace las::water
{
	// Return the shader and fixed-stride contract used to specialise GPU buoyancy for LaS.
	physics::GpuBuoyancy::WaterFieldExtension BuoyancyAdapter::Extension()
	{
		static_assert(sizeof(WaterFieldElement) == 64);
		static_assert(alignof(WaterFieldElement) == 16);
		return physics::GpuBuoyancy::WaterFieldExtension{
			.m_shader_include = "src/world/water/shaders/water_field_buoyancy.hlsli",
			.m_element_stride = sizeof(WaterFieldElement),
		};
	}

	// Copy one immutable LaS field snapshot into GPU buoyancy for the matching simulation time.
	void BuoyancyAdapter::SetField(physics::GpuBuoyancy& buoyancy, Snapshot const& snapshot, double simulation_time_s)
	{
		if (!std::isfinite(simulation_time_s) || snapshot.m_time_s != static_cast<float>(simulation_time_s))
			throw std::invalid_argument("Water snapshot time must match the physics simulation time");
		if (snapshot.m_element_count < 0 || snapshot.m_element_count > MaxWaterFieldElementCount)
			throw std::invalid_argument("Water snapshot element count is outside the fixed field capacity");
		if (!std::isfinite(snapshot.m_water_level))
			throw std::invalid_argument("Water snapshot level must be finite");

		auto elements = snapshot.Elements();
		buoyancy.SetWaterField(std::as_bytes(elements), static_cast<int>(elements.size()), snapshot.m_water_level);
	}
}
