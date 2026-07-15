//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/forward.h"
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

	// Copy one immutable LaS field snapshot into GPU buoyancy.
	void BuoyancyAdapter::SetField(physics::GpuBuoyancy& buoyancy, std::span<WaterFieldElement const> elements, float water_level)
	{
		buoyancy.SetWaterField(std::as_bytes(elements), static_cast<int>(elements.size()), water_level);
	}
}
