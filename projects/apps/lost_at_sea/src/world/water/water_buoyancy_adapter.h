//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"
#include "src/world/water/shaders/water_field_types.hlsli"

namespace las::water
{
	// Connects LaS field snapshots to the generic physics GPU-buoyancy extension.
	struct BuoyancyAdapter
	{
		// Return the shader and fixed-stride contract used to specialise GPU buoyancy for LaS.
		static physics::GpuBuoyancy::WaterFieldExtension Extension();

		// Copy one immutable LaS field snapshot into GPU buoyancy.
		static void SetField(physics::GpuBuoyancy& buoyancy, std::span<WaterFieldElement const> elements, float water_level);
	};
}
