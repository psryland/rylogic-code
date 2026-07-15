//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Adapts the LaS field element and evaluator to the generic GPU-buoyancy shader contract.
#ifndef LAS_WATER_FIELD_BUOYANCY_HLSLI
#define LAS_WATER_FIELD_BUOYANCY_HLSLI
#include "src/world/water/shaders/water_field.hlsli"

#define GpuBuoyancyWaterFieldElement WaterFieldElement

// Evaluate one LaS field element's vertical surface contribution.
float GpuBuoyancyEvaluateWaterHeightElement(WaterFieldElement element, float2 xy_ws, float time_s)
{
	return EvaluateWaterFieldHeightElement(element, xy_ws, time_s);
}

// Evaluate one LaS field element's height and pressure-gradient contribution.
float3 GpuBuoyancyEvaluateWaterHeightAndPressureGradientElement(WaterFieldElement element, float2 xy_ws, float time_s, float gravity)
{
	return EvaluateWaterFieldHeightAndPressureGradientElement(element, xy_ws, time_s, gravity);
}

// Evaluate one LaS field element's water-particle velocity contribution.
float3 GpuBuoyancyEvaluateWaterVelocityElement(WaterFieldElement element, float3 pos_ws, float time_s, float water_level)
{
	return EvaluateWaterFieldVelocityElement(element, pos_ws, time_s, water_level);
}

#endif
