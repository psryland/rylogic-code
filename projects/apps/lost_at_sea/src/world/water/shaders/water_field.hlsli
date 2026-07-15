//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Pure per-element evaluation for the LaS water field.
#ifndef LAS_WATER_FIELD_HLSLI
#define LAS_WATER_FIELD_HLSLI
#include "src/world/water/shaders/water_field_types.hlsli"

// Contributions needed to displace and orient a sampled water surface.
struct WaterFieldSample
{
	float4 displacement_foam;
	float4 normal_delta;
};

// Construct an empty field sample before accumulating elements.
WaterFieldSample WaterFieldSampleZero()
{
	WaterFieldSample sample = (WaterFieldSample)0;
	return sample;
}

// Add one Gerstner component to a water-field sample.
void AccumulateGerstnerWave(WaterFieldElement element, float2 world_xy, float time, inout WaterFieldSample sample)
{
	float2 direction = element.position.xy;
	float amplitude = element.wave.x;
	float wavelength = element.wave.y;
	float speed = element.wave.z;
	float steepness = element.wave.w;

	float k = 6.283185307 / wavelength;
	float phase = k * dot(direction, world_xy) - k * speed * time;
	float s, c;
	sincos(phase, s, c);

	sample.displacement_foam.x -= steepness * amplitude * direction.x * c;
	sample.displacement_foam.y -= steepness * amplitude * direction.y * c;
	sample.displacement_foam.z += amplitude * s;
	sample.displacement_foam.w += steepness * k * amplitude * s;

	sample.normal_delta.x -= direction.x * k * amplitude * c;
	sample.normal_delta.y -= direction.y * k * amplitude * c;
	sample.normal_delta.z -= steepness * k * amplitude * s;
}

// Add one typed field element to a water-field sample.
void AccumulateWaterFieldElement(WaterFieldElement element, float2 world_xy, float time, inout WaterFieldSample sample)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			AccumulateGerstnerWave(element, world_xy, time, sample);
			break;
		}
		case WaterFieldElementTypeStoneDrop:
		{
			// Stone-drop evaluation is introduced with the event-lifecycle phase.
			break;
		}
		default:
		{
			break;
		}
	}
}

#endif
