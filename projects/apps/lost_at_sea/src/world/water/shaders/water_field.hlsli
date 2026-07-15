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

// Return the shared Gerstner phase used by every water-field consumer.
float GerstnerWavePhase(WaterFieldElement element, float2 world_xy, float time)
{
	float k = 6.283185307 / element.wave.y;
	return k * dot(element.position.xy, world_xy) - k * element.wave.z * time;
}

// Add one Gerstner component to a water-field sample.
void AccumulateGerstnerWave(WaterFieldElement element, float2 world_xy, float time, inout WaterFieldSample sample)
{
	float2 direction = element.position.xy;
	float amplitude = element.wave.x;
	float wavelength = element.wave.y;
	float steepness = element.wave.w;

	float k = 6.283185307 / wavelength;
	float phase = GerstnerWavePhase(element, world_xy, time);
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

// Evaluate one typed field element's vertical surface contribution.
float EvaluateWaterFieldHeightElement(WaterFieldElement element, float2 world_xy, float time)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			return element.wave.x * sin(GerstnerWavePhase(element, world_xy, time));
		}
		case WaterFieldElementTypeStoneDrop:
		{
			return 0.0f;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Evaluate one typed field element's height and lateral pressure-gradient contribution.
float3 EvaluateWaterFieldHeightAndPressureGradientElement(WaterFieldElement element, float2 world_xy, float time, float gravity)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			float k = 6.283185307 / element.wave.y;
			float angular_speed = -k * element.wave.z;
			float s, c;
			sincos(GerstnerWavePhase(element, world_xy, time), s, c);
			float height = element.wave.x * s;
			float2 pressure_gradient = element.position.xy * (element.wave.x * angular_speed * angular_speed * c / gravity);
			return float3(height, pressure_gradient);
		}
		case WaterFieldElementTypeStoneDrop:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

// Evaluate one typed field element's water-particle velocity contribution.
float3 EvaluateWaterFieldVelocityElement(WaterFieldElement element, float3 world_pos, float time, float water_level)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			float k = 6.283185307 / element.wave.y;
			float angular_speed = -k * element.wave.z;
			float s, c;
			sincos(GerstnerWavePhase(element, world_pos.xy, time), s, c);
			float depth = min(world_pos.z - water_level, 0.0f);
			float speed = element.wave.x * angular_speed * exp(k * depth);
			return float3((-speed * s) * element.position.xy, speed * c);
		}
		case WaterFieldElementTypeStoneDrop:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

#endif
