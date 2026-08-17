//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Pure per-element evaluation for the LaS water field.
#ifndef LAS_WATER_FIELD_HLSLI
#define LAS_WATER_FIELD_HLSLI
#include "pr/hlsl/core.hlsli"
#include "src/world/water/shaders/water_field_types.hlsli"

#ifdef __cplusplus
namespace pr::hlsl
{
	using namespace las::water;
#endif

// Contributions needed to displace and orient a sampled water surface.
struct WaterFieldSample
{
	float4 displacement_foam;
	float4 normal_delta;
};

// Full differential state of one compact circular packet at a world-space point.
struct StoneDropSample
{
	float height;
	float radial_height_gradient;
	float vertical_velocity;
	float radial_pressure_acceleration;
	float radial_velocity;
	float packet_envelope;
	float2 radial_direction;
};

// Construct an empty field sample before accumulating elements.
odr WaterFieldSample WaterFieldSampleZero()
{
#ifdef __cplusplus
	WaterFieldSample sample = {};
#else
	WaterFieldSample sample = (WaterFieldSample)0;
#endif
	return sample;
}

// Construct an empty stone-drop sample before accumulating elements.
odr StoneDropSample StoneDropSampleZero()
{
#ifdef __cplusplus
	StoneDropSample sample = {};
#else
	StoneDropSample sample = (StoneDropSample)0;
#endif
	return sample;
}

// Return a cubic smooth ramp and its derivative with respect to the input value.
odr float2 SmoothRampWithDerivative(float value, float duration)
{
	float safe_duration = max(duration, 1.0e-4f);
	float x = saturate(value / safe_duration);
	float ramp = x * x * (3.0f - 2.0f * x);
	float derivative = value > 0.0f && value < safe_duration
		? 6.0f * x * (1.0f - x) / safe_duration
		: 0.0f;

	return float2(ramp, derivative);
}

// Evaluate a finite travelling packet and its analytical radial/time derivatives.
odr StoneDropSample EvaluateStoneDrop(WaterFieldElement element, float2 world_xy)
{
	StoneDropSample sample = StoneDropSampleZero();

	float age = element.timing.x;
	float lifetime = element.timing.y;
	float amplitude = element.wave.x;
	float wavelength = element.wave.y;
	float packet_half_width = element.wave.z;
	float propagation_speed = element.wave.w;
	float attenuation_scale = element.timing.w;
	if (age < 0.0f || age >= lifetime || amplitude <= 0.0f || wavelength <= 0.0f ||
		packet_half_width <= 0.0f || propagation_speed <= 0.0f || attenuation_scale <= 0.0f)
		return sample;

	float2 radial_offset = world_xy - element.position.xy;
	float radius = length(radial_offset);
	sample.radial_direction = radius > 1.0e-5f ? radial_offset / radius : float2(0.0f, 0.0f);

	// Compact support follows the outgoing front. Its first two derivatives reach zero at the packet edge, so overlapping events superpose without seams.
	float q = radius - propagation_speed * age;
	float u = q / packet_half_width;
	if (abs(u) >= 1.0f)
		return sample;

	float one_minus_u2 = 1.0f - u * u;
	float packet_envelope = one_minus_u2 * one_minus_u2 * one_minus_u2;
	float packet_envelope_dr = -6.0f * u * one_minus_u2 * one_minus_u2 / packet_half_width;
	float packet_envelope_dt = -propagation_speed * packet_envelope_dr;

	// Attack and end-of-life ramps are multiplied so the event appears and disappears smoothly even when the travelling support still crosses the sample point.
	float fade_duration = min(max(element.timing.z, 0.25f * lifetime), 0.5f * lifetime);
	float2 attack = SmoothRampWithDerivative(age, element.timing.z);
	float2 release_ramp = SmoothRampWithDerivative(age - (lifetime - fade_duration), fade_duration);
	float release = 1.0f - release_ramp.x;
	float release_dt = -release_ramp.y;
	float temporal_fade = attack.x * release;
	float temporal_fade_dt = attack.y * release + attack.x * release_dt;

	// The square-root falloff stays finite at the source and has a simple analytical derivative.
	float attenuation = rsqrt(1.0f + radius / attenuation_scale);
	float attenuation_dr = -0.5f * attenuation * attenuation * attenuation / attenuation_scale;
	float wave_number = tau / wavelength;
	float angular_frequency = wave_number * propagation_speed;
	float phase = wave_number * q;
	float carrier = sin(phase);
	float carrier_dr = wave_number * cos(phase);
	float carrier_dt = -angular_frequency * cos(phase);

	float spatial_profile = attenuation * packet_envelope;
	float spatial_profile_dr = attenuation_dr * packet_envelope + attenuation * packet_envelope_dr;
	float spatial_profile_dt = attenuation * packet_envelope_dt;
	sample.height = amplitude * temporal_fade * spatial_profile * carrier;
	sample.radial_height_gradient = amplitude * temporal_fade * (spatial_profile_dr * carrier + spatial_profile * carrier_dr);
	sample.vertical_velocity = amplitude *
		(temporal_fade_dt * spatial_profile * carrier + temporal_fade * (spatial_profile_dt * carrier + spatial_profile * carrier_dt));

	// This acceleration is converted to a bounded hydrostatic-gradient contribution below. Radial flow is deliberately conservative for stability.
	sample.radial_pressure_acceleration = propagation_speed * propagation_speed * sample.radial_height_gradient;
	float raw_radial_velocity = amplitude * temporal_fade * spatial_profile * angular_frequency * sin(phase);
	float radial_velocity_limit = 0.25f * propagation_speed;
	sample.radial_velocity = raw_radial_velocity / (1.0f + abs(raw_radial_velocity) / radial_velocity_limit);
	sample.packet_envelope = temporal_fade * packet_envelope;
	return sample;
}

// Return the shared Gerstner phase used by every water-field consumer.
odr float GerstnerWavePhase(WaterFieldElement element, float2 world_xy, float time)
{
	float k = tau / element.wave.y;
	return k * dot(element.position.xy, world_xy) - k * element.wave.z * time;
}

// Add one Gerstner component to a water-field sample.
odr void AccumulateGerstnerWave(WaterFieldElement element, float2 world_xy, float time, inout_(WaterFieldSample) sample)
{
	float2 direction = element.position.xy;
	float amplitude = element.wave.x;
	float wavelength = element.wave.y;
	float steepness = element.wave.w;

	float k = 6.283185307f / wavelength;
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
odr void AccumulateWaterFieldElement(WaterFieldElement element, float2 world_xy, float time, inout_(WaterFieldSample) sample)
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
			StoneDropSample stone = EvaluateStoneDrop(element, world_xy);
			sample.displacement_foam.z += stone.height;
			sample.displacement_foam.w += stone.packet_envelope * saturate(abs(stone.radial_height_gradient) * 0.35f);
			sample.normal_delta.xy -= stone.radial_direction * stone.radial_height_gradient;
			break;
		}
		default:
		{
			break;
		}
	}
}

// Evaluate one typed field element's vertical surface contribution.
odr float EvaluateWaterFieldHeightElement(WaterFieldElement element, float2 world_xy, float time)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			return element.wave.x * sin(GerstnerWavePhase(element, world_xy, time));
		}
		case WaterFieldElementTypeStoneDrop:
		{
			return EvaluateStoneDrop(element, world_xy).height;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Evaluate one typed field element's height and lateral pressure-gradient contribution.
odr float3 EvaluateWaterFieldHeightAndPressureGradientElement(WaterFieldElement element, float2 world_xy, float time, float gravity)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			float k = tau / element.wave.y;
			float angular_speed = -k * element.wave.z;
			float s, c;
			sincos(GerstnerWavePhase(element, world_xy, time), s, c);
			float height = element.wave.x * s;
			float2 pressure_gradient = element.position.xy * (element.wave.x * angular_speed * angular_speed * c / gravity);
			return float3(height, pressure_gradient);
		}
		case WaterFieldElementTypeStoneDrop:
		{
			StoneDropSample stone = EvaluateStoneDrop(element, world_xy);

			// The soft bound limits an individual packet to one unit of lateral pressure gradient without discontinuously clipping its shape.
			float bounded_pressure_gradient = stone.radial_pressure_acceleration /
				(max(gravity, 1.0e-4f) + abs(stone.radial_pressure_acceleration));
			return float3(stone.height, stone.radial_direction * bounded_pressure_gradient);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

// Evaluate one typed field element's water-particle velocity contribution.
odr float3 EvaluateWaterFieldVelocityElement(WaterFieldElement element, float3 world_pos, float time, float water_level)
{
	switch (element.info.x)
	{
		case WaterFieldElementTypeGerstnerWave:
		{
			float k = tau / element.wave.y;
			float angular_speed = -k * element.wave.z;
			float s, c;
			sincos(GerstnerWavePhase(element, world_pos.xy, time), s, c);
			float depth = min(world_pos.z - water_level, 0.0f);
			float speed = element.wave.x * angular_speed * exp(k * depth);
			return float3((-speed * s) * element.position.xy, speed * c);
		}
		case WaterFieldElementTypeStoneDrop:
		{
			StoneDropSample stone = EvaluateStoneDrop(element, world_pos.xy);
			return float3(stone.radial_direction * stone.radial_velocity, stone.vertical_velocity);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

#ifdef __cplusplus
}
#endif
#endif
