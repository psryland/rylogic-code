//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Stage-neutral per-element water-field evaluation shared by C++ and HLSL.
// Consumers own the element storage and loop over their active elements, adding each contribution to the still-water level.
// Every function is pure: no resources, registers, or dispatch intrinsics are used, so any shader stage can include this file.
#ifndef PR_PHYSICS_WATER_FIELD_HLSLI
#define PR_PHYSICS_WATER_FIELD_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"

#ifdef __cplusplus
namespace pr::physics::terrain::water::shared
{
#endif

// Surface displacement and normal contributions used to render a displaced water surface.
// displacement_foam.xyz is the vertex offset from the flat still-water position and .w is a foam weight.
// normal_delta accumulates the negative surface slope (-dh/dx, -dh/dy) in .xy and the Gerstner horizontal compression in .z,
// so the unnormalised surface normal is (normal_delta.x, normal_delta.y, 1 + normal_delta.z).
struct WaterFieldSurfaceSample
{
	float4 displacement_foam;
	float4 normal_delta;
};

// Differential state of one radial packet at a world-space point.
struct WaterFieldRadialPacketSample
{
	float height;
	float radial_height_gradient;
	float vertical_velocity;
	float radial_pressure_acceleration;
	float radial_velocity;
	float packet_envelope;
	float2 radial_direction;
};

// Return an empty surface sample before accumulating elements.
odr WaterFieldSurfaceSample WaterFieldSurfaceSampleZero()
{
#ifdef __cplusplus
	WaterFieldSurfaceSample sample = {};
#else
	WaterFieldSurfaceSample sample = (WaterFieldSurfaceSample)0;
#endif
	return sample;
}

// Return an empty radial-packet sample.
odr WaterFieldRadialPacketSample WaterFieldRadialPacketSampleZero()
{
#ifdef __cplusplus
	WaterFieldRadialPacketSample sample = {};
#else
	WaterFieldRadialPacketSample sample = (WaterFieldRadialPacketSample)0;
#endif
	return sample;
}

// Return a cubic smooth ramp over [0, duration] and its derivative with respect to the input value.
odr float2 WaterFieldSmoothRamp(float value, float duration)
{
	float safe_duration = max(duration, 1.0e-4f);
	float x = saturate(value / safe_duration);
	float ramp = x * x * (3.0f - 2.0f * x);
	float derivative = value > 0.0f && value < safe_duration ? 6.0f * x * (1.0f - x) / safe_duration : 0.0f;
	return float2(ramp, derivative);
}

// Evaluate a finite travelling radial packet and its analytic radial and time derivatives.
odr WaterFieldRadialPacketSample WaterFieldEvaluateRadialPacket(WaterFieldElement element, float2 world_xy)
{
	// Inactive or malformed packets contribute nothing.
	WaterFieldRadialPacketSample sample = WaterFieldRadialPacketSampleZero();
	float age = element.timing.x;
	float lifetime = element.timing.y;
	float amplitude = element.wave.x;
	float wavelength = element.wave.y;
	float packet_half_width = element.wave.z;
	float propagation_speed = element.wave.w;
	float attenuation_scale = element.timing.w;
	if (age < 0.0f || age >= lifetime || amplitude <= 0.0f || wavelength <= 0.0f || packet_half_width <= 0.0f || propagation_speed <= 0.0f || attenuation_scale <= 0.0f)
		return sample;

	float2 radial_offset = world_xy - element.position.xy;
	float radius = length(radial_offset);
	sample.radial_direction = radius > 1.0e-5f ? radial_offset / radius : float2(0.0f, 0.0f);

	// Compact support follows the outgoing front. Its first two derivatives reach zero at the packet edge, so overlapping packets add without seams.
	float q = radius - propagation_speed * age;
	float u = q / packet_half_width;
	if (abs(u) >= 1.0f)
		return sample;

	float one_minus_u2 = 1.0f - u * u;
	float packet_envelope = one_minus_u2 * one_minus_u2 * one_minus_u2;
	float packet_envelope_dr = -6.0f * u * one_minus_u2 * one_minus_u2 / packet_half_width;
	float packet_envelope_dt = -propagation_speed * packet_envelope_dr;

	// Attack and end-of-life ramps multiply so the packet appears and disappears smoothly, even while its support still covers the point.
	float fade_duration = min(max(element.timing.z, 0.25f * lifetime), 0.5f * lifetime);
	float2 attack = WaterFieldSmoothRamp(age, element.timing.z);
	float2 release_ramp = WaterFieldSmoothRamp(age - (lifetime - fade_duration), fade_duration);
	float release = 1.0f - release_ramp.x;
	float release_dt = -release_ramp.y;
	float temporal_fade = attack.x * release;
	float temporal_fade_dt = attack.y * release + attack.x * release_dt;

	// The square-root falloff stays finite at the source and has a simple analytic derivative.
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
	sample.vertical_velocity = amplitude * (temporal_fade_dt * spatial_profile * carrier + temporal_fade * (spatial_profile_dt * carrier + spatial_profile * carrier_dt));

	// The acceleration is converted to a bounded pressure-gradient contribution by the caller. Radial flow is softly limited for stability.
	sample.radial_pressure_acceleration = propagation_speed * propagation_speed * sample.radial_height_gradient;
	float raw_radial_velocity = amplitude * temporal_fade * spatial_profile * angular_frequency * sin(phase);
	float radial_velocity_limit = 0.25f * propagation_speed;
	sample.radial_velocity = raw_radial_velocity / (1.0f + abs(raw_radial_velocity) / radial_velocity_limit);
	sample.packet_envelope = temporal_fade * packet_envelope;
	return sample;
}

// Return the phase of a sine or Gerstner wave element. Both use k*dot(d, xy) plus a time term of angular frequency omega and a constant phase offset.
odr float WaterFieldWavePhase(WaterFieldElement element, float2 world_xy, float time)
{
	float k = tau / element.wave.y;
	float omega = element.info.x == WaterFieldElementGerstnerWave ? -k * element.wave.z : element.wave.z;
	return k * dot(element.position.xy, world_xy) + omega * time + element.position.z;
}

// Return the signed angular frequency used by WaterFieldWavePhase for a sine or Gerstner element.
odr float WaterFieldWaveAngularFrequency(WaterFieldElement element)
{
	float k = tau / element.wave.y;
	return element.info.x == WaterFieldElementGerstnerWave ? -k * element.wave.z : element.wave.z;
}

// Return the largest absolute height contribution this element can make at any point and time.
odr float WaterFieldElementAmplitudeBound(WaterFieldElement element)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		case WaterFieldElementRadialPacket:
		{
			// Attenuation, envelope, and fade factors of radial packets are all at most one.
			return abs(element.wave.x);
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Return one element's vertical height contribution at a world-space XY position.
odr float WaterFieldElementHeight(WaterFieldElement element, float2 world_xy, float time)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		{
			return element.wave.x * sin(WaterFieldWavePhase(element, world_xy, time));
		}
		case WaterFieldElementRadialPacket:
		{
			return WaterFieldEvaluateRadialPacket(element, world_xy).height;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Return one element's (height, dh/dx, dh/dy) contribution at a world-space XY position.
odr float3 WaterFieldElementHeightAndGradient(WaterFieldElement element, float2 world_xy, float time)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		{
			float k = tau / element.wave.y;
			float s, c;
			sincos(WaterFieldWavePhase(element, world_xy, time), s, c);
			float slope = element.wave.x * k * c;
			return float3(element.wave.x * s, element.position.x * slope, element.position.y * slope);
		}
		case WaterFieldElementRadialPacket:
		{
			WaterFieldRadialPacketSample packet = WaterFieldEvaluateRadialPacket(element, world_xy);
			return float3(packet.height, packet.radial_direction.x * packet.radial_height_gradient, packet.radial_direction.y * packet.radial_height_gradient);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

// Return one element's height and dimensionless lateral pressure-gradient contribution.
// Waves contribute A*omega^2/g*cos(phase), matching their orbital acceleration; this equals the geometric slope when omega^2 = g*k.
odr float3 WaterFieldElementHeightAndPressureGradient(WaterFieldElement element, float2 world_xy, float time, float gravity)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		{
			float omega = WaterFieldWaveAngularFrequency(element);
			float s, c;
			sincos(WaterFieldWavePhase(element, world_xy, time), s, c);
			float pressure_gradient = element.wave.x * omega * omega * c / gravity;
			return float3(element.wave.x * s, element.position.x * pressure_gradient, element.position.y * pressure_gradient);
		}
		case WaterFieldElementRadialPacket:
		{
			// The soft bound limits one packet to one unit of lateral pressure gradient without clipping its shape.
			WaterFieldRadialPacketSample packet = WaterFieldEvaluateRadialPacket(element, world_xy);
			float bounded_pressure_gradient = packet.radial_pressure_acceleration / (max(gravity, 1.0e-4f) + abs(packet.radial_pressure_acceleration));
			return float3(packet.height, packet.radial_direction.x * bounded_pressure_gradient, packet.radial_direction.y * bounded_pressure_gradient);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

// Return one element's water-particle velocity contribution at a world-space position.
// Waves use linear deep-water orbital flow that decays exponentially below the still-water level; points above the level use the surface value.
odr float3 WaterFieldElementVelocity(WaterFieldElement element, float3 world_pos, float time, float water_level)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		{
			float k = tau / element.wave.y;
			float omega = WaterFieldWaveAngularFrequency(element);
			float s, c;
			sincos(WaterFieldWavePhase(element, world_pos.xy, time), s, c);
			float depth = min(world_pos.z - water_level, 0.0f);
			float speed = element.wave.x * omega * exp(k * depth);
			return float3(-speed * s * element.position.x, -speed * s * element.position.y, speed * c);
		}
		case WaterFieldElementRadialPacket:
		{
			WaterFieldRadialPacketSample packet = WaterFieldEvaluateRadialPacket(element, world_pos.xy);
			return float3(packet.radial_direction.x * packet.radial_velocity, packet.radial_direction.y * packet.radial_velocity, packet.vertical_velocity);
		}
		default:
		{
			return float3(0.0f, 0.0f, 0.0f);
		}
	}
}

// Add one element's rendered displacement, foam, and normal contribution to a surface sample.
odr void WaterFieldAccumulateSurface(WaterFieldElement element, float2 world_xy, float time, inout_(WaterFieldSurfaceSample) sample)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		{
			float k = tau / element.wave.y;
			float s, c;
			sincos(WaterFieldWavePhase(element, world_xy, time), s, c);
			sample.displacement_foam.z += element.wave.x * s;
			sample.normal_delta.x -= element.position.x * k * element.wave.x * c;
			sample.normal_delta.y -= element.position.y * k * element.wave.x * c;
			break;
		}
		case WaterFieldElementGerstnerWave:
		{
			// Steepness moves vertices towards crests, which sharpens them and compresses the normal. The height is A*sin(phase), so a
			// horizontal offset of +Q*A*cos(phase) along the travel direction pulls points on both sides of a crest (sin = 1) towards it.
			float2 direction = element.position.xy;
			float amplitude = element.wave.x;
			float steepness = element.wave.w;
			float k = tau / element.wave.y;
			float s, c;
			sincos(WaterFieldWavePhase(element, world_xy, time), s, c);
			sample.displacement_foam.x += steepness * amplitude * direction.x * c;
			sample.displacement_foam.y += steepness * amplitude * direction.y * c;
			sample.displacement_foam.z += amplitude * s;
			sample.displacement_foam.w += steepness * k * amplitude * s;
			sample.normal_delta.x -= direction.x * k * amplitude * c;
			sample.normal_delta.y -= direction.y * k * amplitude * c;
			sample.normal_delta.z -= steepness * k * amplitude * s;
			break;
		}
		case WaterFieldElementRadialPacket:
		{
			WaterFieldRadialPacketSample packet = WaterFieldEvaluateRadialPacket(element, world_xy);
			sample.displacement_foam.z += packet.height;
			sample.displacement_foam.w += packet.packet_envelope * saturate(abs(packet.radial_height_gradient) * 0.35f);
			sample.normal_delta.xy -= packet.radial_direction * packet.radial_height_gradient;
			break;
		}
		default:
		{
			break;
		}
	}
}

#ifdef __cplusplus
}
#endif
#endif
