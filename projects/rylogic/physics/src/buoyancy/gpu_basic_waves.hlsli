//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Built-in sine-wave implementation of the GPU-buoyancy water-field contract.
#ifndef PR_PHYSICS_GPU_BASIC_WAVES_HLSLI
#define PR_PHYSICS_GPU_BASIC_WAVES_HLSLI

struct GpuBuoyancyWaterFieldElement
{
	float4 direction_wavelength_phase_speed;
	float4 amplitude;
};

// Evaluate one built-in sine-wave element's height contribution.
float GpuBuoyancyEvaluateWaterHeightElement(GpuBuoyancyWaterFieldElement element, float2 xy_ws, float time_s)
{
	float2 direction = element.direction_wavelength_phase_speed.xy;
	float wavelength = element.direction_wavelength_phase_speed.z;
	float phase_speed = element.direction_wavelength_phase_speed.w;
	float amplitude = element.amplitude.x;
	float phase = dot(direction, xy_ws) * tau / wavelength + phase_speed * time_s;
	return amplitude * sin(phase);
}

// Evaluate one built-in sine-wave element's height and pressure-gradient contribution.
float3 GpuBuoyancyEvaluateWaterHeightAndPressureGradientElement(GpuBuoyancyWaterFieldElement element, float2 xy_ws, float time_s, float gravity)
{
	float2 direction = element.direction_wavelength_phase_speed.xy;
	float wavelength = element.direction_wavelength_phase_speed.z;
	float phase_speed = element.direction_wavelength_phase_speed.w;
	float amplitude = element.amplitude.x;
	float phase = dot(direction, xy_ws) * tau / wavelength + phase_speed * time_s;
	float s, c;
	sincos(phase, s, c);
	return float3(amplitude * s, direction * (amplitude * phase_speed * phase_speed * c / gravity));
}

// Evaluate one built-in sine-wave element's orbital-velocity contribution.
float3 GpuBuoyancyEvaluateWaterVelocityElement(GpuBuoyancyWaterFieldElement element, float3 pos_ws, float time_s, float water_level)
{
	float2 direction = element.direction_wavelength_phase_speed.xy;
	float wavelength = element.direction_wavelength_phase_speed.z;
	float phase_speed = element.direction_wavelength_phase_speed.w;
	float amplitude = element.amplitude.x;
	float k = tau / wavelength;
	float phase = dot(direction, pos_ws.xy) * k + phase_speed * time_s;
	float s, c;
	sincos(phase, s, c);
	float depth = min(pos_ws.z - water_level, 0.0f);
	float speed = amplitude * phase_speed * exp(k * depth);
	return float3((-speed * s) * direction, speed * c);
}

#endif
