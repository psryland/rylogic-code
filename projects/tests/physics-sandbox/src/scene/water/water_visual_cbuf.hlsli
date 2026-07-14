//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared water-visual constant-buffer layout.
#ifndef PHYSICS_SANDBOX_WATER_VISUAL_CBUF_HLSLI
#define PHYSICS_SANDBOX_WATER_VISUAL_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace physics_sandbox
{
	using namespace pr::hlsl;
#endif

static const int MaxWaterWaveCount = 64;

struct WaterVisualWave
{
	float4 m_direction_wavelength_phase_speed;
	float4 m_amplitude;
};

struct CBufWaterVisual
{
	WaterVisualWave m_waves[MaxWaterWaveCount];
	int m_wave_count;
	float m_time_s;
	float m_water_level;
	float m_pad;
};

#ifdef __cplusplus
}
#endif
#endif
