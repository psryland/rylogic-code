//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Lightweight water vertex shader matching the physics sine-wave surface.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "src/scene/water/water_visual_cbuf.hlsli"

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufNugget> resource(g_nugget, b1);
ConstantBuffer<CBufWaterVisual> resource(g_water, b3);

// Evaluate the physics water height and gradient together so each wave needs one sincos operation.
float3 EvaluateWater(float2 xy_ws)
{
	float height = g_water.m_water_level;
	float2 gradient = float2(0.0f, 0.0f);
	for (int wave_index = 0; wave_index != g_water.m_wave_count; ++wave_index)
	{
		WaterVisualWave wave = g_water.m_waves[wave_index];
		float2 direction = wave.m_direction_wavelength_phase_speed.xy;
		float wave_number = tau / wave.m_direction_wavelength_phase_speed.z;
		float phase = dot(direction, xy_ws) * wave_number + wave.m_direction_wavelength_phase_speed.w * g_water.m_time_s;
		float sine, cosine;
		sincos(phase, sine, cosine);
		height += wave.m_amplitude.x * sine;
		gradient += direction * (wave.m_amplitude.x * wave_number * cosine);
	}
	return float3(height, gradient);
}

// Displace a static grid in world space and produce the analytical normal used by the physics surface.
PSIn VSWater(VSIn In)
{
	PSIn Out = (PSIn)0;

	float4 os_vert = mul(In.vert, g_nugget.m2o);
	Out.ws_vert = mul(os_vert, g_nugget.o2w);

	float3 height_gradient = EvaluateWater(Out.ws_vert.xy);
	Out.ws_vert.z = height_gradient.x;
	Out.ws_norm = float4(normalize(float3(-height_gradient.yz, 1.0f)), 0.0f);
	Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);

	Out.diff = In.diff * g_nugget.tint;
	Out.tex0 = mul(float4(In.tex0, 0.0f, 1.0f), g_nugget.tex2surf0).xy;
	Out.idx0 = In.idx0;
	return Out;
}
