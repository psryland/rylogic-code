//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Lightweight water vertex shader matching the physics water-field surface.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "src/scene/water/water_visual_cbuf.hlsli"
#include "pr/physics/terrain/water/water_field.hlsli"

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufNugget> resource(g_nugget, b1);
ConstantBuffer<CBufWaterVisual> resource(g_water, b3);

// Evaluate the physics water height and gradient together using the shared per-element evaluator.
float3 EvaluateWater(float2 xy_ws)
{
	float3 height_gradient = float3(g_water.m_water_level, 0.0f, 0.0f);
	for (int element_index = 0; element_index != g_water.m_element_count; ++element_index)
	{
		height_gradient += WaterFieldElementHeightAndGradient(g_water.m_elements[element_index], xy_ws, g_water.m_time_s);
	}
	return height_gradient;
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
