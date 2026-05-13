//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Procedural sky dome shader.
// VS: Passes vertex position as view direction.
// PS: Computes atmospheric sky colour from sun position.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "src/world/sky/shaders/procedural_sky_cbuf.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufNugget> resource(g_nugget, b1);
ConstantBuffer<CBufProceduralSky> resource(g_sky, b3);

struct PSOut
{
	float4 diff semantic(SV_TARGET);
};

// Compute sky colour from a view direction and sun parameters
float3 AtmosphericSky(float3 view_dir, float3 sun_dir, float sun_intensity)
{
	float sun_elev = sun_dir.z;

	// Day factor: smooth transition around sunrise/sunset
	float day = saturate(sun_elev * 5.0 + 0.5);

	// Base sky colours
	float3 zenith_day    = float3(0.15, 0.35, 0.65);
	float3 zenith_night  = float3(0.005, 0.007, 0.02);
	float3 horizon_day   = float3(0.55, 0.65, 0.75);
	float3 horizon_night = float3(0.01, 0.01, 0.02);

	float3 zenith  = lerp(zenith_night,  zenith_day,  day);
	float3 horizon = lerp(horizon_night, horizon_day, day);

	// Sky gradient from horizon to zenith
	float view_elev = max(view_dir.z, 0.0);
	float3 sky = lerp(horizon, zenith, pow(view_elev, 0.4));

	// Sunset/sunrise: warm orange at horizon near the sun, cool purple opposite
	float sunset_band = exp(-sun_elev * sun_elev * 50.0) * saturate(sun_elev + 0.15);
	if (sunset_band > 0.01)
	{
		float2 sun_flat  = normalize(sun_dir.xy + float2(0.0001, 0));
		float2 view_flat = (length(view_dir.xy) > 0.001) ? normalize(view_dir.xy) : sun_flat;
		float az_proximity = saturate(dot(sun_flat, view_flat) * 0.5 + 0.5);

		float3 warm_sunset  = float3(1.0, 0.35, 0.05);
		float3 cool_twilight = float3(0.25, 0.15, 0.35);
		float3 horizon_tint = lerp(cool_twilight, warm_sunset, az_proximity);

		sky += horizon_tint * sunset_band * (1.0 - view_elev * view_elev);
	}

	// Sun disc and Mie-like glow
	float cos_sun = dot(view_dir, sun_dir);
	if (cos_sun > 0 && sun_elev > -0.1)
	{
		float sun_disc = smoothstep(0.9996, 0.9999, cos_sun);
		float sun_glow = pow(cos_sun, 8.0) * 0.4;
		float sun_halo = pow(cos_sun, 32.0) * 0.15;

		float3 sun_color = float3(1.0, 0.95, 0.85) * sun_intensity;
		sky += sun_color * (sun_disc + sun_glow + sun_halo);
	}

	// Below-horizon fade
	if (view_dir.z < 0)
	{
		float below = saturate(-view_dir.z * 3.0);
		sky = lerp(sky, sky * 0.3, below);
	}

	return max(sky, 0.0);
}

// Vertex shader: pass view direction via ws_norm
PSIn VSProceduralSky(VSIn In)
{
	PSIn Out = (PSIn)0;

	// The cube vertex position IS the view direction from the camera
	Out.ws_norm = float4(normalize(In.vert.xyz), 0);

	// Standard transform for rasterisation
	Out.ws_vert = mul(float4(In.vert.xyz, 1), g_nugget.o2w);
	Out.ss_vert = mul(float4(In.vert.xyz, 1), g_nugget.o2s);
	Out.diff = float4(0, 0, 0, 1);
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

	return Out;
}

// Pixel shader: procedural atmospheric sky
PSOut PSProceduralSky(PSIn In)
{
	PSOut Out = (PSOut) 0;

	float3 view_dir = normalize(In.ws_norm.xyz);
	float3 sky = AtmosphericSky(view_dir, g_sky.sun_direction.xyz, g_sky.sun_intensity);

	Out.diff = float4(sky, 1.0);
	return Out;
}

#ifdef __cplusplus
}
#endif
