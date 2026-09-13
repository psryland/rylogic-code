//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Procedural atmospheric background shader.
// VS: Reconstructs world view directions from a full-screen triangle.
// PS: Computes atmospheric sky colour from sun position.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/procedural_sky_cbuf.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::sky
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufProceduralSky> resource(g_sky, b3);
TextureCube<float4> resource(g_background, t13);
SamplerState resource(g_background_sampler, s1);

struct PSOut
{
	float4 diff semantic(SV_TARGET);
};

// Compute sky colour from a view direction and sun parameters
float3 AtmosphericSky(float3 view_dir, float3 sun_dir, float3 sun_colour, float sun_intensity)
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
	float view_elev = saturate(view_dir.z);
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
	float cos_sun = clamp(dot(view_dir, sun_dir), -1.0, 1.0);
	if (cos_sun > 0 && sun_elev > -0.1)
	{
		float sun_disc = smoothstep(0.9996, 0.9999, cos_sun);
		float sun_glow = pow(cos_sun, 8.0) * 0.4;
		float sun_halo = pow(cos_sun, 32.0) * 0.15;

		float3 sun_color = sun_colour * sun_intensity;
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

// Vertex shader: derive world directions independently of camera translation and object transforms.
PSIn VSProceduralSky(VSIn In)
{
	PSIn Out = (PSIn)0;

	// Orthographic rays are parallel; perspective rays also account for off-centre projection.
	float3 camera_direction = float3(0, 0, -1);
	if (g_frame.cam.c2s[3][3] == 0)
	{
		camera_direction.xy = (In.vert.xy + float2(g_frame.cam.c2s[2][0], g_frame.cam.c2s[2][1])) /
			float2(g_frame.cam.c2s[0][0], g_frame.cam.c2s[1][1]);
	}
	Out.ws_norm = mul(float4(camera_direction, 0), g_frame.cam.c2w);

	// Far depth fills only background pixels; interpolation preserves the unnormalized ray until the pixel shader.
	Out.ss_vert = float4(In.vert.xy, 1, 1);
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
	float3 sky_dir = mul(float4(view_dir, 0), g_sky.world_to_sky).xyz;
	float3 sky = AtmosphericSky(sky_dir, g_sky.sun_direction.xyz, g_sky.sun_colour.rgb, g_sky.sun_intensity);

	// Blend linear colour in one opaque background draw; exact endpoints do not depend on the unused source.
	if (g_sky.blend_weight < 1)
	{
		float3 cube_dir = mul(float4(view_dir, 0), g_sky.world_to_cube).xyz;
		float3 background = g_background.SampleLevel(g_background_sampler, cube_dir, 0).rgb;
		sky = lerp(background, sky, g_sky.blend_weight);
	}

	Out.diff = float4(sky, 1.0);
	return Out;
}

#ifdef __cplusplus
}
#endif
