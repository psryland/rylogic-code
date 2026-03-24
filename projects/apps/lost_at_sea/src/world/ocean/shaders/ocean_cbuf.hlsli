//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Shared types for ocean vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_OCEAN_CBUF_HLSLI
#define LAS_OCEAN_CBUF_HLSLI

#include "pr/hlsl/interop.hlsli"

static const int MaxOceanWaves = 4;

// Ocean constant buffer. Bound to b3 (reusing the CBufScreenSpace slot since
// the ocean shader does not use screen-space geometry).
cbuffer CBufOcean :reg(b3)
{
	// Wave component directions. xy = normalised direction, zw = unused
	float4 m_wave_dirs[MaxOceanWaves];

	// Wave component parameters: x=amplitude, y=wavelength, z=speed, w=steepness
	float4 m_wave_params[MaxOceanWaves];

	// Camera world-space position (xyz), w = simulation time
	float4 m_camera_pos_time;

	// Mesh radii: x=inner, y=outer, z=num_rings, w=num_segments
	float4 m_mesh_config;

	// Number of active wave components
	int m_wave_count;

	// PBR parameters
	float m_fresnel_f0;         // Fresnel reflectance at normal incidence (water ~0.02)
	float m_specular_power;     // Specular highlight sharpness
	float m_sss_strength;       // Subsurface scattering intensity

	// Ocean colours
	float4 m_colour_shallow;    // Turquoise for shallow water
	float4 m_colour_deep;       // Dark blue for deep water
	float4 m_colour_foam;       // White foam colour

	// Sun direction (world space, normalised, points toward sun)
	float4 m_sun_direction;

	// Sun colour (RGB intensity)
	float4 m_sun_colour;

	// Environment map: non-zero if a cubemap is bound at t1
	int m_has_env_map;

	// Water transparency at normal incidence (0=opaque, 1=fully clear)
	float m_water_transparency;

	float2 ocean_pad0;
};

#endif
