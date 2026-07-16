//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Shared types for ocean vertex and pixel shaders.
// This file is included from both HLSL and C++ source.
#ifndef LAS_OCEAN_CBUF_HLSLI
#define LAS_OCEAN_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "src/world/water/shaders/water_field_types.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
	using water::MaxWaterFieldElementCount;
	using water::WaterFieldElement;
#endif

// Ocean constant buffer. Bound to b3 (reusing the CBufScreenSpace slot since
// the ocean shader does not use screen-space geometry).
struct CBufOcean //:reg(b3)
{
	// Fixed-capacity field shared with GPU buoyancy. Only water_field_count entries are active.
	WaterFieldElement water_field[MaxWaterFieldElementCount];

	// Camera world-space position (xyz), w = simulation time
	float4 camera_pos_time;

	// Mesh configuration: x=inner radius, y=outer radius, z=ring count, w=segment count
	float4 mesh_config;

	// Number of active field elements
	int water_field_count;

	// PBR parameters
	float fresnel_f0;         // Fresnel reflectance at normal incidence (water ~0.02)
	float specular_power;     // Specular highlight sharpness
	float sss_strength;       // Subsurface scattering intensity

	// Ocean colours
	float4 colour_shallow;    // Turquoise for shallow water
	float4 colour_deep;       // Dark blue for deep water
	float4 colour_foam;       // White foam colour

	// Sun direction (world space, normalised, points toward sun)
	float4 sun_direction;

	// Sun colour (RGB intensity)
	float4 sun_colour;

	// Environment map: non-zero if a cubemap is bound at t1
	int has_env_map;

	// Water transparency at normal incidence (0=opaque, 1=fully clear)
	float water_transparency;

	// Minimum radial distance between adjacent rings
	float min_ring_spacing;

	float pad0;
};

#ifdef __cplusplus
}
#endif
#endif
