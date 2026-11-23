//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Lets particle kernels sample a bound ocean surface (see pr/game-systems/ocean/ocean_surface.h): the height and normal of the drawn waves, and the
// crest foam drawn on them. A particle system binds at most one surface with ParticleSystem::WaterSet. Without one, the water is far below everything and has no foam.
//
// The samples use the surface's constants from its last Update and its procedural buffer, so particles see exactly the waves that are drawn, at the
// ocean's wave time. Positions are in the model space of the ocean's View3D object, which must be the space of the particles' positions.
// The drawn surface filters waves too short for its lattice, and its lattice depends on the camera, so sampled heights depend slightly on the camera
// position. This is a presentation effect, as the drawn surface is.
//
// A type's compute shader includes this file, after particle_environment.hlsli, when it uses the water. Kernels that do not include it ignore the water registers.
#ifndef PR_GAME_SYSTEMS_PARTICLE_WATER_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_WATER_HLSLI
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

#ifndef __cplusplus

// The bound surface's constants and procedural buffer. A dummy buffer and zeroed constants are bound when there is no water.
ConstantBuffer<OceanConstants> g_ocean : register(b3);
ByteAddressBuffer g_ocean_buffer : register(t5);
#include "pr/game-systems/ocean/ocean_buffer.hlsli"

// The water surface at one horizontal position.
struct ParticleWaterSample
{
	float height;     // Height of the drawn surface. Far below everything when there is no water.
	float3 normal;    // Unit normal of the drawn surface.
	float foam;       // Crest foam that the ocean draws here, in [0, 1] (see OceanCrestFoam), so it follows the ocean's crest strength, threshold and range.
	                  // The ocean's pixels add waves too short for its vertex lattice near the camera (see PixelCrest), so the drawn crests there can be a
	                  // little narrower than this. Zero where nothing breaks, and when the ocean draws no foam.
};

// Return the drawn water surface at the horizontal position 'xy'. Positions beyond the ocean's domain take the surface at the domain edge, where it ends.
ParticleWaterSample ParticleSampleWater(float2 xy)
{
	// No water has no surface for particles to meet.
	ParticleWaterSample result;
	result.height = -3.0e38f;
	result.normal = float3(0.0f, 0.0f, 1.0f);
	result.foam = 0.0f;
	if (g_environment.water == 0)
		return result;

	// Mirror the ocean vertex shader's inputs (see Main in ocean_forward_vs.hlsl), so the sample matches the drawn lattice.
	float centre_distance = length(xy);
	if (centre_distance > g_ocean.level_radii.y)
		xy *= g_ocean.level_radii.y / centre_distance;

	float distance = length(xy - g_ocean.camera_time.xy);
	float half_cells = 0.5f * float(g_ocean.layout.x - 1);
	float central_extent = OceanCentralExtent(g_ocean.mesh.x, half_cells, g_ocean.level_radii.z);
	float warp_power = OceanWarpPower(g_ocean.mesh.y, g_ocean.camera_time.z - g_ocean.level_radii.x, g_ocean.level_radii.z);
	float spacing = OceanSpacingAtDistance(distance, central_extent, g_ocean.level_radii.z, warp_power, half_cells);

	// The surface's height and normal, and the crest foam that the ocean's pixels draw from the same crest steepness. An ocean that draws no foam has none here.
	OceanWaveSample water = OceanSample(xy, distance, spacing);
	result.height = g_ocean.level_radii.x + water.surface.displacement_foam.z;
	result.normal = normalize(float3(water.surface.normal_delta.xy, 1.0f));
	if ((g_ocean.layout.y & OceanFlagFoam) != 0)
		result.foam = OceanCrestFoam(water.crest_steepness, water.crest_depth_fade);

	return result;
}

// The most candidate points that one foam search tries.
static const uint ParticleFoamCandidateMax = 32;

// Search for crest foam in the disc of radius 'radius' around 'centre'. Tries up to 'candidates' random points, chosen by 'seed' with random
// indices from 'first_index', and returns true with the first point whose crest foam reaches 'foam_min', in 'xy' and 'water'. Returns false when
// no point does, such as on a calm sea or with no water. 'candidates' is clamped to [1, ParticleFoamCandidateMax].
bool ParticleFindFoam(float2 centre, float radius, uint candidates, float foam_min, uint seed, uint first_index, out float2 xy, out ParticleWaterSample water)
{
	// Points are spread evenly over the disc's area. Each candidate uses two random values.
	uint count = clamp(candidates, 1u, ParticleFoamCandidateMax);
	for (uint i = 0; i != count; ++i)
	{
		// The square root of a uniform value gives a radius that puts equal areas at equal probability.
		float r = radius * sqrt(ParticleRandom(seed, first_index + 2 * i));
		float a = 6.283185307179586f * ParticleRandom(seed, first_index + 2 * i + 1);
		xy = centre + r * float2(cos(a), sin(a));
		water = ParticleSampleWater(xy);
		if (water.foam >= foam_min)
			return true;
	}
	return false;
}

#endif
#endif
