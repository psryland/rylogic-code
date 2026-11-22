//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Lets particle kernels sample a bound ocean surface (see pr/game-systems/ocean/ocean_surface.h): the height and normal of the drawn waves, and the
// breaking crests on them. A particle system binds at most one surface with ParticleSystem::WaterSet. Without one, the water is far below everything and nothing breaks.
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
	float breaking;   // How far the waves here are breaking: the crest foam amount (see OceanCrestFoam) before it is limited to the crest strength.
	                  // It equals the crest foam where that is below its full amount, and keeps rising above it towards the tops of the steepest
	                  // crests, so thresholds above the crest strength select only the ridges of breaking waves. Zero or less where nothing breaks.
};

// Return the drawn water surface at the horizontal position 'xy'. Positions beyond the ocean's domain take the surface at the domain edge, where it ends.
ParticleWaterSample ParticleSampleWater(float2 xy)
{
	// No water has no surface for particles to meet.
	ParticleWaterSample result;
	result.height = -3.0e38f;
	result.normal = float3(0.0f, 0.0f, 1.0f);
	result.breaking = 0.0f;
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

	// The surface's height and normal. The breaking amount follows OceanCrestFoam without its limit: the crest steepness grows from mid height to
	// the crest's top, so the unlimited amount keeps rising to the ridge after the drawn foam has reached its full amount.
	OceanWaveSample water = OceanSample(xy, distance, spacing);
	result.height = g_ocean.level_radii.x + water.surface.displacement_foam.z;
	result.normal = normalize(float3(water.surface.normal_delta.xy, 1.0f));
	result.breaking = g_ocean.foam_crest.x * water.crest_depth_fade * (water.crest_steepness - g_ocean.foam_crest.y) / g_ocean.foam_crest.z;
	return result;
}

// The most candidate points that one crest search tries.
static const uint ParticleCrestCandidateMax = 32;

// Return the breaking amount of 'water', raised on faces that slope down in the horizontal direction of 'lean' and lowered on faces that slope up.
// A face's horizontal normal points the way it slopes down, so the change is the dot product of that normal with 'lean'. Zero 'lean' gives the plain amount.
float ParticleWaterBreaking(ParticleWaterSample water, float2 lean)
{
	// The horizontal part of the unit normal grows with the steepness of the face.
	return water.breaking + dot(water.normal.xy, lean);
}

// Search for a breaking crest in the disc of radius 'radius' around 'centre'. Tries up to 'candidates' random points, chosen by 'seed' with random
// indices from 'first_index', and returns true with the first point whose breaking amount, leaned by 'lean' (see ParticleWaterBreaking), reaches
// 'threshold', in 'xy' and 'water'. Returns false when no point does, such as on a calm sea or with no water. 'candidates' is clamped to [1, ParticleCrestCandidateMax].
bool ParticleFindCrest(float2 centre, float radius, uint candidates, float threshold, float2 lean, uint seed, uint first_index, out float2 xy, out ParticleWaterSample water)
{
	// Points are spread evenly over the disc's area. Each candidate uses two random values.
	uint count = clamp(candidates, 1u, ParticleCrestCandidateMax);
	for (uint i = 0; i != count; ++i)
	{
		// The square root of a uniform value gives a radius that puts equal areas at equal probability.
		float r = radius * sqrt(ParticleRandom(seed, first_index + 2 * i));
		float a = 6.283185307179586f * ParticleRandom(seed, first_index + 2 * i + 1);
		xy = centre + r * float2(cos(a), sin(a));
		water = ParticleSampleWater(xy);
		if (ParticleWaterBreaking(water, lean) >= threshold)
			return true;
	}
	return false;
}

// The most steps that ParticleFindCrestFront takes.
static const uint ParticleCrestFrontSteps = 16;

// Move a breaking point 'xy', with its sample 'water', towards the edge of its crest in steps of 'step', while the next point's breaking amount,
// leaned by 'lean' (see ParticleWaterBreaking), still reaches 'threshold'. The result is the last point that still reaches it. A point at a random
// distance from the edge therefore ends up evenly placed within one step of the edge, so points from many births form a band one step deep along it.
// Crests wider than ParticleCrestFrontSteps steps leave the point further from the edge. Zero 'step' leaves the point where it is.
void ParticleFindCrestFront(inout float2 xy, inout ParticleWaterSample water, float2 step, float threshold, float2 lean)
{
	// Stop at the first point past the edge, keeping the last one inside it.
	if (all(step == float2(0.0f, 0.0f)))
		return;

	for (uint i = 0; i != ParticleCrestFrontSteps; ++i)
	{
		// A point that no longer reaches the threshold is past the edge.
		ParticleWaterSample next = ParticleSampleWater(xy + step);
		if (ParticleWaterBreaking(next, lean) < threshold)
			break;

		xy += step;
		water = next;
	}
}

// Return the horizontal vector of length 'length' along the wind 'wind', or zero with no wind. Waves travel downwind, so this points towards
// the leading edges of crests.
float2 ParticleAlongWind(float2 wind, float length)
{
	// Still air has no direction.
	float speed = sqrt(dot(wind, wind));
	return speed > 0.0f ? wind * (length / speed) : float2(0.0f, 0.0f);
}

#endif
#endif
