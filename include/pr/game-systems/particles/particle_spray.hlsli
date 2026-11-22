//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock spray particle type: droplets thrown up from the breaking crests of a bound water surface (see particle_water.hlsli). Each birth tries
// random points over the emitter's disc of sea and rises from the first point where the crest foam reaches the emitter's threshold. A birth that finds
// no breaking crest makes no droplet, so calm seas throw no spray and rough seas throw spray along their crests. Droplets launch along the surface
// normal, are carried by the wind, fall under gravity, and die when they fall back into the water.
//
// Spray uses the stepped sprite state (see particle_stepped_sprite.hlsli) and draws with the stock sprite batch. It moves only in simulated time.
// Positions are in the space of the type's View3D object, which must be the model space of the bound ocean's object.
#ifndef PR_GAME_SYSTEMS_PARTICLE_SPRAY_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_SPRAY_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The parameters of one spray emitter. Colours are 0xAARRGGBB. The type's constants are ParticleSpriteConstants.
struct ParticleSprayEmitter
{
	float4 area;           // xy = Centre of the disc of sea that spray rises from. z unused. w = Radius of the disc (m).
	float4 launch;         // x = Mean launch speed along the surface normal (m/s). y = Radius of the ball that launch velocities are spread over (m/s).
	                       // z = Fraction of the wind velocity added at launch. w = Drag rate towards the wind velocity (1/s). Non-negative.
	float2 lifetime;       // Range of droplet lifetimes (s). Each droplet picks a lifetime uniformly in [x, y]. Positive.
	float2 size;           // Diameter (m) at birth (x) and at death (y).
	uint colour0;          // Colour at birth.
	uint colour1;          // Colour at death.
	float foam_threshold;  // Crest foam amount (see ParticleSampleWater) that a point must reach to throw spray. (0, 1].
	uint candidates;       // Random points tried per birth. [1, 32] (see ParticleFindCrest). More finds the crests of calmer seas, at more cost.
	float gravity;         // Scale of the environment's gravity.
	uint pad0;
	uint pad1;
	uint pad2;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleSprayEmitter) == 80);
}
#endif
#endif
