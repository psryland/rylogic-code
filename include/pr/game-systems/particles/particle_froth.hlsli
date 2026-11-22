//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock froth particle type: soft puffs of foam that form on the breaking crests of a bound water surface (see particle_water.hlsli) and ride it.
// Births find crests like spray (see particle_spray.hlsli), and a birth that finds no breaking crest makes no froth. Each puff drifts across the
// surface with a fraction of the wind, stays at a height above the waves below it that grows slowly with its age, and grows and fades over its life
// by the sprite curve. Crests travel with the waves while puffs stay behind, so a puff fades over its emitter's linger time once the crest has left it,
// and dies when it has faded. Froth therefore shows only on and just behind breaking crests. Froth dies when the water is removed.
//
// Froth writes ParticleSpriteRecords and draws with the stock sprite batch, as camera-facing sprites. It moves only in simulated time.
// Positions are in the space of the type's View3D object, which must be the model space of the bound ocean's object.
#ifndef PR_GAME_SYSTEMS_PARTICLE_FROTH_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_FROTH_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The parameters of one froth emitter. Colours are 0xAARRGGBB. The type's constants are ParticleSpriteConstants.
struct ParticleFrothEmitter
{
	float4 area;           // xy = Centre of the disc of sea that froth forms on. z unused. w = Radius of the disc (m).
	float2 lifetime;       // Range of puff lifetimes (s). Each puff picks a lifetime uniformly in [x, y]. Positive.
	float2 size;           // Diameter (m) at birth (x) and at death (y), before the sprite curve.
	uint colour0;          // Colour at birth.
	uint colour1;          // Colour at death.
	float breaking_threshold;  // Breaking amount (see ParticleSampleWater) that a point must reach to form froth, and that counts as a full crest. Thresholds
	                           // above the ocean's crest strength confine froth to the ridges of the breaking crests. Positive.
	uint candidates;       // Random points tried per birth. [1, 32] (see ParticleFindCrest).
	float drift;           // Fraction of the wind velocity that puffs drift with.
	float spread;          // Radius of the disc that the horizontal drift velocities are spread over (m/s).
	float lift;            // Height (m) of a puff's centre above the surface at birth.
	float rise;            // Rate (m/s) at which a puff's height above the surface grows.
	float linger;          // Time (s) for a puff to fade once the crest under it has passed. Positive.
	uint pad0;
	uint pad1;
	uint pad2;
};

// The state of one froth puff.
struct ParticleFrothState
{
	float3 position;  // Current centre, above the surface.
	float age;        // Simulated seconds since birth.
	float2 drift;     // Horizontal drift velocity (m/s).
	float lifetime;   // Age at which the puff dies.
	float lift;       // Height above the surface at birth.
	float rise;       // Growth rate of the height above the surface.
	float size0;      // Diameter at birth.
	float size1;      // Diameter at death.
	uint colour0;     // Colour at birth.
	uint colour1;     // Colour at death.
	uint seed;        // Per-puff random seed.
	float presence;   // Crest under the puff in [0, 1]: the breaking amount under it relative to the threshold, decaying over 'linger' once it falls.
	float linger;     // Decay time (s) of the presence.
	float threshold;  // Breaking amount that counts as a full crest.
	uint pad0;
	uint pad1;
	uint pad2;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleFrothEmitter) == 80);
	static_assert(sizeof(ParticleFrothState) == 80);
}
#endif
#endif