//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock froth particle type: patches of foam that form on the breaking crests of a bound water surface (see particle_water.hlsli) and ride it.
// Births find crests like spray (see particle_spray.hlsli), and a birth that finds no breaking crest makes no froth. Each patch drifts across the
// surface with a fraction of the wind, lies flat on the waves just above them, and grows and fades over its life. Froth dies when the water is removed.
//
// Froth writes ParticleMeshRecords and draws with the stock flat sprite batch, as soft discs lying on the surface. It moves only in simulated time.
// Positions are in the space of the type's View3D object, which must be the model space of the bound ocean's object.
#ifndef PR_GAME_SYSTEMS_PARTICLE_FROTH_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_FROTH_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The parameters of one froth emitter. Colours are 0xAARRGGBB. The type's constants are ParticleFrothConstants.
struct ParticleFrothEmitter
{
	float4 area;           // xy = Centre of the disc of sea that froth forms on. z unused. w = Radius of the disc (m).
	float2 lifetime;       // Range of patch lifetimes (s). Each patch picks a lifetime uniformly in [x, y]. Positive.
	float2 size;           // Diameter (m) at birth (x) and at death (y).
	uint colour0;          // Colour at birth.
	uint colour1;          // Colour at death.
	float foam_threshold;  // Crest foam amount (see ParticleSampleWater) that a point must reach to form froth. (0, 1].
	uint candidates;       // Random points tried per birth. [1, 32] (see ParticleFindCrest).
	float drift;           // Fraction of the wind velocity that patches drift with.
	float spread;          // Radius of the disc that the horizontal drift velocities are spread over (m/s).
	float lift;            // Height (m) that patches lie above the surface, so they draw over it.
	uint pad0;
};

// The constants of the froth type.
struct ParticleFrothConstants
{
	float fade_in;   // Seconds over which a patch fades in after birth. Zero for none.
	float fade_out;  // Seconds over which a patch fades out before death. Zero for none.
	uint pad0;
	uint pad1;
};

// The state of one froth patch.
struct ParticleFrothState
{
	float3 position;      // Current position, on the surface.
	float age;            // Simulated seconds since birth.
	float2 drift;         // Horizontal drift velocity (m/s).
	float lifetime;       // Age at which the patch dies.
	float lift;           // Height above the surface.
	float4 orientation;   // Lies the patch on the surface, turned by its random angle.
	float size0;          // Diameter at birth.
	float size1;          // Diameter at death.
	uint colour0;         // Colour at birth.
	uint colour1;         // Colour at death.
	uint seed;            // Per-patch random seed.
	uint pad0;
	uint pad1;
	uint pad2;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleFrothEmitter) == 64);
	static_assert(sizeof(ParticleFrothConstants) == 16);
	static_assert(sizeof(ParticleFrothState) == 80);
}
#endif
#endif
