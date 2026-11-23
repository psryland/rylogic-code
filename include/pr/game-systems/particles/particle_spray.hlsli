//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock spray particle type: droplets torn from a bound water surface (see particle_water.hlsli) by the wind and blown over it. Each birth picks
// a random point over the emitter's disc of sea, spread evenly over its area, and keeps it with a chance that rises with the wind speed there (see
// ParticleSprayEmitter::wind_range). A birth that is not kept makes no droplet, so still air throws no spray and strong winds throw spray all over the
// sea. Droplets start just above the surface, are carried by the wind, fall under gravity, and die when they fall back into the water.
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
	float4 launch;         // x = Mean upward launch speed (m/s). y = Radius of the ball that launch velocities are spread over (m/s).
	                       // z = Fraction of the wind velocity added at launch. w = Drag rate towards the wind velocity (1/s). Non-negative.
	float2 lifetime;       // Range of droplet lifetimes (s). Each droplet picks a lifetime uniformly in [x, y]. Positive.
	float2 size;           // Diameter (m) at birth (x) and at death (y).
	uint colour0;          // Colour at birth.
	uint colour1;          // Colour at death.
	float2 wind_range;     // Horizontal wind speeds (m/s) at the birth point over which the chance that a birth makes a droplet rises linearly from
	                       // zero (x) to one (y). No birth makes a droplet at or below x. y > x >= 0.
	float lift;            // Greatest height (m) above the surface that droplets start at. Each droplet picks a height uniformly in [0, lift]. Non-negative.
	float gravity;         // Scale of the environment's gravity.
	uint pad0;
	uint pad1;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleSprayEmitter) == 80);
}
#endif
#endif
