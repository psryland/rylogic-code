//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock stepped sprite particle type: camera-facing soft discs that fall under the environment's gravity, are carried by its wind, and bounce
// off its collision planes. They change size and colour over their life like sprites, and draw with the sprite batch and vertex shader.
// Stepped sprites suit sparks that bounce, wind-blown dust, rain and spray.
//
// Each tick integrates the motion over the tick and then collides it with the emitter's collision set, so motion depends only on simulated time.
// See particle_environment.hlsli for the integration and collision contract. Positions are in the space of the type's View3D object, which must be
// the space that the environment is described in.
#ifndef PR_GAME_SYSTEMS_PARTICLE_STEPPED_SPRITE_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_STEPPED_SPRITE_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The parameters of one stepped sprite emitter. Colours are 0xAARRGGBB. The type's constants are ParticleSpriteConstants.
struct ParticleSteppedSpriteEmitter
{
	float4 position;    // xyz = Where particles are born. w = Radius of the ball around 'xyz' that births are spread over.
	float4 velocity;    // xyz = Mean birth velocity (m/s). w = Radius of the ball around 'xyz' that birth velocities are spread over.
	float4 accel;       // xyz = Acceleration (m/s²) added to the environment's gravity. w = Drag rate towards the wind velocity (1/s). Non-negative.
	float2 lifetime;    // Range of particle lifetimes (s). Each particle picks a lifetime uniformly in [x, y]. Positive.
	float2 size;        // Diameter (m) at birth (x) and at death (y).
	uint colour0;       // Colour at birth.
	uint colour1;       // Colour at death.
	float radius;       // Collision radius (m): the distance that particle centres are kept from collision planes.
	int collision_set;  // The collision set that particles bounce off, or -1 for none.
};

// The state of one stepped sprite particle.
struct ParticleSteppedSpriteState
{
	float3 position;    // Current position.
	float age;          // Simulated seconds since birth.
	float3 velocity;    // Current velocity.
	float lifetime;     // Age at which the particle dies.
	float3 accel;       // Acceleration added to the environment's gravity.
	float drag;         // Drag rate towards the wind velocity (1/s).
	float radius;       // Collision radius.
	int collision_set;  // Collision set, or -1 for none.
	uint seed;          // Per-particle random seed, for effects that vary per particle.
	uint colour0;       // Colour at birth.
	uint colour1;       // Colour at death.
	float size0;        // Diameter at birth.
	float size1;        // Diameter at death.
	uint pad0;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleSteppedSpriteEmitter) == 80);
	static_assert(sizeof(ParticleSteppedSpriteState) == 80);
}
#endif
#endif
