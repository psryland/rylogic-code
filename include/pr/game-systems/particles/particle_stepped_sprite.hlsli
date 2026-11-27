//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock stepped sprite particle type: camera-facing soft discs that fall under a scale of the environment's gravity, are carried by its wind, and bounce
// off its collision planes, or die when they touch one. They change size and colour over their life like sprites, and draw with the sprite batch and vertex shader.
// Stepped sprites suit sparks that bounce, wind-blown dust, rain and spray.
//
// "Up" is opposite to the environment's gravity. An emitter can push its births outward, level with the ground, away from its position, and can
// roll that outward motion: it slows at the drag rate, curls upward and swirls around the vertical, so a burst spirals out and up like a ring of
// smoke, and each sprite spins with its roll. The wind carries a rolling particle without bending its roll. Both need a non-zero gravity.
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
	float4 accel;       // xyz = Acceleration (m/s²) added to the scaled gravity. w = Drag rate towards the wind velocity (1/s). Non-negative.
	float2 lifetime;    // Range of particle lifetimes (s). Each particle picks a lifetime uniformly in [x, y]. Positive.
	float2 size;        // Diameter (m) at birth (x) and at death (y).
	uint colour0;       // Colour at birth.
	uint colour1;       // Colour at death.
	float radius;       // Collision radius (m): the distance that particle centres are kept from collision planes.
	int collision_set;  // The collision set that particles bounce off, or -1 for none.
	float gravity;      // Scale of the environment's gravity. 1 falls normally, 0 ignores gravity, and negative values rise.
	uint die_on_contact; // Non-zero to remove particles on their first touch of a collision plane, such as rain and snow landing.
	float outward;      // Speed (m/s) away from 'position', level with the ground, along each birth's offset from it. See 'curl' for how it is applied.
	float curl;         // Rate (rad/s) that the outward motion rolls upward. The roll slows at the drag rate and the wind carries it without bending it. A curl no faster than the drag never turns back down.
	float swirl;        // Rate (rad/s) that the outward motion turns about the vertical, right handed about up. With neither curl nor swirl, the outward speed is added to the birth velocity instead of rolling.
	float variety;      // In [0, 1]. Each particle scales its drag, outward speed, curl and swirl by its own random factors in [1 - variety, 1 + variety], so a burst spreads and disperses.
	float2 pad;         // Unused. Rounds the size up to the 16 byte alignment that the C++ vector types give it.
};

// The state of one stepped sprite particle.
struct ParticleSteppedSpriteState
{
	float3 position;    // Current position.
	float age;          // Simulated seconds since birth.
	float3 velocity;    // Current velocity.
	float lifetime;     // Age at which the particle dies.
	float3 accel;       // Acceleration added to the scaled gravity.
	float drag;         // Drag rate towards the wind velocity (1/s).
	float radius;       // Collision radius.
	int collision_set;  // Collision set, or -1 for none.
	uint seed;          // Per-particle random seed, for effects that vary per particle.
	uint colour0;       // Colour at birth.
	uint colour1;       // Colour at death.
	float size0;        // Diameter at birth.
	float size1;        // Diameter at death.
	float gravity;      // Scale of the environment's gravity.
	uint die_on_contact; // Non-zero to die on touching a collision plane.
	float curl;         // Rate (rad/s) that the roll turns upward.
	uint roll_xy;       // The level roll axis scaled by the outward roll speed (m/s), as half floats, x in the low bits. Zero for a sprite that does not roll.
	uint roll_z;        // The scaled axis's z in the low bits, and the rate (rad/s) that the roll swirls about the vertical in the high bits, as half floats.
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleSteppedSpriteEmitter) == 112);
	static_assert(sizeof(ParticleSteppedSpriteState) == 96);
}
#endif
#endif
