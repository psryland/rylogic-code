//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock debris particle type: small tumbling pieces, such as leaves and wood chips, drawn as copies of the mesh of a stock mesh batch.
// Debris falls under a scale of the environment's gravity, is carried by its wind, and bounces off its collision planes like stepped sprites.
// Each piece spins at a constant rate about a random axis, and can flutter: a sideways push that swings around in the horizontal plane, so light
// pieces such as leaves drift and swirl as they fall. Contacts slow the spin and flutter, so pieces settle where they land.
//
// Each tick integrates the motion over the tick and then collides it with the emitter's collision set, so motion depends only on simulated time.
// Flutter is held constant for each tick at its value in the middle of the tick, so it changes slightly with the tick length. Flutter acts in the
// xy plane, so the type's View3D object must have z up. Positions are in the space of the type's View3D object, which must be the space that the
// environment is described in. See particle_environment.hlsli.
#ifndef PR_GAME_SYSTEMS_PARTICLE_DEBRIS_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_DEBRIS_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The constants of a debris particle type.
struct ParticleDebrisConstants
{
	float grow;         // Seconds over which pieces grow from zero after birth. Zero for none.
	float shrink;       // Seconds over which pieces shrink to zero before death. Zero for none.
	float mesh_radius;  // Radius of a ball around the origin that contains the batch's mesh, in units of the record's size. Used for culling.
	float pad0;
};

// The parameters of one debris emitter. Colours are 0xAARRGGBB.
struct ParticleDebrisEmitter
{
	float4 position;     // xyz = Where pieces are born. w = Radius of the ball around 'xyz' that births are spread over.
	float4 velocity;     // xyz = Mean birth velocity (m/s). w = Radius of the ball around 'xyz' that birth velocities are spread over.
	float4 accel;        // xyz = Acceleration (m/s²) added to the scaled gravity. w = Drag rate towards the wind velocity (1/s). Non-negative.
	float2 lifetime;     // Range of lifetimes (s). Each piece picks a lifetime uniformly in [x, y]. Positive.
	float2 size;         // Range of sizes: the scale of the mesh. Each piece picks a size uniformly in [x, y].
	float2 spin;         // Range of spin rates (rad/s). Each piece picks a rate uniformly in [x, y], about a random axis.
	float flutter;       // Strength of the sideways flutter push (m/s²). Zero for none.
	float flutter_rate;  // Typical swings of the flutter per second.
	uint colour0;        // One end of the range of colours. Each piece picks a colour between 'colour0' and 'colour1'.
	uint colour1;        // The other end of the range of colours.
	float radius;        // Collision radius (m): the distance that piece centres are kept from collision planes.
	int collision_set;   // The collision set that pieces bounce off, or -1 for none.
	float settle;        // Fraction of the spin and flutter removed by each contact. [0, 1].
	float gravity;       // Scale of the environment's gravity. 1 falls normally, 0 ignores gravity.
	uint pad0;
	uint pad1;
};

// The state of one debris particle.
struct ParticleDebrisState
{
	float3 position;     // Current position.
	float age;           // Simulated seconds since birth.
	float3 velocity;     // Current velocity.
	float lifetime;      // Age at which the piece dies.
	float4 orientation;  // Current orientation, as a unit quaternion.
	float3 spin_axis;    // Unit axis of the spin.
	float spin_rate;     // Spin rate (rad/s).
	float3 accel;        // Acceleration added to the scaled gravity.
	float drag;          // Drag rate towards the wind velocity (1/s).
	float size;          // Scale of the mesh.
	float radius;        // Collision radius.
	int collision_set;   // Collision set, or -1 for none.
	uint seed;           // Per-particle random seed.
	uint colour;         // Colour.
	float gravity;       // Scale of the environment's gravity.
	float settle;        // Fraction of the spin and flutter removed by each contact.
	float flutter;       // Strength of the flutter push.
	float flutter_rate;  // Typical swings of the flutter per second.
	uint pad0;
	uint pad1;
	uint pad2;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleDebrisConstants) == 16);
	static_assert(sizeof(ParticleDebrisEmitter) == 112);
	static_assert(sizeof(ParticleDebrisState) == 128);
}
#endif
#endif
