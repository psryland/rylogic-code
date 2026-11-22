//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock bird particle type: distant birds that circle, bob up and down, and flap or glide, drawn by the bird vertex shader as a small
// procedural bird with flapping wings. Birds are a background effect for distant flocks, so they ignore wind, gravity and collision planes.
//
// Each bird flies a circle around its own centre, near the emitter, at its own radius, speed and direction, and bobs vertically on a sine wave.
// It banks into the turn, and switches between flapping and gliding at a slow, per-bird pace. Its position, orientation and wing angle are pure
// functions of its age, so they are exact at any tick rate and in slow motion. Birds circle in the xy plane, so the type's View3D object must have
// z up. The bird's local frame has +y forward, +x right and +z up, and its wingspan is its size.
#ifndef PR_GAME_SYSTEMS_PARTICLE_BIRD_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_BIRD_HLSLI
#include "pr/game-systems/particles/particles.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// Index count of the bird mesh drawn by the bird vertex shader: seven triangles, each drawn in both windings so the bird has no back face.
static const uint ParticleBirdIndexCount = 42;

// Number of logical vertices of the bird mesh: the head, the rump, three per wing, and two tail tips.
static const uint ParticleBirdVertexCount = 10;

// The constants of a bird particle type.
struct ParticleBirdConstants
{
	float bank;            // Roll into the turn (radians).
	float flap_amplitude;  // Largest wing angle above and below the body while flapping (radians).
	float dihedral;        // Wing angle above the body while gliding (radians).
	float fade;            // Seconds over which birds grow from zero after birth and shrink to zero before death. Zero for none.
};

// The parameters of one bird emitter. The colour is 0xAARRGGBB.
struct ParticleBirdEmitter
{
	float4 position;    // xyz = The middle of the flock. w = Radius of the ball around 'xyz' that the birds' circle centres are spread over.
	float2 orbit;       // Range of circle radii (m). Each bird picks a radius uniformly in [x, y]. Positive.
	float2 speed;       // Range of flight speeds (m/s). Each bird picks a speed uniformly in [x, y], and a random direction around its circle.
	float2 bob;         // x = Height of the vertical bob (m). y = Seconds per bob. Each bird varies both a little.
	float2 lifetime;    // Range of lifetimes (s). Each bird picks a lifetime uniformly in [x, y]. Positive.
	float size;         // Wingspan (m).
	float flap_rate;    // Wing beats per second.
	uint colour;        // Colour of every bird.
	uint pad0;
};

// The state of one bird particle.
struct ParticleBirdState
{
	float3 centre;      // Centre of the bird's circle.
	float age;          // Simulated seconds since birth.
	float radius;       // Radius of the circle.
	float omega;        // Signed angular speed around the circle (rad/s). Positive flies anticlockwise seen from above.
	float theta0;       // Angle around the circle at birth (radians).
	float lifetime;     // Age at which the bird dies.
	float bob_height;   // Height of the vertical bob.
	float bob_period;   // Seconds per bob.
	float bob_phase;    // Phase of the bob (radians).
	float size;         // Wingspan.
	float flap_rate;    // Wing beats per second.
	uint colour;        // Colour.
	uint seed;          // Per-particle random seed.
	uint pad0;
};

// What the bird vertex shader draws for one bird.
struct ParticleBirdRecord
{
	float4 pos_size;     // xyz = Position of the bird's body. w = Wingspan.
	float4 orientation;  // Unit quaternion (x, y, z, w) that turns the bird's local frame.
	uint colour;         // 0xAARRGGBB.
	float flap;          // Wing angle above the body (radians).
	uint pad0;
	uint pad1;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleBirdConstants) == 16);
	static_assert(sizeof(ParticleBirdEmitter) == 64);
	static_assert(sizeof(ParticleBirdState) == 64);
	static_assert(sizeof(ParticleBirdRecord) == 48);
}
#endif
#endif
