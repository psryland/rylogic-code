//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock sprite particle type: camera-facing soft discs that fly on a constant acceleration, and change size and colour over their life.
// Sprites suit sparks, embers, spray, dust and other small particles that need no texture.
//
// A sprite's position is a pure function of its age, so it is exact at any tick rate and in slow motion:
//   position(age) = p0 + v0 * age + 0.5 * accel * age^2
// Positions are in the space of the type's View3D object.
#ifndef PR_GAME_SYSTEMS_PARTICLE_SPRITE_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_SPRITE_HLSLI
#include "pr/game-systems/particles/particles.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The parameters of one sprite emitter. Colours are 0xAARRGGBB.
struct ParticleSpriteEmitter
{
	float4 position;  // xyz = Where particles are born. w = Radius of the ball around 'xyz' that births are spread over.
	float4 velocity;  // xyz = Mean birth velocity (m/s). w = Radius of the ball around 'xyz' that birth velocities are spread over.
	float4 accel;     // xyz = Constant acceleration (m/s²), e.g. gravity. w unused.
	float2 lifetime;  // Range of particle lifetimes (s). Each particle picks a lifetime uniformly in [x, y]. Positive.
	float2 size;      // Diameter (m) at birth (x) and at death (y).
	uint colour0;     // Colour at birth.
	uint colour1;     // Colour at death.
	uint pad0;
	uint pad1;
};

// The constants of a sprite particle type.
struct ParticleSpriteConstants
{
	float fade_in;   // Fraction of the lifetime over which alpha rises from zero. [0, 1].
	float fade_out;  // Fraction of the lifetime over which alpha falls to zero. [0, 1].
	float pad0;
	float pad1;
};

// The state of one sprite particle.
struct ParticleSpriteState
{
	float3 p0;       // Position at age zero.
	float age;       // Simulated seconds since birth.
	float3 v0;       // Velocity at age zero.
	float lifetime;  // Age at which the particle dies.
	float3 accel;    // Constant acceleration.
	uint seed;       // Per-particle random seed, for effects that vary per particle.
	uint colour0;    // Colour at birth.
	uint colour1;    // Colour at death.
	float size0;     // Diameter at birth.
	float size1;     // Diameter at death.
};

// What the sprite vertex shader draws for one particle.
struct ParticleSpriteRecord
{
	float4 pos_size;  // xyz = Centre. w = Diameter.
	uint colour;      // 0xAARRGGBB.
	float angle;      // Rotation about the view direction (radians).
	uint pad0;
	uint pad1;
};

// The sprite index list draws an octagon as a fan of triangles from the centre (logical vertex 8) to the rim (logical vertices 0-7).
static const uint ParticleSpriteRimCount = 8;
static const uint ParticleSpriteIndexCount = 3 * ParticleSpriteRimCount;
static const uint ParticleSpriteVertexCount = ParticleSpriteRimCount + 1;

#ifndef __cplusplus

// Unpack a 0xAARRGGBB colour to (r, g, b, a) in [0, 1].
float4 ParticleUnpackColour(uint argb)
{
	return float4((argb >> 16) & 0xFF, (argb >> 8) & 0xFF, argb & 0xFF, argb >> 24) / 255.0f;
}

// Pack (r, g, b, a) in [0, 1] to a 0xAARRGGBB colour.
uint ParticlePackColour(float4 colour)
{
	uint4 c = uint4(saturate(colour) * 255.0f + 0.5f);
	return (c.a << 24) | (c.r << 16) | (c.g << 8) | c.b;
}

// Return a random point in the unit ball from three values in [0, 1). Points are spread evenly over the ball's volume.
float3 ParticleRandomInBall(float3 u)
{
	// Pick a direction evenly over the sphere, and a radius that puts equal volumes at equal probability.
	float z = 2.0f * u.x - 1.0f;
	float r = sqrt(saturate(1.0f - z * z));
	float sine;
	float cosine;
	sincos(6.283185307179586f * u.y, sine, cosine);
	return float3(r * cosine, r * sine, z) * pow(u.z, 1.0f / 3.0f);
}

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleSpriteEmitter) == 80);
	static_assert(sizeof(ParticleSpriteConstants) == 16);
	static_assert(sizeof(ParticleSpriteState) == 64);
	static_assert(sizeof(ParticleSpriteRecord) == 32);
}
#endif
#endif
