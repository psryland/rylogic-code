//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock sprite particle type: camera-facing soft discs that fly on a constant acceleration, and change size and colour over their life.
// Sprites suit sparks, embers, spray, dust and other small particles that need no texture.
//
// A sprite type's constants shape every sprite's life: a curve of size and colour keys multiplies the emitter's birth-to-death blend, brightness can
// flicker, and a streak stretches each sprite behind its centre along its velocity, for sparks and rain. All are functions of the particle's age.
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

// One key of a sprite's curve over its life.
struct ParticleCurveKey
{
	float t;      // Fraction of the lifetime at the key. [0, 1]. Keys are in increasing order of 't'.
	float size;   // Multiplier of the emitter's diameter.
	uint colour;  // Multiplier of the emitter's colour, as 0xAARRGGBB.
	float pad0;
};

// The most keys in a sprite curve.
static const uint ParticleCurveKeyMax = 4;

// The constants of a sprite particle type. All zeros draws the emitter's size and colour blend unchanged.
struct ParticleSpriteConstants
{
	uint key_count;       // Number of keys used in 'keys'. [0, ParticleCurveKeyMax]. With no keys, the curve is one, so size and colour are unchanged.
	float flicker;        // Largest fraction of the brightness removed by flicker. [0, 1]. Zero for none.
	float flicker_rate;   // Typical flickers per second.
	float streak;         // Seconds of motion that a sprite is stretched along, behind its centre. Zero for round sprites.
	ParticleCurveKey keys[ParticleCurveKeyMax]; // The curve, interpolated linearly between keys and held constant beyond the first and last keys.
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
	uint streak_xy;   // The streak: the displacement from the tail to the head of the sprite, in the object's space, as half floats. x in the low bits.
	uint streak_z;    // The streak's z, as a half float in the low bits. A zero streak draws a round sprite.
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

// Return the size and colour multipliers of the curve in 'c' at the fraction 't' of a sprite's life.
void ParticleSpriteCurve(ParticleSpriteConstants c, float t, out float size, out float4 colour)
{
	// No keys leaves the emitter's blend unchanged.
	size = 1.0f;
	colour = float4(1.0f, 1.0f, 1.0f, 1.0f);
	if (c.key_count == 0)
		return;

	// Each key replaces the value once 't' passes it, blending in from the previous key. Keys are in order, so the value at the previous key is
	// already exact when the blend to the next key starts. Keys at equal 't' make a step.
	size = c.keys[0].size;
	colour = ParticleUnpackColour(c.keys[0].colour);
	for (uint i = 1; i < min(c.key_count, ParticleCurveKeyMax); ++i)
	{
		ParticleCurveKey a = c.keys[i - 1];
		ParticleCurveKey b = c.keys[i];
		float f = b.t > a.t ? saturate((t - a.t) / (b.t - a.t)) : (t >= b.t ? 1.0f : 0.0f);
		size = lerp(size, b.size, f);
		colour = lerp(colour, ParticleUnpackColour(b.colour), f);
	}
}

// Make the record that the sprite vertex shader draws for a sprite at 'position', moving at 'velocity', at 'age'. Size and colour blend from birth to
// death over 'lifetime' and are multiplied by the type's curve. Brightness flickers and the rotation is fixed per particle, both chosen by 'seed'.
ParticleSpriteRecord ParticleSpriteMakeRecord(float3 position, float3 velocity, float age, float lifetime, uint seed, uint colour0, uint colour1, float size0, float size1, ParticleSpriteConstants c)
{
	// Blend size and colour over the life, shaped by the curve.
	float t = saturate(age / lifetime);
	float curve_size;
	float4 curve_colour;
	ParticleSpriteCurve(c, t, curve_size, curve_colour);
	float size = lerp(size0, size1, t) * curve_size;
	float4 colour = lerp(ParticleUnpackColour(colour0), ParticleUnpackColour(colour1), t) * curve_colour;

	// Flicker is two sine waves at unrelated rates with per-particle phases, so it is a function of age alone and never repeats exactly.
	if (c.flicker > 0.0f)
	{
		float phase = 6.283185307179586f * c.flicker_rate * age;
		float wave = 0.5f + 0.25f * (sin(phase + 6.283185307179586f * ParticleRandom(seed, 8)) + sin(2.3f * phase + 6.283185307179586f * ParticleRandom(seed, 9)));
		colour.rgb *= 1.0f - c.flicker * wave;
	}

	// The streak is the distance moved in the streak time, stored as half floats.
	float3 streak = velocity * c.streak;

	ParticleSpriteRecord record;
	record.pos_size = float4(position, size);
	record.colour = ParticlePackColour(colour);
	record.angle = 6.283185307179586f * ParticleRandom(seed, 7);
	record.streak_xy = f32tof16(streak.x) | (f32tof16(streak.y) << 16);
	record.streak_z = f32tof16(streak.z);
	return record;
}

// Return the position of the sprite vertex 'vertex_id' relative to the sprite's centre, in the camera's view plane, and its texture coordinates.
// Rim vertices sit on a circle of 'radius', turned by 'angle'. The texture's unit square fits around the rim, with the centre vertex at the texture's
// centre and v down the screen, so the texture turns with the sprite. Returns true for rim vertices and false for the centre vertex.
bool ParticleSpriteVertex(uint vertex_id, float angle, float radius, out float2 offset_cs, out float2 uv)
{
	// The centre vertex is the last logical vertex.
	offset_cs = float2(0.0f, 0.0f);
	uv = float2(0.5f, 0.5f);
	if (vertex_id >= ParticleSpriteRimCount)
		return false;

	float sine;
	float cosine;
	sincos(angle + 6.283185307179586f * float(vertex_id) / float(ParticleSpriteRimCount), sine, cosine);
	offset_cs = radius * float2(cosine, sine);
	uv = float2(0.5f + 0.5f * cosine, 0.5f - 0.5f * sine);
	return true;
}

// Return the streak of 'record': the displacement from the tail of the sprite to its centre.
float3 ParticleSpriteStreak(ParticleSpriteRecord record)
{
	return float3(f16tof32(record.streak_xy), f16tof32(record.streak_xy >> 16), f16tof32(record.streak_z));
}

// Return a sphere that contains what the sprite vertex shader draws for 'record': the disc at its centre and the streak behind it. The radius is in
// world units and the cull scales it by the object's largest axis scale, so the sphere contains the sprite when the batch's object is unscaled or enlarged.
float4 ParticleSpriteRecordBounds(ParticleSpriteRecord record)
{
	// The sprite spans from its centre back along its streak, with its radius around both ends.
	float3 streak = ParticleSpriteStreak(record);
	return float4(record.pos_size.xyz - 0.5f * streak, 0.5f * (record.pos_size.w + length(streak)));
}

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleSpriteEmitter) == 80);
	static_assert(sizeof(ParticleCurveKey) == 16);
	static_assert(sizeof(ParticleSpriteConstants) == 80);
	static_assert(sizeof(ParticleSpriteState) == 64);
	static_assert(sizeof(ParticleSpriteRecord) == 32);
}
#endif
#endif
