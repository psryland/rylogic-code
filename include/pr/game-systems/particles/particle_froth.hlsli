//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock froth particle type: soft puffs of foam that form on the breaking crests of a bound water surface (see particle_water.hlsli) and ride it.
// Births find crests like spray (see particle_spray.hlsli), leaning towards the leading faces of the crests, and a birth that finds no breaking crest
// makes no froth. Each puff drifts across the surface with a fraction of the wind, stays at a height above the waves below it that grows slowly with
// its age, and swells and shrinks along a half sine over its life while the sprite curve shapes its colour and size. Crests travel with the waves while
// puffs stay behind, so a puff fades over its emitter's linger time once the crest has left it, and dies when it has faded. Froth therefore shows only
// on and just behind breaking crests. Froth dies when the water is removed.
//
// Froth writes ParticleFrothRecords and draws with the froth batch (see FrothDrawShaders) as camera-facing sprites cut off at the water's surface, so
// only the part of a puff above the water is drawn and a puff rises out of the wave like a dome. The texture gives each puff its outline, so the froth
// batch needs a texture. Froth moves only in simulated time. Positions are in the space of the type's View3D object, which must be the model space of
// the bound ocean's object, and that object must not be scaled unevenly.
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
	float2 size;           // Diameter (m) at birth (x) and at death (y), before the sprite curve and the half-sine swell over the puff's life.
	uint colour0;          // Colour at birth.
	uint colour1;          // Colour at death.
	float breaking_threshold;  // Breaking amount (see ParticleSampleWater) that a point must reach to form froth, and that counts as a full crest. Thresholds
	                           // above the ocean's crest strength confine froth to the ridges of the breaking crests. Positive.
	uint candidates;       // Random points tried per birth. [1, 32] (see ParticleFindCrest).
	float drift;           // Fraction of the wind velocity that puffs drift with.
	float spread;          // Radius of the disc that the horizontal drift velocities are spread over (m/s).
	float lift;            // Height (m) of a puff's centre above the surface at birth. The part of a puff below the surface is not drawn, so zero shows the
	                       // upper half of each puff. Not negative.
	float rise;            // Rate (m/s) at which a puff's height above the surface grows.
	float linger;          // Time (s) for a puff to fade once the crest under it has passed. Positive.
	float lead;            // Breaking amount added per unit of a face's horizontal normal along the wind (see ParticleWaterBreaking). Waves travel downwind,
	                       // so a positive lead forms froth earlier on the leading faces of crests and less on their backs. Zero for no lean.
	float front;           // Depth (m) of the band behind the leading edge of a breaking crest that puffs form in (see ParticleFindCrestFront), so the
	                       // crest gets a sharp foam front. Zero forms puffs anywhere on the crest.
	uint pad1;
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
	float2 lean;      // Lean of the breaking amount (see ParticleWaterBreaking): the emitter's lead along the wind direction at birth.
	uint normal_xy;   // x and y of the surface's unit normal under the puff, as half floats. x in the low bits. The normal's z is positive.
};

// What the froth vertex shader draws for one puff: a sprite, and the plane of the water surface under it that the sprite is cut off at.
struct ParticleFrothRecord
{
	float4 pos_size;  // xyz = Centre. w = Diameter.
	uint colour;      // 0xAARRGGBB.
	float angle;      // Rotation about the view direction (radians).
	uint normal_xy;   // x and y of the surface's unit normal, as half floats. x in the low bits. The normal's z is positive.
	float height;     // Height of the centre above the surface, along the object's z axis. The surface plane passes through the point this far below the centre.
};

#ifndef __cplusplus

// Pack the x and y of a unit normal with positive z as half floats.
uint ParticleFrothPackNormal(float3 normal)
{
	return f32tof16(normal.x) | (f32tof16(normal.y) << 16);
}

// Unpack a unit normal with positive z from its x and y as half floats.
float3 ParticleFrothUnpackNormal(uint normal_xy)
{
	float2 xy = float2(f16tof32(normal_xy), f16tof32(normal_xy >> 16));
	return float3(xy, sqrt(saturate(1.0f - dot(xy, xy))));
}

// Return a sphere that contains what the froth vertex shader draws for 'record'. The radius is in world units (see ParticleSpriteRecordBounds).
float4 ParticleFrothRecordBounds(ParticleFrothRecord record)
{
	// The cut sprite lies within its uncut disc.
	return float4(record.pos_size.xyz, 0.5f * record.pos_size.w);
}

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleFrothEmitter) == 80);
	static_assert(sizeof(ParticleFrothState) == 80);
	static_assert(sizeof(ParticleFrothRecord) == 32);
}
#endif
#endif