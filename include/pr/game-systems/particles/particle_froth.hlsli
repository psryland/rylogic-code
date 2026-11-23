//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock froth particle type: soft puffs of foam that form where a bound water surface draws strong crest foam (see particle_water.hlsli) and ride it.
// Each birth tries random points over the emitter's disc of sea and uses the first whose crest foam reaches the emitter's minimum (see ParticleFindFoam),
// and a birth that finds none makes no froth. Froth therefore follows the ocean's own crest foam settings. Each puff
// drifts across the surface with a fraction of the wind, stays at a height above the waves below it that grows slowly with
// its age, and swells and shrinks along a half sine over its life while the sprite curve shapes its colour and size. Crests travel with the waves while
// puffs stay behind, so a puff fades over its emitter's linger time once the crest foam under it has gone, and dies when it has faded. Froth therefore shows only
// on and just behind breaking crests. Froth dies when the water is removed. Each puff is squashed along the surface normal by its emitter's
// squash, so puffs can lie flat on the water. Puffs fade out with their distance from the camera, between the emitter's
// fade distances. This fade is applied when puffs are drawn, so it changes only how they look, never how they move or how long they live.
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
	float foam_min;        // Crest foam (see ParticleWaterSample), in (0, 1], that a point must have to form froth, and that counts as full foam under a puff.
	                       // Values near one keep froth to the strongest crest foam.
	uint candidates;       // Random points tried per birth. [1, 32] (see ParticleFindFoam).
	float drift;           // Fraction of the wind velocity that puffs drift with.
	float spread;          // Radius of the disc that the horizontal drift velocities are spread over (m/s).
	float lift;            // Height (m) of a puff's centre above the surface at birth. The part of a puff below the surface is not drawn, so zero shows the
	                       // upper half of each puff. Not negative.
	float rise;            // Rate (m/s) at which a puff's height above the surface grows.
	float linger;          // Time (s) for a puff to fade once the crest foam under it has gone. Positive.
	float fade_near;       // Distance (m) from the camera at which puffs start to fade out. Not negative.
	float fade_far;        // Distance (m) from the camera beyond which puffs are not drawn. Greater than 'fade_near'.
	float squash;          // Height of a puff along the surface normal as a fraction of its width, in (0, 1]. One draws round puffs; smaller values draw
	                       // flat puffs that lie low on the water when seen from the side, and still look round from above.
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
	float presence;   // Crest foam under the puff in [0, 1], relative to 'foam_min', decaying over 'linger' once it falls.
	float linger;     // Decay time (s) of the presence.
	float foam_min;   // Crest foam that counts as full foam under the puff.
	uint normal_xy;   // x and y of the surface's unit normal under the puff, as half floats. x in the low bits. The normal's z is positive.
	uint fade;        // The emitter's 'fade_near' (low bits) and 'fade_far', as half floats.
	float squash;     // Height along the surface normal as a fraction of the width.
};

// What the froth vertex shader draws for one puff: a sprite, the plane of the water surface under it that the sprite is cut off at, and the camera
// distances over which it fades out.
struct ParticleFrothRecord
{
	float4 pos_size;      // xyz = Centre. w = Diameter.
	uint colour;          // 0xAARRGGBB.
	uint shape;           // The puff's rotation, squash and height above the surface. See ParticleFrothPackShape.
	uint normal_xy;       // x and y of the surface's unit normal, as half floats. x in the low bits. The normal's z is positive.
	uint fade;            // Camera distances (m) at which the puff starts to fade (low bits) and is gone, as half floats. Distances are in world units.
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

// The shape of a puff, packed into one record field.
struct ParticleFrothShape
{
	float angle;   // Rotation about the view direction (radians), in [-pi, pi]. Stored to 8 bits.
	float squash;  // Height along the surface normal as a fraction of the width, in (0, 1]. Stored to 8 bits.
	float height;  // Height of the centre above the surface, along the object's z axis. The surface plane passes through the point this far below
	               // the centre. Stored as a half float.
};

// Pack a puff's shape (see ParticleFrothShape): the angle in the low 8 bits, then the squash, then the height in the high 16 bits.
uint ParticleFrothPackShape(float angle, float squash, float height)
{
	uint a = uint(round(saturate(angle / 6.283185307179586f + 0.5f) * 255.0f));
	uint s = uint(round(saturate(squash) * 255.0f));
	return a | (s << 8) | (f32tof16(height) << 16);
}

// Unpack a puff's shape. See ParticleFrothPackShape.
ParticleFrothShape ParticleFrothUnpackShape(uint shape)
{
	// The squash is kept above zero so a puff never collapses to a line.
	ParticleFrothShape result;
	result.angle = ((shape & 0xFF) / 255.0f - 0.5f) * 6.283185307179586f;
	result.squash = max((shape >> 8) & 0xFF, 1) / 255.0f;
	result.height = f16tof32(shape >> 16);
	return result;
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