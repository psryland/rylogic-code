//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock marker particle type: gameplay markers, such as beacon pulses and target rings, drawn as copies of the mesh of a stock mesh batch.
// Each marker stays at its emitter's position, faces along the emitter's normal, turns about it, bobs along it, and changes size and colour over
// its life. A steady emitter rate gives a train of expanding pulses. Markers are pure functions of their age, so they are exact at any tick rate
// and in slow motion. The stock marker mesh is a flat ring in the mesh's xy plane, facing +z, which the normal turns to face along.
#ifndef PR_GAME_SYSTEMS_PARTICLE_MARKER_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_MARKER_HLSLI
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// The constants of a marker particle type.
struct ParticleMarkerConstants
{
	float mesh_radius;  // Radius of a ball around the origin that contains the batch's mesh, in units of the record's size. Used for culling.
	float pad0;
	float pad1;
	float pad2;
};

// The parameters of one marker emitter. Colours are 0xAARRGGBB.
struct ParticleMarkerEmitter
{
	float4 position;   // xyz = Where markers are drawn. w is unused.
	float4 normal;     // xyz = Unit direction that markers face. w = Spin rate about the normal (rad/s).
	float2 size;       // Scale of the mesh at birth (x) and at death (y).
	float2 lifetime;   // Range of lifetimes (s). Each marker picks a lifetime uniformly in [x, y]. Positive.
	uint colour0;      // Colour at birth.
	uint colour1;      // Colour at death.
	float bob;         // Distance that markers bob along the normal (m). Zero for none.
	float bob_rate;    // Bobs per second.
};

// The state of one marker particle.
struct ParticleMarkerState
{
	float3 position;   // Where the marker is drawn.
	float age;         // Simulated seconds since birth.
	float3 normal;     // Unit direction that the marker faces.
	float spin;        // Spin rate about the normal (rad/s).
	float2 size;       // Scale at birth (x) and at death (y).
	float lifetime;    // Age at which the marker dies.
	uint seed;         // Per-particle random seed.
	uint colour0;      // Colour at birth.
	uint colour1;      // Colour at death.
	float bob;         // Distance of the bob along the normal.
	float bob_rate;    // Bobs per second.
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleMarkerConstants) == 16);
	static_assert(sizeof(ParticleMarkerEmitter) == 64);
	static_assert(sizeof(ParticleMarkerState) == 64);
}
#endif
#endif
