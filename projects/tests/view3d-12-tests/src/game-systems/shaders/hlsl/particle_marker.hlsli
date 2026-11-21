//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Layouts of the test marker particle type, shared by its kernels and the tests. See particle_marker_cs.hlsl.
#ifndef GAME_SYSTEMS_TESTS_PARTICLE_MARKER_HLSLI
#define GAME_SYSTEMS_TESTS_PARTICLE_MARKER_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace game_systems_tests
{
	using namespace pr::hlsl;
#endif

// The parameters of one marker emitter. 'colour' is 0xAARRGGBB.
struct ParticleMarkerEmitter
{
	float3 position;  // Where the first marker of the emitter is placed.
	float spacing;    // Distance along x between consecutive markers of the emitter.
	uint colour;      // Colour of every marker of the emitter.
	uint pad0;
	uint pad1;
	uint pad2;
};

// The constants of the marker type.
struct ParticleMarkerConstants
{
	float size;  // Diameter of every marker.
	float pad0;
	float pad1;
	float pad2;
};

// The state of one marker.
struct ParticleMarkerState
{
	float3 position;
	uint colour;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleMarkerEmitter) == 32);
	static_assert(sizeof(ParticleMarkerConstants) == 16);
	static_assert(sizeof(ParticleMarkerState) == 16);
}
#endif
#endif
