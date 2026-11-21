//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// A caller-defined particle type for the tests: stationary markers that never die. Its state, emitter parameters and constants differ from
// the stock sprite's, but it writes sprite records, so it can share a batch with sprite types and be drawn by the sprite vertex shader.
#include "particle_marker.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

#define ParticleState ParticleMarkerState
#define ParticleRecord ParticleSpriteRecord
#define EmitterParams ParticleMarkerEmitter
#define TypeConstants ParticleMarkerConstants

// Place a marker at its emitter's position, offset along x by its sequence number so that each marker of a burst is visible.
ParticleMarkerState ParticleInit(ParticleMarkerEmitter params, ParticleSpawn spawn, ParticleMarkerConstants c)
{
	ParticleMarkerState state;
	state.position = params.position + float3(params.spacing * float(spawn.sequence), 0.0f, 0.0f);
	state.colour = params.colour;
	return state;
}

// Markers never move or die.
bool ParticleUpdate(inout ParticleMarkerState state, float dt, ParticleMarkerConstants c)
{
	return true;
}

// Draw a marker as a sprite of the type's size.
ParticleSpriteRecord ParticleProject(ParticleMarkerState state, ParticleMarkerConstants c)
{
	ParticleSpriteRecord record;
	record.pos_size = float4(state.position, c.size);
	record.colour = state.colour;
	record.angle = 0.0f;
	record.pad0 = 0;
	record.pad1 = 0;
	return record;
}

// Bound a marker by a sphere of the sprite's radius.
float4 ParticleBounds(ParticleMarkerState state, ParticleMarkerConstants c)
{
	// Markers are sprites of the type's size.
	return float4(state.position, 0.5f * c.size);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
