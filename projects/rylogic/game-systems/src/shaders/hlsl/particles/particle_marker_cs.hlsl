//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock marker particle type. See particle_marker.hlsli.
#include "pr/game-systems/particles/particle_marker.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

#define ParticleState ParticleMarkerState
#define ParticleRecord ParticleMeshRecord
#define EmitterParams ParticleMarkerEmitter
#define TypeConstants ParticleMarkerConstants

// Create a marker at its emitter's position, facing along its normal.
ParticleMarkerState ParticleInit(ParticleMarkerEmitter params, ParticleSpawn spawn, ParticleMarkerConstants c)
{
	// Copy the emitter's shape, and pick the lifetime within its range.
	ParticleMarkerState state;
	state.position = params.position.xyz;
	state.age = spawn.age;
	state.normal = params.normal.xyz;
	state.spin = params.normal.w;
	state.size = params.size;
	state.lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 0));
	state.seed = spawn.seed;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.bob = params.bob;
	state.bob_rate = params.bob_rate;
	return state;
}

// Age a marker. Its appearance is a function of its age, so only the age changes.
bool ParticleUpdate(inout ParticleMarkerState state, float dt, ParticleMarkerConstants c)
{
	state.age += dt;
	return state.age < state.lifetime;
}

// Evaluate a marker's position, orientation, size and colour at its age.
ParticleMeshRecord ParticleProject(ParticleMarkerState state, ParticleMarkerConstants c)
{
	// Blend size and colour over the life, and bob along the normal.
	float t = saturate(state.age / state.lifetime);
	float4 colour = lerp(ParticleUnpackColour(state.colour0), ParticleUnpackColour(state.colour1), t);
	float3 position = state.position + state.normal * (state.bob * sin(6.283185307179586f * state.bob_rate * state.age));

	// Face along the normal, turned about it from a random start.
	float spin = state.spin * state.age + 6.283185307179586f * ParticleRandom(state.seed, 1);
	float4 orientation = ParticleQuatMul(ParticleQuatFromZ(state.normal), ParticleQuatAxisAngle(float3(0.0f, 0.0f, 1.0f), spin));

	ParticleMeshRecord record;
	record.pos_size = float4(position, lerp(state.size.x, state.size.y, t));
	record.orientation = orientation;
	record.colour = ParticlePackColour(colour);
	record.pad0 = 0;
	record.pad1 = 0;
	record.pad2 = 0;
	return record;
}

// Bound a marker by the ball that contains the batch's mesh at the marker's size.
float4 ParticleBounds(ParticleMarkerState state, ParticleMarkerConstants c)
{
	// The record holds the position and size that the vertex shader draws.
	ParticleMeshRecord record = ParticleProject(state, c);
	return float4(record.pos_size.xyz, c.mesh_radius * record.pos_size.w);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
