//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock sprite particle type. See particle_sprite.hlsli.
#include "pr/game-systems/particles/particle_sprite.hlsli"

#define ParticleState ParticleSpriteState
#define ParticleRecord ParticleSpriteRecord
#define EmitterParams ParticleSpriteEmitter
#define TypeConstants ParticleSpriteConstants

// Create a sprite from its emitter's parameters. Every random choice comes from the particle's own seed.
ParticleSpriteState ParticleInit(ParticleSpriteEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Spread the birth position and velocity over their balls, and pick the lifetime within its range.
	float3 dp = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 0), ParticleRandom(spawn.seed, 1), ParticleRandom(spawn.seed, 2)));
	float3 dv = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 3), ParticleRandom(spawn.seed, 4), ParticleRandom(spawn.seed, 5)));
	float lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));

	ParticleSpriteState state;
	state.p0 = params.position.xyz + dp * params.position.w;
	state.age = spawn.age;
	state.v0 = params.velocity.xyz + dv * params.velocity.w;
	state.lifetime = lifetime;
	state.accel = params.accel.xyz;
	state.seed = spawn.seed;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	return state;
}

// Age a sprite. Its position is a function of its age, so only the age changes.
bool ParticleUpdate(inout ParticleSpriteState state, float dt, ParticleSpriteConstants c)
{
	state.age += dt;
	return state.age < state.lifetime;
}

// Evaluate a sprite's position, size and colour at its age.
ParticleSpriteRecord ParticleProject(ParticleSpriteState state, ParticleSpriteConstants c)
{
	// Fly on constant acceleration.
	float3 position = state.p0 + state.v0 * state.age + 0.5f * state.accel * state.age * state.age;
	float3 velocity = state.v0 + state.accel * state.age;
	return ParticleSpriteMakeRecord(position, velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
}

// Bound a sprite by the sphere around its disc and streak. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleSpriteState state, ParticleSpriteConstants c)
{
	// The record holds the position, size and streak that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
