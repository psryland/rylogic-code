//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock stepped sprite particle type. See particle_stepped_sprite.hlsli.
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"

#define ParticleState ParticleSteppedSpriteState
#define ParticleRecord ParticleSpriteRecord
#define EmitterParams ParticleSteppedSpriteEmitter
#define TypeConstants ParticleSpriteConstants

// Move a stepped sprite through 'dt' seconds of its environment, bouncing off its collision planes. Returns the number of planes touched.
uint ParticleSteppedSpriteMove(inout ParticleSteppedSpriteState state, float dt)
{
	// Integrate the free flight with the wind at the start of the tick, then resolve contacts along the tick's motion.
	float3 start = state.position;
	float3 end = state.position;
	float3 velocity = state.velocity;
	float3 wind = ParticleSampleWind(start);
	ParticleIntegrate(end, velocity, state.gravity * g_environment.gravity + state.accel, state.drag, wind, dt);
	uint touched = ParticleCollidePlanes(state.collision_set, state.radius, start, end, velocity);
	state.position = end;
	state.velocity = velocity;
	return touched;
}

// Create a stepped sprite from its emitter's parameters, and move it through the part of the tick after its birth.
ParticleSteppedSpriteState ParticleInit(ParticleSteppedSpriteEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Spread the birth position and velocity over their balls, and pick the lifetime within its range. These match the sprite type's choices.
	float3 dp = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 0), ParticleRandom(spawn.seed, 1), ParticleRandom(spawn.seed, 2)));
	float3 dv = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 3), ParticleRandom(spawn.seed, 4), ParticleRandom(spawn.seed, 5)));
	float lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));

	ParticleSteppedSpriteState state;
	state.position = params.position.xyz + dp * params.position.w;
	state.age = spawn.age;
	state.velocity = params.velocity.xyz + dv * params.velocity.w;
	state.lifetime = lifetime;
	state.accel = params.accel.xyz;
	state.drag = params.accel.w;
	state.radius = params.radius;
	state.collision_set = params.collision_set;
	state.seed = spawn.seed;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	state.gravity = params.gravity;
	state.die_on_contact = params.die_on_contact;
	state.pad0 = 0;
	state.pad1 = 0;
	state.pad2 = 0;

	// A particle born part way through the tick has already lived for 'spawn.age'. New particles are always kept for one tick, so one that dies on
	// contact and touched a plane already ends its life now, and the next update removes it.
	uint touched = ParticleSteppedSpriteMove(state, spawn.age);
	if (touched != 0 && state.die_on_contact != 0)
		state.lifetime = max(state.age, 1e-6f);

	return state;
}

// Age and move a stepped sprite.
bool ParticleUpdate(inout ParticleSteppedSpriteState state, float dt, ParticleSpriteConstants c)
{
	// Particles at the end of their life are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime)
		return false;

	// Particles that die on contact are removed at the end of the tick that touches a plane.
	uint touched = ParticleSteppedSpriteMove(state, dt);
	return touched == 0 || state.die_on_contact == 0;
}

// Evaluate a stepped sprite's size and colour at its age.
ParticleSpriteRecord ParticleProject(ParticleSteppedSpriteState state, ParticleSpriteConstants c)
{
	// The position and velocity are the integrated state.
	return ParticleSpriteMakeRecord(state.position, state.velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
}

// Bound a stepped sprite by the sphere around its disc and streak. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleSteppedSpriteState state, ParticleSpriteConstants c)
{
	// The record holds the position, size and streak that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
