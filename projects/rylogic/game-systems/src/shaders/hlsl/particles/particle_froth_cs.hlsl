//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock froth particle type. See particle_froth.hlsli.
#include "pr/game-systems/particles/particle_froth.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/particles/particle_water.hlsli"

#define ParticleState ParticleFrothState
#define ParticleRecord ParticleSpriteRecord
#define EmitterParams ParticleFrothEmitter
#define TypeConstants ParticleSpriteConstants

// Drift a puff through 'dt' seconds and keep it at its height above the water below its new position.
void ParticleFrothMove(inout ParticleFrothState state, float dt)
{
	// The drift is horizontal and constant, and the height above the surface is a function of age, so the motion is exact at any tick rate.
	state.position.xy += state.drift * dt;
	state.position.z = ParticleSampleWater(state.position.xy).height + state.lift + state.rise * state.age;
}

// Form a puff on a breaking crest within the emitter's disc, and move it through the part of the tick after its birth.
ParticleFrothState ParticleInit(ParticleFrothEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Random indices below 16 belong to the drift, lifetime and sprite record; the crest search uses those from 16.
	float2 xy;
	ParticleWaterSample water;
	bool found = ParticleFindCrest(params.area.xy, params.area.w, params.candidates, params.foam_threshold, spawn.seed, 16, xy, water);
	float spread_r = params.spread * sqrt(ParticleRandom(spawn.seed, 3));
	float spread_a = 6.283185307179586f * ParticleRandom(spawn.seed, 4);
	float lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));

	// Puffs drift with a fraction of the wind, spread evenly over a disc of velocities.
	ParticleFrothState state;
	state.position = float3(xy, 0.0f);
	state.age = spawn.age;
	state.drift = ParticleSampleWind(float3(xy, water.height)).xy * params.drift + spread_r * float2(cos(spread_a), sin(spread_a));
	state.lifetime = lifetime;
	state.lift = params.lift;
	state.rise = params.rise;
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.seed = spawn.seed;
	state.pad0 = 0;
	state.pad1 = 0;

	// A birth without a crest makes no froth. New particles are always kept for one tick, so it is drawn at zero size, away from the water's
	// far-below height, and the next update removes it.
	if (!found)
	{
		state.position = float3(params.area.xy, 0.0f);
		state.drift = float2(0.0f, 0.0f);
		state.size0 = 0.0f;
		state.size1 = 0.0f;
		state.lifetime = max(state.age, 1e-6f);
		return state;
	}

	// A puff born part way through the tick has already drifted for 'spawn.age'.
	ParticleFrothMove(state, spawn.age);
	return state;
}

// Age a puff and move it with the water. Puffs die with the water.
bool ParticleUpdate(inout ParticleFrothState state, float dt, ParticleSpriteConstants c)
{
	// Puffs at the end of their life, or without water to ride, are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime || g_environment.water == 0)
		return false;

	ParticleFrothMove(state, dt);
	return true;
}

// Evaluate a puff's size and colour at its age.
ParticleSpriteRecord ParticleProject(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The sprite curve shapes the growth and fade. The velocity only matters to a streak, and excludes the waves' motion.
	float3 velocity = float3(state.drift, state.rise);
	return ParticleSpriteMakeRecord(state.position, velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
}

// Bound a puff by the sphere around its sprite. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The record holds the position and size that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"