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

// Drift a puff through 'dt' seconds, keep it at its height above the water below its new position, and track the crest under it.
void ParticleFrothMove(inout ParticleFrothState state, float dt)
{
	// The drift is horizontal and constant, and the height above the surface is a function of age, so the motion is exact at any tick rate.
	state.position.xy += state.drift * dt;
	ParticleWaterSample water = ParticleSampleWater(state.position.xy);
	state.position.z = water.height + state.lift + state.rise * state.age;

	// The presence follows the crest under the puff as it falls, and decays exponentially over the linger time once the crest moves on.
	// Presence never rises, so a puff belongs to the crest that formed it, and a later crest passing under a faded puff does not make it pop back.
	// A breaking amount below half the threshold counts as no crest, so the lower faces of breaking crests do not keep puffs alive.
	// The water is sampled once per tick, so the fade depends only slightly on the tick length.
	float crest = smoothstep(0.5f * state.threshold, state.threshold, ParticleWaterBreaking(water, state.lean));
	state.presence = min(state.presence, max(crest, state.presence * exp(-dt / state.linger)));
}

// Form a puff on a breaking crest within the emitter's disc, and move it through the part of the tick after its birth.
ParticleFrothState ParticleInit(ParticleFrothEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Random indices below 16 belong to the drift, lifetime and sprite record; the crest search uses those from 16.
	// The search leans towards faces that slope down along the wind, which are the leading faces of the downwind-travelling crests.
	float2 wind = ParticleSampleWind(float3(params.area.xy, 0.0f)).xy;
	float wind_speed = length(wind);
	float2 lean = wind_speed > 0.0f ? wind * (params.lead / wind_speed) : float2(0.0f, 0.0f);
	float2 xy;
	ParticleWaterSample water;
	bool found = ParticleFindCrest(params.area.xy, params.area.w, params.candidates, params.breaking_threshold, lean, spawn.seed, 16, xy, water);
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
	state.presence = 1.0f;
	state.linger = params.linger;
	state.threshold = params.breaking_threshold;
	state.lean = lean;
	state.pad0 = 0;

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

// Age a puff and move it with the water. Puffs die with the water, and once they have faded behind the crest that formed them.
bool ParticleUpdate(inout ParticleFrothState state, float dt, ParticleSpriteConstants c)
{
	// Puffs at the end of their life, or without water to ride, are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime || g_environment.water == 0)
		return false;

	// Puffs that have faded to almost nothing are removed.
	ParticleFrothMove(state, dt);
	return state.presence > 0.02f;
}

// Evaluate a puff's size and colour at its age.
ParticleSpriteRecord ParticleProject(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The sprite curve shapes the colour and size. The velocity only matters to a streak, and excludes the waves' motion.
	// The presence fades the puff where the crest has moved on. Puffs stay nearly upright, so textures lit from above keep their light on top.
	float3 velocity = float3(state.drift, state.rise);
	ParticleSpriteRecord record = ParticleSpriteMakeRecord(state.position, velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
	float4 colour = ParticleUnpackColour(record.colour);
	colour.a *= state.presence;
	record.colour = ParticlePackColour(colour);
	record.angle = 0.6f * (ParticleRandom(state.seed, 7) - 0.5f);

	// Puffs swell from a small start to their full size at mid life and shrink again, along a half sine, so they never stop growing abruptly.
	float t = saturate(state.age / state.lifetime);
	record.pos_size.w *= 0.3f + 0.7f * sin(3.141592653589793f * t);
	return record;
}

// Bound a puff by the sphere around its sprite. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The record holds the position and size that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"