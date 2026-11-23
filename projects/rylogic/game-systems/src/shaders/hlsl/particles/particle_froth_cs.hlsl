//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock froth particle type. See particle_froth.hlsli.
#include "pr/game-systems/particles/particle_froth.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/particles/particle_water.hlsli"

#define ParticleState ParticleFrothState
#define ParticleRecord ParticleFrothRecord
#define EmitterParams ParticleFrothEmitter
#define TypeConstants ParticleSpriteConstants

// Drift a puff through 'dt' seconds, keep it at its height above the water below its new position, and track the crest and surface under it.
void ParticleFrothMove(inout ParticleFrothState state, float dt)
{
	// The drift is horizontal and constant, and the height above the surface is a function of age, so the motion is exact at any tick rate.
	// The surface normal under the puff is kept for the draw, which cuts the puff off at the surface.
	state.position.xy += state.drift * dt;
	ParticleWaterSample water = ParticleSampleWater(state.position.xy);
	state.position.z = water.height + state.lift + state.rise * state.age;
	state.normal_xy = ParticleFrothPackNormal(water.normal);

	// The presence follows the crest foam under the puff as the crest passes, and decays exponentially over the linger time once the foam has gone.
	// Presence never rises, so a puff belongs to the crest that formed it, and a later crest passing under a faded puff does not make it pop back.
	// Foam below half the minimum counts as none, so the fringes of the crest foam do not keep puffs alive.
	// The water is sampled once per tick, so the fade depends only slightly on the tick length.
	float crest = smoothstep(0.5f * state.foam_min, state.foam_min, water.foam);
	state.presence = min(state.presence, max(crest, state.presence * exp(-dt / state.linger)));
}

// Form a puff on strong crest foam within the emitter's disc, and move it through the part of the tick after its birth.
ParticleFrothState ParticleInit(ParticleFrothEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Random indices below 16 belong to the drift, lifetime and sprite record; the foam search uses those from 16.
	float2 xy;
	ParticleWaterSample water;
	bool found = ParticleFindFoam(params.area.xy, params.area.w, params.candidates, params.foam_min, spawn.seed, 16, xy, water);

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
	state.foam_min = params.foam_min;
	state.normal_xy = ParticleFrothPackNormal(water.normal);
	state.fade = f32tof16(params.fade_near) | (f32tof16(params.fade_far) << 16);
	state.squash = params.squash;

	// A birth without crest foam makes no froth. New particles are always kept for one tick, so it is drawn at zero size, away from the water's
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

// Evaluate a puff's size, colour and the surface it is cut off at, at its age.
ParticleFrothRecord ParticleProject(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The sprite curve shapes the colour and size. Froth never streaks, so the velocity is unused.
	// The presence fades the puff where the crest has moved on. Puffs stay nearly upright, so textures lit from above keep their light on top.
	ParticleSpriteRecord sprite = ParticleSpriteMakeRecord(state.position, float3(0.0f, 0.0f, 0.0f), state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
	float4 colour = ParticleUnpackColour(sprite.colour);
	colour.a *= state.presence;

	// Puffs swell from a small start to their full size at mid life and shrink again, along a half sine, so they never stop growing abruptly.
	float t = saturate(state.age / state.lifetime);
	ParticleFrothRecord record;
	record.pos_size = float4(sprite.pos_size.xyz, sprite.pos_size.w * (0.3f + 0.7f * sin(3.141592653589793f * t)));
	record.colour = ParticlePackColour(colour);
	record.shape = ParticleFrothPackShape(0.6f * (ParticleRandom(state.seed, 7) - 0.5f), state.squash, state.lift + state.rise * state.age);
	record.normal_xy = state.normal_xy;
	record.fade = state.fade;
	return record;
}

// Bound a puff by the sphere around its sprite. See ParticleFrothRecordBounds.
float4 ParticleBounds(ParticleFrothState state, ParticleSpriteConstants c)
{
	// The record holds the position and size that the vertex shader draws.
	return ParticleFrothRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"