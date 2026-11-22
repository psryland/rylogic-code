//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock froth particle type. See particle_froth.hlsli.
#include "pr/game-systems/particles/particle_froth.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/particles/particle_water.hlsli"

#define ParticleState ParticleFrothState
#define ParticleRecord ParticleMeshRecord
#define EmitterParams ParticleFrothEmitter
#define TypeConstants ParticleFrothConstants

// Drift a patch through 'dt' seconds and lay it on the water below its new position, turned by its fixed random angle.
void ParticleFrothMove(inout ParticleFrothState state, float dt)
{
	// The drift is horizontal and constant, so the motion is exact at any tick rate. The height and tilt follow the waves.
	state.position.xy += state.drift * dt;
	ParticleWaterSample water = ParticleSampleWater(state.position.xy);
	state.position.z = water.height + state.lift;
	float4 spin = ParticleQuatAxisAngle(float3(0.0f, 0.0f, 1.0f), 6.283185307179586f * ParticleRandom(state.seed, 7));
	state.orientation = ParticleQuatMul(ParticleQuatFromZ(water.normal), spin);
}

// Form a patch on a breaking crest within the emitter's disc, and move it through the part of the tick after its birth.
ParticleFrothState ParticleInit(ParticleFrothEmitter params, ParticleSpawn spawn, ParticleFrothConstants c)
{
	// Random indices below 16 belong to the drift, lifetime and spin; the crest search uses those from 16.
	float2 xy;
	ParticleWaterSample water;
	bool found = ParticleFindCrest(params.area.xy, params.area.w, params.candidates, params.foam_threshold, spawn.seed, 16, xy, water);
	float spread_r = params.spread * sqrt(ParticleRandom(spawn.seed, 3));
	float spread_a = 6.283185307179586f * ParticleRandom(spawn.seed, 4);
	float lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));

	// Patches drift with a fraction of the wind, spread evenly over a disc of velocities.
	ParticleFrothState state;
	state.position = float3(xy, 0.0f);
	state.age = spawn.age;
	state.drift = ParticleSampleWind(float3(xy, water.height)).xy * params.drift + spread_r * float2(cos(spread_a), sin(spread_a));
	state.lifetime = lifetime;
	state.lift = params.lift;
	state.orientation = float4(0.0f, 0.0f, 0.0f, 1.0f);
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.seed = spawn.seed;
	state.pad0 = 0;
	state.pad1 = 0;
	state.pad2 = 0;

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

	// A patch born part way through the tick has already drifted for 'spawn.age'.
	ParticleFrothMove(state, spawn.age);
	return state;
}

// Age a patch and move it with the water. Patches die with the water.
bool ParticleUpdate(inout ParticleFrothState state, float dt, ParticleFrothConstants c)
{
	// Patches at the end of their life, or without water to ride, are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime || g_environment.water == 0)
		return false;

	ParticleFrothMove(state, dt);
	return true;
}

// Evaluate a patch's size and colour at its age.
ParticleMeshRecord ParticleProject(ParticleFrothState state, ParticleFrothConstants c)
{
	// Size and colour blend from birth to death, and the alpha fades in and out at the ends of the life.
	float t = saturate(state.age / state.lifetime);
	float4 colour = lerp(ParticleUnpackColour(state.colour0), ParticleUnpackColour(state.colour1), t);
	colour.a *= ParticleMeshScaleRamp(state.age, state.lifetime, c.fade_in, c.fade_out);

	ParticleMeshRecord record;
	record.pos_size = float4(state.position, lerp(state.size0, state.size1, t));
	record.orientation = state.orientation;
	record.colour = ParticlePackColour(colour);
	record.pad0 = 0;
	record.pad1 = 0;
	record.pad2 = 0;
	return record;
}

// Bound a patch by the sphere around its disc.
float4 ParticleBounds(ParticleFrothState state, ParticleFrothConstants c)
{
	// The flat sprite vertex shader draws a disc of the record's diameter around its position.
	return float4(state.position, 0.5f * ParticleProject(state, c).pos_size.w);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
