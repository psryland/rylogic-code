//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock spray particle type. See particle_spray.hlsli.
#include "pr/game-systems/particles/particle_spray.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/particles/particle_water.hlsli"

#define ParticleState ParticleSteppedSpriteState
#define ParticleRecord ParticleSpriteRecord
#define EmitterParams ParticleSprayEmitter
#define TypeConstants ParticleSpriteConstants

// Move a droplet through 'dt' seconds of its environment. Returns true when it falls back into the water.
bool ParticleSprayMove(inout ParticleSteppedSpriteState state, float dt)
{
	// Integrate the free flight with the wind at the start of the tick.
	float3 wind = ParticleSampleWind(state.position);
	ParticleIntegrate(state.position, state.velocity, state.gravity * g_environment.gravity + state.accel, state.drag, wind, dt);

	// Only falling droplets can land, so droplets leaving the surface are not lost to a wave that rises under them.
	return state.velocity.z < 0.0f && state.position.z < ParticleSampleWater(state.position.xy).height;
}

// Throw a droplet from a breaking crest within the emitter's disc, and move it through the part of the tick after its birth.
ParticleSteppedSpriteState ParticleInit(ParticleSprayEmitter params, ParticleSpawn spawn, ParticleSpriteConstants c)
{
	// Random indices below 16 belong to the launch and the sprite record; the crest search uses those from 16.
	// With a lead and a front, droplets rise from the same leading foam front as froth, as if the wind tears them off the foam.
	float2 wind = ParticleSampleWind(float3(params.area.xy, 0.0f)).xy;
	float2 lean = ParticleAlongWind(wind, params.lead);
	float2 xy;
	ParticleWaterSample water;
	bool found = ParticleFindCrest(params.area.xy, params.area.w, params.candidates, params.breaking_threshold, lean, spawn.seed, 16, xy, water);
	if (found)
		ParticleFindCrestFront(xy, water, ParticleAlongWind(wind, params.front), params.breaking_threshold, lean);

	float3 dv = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 3), ParticleRandom(spawn.seed, 4), ParticleRandom(spawn.seed, 5)));
	float lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));

	// Droplets leave the surface along its normal, picking up some of the wind.
	ParticleSteppedSpriteState state;
	state.position = float3(xy, water.height);
	state.age = spawn.age;
	state.velocity = water.normal * params.launch.x + dv * params.launch.y + ParticleSampleWind(state.position) * params.launch.z;
	state.lifetime = lifetime;
	state.accel = float3(0.0f, 0.0f, 0.0f);
	state.drag = params.launch.w;
	state.radius = 0.0f;
	state.collision_set = -1;
	state.seed = spawn.seed;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	state.gravity = params.gravity;
	state.die_on_contact = 0;
	state.pad0 = 0;
	state.pad1 = 0;
	state.pad2 = 0;

	// A birth without a crest makes no droplet. New particles are always kept for one tick, so it is drawn at zero size, away from the water's
	// far-below height, and the next update removes it.
	if (!found)
	{
		state.position = float3(params.area.xy, 0.0f);
		state.velocity = float3(0.0f, 0.0f, 0.0f);
		state.size0 = 0.0f;
		state.size1 = 0.0f;
		state.lifetime = max(state.age, 1e-6f);
		return state;
	}

	// A droplet born part way through the tick has already flown for 'spawn.age'. One that is already back in the water ends its life now.
	if (ParticleSprayMove(state, spawn.age))
		state.lifetime = max(state.age, 1e-6f);

	return state;
}

// Age and move a droplet.
bool ParticleUpdate(inout ParticleSteppedSpriteState state, float dt, ParticleSpriteConstants c)
{
	// Droplets at the end of their life are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime)
		return false;

	// Droplets are removed at the end of the tick that takes them back into the water.
	return !ParticleSprayMove(state, dt);
}

// Evaluate a droplet's size and colour at its age.
ParticleSpriteRecord ParticleProject(ParticleSteppedSpriteState state, ParticleSpriteConstants c)
{
	// The position and velocity are the integrated state.
	return ParticleSpriteMakeRecord(state.position, state.velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
}

// Bound a droplet by the sphere around its disc and streak. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleSteppedSpriteState state, ParticleSpriteConstants c)
{
	// The record holds the position, size and streak that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
