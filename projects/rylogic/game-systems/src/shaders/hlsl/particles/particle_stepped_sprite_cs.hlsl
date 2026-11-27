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

// Return the level axis that a stepped sprite rolls about, scaled by its outward roll speed (m/s). Zero for a sprite that does not roll.
float3 ParticleSteppedSpriteRoll(ParticleSteppedSpriteState state)
{
	// The axis and speed are packed together as half floats.
	return float3(f16tof32(state.roll_xy & 0xFFFF), f16tof32(state.roll_xy >> 16), f16tof32(state.roll_z & 0xFFFF));
}

// Return the rate (rad/s) that a rolling stepped sprite's roll swirls about the vertical.
float ParticleSteppedSpriteSwirl(ParticleSteppedSpriteState state)
{
	// The swirl shares a word with the roll axis's z.
	return f16tof32(state.roll_z >> 16);
}

// Return the integrals from 0 to 't' of e^(-k τ) cos(f τ) (x) and e^(-k τ) sin(f τ) (y), for a non-negative decay rate 'k' and any frequency 'f'.
float2 ParticleDecayingWave(float k, float f, float t)
{
	// With d = k² + f² and e = e^(-k t), the integrals are (k (1 - e cos) + f e sin) / d and (f (1 - e cos) - k e sin) / d. Without decay or
	// turning, the wave is a constant one.
	float d = k * k + f * f;
	if (d == 0.0f)
		return float2(t, 0.0f);

	float e = exp(-k * t);
	float s, c;
	sincos(f * t, s, c);
	return float2(k * (1.0f - e * c) + f * e * s, f * (1.0f - e * c) - k * e * s) / d;
}

// Return how far a rolling stepped sprite has rolled by 'age'. The roll starts outward and level, slows at the drag rate, turns upward about the
// roll axis at the curl rate, and turns about the vertical at the swirl rate. A curl no faster than the drag fades the roll out before it can turn
// back down.
float3 ParticleSteppedSpriteRollOffset(ParticleSteppedSpriteState state, float age)
{
	// The roll velocity is speed * e^(-k t) * (level(t) * cos(w t) + up * sin(w t)) for drag k and curl w, where the level direction
	// level(t) = out * cos(W t) + side * sin(W t) turns at the swirl rate W. Products of the cosines and sines become sums of single waves at the
	// frequencies w ± W, so each part of the offset is a sum of decaying wave integrals.
	float3 roll = ParticleSteppedSpriteRoll(state);
	float speed = length(roll);
	if (speed == 0.0f)
		return float3(0, 0, 0);

	float3 axis = roll / speed;
	float3 up = -normalize(g_environment.gravity);
	float3 out_dir = cross(up, axis);
	float3 side = cross(up, out_dir);
	float k = state.drag;
	float w = state.curl;
	float W = ParticleSteppedSpriteSwirl(state);
	float rise = ParticleDecayingWave(k, w, age).y;
	float outward = 0.5f * (ParticleDecayingWave(k, w - W, age).x + ParticleDecayingWave(k, w + W, age).x);
	float sideways = 0.5f * (ParticleDecayingWave(k, W + w, age).y + ParticleDecayingWave(k, W - w, age).y);
	return speed * (out_dir * outward + side * sideways + up * rise);
}
// Move a stepped sprite through the 'dt' seconds of its environment that end at its current age, bouncing off its collision planes. Returns the
// number of planes touched.
uint ParticleSteppedSpriteMove(inout ParticleSteppedSpriteState state, float dt)
{
	// Integrate the free flight with the wind at the start of the tick, then resolve contacts along the tick's motion. The roll is a fixed motion
	// over the particle's age that is added on top of the drift, so the wind carries a rolling puff without bending its roll.
	float3 start = state.position;
	float3 end = state.position;
	float3 velocity = state.velocity;
	float3 wind = ParticleSampleWind(start);
	ParticleIntegrate(end, velocity, state.gravity * g_environment.gravity + state.accel, state.drag, wind, dt);
	end += ParticleSteppedSpriteRollOffset(state, state.age) - ParticleSteppedSpriteRollOffset(state, state.age - dt);
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

	// Each particle varies its drag and roll by its own factors, so the particles of a burst move apart rather than together.
	float4 vary = 1.0f + params.variety * (2.0f * float4(ParticleRandom(spawn.seed, 7), ParticleRandom(spawn.seed, 8), ParticleRandom(spawn.seed, 9), ParticleRandom(spawn.seed, 10)) - 1.0f);

	ParticleSteppedSpriteState state;
	state.position = params.position.xyz + dp * params.position.w;
	state.age = spawn.age;
	state.velocity = params.velocity.xyz + dv * params.velocity.w;
	state.lifetime = lifetime;
	state.accel = params.accel.xyz;
	state.drag = params.accel.w * vary.x;
	state.radius = params.radius;
	state.collision_set = params.collision_set;
	state.seed = spawn.seed;
	state.colour0 = params.colour0;
	state.colour1 = params.colour1;
	state.size0 = params.size.x;
	state.size1 = params.size.y;
	state.gravity = params.gravity;
	state.die_on_contact = params.die_on_contact;
	state.curl = 0.0f;
	state.roll_xy = 0;
	state.roll_z = 0;

	// Outward motion is level with the ground, along the birth's level offset from the emitter, so a burst spreads out as a ring. The offset in the
	// unit ball is used, so the direction is defined even when births are not spread. A curling or swirling sprite rolls outward instead of carrying
	// the outward speed in its velocity, so the wind carries its roll without bending it (see ParticleSteppedSpriteRollOffset). Its roll axis is level
	// and across the outward direction, so that a positive curl turns the roll upward. A birth straight above or below the emitter has no level direction.
	float3 gravity = g_environment.gravity;
	if (params.outward != 0.0f && dot(gravity, gravity) > 0.0f)
	{
		float3 up = -normalize(gravity);
		float3 level = dp - dot(dp, up) * up;
		float len = length(level);
		if (len > 1e-4f)
		{
			float3 out_dir = level / len;
			if (params.curl != 0.0f || params.swirl != 0.0f)
			{
				float3 roll = cross(out_dir, up) * (params.outward * vary.y);
				state.curl = params.curl * vary.z;
				state.roll_xy = f32tof16(roll.x) | (f32tof16(roll.y) << 16);
				state.roll_z = f32tof16(roll.z) | (f32tof16(params.swirl * vary.w) << 16);
			}
			else
			{
				state.velocity += out_dir * (params.outward * vary.y);
			}
		}
	}

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
	// The position and velocity are the integrated state. A rolling sprite spins with its roll: about the roll axis at the curl rate and about the
	// vertical at the swirl rate, combined into one spin about their sum.
	ParticleSpriteRecord record = ParticleSpriteMakeRecord(state.position, state.velocity, state.age, state.lifetime, state.seed, state.colour0, state.colour1, state.size0, state.size1, c);
	float3 roll = ParticleSteppedSpriteRoll(state);
	float3 spin = state.curl * normalize(roll) - ParticleSteppedSpriteSwirl(state) * normalize(g_environment.gravity);
	if (dot(roll, roll) != 0.0f && dot(spin, spin) != 0.0f)
		ParticleSpriteSetSpin(record, normalize(spin), length(spin) * state.age);

	return record;
}

// Bound a stepped sprite by the sphere around its disc and streak. See ParticleSpriteRecordBounds.
float4 ParticleBounds(ParticleSteppedSpriteState state, ParticleSpriteConstants c)
{
	// The record holds the position, size and streak that the vertex shader draws.
	return ParticleSpriteRecordBounds(ParticleProject(state, c));
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
