//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock bird particle type. See particle_bird.hlsli.
#include "pr/game-systems/particles/particle_bird.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"

#define ParticleState ParticleBirdState
#define ParticleRecord ParticleBirdRecord
#define EmitterParams ParticleBirdEmitter
#define TypeConstants ParticleBirdConstants

// Create a bird on its own circle near the emitter, with its own speed, direction, bob and wing beat.
ParticleBirdState ParticleInit(ParticleBirdEmitter params, ParticleSpawn spawn, ParticleBirdConstants c)
{
	// Spread the circle centres over their ball, and pick the radius, speed and direction around the circle.
	float3 dp = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 0), ParticleRandom(spawn.seed, 1), ParticleRandom(spawn.seed, 2)));
	float radius = lerp(params.orbit.x, params.orbit.y, ParticleRandom(spawn.seed, 3));
	float speed = lerp(params.speed.x, params.speed.y, ParticleRandom(spawn.seed, 4));
	float direction = ParticleRandom(spawn.seed, 5) < 0.5f ? -1.0f : 1.0f;

	ParticleBirdState state;
	state.centre = params.position.xyz + dp * params.position.w;
	state.age = spawn.age;
	state.radius = radius;
	state.omega = direction * speed / radius;
	state.theta0 = 6.283185307179586f * ParticleRandom(spawn.seed, 6);
	state.lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 7));
	state.bob_height = params.bob.x * lerp(0.75f, 1.25f, ParticleRandom(spawn.seed, 8));
	state.bob_period = params.bob.y * lerp(0.75f, 1.25f, ParticleRandom(spawn.seed, 9));
	state.bob_phase = 6.283185307179586f * ParticleRandom(spawn.seed, 10);
	state.size = params.size;
	state.flap_rate = params.flap_rate * lerp(0.9f, 1.1f, ParticleRandom(spawn.seed, 11));
	state.colour = params.colour;
	state.seed = spawn.seed;
	state.pad0 = 0;
	return state;
}

// Age a bird. Its flight is a function of its age, so only the age changes.
bool ParticleUpdate(inout ParticleBirdState state, float dt, ParticleBirdConstants c)
{
	state.age += dt;
	return state.age < state.lifetime;
}

// Evaluate a bird's position, orientation and wing angle at its age.
ParticleBirdRecord ParticleProject(ParticleBirdState state, ParticleBirdConstants c)
{
	// Fly around the circle and bob vertically.
	float t = state.age;
	float theta = state.theta0 + state.omega * t;
	float sine;
	float cosine;
	sincos(theta, sine, cosine);
	float bob = state.bob_period > 0.0f ? state.bob_height * sin(6.283185307179586f * t / state.bob_period + state.bob_phase) : 0.0f;
	float3 position = state.centre + float3(state.radius * cosine, state.radius * sine, bob);

	// Face along the circle and bank into the turn. Turning +y by 'theta' about z points it along the anticlockwise tangent, and a half turn more points
	// it along the clockwise one. A negative roll about the forward axis lowers the left wing, which is the inside of an anticlockwise turn.
	float turn = state.omega >= 0.0f ? 1.0f : -1.0f;
	float4 yaw = ParticleQuatAxisAngle(float3(0.0f, 0.0f, 1.0f), theta + (turn > 0.0f ? 0.0f : 3.141592653589793f));
	float4 roll = ParticleQuatAxisAngle(float3(0.0f, 1.0f, 0.0f), -turn * c.bank);

	// Switch between flapping and gliding at a slow pace that differs per bird. The blend is smooth, so the wings never jump.
	float pace = 6.283185307179586f * 0.15f * lerp(0.7f, 1.3f, ParticleRandom(state.seed, 12));
	float gate = smoothstep(-0.3f, 0.3f, sin(pace * t + 6.283185307179586f * ParticleRandom(state.seed, 13)));
	float beat = c.flap_amplitude * sin(6.283185307179586f * state.flap_rate * t + 6.283185307179586f * ParticleRandom(state.seed, 14));
	float flap = lerp(c.dihedral, beat, gate);

	// Grow in after birth and shrink out before death, so birds never pop.
	ParticleBirdRecord record;
	record.pos_size = float4(position, state.size * ParticleMeshScaleRamp(t, state.lifetime, c.fade, c.fade));
	record.orientation = ParticleQuatMul(yaw, roll);
	record.colour = state.colour;
	record.flap = flap;
	record.pad0 = 0;
	record.pad1 = 0;
	return record;
}

// Bound a bird by a ball of its wingspan, which contains the wings at any flap angle and the tail.
float4 ParticleBounds(ParticleBirdState state, ParticleBirdConstants c)
{
	// The record holds the position that the vertex shader draws.
	ParticleBirdRecord record = ParticleProject(state, c);
	return float4(record.pos_size.xyz, state.size);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
