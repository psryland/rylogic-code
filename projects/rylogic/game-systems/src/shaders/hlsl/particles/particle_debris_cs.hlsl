//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Emit and update kernels of the stock debris particle type. See particle_debris.hlsli.
#include "pr/game-systems/particles/particle_debris.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"

#define ParticleState ParticleDebrisState
#define ParticleRecord ParticleMeshRecord
#define EmitterParams ParticleDebrisEmitter
#define TypeConstants ParticleDebrisConstants

// Move a piece through 'dt' seconds of its environment, spinning it and bouncing it off its collision planes.
void ParticleDebrisMove(inout ParticleDebrisState state, float dt)
{
	// The flutter push swings around the horizontal plane at two unrelated rates, from per-piece phases. It is held at its value in the middle of the tick.
	float flutter_phase = 6.283185307179586f * state.flutter_rate * (state.age - 0.5f * dt);
	float3 flutter = state.flutter * float3(
		cos(flutter_phase + 6.283185307179586f * ParticleRandom(state.seed, 10)),
		sin(0.73f * flutter_phase + 6.283185307179586f * ParticleRandom(state.seed, 11)),
		0.0f);

	// Integrate the free flight with the wind at the start of the tick, then resolve contacts along the tick's motion.
	float3 start = state.position;
	float3 end = state.position;
	float3 velocity = state.velocity;
	float3 wind = ParticleSampleWind(start);
	ParticleIntegrate(end, velocity, state.gravity * g_environment.gravity + state.accel + flutter, state.drag, wind, dt);
	uint touched = ParticleCollidePlanes(state.collision_set, state.radius, start, end, velocity);
	state.position = end;
	state.velocity = velocity;

	// Turn about the spin axis. Turns about one axis add, so the orientation does not depend on how time is divided into ticks.
	state.orientation = normalize(ParticleQuatMul(ParticleQuatAxisAngle(state.spin_axis, state.spin_rate * dt), state.orientation));

	// Each contact takes some of the spin and flutter, so pieces come to rest on the surfaces they land on.
	if (touched != 0)
	{
		float keep = pow(saturate(1.0f - state.settle), float(touched));
		state.spin_rate *= keep;
		state.flutter *= keep;
	}
}

// Create a piece from its emitter's parameters, and move it through the part of the tick after its birth.
ParticleDebrisState ParticleInit(ParticleDebrisEmitter params, ParticleSpawn spawn, ParticleDebrisConstants c)
{
	// Spread the birth position and velocity over their balls, and pick the lifetime, size, spin and colour within their ranges.
	float3 dp = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 0), ParticleRandom(spawn.seed, 1), ParticleRandom(spawn.seed, 2)));
	float3 dv = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 3), ParticleRandom(spawn.seed, 4), ParticleRandom(spawn.seed, 5)));
	float4 colour = lerp(ParticleUnpackColour(params.colour0), ParticleUnpackColour(params.colour1), ParticleRandom(spawn.seed, 9));

	// The starting orientation and the spin axis are uniformly random.
	float3 turn_axis = ParticleRandomDirection(float2(ParticleRandom(spawn.seed, 12), ParticleRandom(spawn.seed, 13)));
	float turn_angle = 6.283185307179586f * ParticleRandom(spawn.seed, 14);
	float3 spin_axis = ParticleRandomDirection(float2(ParticleRandom(spawn.seed, 15), ParticleRandom(spawn.seed, 16)));

	ParticleDebrisState state;
	state.position = params.position.xyz + dp * params.position.w;
	state.age = spawn.age;
	state.velocity = params.velocity.xyz + dv * params.velocity.w;
	state.lifetime = lerp(params.lifetime.x, params.lifetime.y, ParticleRandom(spawn.seed, 6));
	state.orientation = ParticleQuatAxisAngle(turn_axis, turn_angle);
	state.spin_axis = spin_axis;
	state.spin_rate = lerp(params.spin.x, params.spin.y, ParticleRandom(spawn.seed, 8));
	state.accel = params.accel.xyz;
	state.drag = params.accel.w;
	state.size = lerp(params.size.x, params.size.y, ParticleRandom(spawn.seed, 7));
	state.radius = params.radius;
	state.collision_set = params.collision_set;
	state.seed = spawn.seed;
	state.colour = ParticlePackColour(colour);
	state.gravity = params.gravity;
	state.settle = params.settle;
	state.flutter = params.flutter;
	state.flutter_rate = params.flutter_rate;
	state.pad0 = 0;
	state.pad1 = 0;
	state.pad2 = 0;

	// A piece born part way through the tick has already lived for 'spawn.age'.
	ParticleDebrisMove(state, spawn.age);
	return state;
}

// Age and move a piece.
bool ParticleUpdate(inout ParticleDebrisState state, float dt, ParticleDebrisConstants c)
{
	// Pieces at the end of their life are removed without moving.
	state.age += dt;
	if (state.age >= state.lifetime)
		return false;

	ParticleDebrisMove(state, dt);
	return true;
}

// Place a piece's mesh, scaled by the type's grow and shrink ramps.
ParticleMeshRecord ParticleProject(ParticleDebrisState state, ParticleDebrisConstants c)
{
	// The record turns and scales the batch's mesh.
	ParticleMeshRecord record;
	record.pos_size = float4(state.position, state.size * ParticleMeshScaleRamp(state.age, state.lifetime, c.grow, c.shrink));
	record.orientation = state.orientation;
	record.colour = state.colour;
	record.pad0 = 0;
	record.pad1 = 0;
	record.pad2 = 0;
	return record;
}

// Bound a piece by the ball that contains the batch's mesh at the piece's full size.
float4 ParticleBounds(ParticleDebrisState state, ParticleDebrisConstants c)
{
	// The ramps only shrink the mesh, so the full size is conservative.
	return float4(state.position, c.mesh_radius * state.size);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
