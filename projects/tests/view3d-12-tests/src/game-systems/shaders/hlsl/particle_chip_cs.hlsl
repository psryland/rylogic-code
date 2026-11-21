//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Emit and update kernels of the test chip particle type. See particle_chip.hlsli.
#include "particle_chip.hlsli"

#define ParticleState ParticleChipState
#define ParticleRecord ParticleChipRecord
#define EmitterParams ParticleChipEmitter
#define TypeConstants ParticleChipConstants

// Create a chip at its emitter's position, offset along x by its sequence number, with a random velocity spread and spin axis.
ParticleChipState ParticleInit(ParticleChipEmitter params, ParticleSpawn spawn, ParticleChipConstants c)
{
	// A uniformly random unit axis from a random height and angle around it.
	float3 dv = ParticleRandomInBall(float3(ParticleRandom(spawn.seed, 0), ParticleRandom(spawn.seed, 1), ParticleRandom(spawn.seed, 2)));
	float z = 2.0f * ParticleRandom(spawn.seed, 3) - 1.0f;
	float phi = 6.283185307179586f * ParticleRandom(spawn.seed, 4);
	float r = sqrt(saturate(1.0f - z * z));

	ParticleChipState state;
	state.p0 = params.position.xyz + float3(params.spacing * float(spawn.sequence), 0.0f, 0.0f);
	state.age = spawn.age;
	state.v0 = params.velocity.xyz + dv * params.velocity.w;
	state.lifetime = params.lifetime;
	state.axis = float3(r * cos(phi), r * sin(phi), z);
	state.spin = params.spin;
	state.colour = params.colour;
	state.size = params.position.w;
	state.pad0 = 0;
	state.pad1 = 0;
	return state;
}

// Age a chip. Its position and orientation are functions of its age, so only the age changes.
bool ParticleUpdate(inout ParticleChipState state, float dt, ParticleChipConstants c)
{
	state.age += dt;
	return state.age < state.lifetime;
}

// Evaluate a chip's position and orientation at its age.
ParticleChipRecord ParticleProject(ParticleChipState state, ParticleChipConstants c)
{
	// Fly on constant acceleration while turning at a constant rate about the chip's axis.
	float half_angle = 0.5f * state.spin * state.age;

	ParticleChipRecord record;
	record.pos_size = float4(state.p0 + state.v0 * state.age + 0.5f * c.gravity.xyz * state.age * state.age, state.size);
	record.orientation = float4(state.axis * sin(half_angle), cos(half_angle));
	record.colour = state.colour;
	record.pad0 = 0;
	record.pad1 = 0;
	record.pad2 = 0;
	return record;
}

// Bound a chip by the sphere around a cube of its length. Chip width and thickness are fractions of the length, so the box fits inside.
float4 ParticleBounds(ParticleChipState state, ParticleChipConstants c)
{
	// Half the diagonal of a cube of the chip's length.
	ParticleChipRecord record = ParticleProject(state, c);
	return float4(record.pos_size.xyz, 0.8660254f * record.pos_size.w);
}

#include "pr/game-systems/particles/particle_kernels.hlsli"
