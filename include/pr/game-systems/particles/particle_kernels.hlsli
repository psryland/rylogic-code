//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The emit and update kernels of a particle type. A particle type's compute shader defines its own types and functions, then includes this file,
// and is compiled with the entry points 'ParticleEmitCS' and 'ParticleUpdateCS'.
//
// Define before including:
//   struct ParticleState    - The state of one particle. Its size is the type's state stride.
//   struct ParticleRecord   - What the type's vertex shader reads for one particle. Its size is the type's record stride.
//   struct EmitterParams    - The caller-defined parameters of one emitter. Its size is the type's parameter stride.
//   struct TypeConstants    - The type's immutable constants, which its vertex shader also reads.
//
//   ParticleState ParticleInit(EmitterParams params, ParticleSpawn spawn, TypeConstants c)
//     Return a new particle's state at the end of the tick, 'spawn.age' simulated seconds after its birth.
//     A new particle is always drawn and is next given to ParticleUpdate in the next tick, even if it should already have died.
//   bool ParticleUpdate(inout ParticleState state, float dt, TypeConstants c)
//     Advance 'state' by 'dt' simulated seconds (which may be zero). Return false when the particle has died, and its slot is then reused.
//     The result must depend only on the inputs, so particles behave the same at any tick rate and in slow motion.
//   ParticleRecord ParticleProject(ParticleState state, TypeConstants c)
//     Return the render record of a live particle.
//
// Particles never read emitters or other particles after they are born, so every particle updates independently.
#ifndef PR_GAME_SYSTEMS_PARTICLE_KERNELS_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_KERNELS_HLSLI
#include "pr/game-systems/particles/particles.hlsli"

ConstantBuffer<ParticleTickConstants> g_tick : register(b0);
ConstantBuffer<TypeConstants> g_type : register(b1);
RWStructuredBuffer<uint> g_counters : register(u0);
RWStructuredBuffer<uint> g_free : register(u1);
RWStructuredBuffer<uint> g_live : register(u2);
RWStructuredBuffer<ParticleState> g_states : register(u3);
RWStructuredBuffer<ParticleRecord> g_records : register(u4);
RWStructuredBuffer<ParticleEmitterRuntime> g_runtime : register(u5);
StructuredBuffer<ParticleEmitterHeader> g_headers : register(t0);
StructuredBuffer<EmitterParams> g_params : register(t1);

// Append the live particle in 'slot' to the written live list, with its render record at the same position.
void ParticleKeep(uint slot, ParticleState state)
{
	// The written list is the one not read this tick.
	uint next = 1u - g_tick.cur;
	uint index;
	InterlockedAdd(g_counters[ParticleCounterLive0 + next], 1u, index);
	g_live[next * g_tick.capacity + index] = slot;
	g_records[index] = ParticleProject(state, g_type);
}

// Advance each particle in the read live list, keeping the survivors and returning the slots of the dead to the free stack.
// The dispatch size comes from the args buffer, so threads beyond the list length do nothing.
numthreads(ParticleUpdateCS, 64, 1, 1)
void ParticleUpdateCS(uint3 dtid : SV_DispatchThreadID)
{
	// Each thread owns one entry of the read list.
	uint cur = g_tick.cur;
	if (dtid.x >= g_counters[ParticleCounterLive0 + cur])
		return;

	uint slot = g_live[cur * g_tick.capacity + dtid.x];
	ParticleState state = g_states[slot];
	if (ParticleUpdate(state, g_tick.dt, g_type))
	{
		// Survivors keep their slot.
		g_states[slot] = state;
		ParticleKeep(slot, state);
	}
	else
	{
		// The free stack only grows during the update pass, so pushes cannot overlap the pops of the emit pass that follows.
		uint index;
		InterlockedAdd(g_counters[ParticleCounterFree], 1u, index);
		g_free[index] = slot;
	}
}

groupshared uint gs_base;
groupshared uint gs_free;
groupshared uint gs_admitted;
groupshared uint gs_continuous;
groupshared uint gs_sequence;

// Emit the particles due from one emitter slot per group this tick.
numthreads(ParticleEmitCS, 64, 1, 1)
void ParticleEmitCS(uint3 gid : SV_GroupID, uint gi : SV_GroupIndex)
{
	// One thread decides how many particles the emitter makes this tick, and claims their slots.
	uint emitter = gid.x;
	ParticleEmitterHeader header = g_headers[emitter];
	if (gi == 0)
	{
		// A slot given to a new emitter starts its emission progress afresh.
		ParticleEmitterRuntime runtime = g_runtime[emitter];
		if (runtime.generation != header.generation)
		{
			runtime.accum = 0.0f;
			runtime.burst_emitted = 0;
			runtime.generation = header.generation;
			runtime.sequence = 0;
		}

		// Continuous emission carries the fraction of a particle over to later ticks, so the count over time matches the rate at any tick rate.
		// Bursts requested while the emitter is inactive are discarded.
		uint continuous = 0;
		uint burst = 0;
		if ((header.flags & ParticleEmitterActive) != 0)
		{
			float due = runtime.accum + header.rate * g_tick.dt;
			continuous = uint(floor(due));
			runtime.accum = due - float(continuous);
			burst = header.burst_total - runtime.burst_emitted;
		}
		runtime.burst_emitted = header.burst_total;

		// Claim slots from the top of the free stack. The stack does not change during the emit pass, so the admitted particles are those that fit
		// below its top. Particles that do not fit are dropped, bursts first.
		uint requested = continuous + burst;
		uint base = 0;
		if (requested != 0)
			InterlockedAdd(g_counters[ParticleCounterTaken], requested, base);

		uint free = g_counters[ParticleCounterFree];
		uint admitted = base < free ? min(requested, free - base) : 0u;
		if (requested != 0)
		{
			// Cumulative counts for diagnostics.
			InterlockedAdd(g_counters[ParticleCounterRequested], requested);
			InterlockedAdd(g_counters[ParticleCounterAdmitted], admitted);
			InterlockedAdd(g_counters[ParticleCounterDropped], requested - admitted);
		}

		gs_base = base;
		gs_free = free;
		gs_admitted = admitted;
		gs_continuous = continuous;
		gs_sequence = runtime.sequence;
		runtime.sequence += requested;
		g_runtime[emitter] = runtime;
	}
	GroupMemoryBarrierWithGroupSync();

	// The group creates the admitted particles together.
	EmitterParams params = g_params[emitter];
	for (uint i = gi; i < gs_admitted; i += ParticleGroupSize)
	{
		// Continuous particles are born evenly through the tick, and burst particles at its end. Each particle's random seed comes from its
		// emitter's seed and its sequence number only, so emitters with equal seeds make equal particles wherever they and their particles land.
		ParticleSpawn spawn;
		spawn.emitter = emitter;
		spawn.sequence = gs_sequence + i;
		spawn.seed = ParticleHash(header.seed ^ ParticleHash(spawn.sequence));
		spawn.age = i < gs_continuous ? g_tick.dt * (float(gs_continuous - i) - 0.5f) / float(gs_continuous) : 0.0f;

		uint slot = g_free[gs_free - 1 - (gs_base + i)];
		ParticleState state = ParticleInit(params, spawn, g_type);
		g_states[slot] = state;
		ParticleKeep(slot, state);
	}
}

#endif
