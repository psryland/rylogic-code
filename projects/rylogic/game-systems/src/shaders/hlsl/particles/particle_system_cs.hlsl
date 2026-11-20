//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Kernels that the particle system runs for every particle type, around the type's own emit and update kernels (see particle_kernels.hlsli).
#include "pr/game-systems/particles/particles.hlsli"

ConstantBuffer<ParticleTickConstants> g_tick : register(b0);
RWStructuredBuffer<uint> g_counters : register(u0);
RWStructuredBuffer<uint> g_free : register(u1);
RWStructuredBuffer<ParticleEmitterRuntime> g_runtime : register(u5);
RWStructuredBuffer<uint> g_args : register(u6);

// Empty the pool: every slot is free, no particle is live, and every emitter slot restarts its emission progress.
// Dispatched with enough threads for the larger of the capacity and 'emitter_hwm', which the caller sets to the emitter slot count.
numthreads(ParticleResetCS, 64, 1, 1)
void ParticleResetCS(uint3 dtid : SV_DispatchThreadID)
{
	// Each thread resets one pool slot and one emitter slot.
	uint i = dtid.x;
	if (i < g_tick.capacity)
		g_free[i] = i;

	if (i < g_tick.emitter_hwm)
	{
		ParticleEmitterRuntime runtime = (ParticleEmitterRuntime)0;
		g_runtime[i] = runtime;
	}

	// One thread clears the counters and the indirect arguments.
	if (i == 0)
	{
		for (uint c = 0; c != ParticleCounterCount; ++c)
			g_counters[c] = 0;

		g_counters[ParticleCounterFree] = g_tick.capacity;
		g_args[ParticleArgsInstanceCount] = 0;
		g_args[ParticleArgsDispatchX] = 0;
	}
}

// Close a tick: remove the slots that the emit pass took from the free stack, empty the list that was read, and point the draw and the next
// update at the list that was written.
numthreads(ParticleFinalizeCS, 1, 1, 1)
void ParticleFinalizeCS()
{
	// 'taken' counts requests, which may exceed the free slots when the pool is full.
	uint free = g_counters[ParticleCounterFree];
	uint taken = g_counters[ParticleCounterTaken];
	g_counters[ParticleCounterFree] = free - min(taken, free);
	g_counters[ParticleCounterTaken] = 0;

	// The read list becomes the written list of the next tick.
	uint cur = g_tick.cur;
	uint next = 1u - cur;
	uint live = g_counters[ParticleCounterLive0 + next];
	g_counters[ParticleCounterLive0 + cur] = 0;
	g_args[ParticleArgsInstanceCount] = live;
	g_args[ParticleArgsDispatchX] = (live + ParticleGroupSize - 1) / ParticleGroupSize;
}
