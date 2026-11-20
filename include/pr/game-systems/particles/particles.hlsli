//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU layout shared by the particle system's native owner and its compute kernels.
//
// A particle type is a pool with a fixed capacity. Its particles live only in GPU memory:
//  - 'states' holds each particle's state, in a caller-defined layout, at its pool slot.
//  - 'free' is a stack of unused slots, and 'live' holds two lists of used slots. Each tick reads one live list and writes the other.
//  - 'records' holds each live particle's render record, in a caller-defined layout, at its position in the written live list.
//    The type's vertex shader draws one instance per record.
//  - 'counters' holds the list lengths and the cumulative emission counts (see ParticleCounterFree...).
//  - 'args' holds the draw arguments and the update dispatch arguments, both written on the GPU (see ParticleArgsIndexCount...).
// Emitters live in CPU memory. The CPU uploads a header and caller-defined parameters for each emitter slot that changes. The GPU keeps the
// emission progress of each slot in 'runtime', so the CPU never reads particle data back.
#ifndef PR_GAME_SYSTEMS_PARTICLES_HLSLI
#define PR_GAME_SYSTEMS_PARTICLES_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// Threads per group of the particle kernels.
static const uint ParticleGroupSize = 64;

// Indices into the counters buffer.
static const uint ParticleCounterFree = 0;      // Number of slots on the free stack.
static const uint ParticleCounterLive0 = 1;     // Length of live list 0. List 1 follows.
static const uint ParticleCounterTaken = 3;     // Slots requested from the free stack during the current tick.
static const uint ParticleCounterRequested = 4; // Cumulative particles that emitters asked for.
static const uint ParticleCounterAdmitted = 5;  // Cumulative particles that were created.
static const uint ParticleCounterDropped = 6;   // Cumulative particles not created because the pool was full.
static const uint ParticleCounterCount = 8;

// Indices into the args buffer. The draw arguments are a D3D12_DRAW_INDEXED_ARGUMENTS record, followed by a D3D12_DISPATCH_ARGUMENTS record.
static const uint ParticleArgsIndexCount = 0;
static const uint ParticleArgsInstanceCount = 1;
static const uint ParticleArgsDispatchX = 5;
static const uint ParticleArgsCount = 8;
static const uint ParticleArgsDispatchOffset = 20; // Byte offset of the dispatch record.

// Emitter header flags.
static const uint ParticleEmitterActive = 1u << 0;

// The system-owned part of an emitter slot, written by the CPU.
struct ParticleEmitterHeader
{
	uint flags;       // See ParticleEmitterActive.
	uint generation;  // Changes when the slot is given to a new emitter, so the GPU restarts its emission progress.
	float rate;       // Particles per simulated second.
	uint burst_total; // Cumulative particles requested as bursts. The GPU emits the difference from the total it has already emitted.
	uint seed;        // Per-emitter random seed.
	uint pad0;
	uint pad1;
	uint pad2;
};

// The emission progress of an emitter slot, owned by the GPU.
struct ParticleEmitterRuntime
{
	float accum;         // Fraction of a particle carried over to the next tick.
	uint burst_emitted;  // The 'burst_total' already emitted.
	uint generation;     // The header generation this progress belongs to.
	uint sequence;       // Particles requested by this emitter so far. Each particle's random stream comes from its sequence number.
};

// Per-tick constants of a particle type's kernels.
struct ParticleTickConstants
{
	float dt;          // Simulated seconds in this tick.
	uint cur;          // The live list read this tick (0 or 1). The other list is written.
	uint emitter_hwm;  // One past the highest emitter slot in use.
	uint capacity;     // The pool capacity.
	uint tick;         // Count of ticks since the system was created.
	uint pad0;
	uint pad1;
	uint pad2;
};

// What the emit kernel tells a particle's initialiser about its birth.
struct ParticleSpawn
{
	uint emitter;   // Emitter slot.
	uint sequence;  // Sequence number of this particle within its emitter.
	uint seed;      // Random seed unique to this particle. See ParticleRandom.
	float age;      // Simulated seconds between the particle's birth and the end of the tick.
};

// Mix the bits of 'x' so that nearby inputs give unrelated outputs.
odr inline uint ParticleHash(uint x)
{
	// Alternate multiplies and shifts, so every input bit affects every output bit.
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

// Return a random value in [0, 1) from 'seed' and the stream position 'index'. The same inputs always give the same value.
odr inline float ParticleRandom(uint seed, uint index)
{
	// Keep the top 24 bits, which a float holds exactly.
	return float(ParticleHash(seed ^ ParticleHash(index + 0x9e3779b9u)) >> 8) * (1.0f / 16777216.0f);
}

#ifdef __cplusplus
}
#endif
#endif
