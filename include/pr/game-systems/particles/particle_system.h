//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU-resident particles. A particle system owns one pool per particle type. Particles are born, updated, and die in compute kernels, and each
// type draws all of its particles as one View3D object with one indirect draw, whose instance count the GPU writes. The CPU only uploads the
// emitters that change, so a step costs a few small uploads and dispatches however many particles are alive.
//
// Particle types supply their own state and render record layouts and their own shaders (see particle_kernels.hlsli). The stock sprite type
// (see particle_sprite.hlsli) serves small untextured particles.
//
// Particles evolve only in Step, in simulated seconds, so they move correctly at any step rate, in slow motion, and not at all while paused.
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems::particles
{
	// A set of particle types and their emitters on one renderer. The type's View3D objects are created by TypeCreate and owned by the caller.
	// Emitter ids are 64-bit handles that become invalid when the emitter is deleted; using a stale id throws.
	struct ParticleSystem
	{
		virtual ~ParticleSystem() = default;

		// Create an empty particle system on 'rdr'.
		static std::unique_ptr<ParticleSystem> Create(Renderer& rdr, char const* name);

		// The shaders of the stock sprite type. The data is static.
		static pr::view3d::ParticleTypeShaders SpriteShaders();

		// Validate 'options' and add a particle type, returning its index. Its View3D object is returned in 'out_object'.
		virtual int TypeCreate(pr::view3d::ParticleTypeOptions const& options, char const* name, pr::view3d::Object& out_object) = 0;

		// Add an emitter of particles of 'type'. 'params' must be the type's parameter stride in size.
		virtual uint64_t EmitterCreate(int type, pr::view3d::ParticleEmitter const& emitter, std::span<std::byte const> params) = 0;

		// Replace the settings and parameters of an emitter from the next step.
		virtual void EmitterUpdate(uint64_t emitter, pr::view3d::ParticleEmitter const& settings, std::span<std::byte const> params) = 0;

		// Ask an emitter for 'count' extra particles, born at the end of the next step.
		virtual void EmitterBurst(uint64_t emitter, uint32_t count) = 0;

		// Remove an emitter. Its living particles live out their lives.
		virtual void EmitterDelete(uint64_t emitter) = 0;

		// Advance every particle by 'dt' simulated seconds. Zero does nothing.
		virtual void Step(double dt) = 0;

		// Remove every particle and restart every emitter's emission.
		virtual void Reset() = 0;

		// Read the pool usage of 'type'. Waits for the GPU.
		virtual pr::view3d::ParticleStats Stats(int type) = 0;
	};
}
