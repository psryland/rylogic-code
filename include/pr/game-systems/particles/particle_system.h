//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU-resident particles. A particle system owns one pool per particle type and one View3D object per batch. Particles are born, updated, and
// die in compute kernels. Each type writes the render records of its live particles into its batch, and each batch draws all of its records with
// one indirect draw, whose instance count the GPU writes. The CPU only uploads the emitters that change, so a step costs a few small uploads and
// dispatches however many particles are alive.
//
// Particle types supply their own state layouts and simulation shaders (see particle_kernels.hlsli). Batches supply the render record layout
// and its vertex shader, so types with different states share one draw when they write the same record. The stock sprite type and batch
// (see particle_sprite.hlsli) serve small untextured particles.
//
// Particles evolve only in Step, in simulated seconds, so they move correctly at any step rate, in slow motion, and not at all while paused.
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems::particles
{
	// A set of particle batches, types and emitters on one renderer. The batches' View3D objects are created by BatchCreate and owned by the caller.
	// Emitter ids are 64-bit handles that become invalid when the emitter is deleted; using a stale id throws.
	struct ParticleSystem
	{
		virtual ~ParticleSystem() = default;

		// Create an empty particle system on 'rdr'.
		static std::unique_ptr<ParticleSystem> Create(Renderer& rdr, char const* name);

		// The simulation shaders of the stock sprite type. The data is static.
		static pr::view3d::ParticleSimShaders SpriteSimShaders();

		// The drawing shaders of the stock sprite batch. The data is static.
		static pr::view3d::ParticleDrawShaders SpriteDrawShaders();

		// The vertex shader, outputs and vertex stride of the stock mesh batch (see particle_mesh.hlsli). The caller supplies the mesh: its
		// ParticleMeshVertex array as 'm_vertex_data', its triangle list as 'm_indices', and their counts. The data is static.
		static pr::view3d::ParticleDrawShaders MeshDrawShaders();

		// The simulation shaders of the stock stepped sprite type (see particle_stepped_sprite.hlsli). It draws with the stock sprite batch. The data is static.
		static pr::view3d::ParticleSimShaders SteppedSpriteSimShaders();

		// The simulation shaders of the stock debris type (see particle_debris.hlsli). It draws with the stock mesh batch. The data is static.
		static pr::view3d::ParticleSimShaders DebrisSimShaders();

		// The simulation shaders of the stock bird type (see particle_bird.hlsli). The data is static.
		static pr::view3d::ParticleSimShaders BirdSimShaders();

		// The drawing shaders of a batch of stock bird records. The bird is procedural, so the batch has no vertex data. The data is static.
		static pr::view3d::ParticleDrawShaders BirdDrawShaders();

		// The simulation shaders of the stock marker type (see particle_marker.hlsli). It draws with the stock mesh batch. The data is static.
		static pr::view3d::ParticleSimShaders MarkerSimShaders();

		// The simulation shaders of the stock spray type (see particle_spray.hlsli). It draws with the stock sprite batch, and needs a water surface
		// (see WaterSet). The data is static.
		static pr::view3d::ParticleSimShaders SpraySimShaders();

		// The simulation shaders of the stock froth type (see particle_froth.hlsli). It draws with the stock flat sprite batch, and needs a water
		// surface (see WaterSet). The data is static.
		static pr::view3d::ParticleSimShaders FrothSimShaders();

		// The drawing shaders of the stock flat sprite batch: soft discs lying in the plane of each record's orientation, drawn from ParticleMeshRecord
		// records (see particle_mesh.hlsli). The disc is procedural, so the batch has no vertex data. The data is static.
		static pr::view3d::ParticleDrawShaders FlatSpriteDrawShaders();

		// Validate 'options' and add a particle batch, returning its index. Its View3D object is returned in 'out_object'.
		virtual int BatchCreate(pr::view3d::ParticleBatchOptions const& options, char const* name, pr::view3d::Object& out_object) = 0;

		// Validate 'options' and add a particle type to the batch 'options.m_batch', returning the type's index.
		virtual int TypeCreate(pr::view3d::ParticleTypeOptions const& options, char const* name) = 0;

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

		// Set the gravity, uniform wind and contact budget of the environment, from the next step. See particle_environment.hlsli.
		virtual void EnvironmentSet(pr::view3d::ParticleEnvironment const& environment) = 0;

		// Set the wind grid, from the next step: 'cells_x' by 'cells_y' columns of horizontal wind, row-major, with the minimum corner at 'origin' in
		// the XY plane and square cells of side 'cell_size'. Zero columns removes the grid.
		virtual void WindGridSet(v2 origin, float cell_size, int cells_x, int cells_y, std::span<v2 const> wind) = 0;

		// Add a collision set of planes, returning its index. Indices of deleted sets are reused, so emitters must stop naming a set before it is deleted.
		virtual int CollisionSetCreate(std::span<pr::view3d::ParticlePlane const> planes) = 0;

		// Replace the planes of a collision set, from the next step.
		virtual void CollisionSetUpdate(int set, std::span<pr::view3d::ParticlePlane const> planes) = 0;

		// Remove a collision set. Particles that still name it collide with nothing.
		virtual void CollisionSetDelete(int set) = 0;

		// Bind the water surface that types sample (see particle_water.hlsli), from the next step, or remove it when 'ocean' is null. 'object' must be
		// the View3D object of 'ocean'. Both must outlive the binding, and the ocean must be updated before each step, so particles see the frame being drawn.
		virtual void WaterSet(ocean::OceanSurface const* ocean, pr::view3d::Object object) = 0;
	};
}
