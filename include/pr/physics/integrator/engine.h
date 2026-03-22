//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/integrator/engine_config.h"

namespace pr::physics
{
	struct Engine
	{
		// Notes:
		//  - The engine does not own the bodies. The caller is responsible for managing their
		//    lifetime and ensuring they remain valid while being used by the engine.
		//  - The engine does not have a universal gravity setting, Gravity should be applied
		//    as a force to bodies each frame before calling Step().
		//  - Collision resolution and 'sleeping objects' require a concept of "down" however,
		//    even if it is a zero vector.

	private:

		// Engine configuration parameters.
		EngineConfig const m_config;

		// GPU device and command queue wrapper, shared by the integrator and collision detector.
		GpuPtr m_gpu;

		// GPU integrator (opaque)
		GpuIntegratorPtr m_gpu_integrator;

		// GPU broadphase (opaque)
		GpuSortAndSweepPtr m_gpu_sort_and_sweep;

		// GPU collision detector (opaque)
		GpuCollisionDetectorPtr m_gpu_collision_detector;

		// GPU collision resolver (opaque)
		GpuResolverPtr m_gpu_resolver;

		// Material map for looking up combined material properties during collision resolution.
		MaterialMapPtr m_materials;

		// Buffers for preparing GPU data
		CachePtr m_cache;

		// Storage for body pointers
		std::vector<RigidBody*> m_body_ptrs;
		
		friend struct DbgPhysics;
		bool m_gpu_integrate = true;
		bool m_gpu_sorter = true;
		bool m_gpu_detect = true;
		bool m_gpu_resolve = true;

	public:

		explicit Engine(EngineConfig const& config = {}, ID3D12Device4* existing_device = nullptr);

		// Get/Set whether the GPU is used for integration and collision detection.
		bool UseGpu() const;
		void UseGpu(bool use_gpu);

		// Get/Set whether the GPU is used for narrow-phase collision detection (GJK).
		bool UseGpuDetect() const;
		void UseGpuDetect(bool use);

		// Get/Set whether the GPU is used for collision resolution (impulse application).
		bool UseGpuResolve() const;
		void UseGpuResolve(bool use);

		// Evolve the physics objects forward in time and resolve any collisions.
		void Step(float dt, std::span<RigidBody*> bodies);
		void Step(float dt, RigidBodyRange auto&& bodies)
		{
			m_body_ptrs.resize(0);
			for (auto& body : bodies)
				m_body_ptrs.push_back(&body);

			Step(dt, m_body_ptrs);
		}

		// Get/set the physics material properties for a given material ID.
		physics::Material Material(int id) const;
		void Material(physics::Material mat);

	private:

		// CPU integration for testing and debugging.
		void CpuIntegrate(std::span<GpuRigidBody> bodies, float dt);

		// CPU broadphase for testing and debugging
		void CpuSweep();

		// CPU collision detection for testing and debugging.
		void CpuCollide(std::span<GpuCollisionPair> pairs);

		// CPU collision resolution for testing and debugging
		void CpuResolve();

		// Narrow phase collision detection.
		// Tests whether the two bodies in 'c' are geometrically in contact using GJK/SAT.
		bool NarrowPhaseCollision(float dt, RbContact& c);

		// Calculate and apply the restitution impulse to resolve a collision.
		void ResolveCollision(RbContact& c);
	};
}
