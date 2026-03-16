//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// ToDo:
	//  - Make use of sub step collision time

	// A container object that groups the parts of a physics system together.
	// IBroadphase provides spatial overlap queries (e.g. brute-force, sweep-and-prune).
	// IMaterials maps material ID pairs to combined material properties (friction, elasticity).
	// The broadphase and material map are owned externally and passed by reference.
	struct Engine
	{
	private:

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

		// Recycled buffer of rigid body pointers
		std::vector<RigidBody*> m_body_ptrs;

		bool m_gpu_integrate = true;
		bool m_gpu_sorter = true;
		bool m_gpu_detect = true;
		bool m_gpu_resolve = true;

	public:

		Engine(ID3D12Device4* existing_device = nullptr);

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

		// Raised after collision detection, but before resolution.
		// Subscribers can inspect, modify, add, or remove contacts before impulses are applied.
		struct PostCollisionDetectionArgs { std::span<RbContact> m_contacts; };
		EventHandler<Engine&, PostCollisionDetectionArgs> PostCollisionDetection;

		// Get/set the physics material properties for a given material ID.
		physics::Material Material(int id) const;
		void Material(physics::Material mat);

	private:

		// Broad phase overlap query → narrow phase collision detection → impulse resolution.
		void DetectAndResolveCollisions(float dt);

		// Narrow phase collision detection.
		// Tests whether the two bodies in 'c' are geometrically in contact using GJK/SAT.
		bool NarrowPhaseCollision(float dt, RbContact& c);

		// Calculate and apply the restitution impulse to resolve a collision.
		void ResolveCollision(RbContact& c);
	};
}
