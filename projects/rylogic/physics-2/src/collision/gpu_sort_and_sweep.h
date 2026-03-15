//*********************************************
// Physics Sandbox — GPU Sort-and-Sweep Broadphase
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// GPU-accelerated sort-and-sweep broadphase using radix sort.
//
// Algorithm:
//   1. Choose the primary sort axis (axis with max position variance)
//   2. Pack 2N AABB endpoints (min + max per body) as float keys
//   3. GPU radix sort the endpoints
//   4. CPU sweep: linear scan tracking an "active set" of open intervals.
//      For each "begin" endpoint: test Y and Z overlap against all active bodies.
//      For each "end" endpoint: remove from active set.
//   5. Emit pairs that overlap on all 3 axes.
//
// Full 3-axis AABB filtering with only one GPU sort. The Y/Z tests during
// the sweep are 4 comparisons per candidate — trivial cost vs the sort.
//
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"
#include "pr/physics-2/collision/ibroadphase.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuSortAndSweep : IBroadphase
	{
	private:

		// Lightweight D3D12 wrapper (device + command queue)
		Gpu& m_gpu;

		// The GPU radix sorter — long-lived, resized as needed.
		// Sorts float keys (AABB endpoints) with uint32 payloads (body_index << 1 | begin/end).
		// Uses COMPUTE queue type to match the physics engine's Gpu.
		mutable BoundsSorter m_sorter;

		// Staging buffers for sort input/output (reused across frames)
		mutable std::vector<float> m_keys;
		mutable std::vector<uint32_t> m_payloads;
		mutable std::vector<int> m_sweep;

		// The registered bodies for overlap testing
		std::vector<RigidBody const*> m_entity;

	public:

		explicit GpuSortAndSweep(Gpu& gpu);

		// Remove all registered bodies
		void Clear() override;

		// Register a body for overlap testing
		void Add(RigidBody const& obj) override;

		// Unregister a body
		void Remove(RigidBody const& obj) override;

		//// Enumerate all pairs of bodies whose bounding boxes overlap.
		//// Uses GPU radix sort for the primary axis, then CPU sweep with Y/Z filtering.
		//void EnumOverlappingPairs(GpuJob& job, std::function<void(RigidBody const&, RigidBody const&)> cb) const;

		// Enumerate overlapping pairs using pre-computed world-space AABBs from the GPU integrate step.
		void EnumOverlappingPairs(GpuJob& job, std::span<IntegrateAABB const> aabbs, std::function<void(RigidBody const&, RigidBody const&)> cb) const;
	};
}
