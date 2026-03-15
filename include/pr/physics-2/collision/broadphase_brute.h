//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/collision/ibroadphase.h"

namespace pr::physics
{
	struct BroadphaseBrute : IBroadphase
	{
		// Notes:
		// A simple O(n²) broad phase implementation.
		// Tests every pair of registered bodies for bounding box overlap.
		// Suitable for small scenes (< ~100 bodies). For larger scenes,
		// consider sweep-and-prune or spatial hashing.
	private:

		std::vector<RigidBody const*> m_entity;

	public:

		BroadphaseBrute();

		// Remove all registered bodies
		void Clear() override;

		// Register a body for overlap testing
		void Add(RigidBody const& obj) override;

		// Unregister a body
		void Remove(RigidBody const& obj) override;

		// Enumerate all pairs of entities whose bounding boxes overlap
		void EnumOverlappingPairs(std::function<void(RigidBody const&, RigidBody const&)> cb) const;
	};
}
