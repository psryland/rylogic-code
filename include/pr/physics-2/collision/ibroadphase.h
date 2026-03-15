//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"

namespace pr::physics
{
	// Abstract interface for broad-phase spatial overlap queries.
	// Broad-phase is the first stage of collision detection: it cheaply identifies
	// pairs of bodies whose bounding volumes overlap. Narrow-phase then tests each
	// pair for exact geometric contact. Different implementations trade accuracy
	// for speed (e.g. brute-force O(n²), sweep-and-prune O(n log n), spatial hash O(n)).
	struct IBroadphase
	{
		virtual ~IBroadphase() = default;

		// Remove all registered bodies
		virtual void Clear() = 0;

		// Register a body for overlap testing
		virtual void Add(RigidBody const& obj) = 0;

		// Unregister a body
		virtual void Remove(RigidBody const& obj) = 0;

		//// Call 'cb' for each pair of bodies whose bounding volumes overlap.
		//// The callback receives the two overlapping bodies in no particular order.
		//virtual void EnumOverlappingPairs(std::function<void(RigidBody const&, RigidBody const&)> cb) const = 0;
	};
}
