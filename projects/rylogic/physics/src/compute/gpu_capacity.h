//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Return a buffer capacity of at least 'required' elements, and at least one.
	// When 'required' exceeds 'capacity', the result is at least double 'capacity', so a count that grows one element at a time
	// reallocates only a logarithmic number of times. The result never exceeds the largest 'int'.
	inline int GrowCapacity(int required, int capacity)
	{
		// Keep the current capacity while it is large enough.
		if (required <= capacity)
			return std::max(1, capacity);

		// Double without overflowing, but never return less than the requirement.
		auto const doubled = capacity <= std::numeric_limits<int>::max() / 2 ? 2 * capacity : std::numeric_limits<int>::max();
		return std::max({1, required, doubled});
	}
}
