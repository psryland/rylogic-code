//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Stable identity for one articulation; zero is reserved for an invalid identity.
	struct ArticulationId
	{
		uint64_t m_value = 0;

		// True when this identity names an articulation.
		explicit operator bool() const
		{
			return m_value != 0;
		}

		// Compare articulation identities.
		friend bool operator==(ArticulationId lhs, ArticulationId rhs)
		{
			return lhs.m_value == rhs.m_value;
		}

		// Compare articulation identities.
		friend bool operator!=(ArticulationId lhs, ArticulationId rhs)
		{
			return !(lhs == rhs);
		}
	};

	// Generational identity for one articulation link; it never denotes a transient solver index.
	struct LinkHandle
	{
		uint32_t m_index = std::numeric_limits<uint32_t>::max();
		uint32_t m_generation = 0;

		// True when this handle has a potentially addressable link slot.
		explicit operator bool() const
		{
			return m_index != std::numeric_limits<uint32_t>::max() && m_generation != 0;
		}

		// Compare articulation link handles.
		friend bool operator==(LinkHandle lhs, LinkHandle rhs)
		{
			return lhs.m_index == rhs.m_index && lhs.m_generation == rhs.m_generation;
		}

		// Compare articulation link handles.
		friend bool operator!=(LinkHandle lhs, LinkHandle rhs)
		{
			return !(lhs == rhs);
		}
	};
}
