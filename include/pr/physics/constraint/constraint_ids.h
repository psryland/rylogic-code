//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Stable identity for a caller-owned rigid body; zero is reserved for an invalid identity.
	struct BodyId
	{
		uint64_t m_value = 0;

		// True when this identity names a rigid body.
		explicit operator bool() const
		{
			return m_value != 0;
		}

		// Compare rigid-body identities.
		friend bool operator==(BodyId lhs, BodyId rhs)
		{
			return lhs.m_value == rhs.m_value;
		}

		// Compare rigid-body identities.
		friend bool operator!=(BodyId lhs, BodyId rhs)
		{
			return !(lhs == rhs);
		}
	};

	// Generational identity for a persistent constraint slot.
	struct ConstraintHandle
	{
		uint32_t m_index = std::numeric_limits<uint32_t>::max();
		uint32_t m_generation = 0;

		// True when this handle has a potentially addressable slot.
		explicit operator bool() const
		{
			return m_index != std::numeric_limits<uint32_t>::max() && m_generation != 0;
		}

		// Compare persistent constraint handles.
		friend bool operator==(ConstraintHandle lhs, ConstraintHandle rhs)
		{
			return lhs.m_index == rhs.m_index && lhs.m_generation == rhs.m_generation;
		}

		// Compare persistent constraint handles.
		friend bool operator!=(ConstraintHandle lhs, ConstraintHandle rhs)
		{
			return !(lhs == rhs);
		}
	};

	// Generational identity for a future articulation link; it never denotes a transient proxy index.
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
