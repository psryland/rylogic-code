//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

namespace pr::physics
{
	enum class ERigidBodyStateFlags :int
	{
		// No flags set
		None = 0,

		// True if the body is static or temporarily static (i.e.
		// has non-zero inv mass but should be treated as static).
		Static = 1 << 0,

		// True if a body is in the sleep state. Impulses can wake it up.
		Sleeping = 1 << 1,

		// True if the body is immune to automatic sleeping.
		NeverSleep = 1 << 2,

		// True if the body was involved in a collision and recieved an impulse
		// in the current step. Cleared at the start of each step.
		Collided = 1 << 3,

		_flags_enum = 0,
	};
}
