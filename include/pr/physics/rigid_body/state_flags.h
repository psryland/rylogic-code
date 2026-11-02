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

		// True if the body follows a caller-driven velocity (twist) and has infinite mass. Static is clear for kinematic bodies.
		Kinematic = 1 << 4,

		// True for the first step after a motion type change. Contacts involving the body ignore cached impulses from before the change.
		// The engine clears it after that step.
		ResetWarmStart = 1 << 5,

		_flags_enum = 0,
	};

	// How a rigid body moves.
	//  - Static: infinite mass, never moves.
	//  - Dynamic: finite mass, moved by forces, impulses and contacts.
	//  - Kinematic: infinite mass, moved only by its caller-driven velocity (twist). It pushes dynamic bodies but is never pushed.
	enum class ERigidBodyMotion :int
	{
		Static,
		Dynamic,
		Kinematic,
	};
}
