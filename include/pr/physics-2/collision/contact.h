//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/materials/material.h"

namespace pr::physics
{
	// A description of contact between 'objA' and 'objB'
	struct RbContact :collision::Contact
	{
		// Notes:
		//  - Collision detection is performed in objA space to prevent floating point accuracy issues.
		//  - The results of the collision; 'axis' and 'point', are in 'objA' space.

		// Transform from B to A space
		m4x4 m_b2a;

		// The relative velocity of objB in objA space (measured at objA 0,0,0)
		v8motion m_velocity;

		// The collision point adjusted by the collision time
		v4 m_point_at_t;

		// The objects that are colliding, defining who is 'A' and 'B'
		RigidBody const* m_objA;
		RigidBody const* m_objB;
		
		// The combined material properties of the two colliding objects
		Material m_mat;

		// The relative time of the collision (in seconds). 0 = now, -dt = previous step. (used to order collision resolution)
		float m_time;

		RbContact();
		RbContact(RigidBody const& objA, RigidBody const& objB);
		RbContact(RigidBody const& objA, RigidBody const& objB, GpuContact const& contact);

		// Adjust the collision data to the given sub-step time.
		void Update(float dt_sub);

		// Dump the collision scene to LDraw script (best-effort, won't throw)
		friend void Dump(RbContact const& c);
	};
}
