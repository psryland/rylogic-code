#pragma once
#include "pr/common/ldraw.h"
#include "pr/collision/ldraw.h"
#include "pr/physics-2/forward.h"
#include "pr/physics-2/rigid_body/rigid_body.h"

namespace pr::ldraw
{
	enum class ERigidBodyFlags
	{
		None    = 0,
		Origin  = 1 << 0,
		CoM     = 1 << 1,
		AVel    = 1 << 2,
		LVel    = 1 << 3,
		AMom    = 1 << 4,
		LMom    = 1 << 5,
		Force   = 1 << 6,
		Torque  = 1 << 7,
		Default = Origin,
		All     = ~0,
		_flags_enum = 0,
	};

	struct LdrRigidBody : LdrGroup
	{
		physics::RigidBody m_rb;
		LdrRigidBody(seri::Name name = {}, seri::Colour colour = {})
			: LdrGroup(name, colour)
		{
			group_colour(m_colour);
		}

		LdrRigidBody& rigid_body(physics::RigidBody const& rb)
		{
			m_rb = rb;
			if (m_rb.HasShape())
				Add<LdrCollisionShape>("shape").shape(m_rb.Shape());

			return *this;
		}
		LdrRigidBody& origin()
		{
			CoordFrame("origin");
			return *this;
		}
	};
}