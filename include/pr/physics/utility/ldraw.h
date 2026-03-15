#pragma once
#include "pr/common/ldraw.h"
#include "pr/collision/ldraw.h"
#include "pr/physics/forward.h"
#include "pr/physics/rigid_body/rigid_body.h"

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

	struct LdrRigidBody : LdrBase
	{
		physics::RigidBody const* m_rb;
		ERigidBodyFlags m_flags;

		LdrRigidBody(seri::Name name = {}, seri::Colour colour = {})
			: LdrBase(name, colour)
			, m_rb()
			, m_flags(ERigidBodyFlags::Default)
		{
		}

		LdrRigidBody& rigid_body(physics::RigidBody const& rb)
		{
			m_rb = &rb;
			return *this;
		}
		LdrRigidBody& flags(ERigidBodyFlags f)
		{
			m_flags = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			if (!m_rb || !m_rb->HasShape())
				return;

			Builder tmp;
			auto& grp = tmp.Group(m_name, m_colour);
			grp.group_colour(m_colour);
			AddShape(grp, m_rb->Shape());
			tmp.ToString(out);
			LdrBase::Write(out);
		}
		virtual void Write(bytebuf& out) const override
		{
			if (!m_rb || !m_rb->HasShape())
				return;

			Builder tmp;
			auto& grp = tmp.Group(m_name, m_colour);
			grp.group_colour(m_colour);
			AddShape(grp, m_rb->Shape());
			tmp.ToBinary(out);
			LdrBase::Write(out);
		}
	};
}