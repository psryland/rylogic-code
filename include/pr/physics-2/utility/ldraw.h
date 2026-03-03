#pragma once
#include "pr/physics-2/forward.h"
#include "pr/collision/ldraw.h"
#include "pr/physics-2/rigid_body/rigid_body.h"

namespace pr::rdr12::ldraw
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

	struct LdrRigidBody : fluent::LdrBase<LdrRigidBody>
	{
		physics::RigidBody const* m_rb;
		ERigidBodyFlags m_flags;
		float m_scale;

		LdrRigidBody()
			: m_rb()
			, m_flags(ERigidBodyFlags::Default)
			, m_scale(0.1f)
		{
		}

		LdrRigidBody& rigid_body(physics::RigidBody const& rb)
		{
			m_rb = &rb;
			return *this;
		}
		LdrRigidBody& flags(ERigidBodyFlags flags)
		{
			m_flags = flags;
			return *this;
		}
		LdrRigidBody& scale(float s)
		{
			m_scale = s;
			return *this;
		}

		// Write to 'out'
		template <fluent::WriterType Writer, typename TOut>
		void WriteTo(TOut& out) const
		{
			(void)out;
			#if 0 // todo
			{
				GroupStart(str, name, colour);
				Shape(str, "Shape", colour, rb.Shape());
				if (flags != ERigidBodyFlags::None)
				{
					auto os_momentum = scale * rb.MomentumOS();
					auto os_velocity = scale * rb.VelocityOS();
					auto os_force = scale * rb.ForceOS();
					if (AllSet(flags, ERigidBodyFlags::Origin))
						CoordFrame(str, "Origin", 0xFFFFFFFF, m4x4::Identity(), 0.1f);
					if (AllSet(flags, ERigidBodyFlags::CoM))
						CoordFrame(str, "CoM", 0xFF404040, m4x4::Translation(rb.CentreOfMassOS().w1()), 0.1f);
					if (AllSet(flags, ERigidBodyFlags::LVel))
						Arrow(str, "LVel", 0xFF00FFFF, EArrowType::Fwd, v4Origin, os_velocity.lin, 2);
					if (AllSet(flags, ERigidBodyFlags::AVel))
						Arrow(str, "AVel", 0xFFFF00FF, EArrowType::Fwd, v4Origin, os_velocity.ang, 2);
					if (AllSet(flags, ERigidBodyFlags::LMom))
						Arrow(str, "LMom", 0xFF008080, EArrowType::Fwd, v4Origin, os_momentum.lin, 5);
					if (AllSet(flags, ERigidBodyFlags::AMom))
						Arrow(str, "AMom", 0xFF800080, EArrowType::Fwd, v4Origin, os_momentum.ang, 5);
					if (AllSet(flags, ERigidBodyFlags::Force))
						Arrow(str, "Force", 0xFF0000FF, EArrowType::Back, v4Origin, -os_force.lin.w0(), 8);
					if (AllSet(flags, ERigidBodyFlags::Torque))
						Arrow(str, "Torque", 0xFF000080, EArrowType::Fwd, v4Origin, os_force.ang.w1(), 8);
				}
				GroupEnd(str, o2w ? *o2w : rb.O2W());
			}
			#endif
		}
	};
}