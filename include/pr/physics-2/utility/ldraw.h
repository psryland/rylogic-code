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
			if (!m_rb || !m_rb->HasShape())
				return;

			// Write the collision shape with this element's color
			auto& shape = m_rb->Shape();
			using namespace collision;
			switch (shape.m_type)
			{
				case EShape::Box:
				{
					auto& box = shape_cast<ShapeBox>(shape);
					fluent::LdrBox().colour(m_colour).dim(box.m_radius * 2).o2w(box.m_base.m_s2p).WriteTo<Writer>(out);
					break;
				}
				case EShape::Sphere:
				{
					auto& sph = shape_cast<ShapeSphere>(shape);
					fluent::LdrSphere().colour(m_colour).radius(sph.m_radius).o2w(sph.m_base.m_s2p).WriteTo<Writer>(out);
					break;
				}
				default:
				{
					LdrPhysicsShape().shape(shape).WriteTo<Writer>(out);
					break;
				}
			}
			LdrBase::WriteTo<Writer>(out);
		}
	};
}