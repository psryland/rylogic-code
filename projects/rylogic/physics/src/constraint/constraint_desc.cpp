//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/constraint/constraint_desc.h"
#include "pr/physics/articulation/articulation.h"
#include "pr/physics/rigid_body/rigid_body.h"

namespace pr::physics
{
	namespace
	{
		// Return a hard-locked axis with zero relative position and velocity targets.
		ConstraintAxisDesc LockedAxis()
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			return axis;
		}

		// Copy common named-joint properties into a D6 descriptor.
		template <typename TDesc>
		D6ConstraintDesc CommonD6(TDesc const& desc)
		{
			auto d6 = D6ConstraintDesc{};
			d6.m_frame_a = desc.m_frame_a;
			d6.m_frame_b = desc.m_frame_b;
			d6.m_break_force = desc.m_break_force;
			d6.m_break_torque = desc.m_break_torque;
			d6.m_collide_connected = desc.m_collide_connected;
			d6.m_enabled = desc.m_enabled;
			return d6;
		}
	}

	// Return the fixed world endpoint.
	BodyRef BodyRef::World()
	{
		return BodyRef{
			.m_type = EConstraintBodyType::World,
			.m_body_id = {},
			.m_articulation_id = {},
			.m_link = {},
		};
	}

	// Return a stable endpoint for a caller-owned rigid body.
	BodyRef BodyRef::Rigid(RigidBody const& body)
	{
		return BodyRef{
			.m_type = EConstraintBodyType::Rigid,
			.m_body_id = body.Id(),
			.m_articulation_id = {},
			.m_link = {},
		};
	}

	// Return a stable endpoint for one validated articulation link.
	BodyRef BodyRef::Link(Articulation const& articulation, LinkHandle link)
	{
		// Validate ownership immediately so a foreign or stale handle never enters persistent constraint storage.
		(void)articulation.LinkDescription(link);
		return BodyRef{
			.m_type = EConstraintBodyType::ArticulationLink,
			.m_body_id = {},
			.m_articulation_id = articulation.Id(),
			.m_link = link,
		};
	}

	// True when this endpoint represents fixed world space.
	bool BodyRef::IsWorld() const
	{
		return m_type == EConstraintBodyType::World;
	}

	// True when this endpoint represents an ordinary rigid body.
	bool BodyRef::IsRigid() const
	{
		return m_type == EConstraintBodyType::Rigid;
	}

	// True when this endpoint represents a reduced-coordinate articulation link.
	bool BodyRef::IsLink() const
	{
		return m_type == EConstraintBodyType::ArticulationLink;
	}

	// Convert a ball-and-socket descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(BallSocketConstraintDesc const& desc)
	{
		auto d6 = CommonD6(desc);
		d6.m_linear.fill(LockedAxis());
		return d6;
	}

	// Convert a hinge descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(HingeConstraintDesc const& desc)
	{
		auto d6 = CommonD6(desc);
		d6.m_linear.fill(LockedAxis());
		d6.m_angular = {desc.m_axis, LockedAxis(), LockedAxis()};
		return d6;
	}

	// Convert a slider descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(SliderConstraintDesc const& desc)
	{
		auto d6 = CommonD6(desc);
		d6.m_linear = {desc.m_axis, LockedAxis(), LockedAxis()};
		d6.m_angular.fill(LockedAxis());
		return d6;
	}

	// Convert a weld descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(WeldConstraintDesc const& desc)
	{
		auto d6 = CommonD6(desc);
		d6.m_linear.fill(LockedAxis());
		d6.m_angular.fill(LockedAxis());
		return d6;
	}
}
