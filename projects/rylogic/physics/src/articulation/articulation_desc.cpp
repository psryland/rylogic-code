//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/articulation/articulation_desc.h"

namespace pr::physics
{
	// Return the configuration dimension for the selected joint representation.
	int ArticulationJointDesc::PositionCount() const
	{
		// Quaternion orientation has one more coordinate than its angular velocity.
		switch (m_type)
		{
			case EArticulationJointType::Axes: { return m_dof_count; }
			case EArticulationJointType::Spherical: { return 4; }
			default: { throw std::invalid_argument("Articulation joint type is invalid"); }
		}
	}

	// Return an unconstrained three-axis rotation without Euler-angle coordinate singularities.
	ArticulationJointDesc ArticulationJointDesc::Spherical(m4x4 const& joint_to_parent, m4x4 const& joint_to_child)
	{
		// The quaternion maps the moving child-joint frame into its parent-joint frame.
		return ArticulationJointDesc{
			.m_joint_to_parent = joint_to_parent,
			.m_joint_to_child = joint_to_child,
			.m_initial_position = {0, 0, 0, 1},
			.m_dof_count = 3,
			.m_type = EArticulationJointType::Spherical,
		};
	}

	// Return a fixed joint whose two joint frames remain coincident.
	ArticulationJointDesc ArticulationJointDesc::Fixed(m4x4 const& joint_to_parent, m4x4 const& joint_to_child)
	{
		return ArticulationJointDesc{
			.m_joint_to_parent = joint_to_parent,
			.m_joint_to_child = joint_to_child,
			.m_dof_count = 0,
		};
	}

	// Return a one-DOF revolute joint about a joint-local unit axis.
	ArticulationJointDesc ArticulationJointDesc::Revolute(v4 axis, m4x4 const& joint_to_parent, m4x4 const& joint_to_child)
	{
		auto joint = Fixed(joint_to_parent, joint_to_child);
		joint.m_axes[0] = ArticulationAxisDesc{
			.m_type = EArticulationAxisType::Revolute,
			.m_axis = axis,
		};
		joint.m_dof_count = 1;
		return joint;
	}

	// Return a one-DOF prismatic joint along a joint-local unit axis.
	ArticulationJointDesc ArticulationJointDesc::Prismatic(v4 axis, m4x4 const& joint_to_parent, m4x4 const& joint_to_child)
	{
		auto joint = Fixed(joint_to_parent, joint_to_child);
		joint.m_axes[0] = ArticulationAxisDesc{
			.m_type = EArticulationAxisType::Prismatic,
			.m_axis = axis,
		};
		joint.m_dof_count = 1;
		return joint;
	}
}
