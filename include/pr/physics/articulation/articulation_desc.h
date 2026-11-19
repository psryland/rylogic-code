//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/articulation/articulation_ids.h"
#include "pr/physics/shape/inertia.h"

namespace pr::physics
{
	// Selects whether the root link is anchored to world or contributes six floating-base velocities.
	enum class EArticulationRootType
	{
		Fixed,
		Floating,
	};

	// Selects the one-dimensional screw motion represented by a joint axis.
	enum class EArticulationAxisType
	{
		Revolute,
		Prismatic,
	};

	// Selects scalar screw coordinates or a unit quaternion with three child-joint angular velocities.
	enum class EArticulationJointType
	{
		Axes,
		Spherical,
	};

	// One ordered scalar degree of freedom in a reduced-coordinate joint.
	struct ArticulationAxisDesc
	{
		EArticulationAxisType m_type = EArticulationAxisType::Revolute;
		v4 m_axis = v4::ZAxis();
	};

	// A reduced joint with scalar screw coordinates, or a spherical quaternion (x,y,z,w) mapping child joint frame to parent joint frame.
	// Spherical velocity and force entries are angular velocity and torque in the child joint frame; m_axes is unused for this type.
	struct ArticulationJointDesc
	{
		m4x4 m_joint_to_parent = m4x4::Identity();
		m4x4 m_joint_to_child = m4x4::Identity();
		std::array<ArticulationAxisDesc, 6> m_axes = {};
		std::array<float, 6> m_initial_position = {};
		std::array<float, 6> m_initial_velocity = {};
		int m_dof_count = 0;
		EArticulationJointType m_type = EArticulationJointType::Axes;

		// Return the configuration dimension, which differs from the velocity dimension for spherical joints.
		int PositionCount() const;

		// Return a fixed joint whose two joint frames remain coincident.
		static ArticulationJointDesc Fixed(m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());

		// Return a one-DOF revolute joint about a joint-local unit axis.
		static ArticulationJointDesc Revolute(v4 axis = v4::ZAxis(), m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());

		// Return a one-DOF prismatic joint along a joint-local unit axis.
		static ArticulationJointDesc Prismatic(v4 axis = v4::ZAxis(), m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());

		// Return a spherical joint with identity orientation and three child-joint angular velocities.
		static ArticulationJointDesc Spherical(m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());
	};

	// Immutable mass and collision policy for one articulation link.
	struct ArticulationLinkDesc
	{
		Inertia m_inertia = {};
		collision::Shape const* m_shape = nullptr;
		m4x4 m_shape_to_link = m4x4::Identity();

		// Allow this link's collision shape to contact its immediate parent link.
		bool m_collide_parent = false;

		// Allow this link's collision shape to contact non-adjacent links in the same articulation.
		bool m_collide_self = true;
	};

	// One link-frame spatial impulse to accumulate before a single linear-time impulse ABA traversal.
	struct ArticulationImpulse
	{
		LinkHandle m_link = {};
		v8force m_impulse = {};
	};
}
