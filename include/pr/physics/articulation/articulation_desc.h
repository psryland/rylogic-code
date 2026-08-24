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

	// One ordered scalar degree of freedom in a reduced-coordinate joint.
	struct ArticulationAxisDesc
	{
		EArticulationAxisType m_type = EArticulationAxisType::Revolute;
		v4 m_axis = v4::ZAxis();
	};

	// A reduced-coordinate joint expressed as an ordered product of zero to six scalar screw motions.
	struct ArticulationJointDesc
	{
		m4x4 m_joint_to_parent = m4x4::Identity();
		m4x4 m_joint_to_child = m4x4::Identity();
		std::array<ArticulationAxisDesc, 6> m_axes = {};
		std::array<float, 6> m_initial_position = {};
		std::array<float, 6> m_initial_velocity = {};
		int m_dof_count = 0;

		// Return a fixed joint whose two joint frames remain coincident.
		static ArticulationJointDesc Fixed(m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());

		// Return a one-DOF revolute joint about a joint-local unit axis.
		static ArticulationJointDesc Revolute(v4 axis = v4::ZAxis(), m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());

		// Return a one-DOF prismatic joint along a joint-local unit axis.
		static ArticulationJointDesc Prismatic(v4 axis = v4::ZAxis(), m4x4 const& joint_to_parent = m4x4::Identity(), m4x4 const& joint_to_child = m4x4::Identity());
	};

	// Immutable mass and future collision-proxy data for one articulation link.
	struct ArticulationLinkDesc
	{
		Inertia m_inertia = {};
		collision::Shape const* m_shape = nullptr;
		m4x4 m_shape_to_link = m4x4::Identity();
	};
}
