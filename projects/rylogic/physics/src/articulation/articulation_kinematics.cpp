//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/articulation/articulation.h"
#include "src/articulation/articulation_internal.h"

namespace pr::physics
{
	namespace
	{
		// Return the child-side frame to parent-side frame transform for one scalar joint displacement.
		m4x4 AxisTransform(ArticulationAxisDesc const& axis, float position)
		{
			switch (axis.m_type)
			{
				case EArticulationAxisType::Revolute:
				{
					return m4x4::Transform(axis.m_axis, position, v4::Origin());
				}
				case EArticulationAxisType::Prismatic:
				{
					return m4x4::Translation(axis.m_axis * position);
				}
				default:
				{
					throw std::runtime_error("Articulation joint axis type is invalid");
				}
			}
		}

		// Return the constant unit motion subspace of one scalar axis in its child-side joint frame.
		v8motion AxisMotion(ArticulationAxisDesc const& axis)
		{
			switch (axis.m_type)
			{
				case EArticulationAxisType::Revolute:
				{
					return v8motion{axis.m_axis, v4{}};
				}
				case EArticulationAxisType::Prismatic:
				{
					return v8motion{v4{}, axis.m_axis};
				}
				default:
				{
					throw std::runtime_error("Articulation joint axis type is invalid");
				}
			}
		}

		// Evaluate the joint transform, child-frame velocity Jacobian, and velocity-product bias.
		void EvaluateJoint(detail::ArticulationLinkState& link, std::span<float const> position, std::span<float const> velocity)
		{
			// Spherical angular coordinates already use the child joint frame, so their Jacobian is constant and has no internal bias.
			switch (link.m_joint.m_type)
			{
				case EArticulationJointType::Axes: { break; }
				case EArticulationJointType::Spherical:
				{
					auto const orientation = quat{position[0], position[1], position[2], position[3]};
					link.m_child_to_parent = link.m_joint.m_joint_to_parent * m4x4::Transform(orientation, v4::Origin()) * InvertOrthonormal(link.m_joint.m_joint_to_child);
					link.m_parent_to_child = InvertOrthonormal(link.m_child_to_parent);
					link.m_joint_velocity = link.m_joint.m_joint_to_child * v8motion{v4{velocity[0], velocity[1], velocity[2], 0}, v4{}};
					link.m_joint_bias = {};
					for (int column = 0; column != 3; ++column)
					{
						auto angular_axis = v4{};
						angular_axis[column] = 1.0f;
						link.m_motion_subspace[column] = link.m_joint.m_joint_to_child * v8motion{angular_axis, v4{}};
					}
					return;
				}
				default: { throw std::invalid_argument("Articulation joint type is invalid"); }
			}

			auto const dof_count = link.m_joint.m_dof_count;
			auto motion_to_parent_joint = m4x4::Identity();
			auto joint_velocity = v8motion{};
			auto joint_bias = v8motion{};

			// Propagate zero parent motion through each moving scalar frame to recover the aggregate joint velocity and cJ bias.
			for (int axis_index = 0; axis_index != dof_count; ++axis_index)
			{
				auto const axis_to_parent = AxisTransform(link.m_joint.m_axes[axis_index], position[axis_index]);
				auto const parent_to_axis = InvertOrthonormal(axis_to_parent);
				auto const axis_velocity = AxisMotion(link.m_joint.m_axes[axis_index]) * velocity[axis_index];
				motion_to_parent_joint = motion_to_parent_joint * axis_to_parent;
				joint_velocity = parent_to_axis * joint_velocity + axis_velocity;
				joint_bias = parent_to_axis * joint_bias + Cross(joint_velocity, axis_velocity);
			}

			// Transform the terminal joint frame into the physical child-link frame.
			link.m_child_to_parent = link.m_joint.m_joint_to_parent * motion_to_parent_joint * InvertOrthonormal(link.m_joint.m_joint_to_child);
			link.m_parent_to_child = InvertOrthonormal(link.m_child_to_parent);
			link.m_joint_velocity = link.m_joint.m_joint_to_child * joint_velocity;
			link.m_joint_bias = link.m_joint.m_joint_to_child * joint_bias;

			// Probe each unit generalized speed through the same bounded serial transform to form S(q) without representation-specific formulas.
			for (int column = 0; column != dof_count; ++column)
			{
				auto unit_velocity = v8motion{};
				for (int axis_index = 0; axis_index != dof_count; ++axis_index)
				{
					auto const parent_to_axis = InvertOrthonormal(AxisTransform(link.m_joint.m_axes[axis_index], position[axis_index]));
					unit_velocity = parent_to_axis * unit_velocity;
					if (axis_index == column)
						unit_velocity += AxisMotion(link.m_joint.m_axes[axis_index]);
				}
				link.m_motion_subspace[column] = link.m_joint.m_joint_to_child * unit_velocity;
			}
			for (int column = dof_count; column != isize(link.m_motion_subspace); ++column)
				link.m_motion_subspace[column] = {};
		}
	}

	// Refresh all world transforms, motion subspaces, bias accelerations, and link velocities in parent-before-child order.
	void Articulation::UpdateKinematics()
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");

		auto& state = *m_state;
		auto& root = state.m_links.front();
		switch (state.m_root_type)
		{
			case EArticulationRootType::Fixed:
			{
				root.m_link_velocity = {};
				break;
			}
			case EArticulationRootType::Floating:
			{
				root.m_link_velocity = detail::LoadSpatialMotion(state.m_velocity, 0);
				break;
			}
			default:
			{
				throw std::runtime_error("Articulation root type is invalid");
			}
		}
		root.m_child_to_parent = m4x4::Identity();
		root.m_parent_to_child = m4x4::Identity();
		root.m_joint_velocity = {};
		root.m_joint_bias = {};

		// Builder insertion order is topological, so every parent cache is complete before its children are evaluated.
		for (int link_index = 1; link_index != isize(state.m_links); ++link_index)
		{
			auto& link = state.m_links[link_index];
			auto const& parent = state.m_links[link.m_parent_index];
			auto position = std::span<float const>{state.m_position}.subspan(link.m_position_offset, link.m_joint.PositionCount());
			auto velocity = std::span<float const>{state.m_velocity}.subspan(link.m_velocity_offset, link.m_joint.m_dof_count);
			EvaluateJoint(link, position, velocity);
			link.m_link_to_world = parent.m_link_to_world * link.m_child_to_parent;
			link.m_link_velocity = link.m_parent_to_child * parent.m_link_velocity + link.m_joint_velocity;

			// Complete the serial joint's internal bias with the parent/joint velocity coupling required by spatial acceleration recursion.
			link.m_joint_bias += Cross(link.m_link_velocity, link.m_joint_velocity);
		}

		state.m_kinematics_dirty = false;
	}
}
