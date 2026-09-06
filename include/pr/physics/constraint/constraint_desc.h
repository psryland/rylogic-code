//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/constraint/constraint_ids.h"
#include "pr/physics/articulation/articulation_ids.h"

namespace pr::physics
{
	// Selects how one translational or rotational degree of freedom contributes to the constraint solve.
	enum class EConstraintAxisMode
	{
		Free,
		Locked,
		Limited,
		Driven,
	};

	// Identifies the dynamics owner represented by one stable constraint endpoint.
	enum class EConstraintBodyType
	{
		World,
		Rigid,
		ArticulationLink,
	};

	// A stable constraint endpoint that never stores an object pointer or transient packed index.
	struct BodyRef
	{
		EConstraintBodyType m_type = EConstraintBodyType::World;
		BodyId m_body_id = {};
		ArticulationId m_articulation_id = {};
		LinkHandle m_link = {};

		// Return the fixed world endpoint.
		static BodyRef World();

		// Return a stable endpoint for a caller-owned rigid body.
		static BodyRef Rigid(RigidBody const& body);

		// Return a stable endpoint for one validated articulation link.
		static BodyRef Link(Articulation const& articulation, LinkHandle link);

		// True when this endpoint represents fixed world space.
		bool IsWorld() const;

		// True when this endpoint represents an ordinary rigid body.
		bool IsRigid() const;

		// True when this endpoint represents a reduced-coordinate articulation link.
		bool IsLink() const;

		// Compare stable constraint endpoints.
		friend bool operator==(BodyRef lhs, BodyRef rhs)
		{
			return
				lhs.m_type == rhs.m_type &&
				lhs.m_body_id == rhs.m_body_id &&
				lhs.m_articulation_id == rhs.m_articulation_id &&
				lhs.m_link == rhs.m_link;
		}

		// Compare stable constraint endpoints.
		friend bool operator!=(BodyRef lhs, BodyRef rhs)
		{
			return !(lhs == rhs);
		}
	};

	// A constraint coordinate frame expressed in endpoint-local coordinates, or directly in world coordinates for the world endpoint.
	struct BodyFrame
	{
		BodyRef m_body = BodyRef::World();
		m4x4 m_constraint_to_body = m4x4::Identity();
	};

	// Configuration for one D6 translational or rotational axis.
	struct ConstraintAxisDesc
	{
		EConstraintAxisMode m_mode = EConstraintAxisMode::Free;
		Range<float> m_limits = {
			-std::numeric_limits<float>::infinity(),
			+std::numeric_limits<float>::infinity(),
		};
		float m_target_position = 0.0f;
		float m_target_velocity = 0.0f;
		float m_stiffness = 0.0f;
		float m_damping = 0.0f;
		float m_max_force = std::numeric_limits<float>::infinity();
	};

	// General six-degree-of-freedom constraint between two endpoint-local frames.
	struct D6ConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		std::array<ConstraintAxisDesc, 3> m_linear = {};
		std::array<ConstraintAxisDesc, 3> m_angular = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;
	};

	// Convenience descriptor for a ball-and-socket joint whose frame origins must coincide.
	struct BallSocketConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;
	};

	// Convenience descriptor for a hinge whose free, limited, or driven rotation is frame-local X.
	struct HingeConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		ConstraintAxisDesc m_axis = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;
	};

	// Convenience descriptor for a slider whose free, limited, or driven translation is frame-local X.
	struct SliderConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		ConstraintAxisDesc m_axis = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;
	};

	// Convenience descriptor for a weld that locks all relative translation and rotation.
	struct WeldConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;
	};

	// Convert a ball-and-socket descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(BallSocketConstraintDesc const& desc);

	// Convert a hinge descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(HingeConstraintDesc const& desc);

	// Convert a slider descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(SliderConstraintDesc const& desc);

	// Convert a weld descriptor to the general D6 representation.
	D6ConstraintDesc ToD6(WeldConstraintDesc const& desc);
}
