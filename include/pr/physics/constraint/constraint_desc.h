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

	// Configuration for one D6 coordinate. Velocity, stiffness, damping and force bounds refer to that coordinate and its conjugate force.
	// Rotational coordinate damping is not an isotropic physical angular damper away from zero relative orientation.
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

	// Selects the coordinate law; every kind uses the same persistent ownership and row solver.
	enum class EConstraintType
	{
		D6,
		SphericalAngular,
	};

	// Six-degree-of-freedom coordinate parameters.
	// Angular coordinates are components of the principal relative rotation vector in frame A, differentiable for angles strictly below pi.
	struct D6ConstraintParameters
	{
		std::array<ConstraintAxisDesc, 3> m_linear = {};
		std::array<ConstraintAxisDesc, 3> m_angular = {};
	};

	// Angular-only swing-ellipse/Y-twist limits and isotropic physical viscous damping; translation is unconstrained.
	// Ordered ranges contain zero and lie in [-pi,pi]. Swing range ends are quadrant half-axes, with zero-width sides acting as hinges.
	// Damping has units N*m*s/rad and applies implicit torque -m_damping*relative_angular_velocity, not coordinate-rate damping.
	struct SphericalAngularConstraintParameters
	{
		Range<float> m_swing_x = {-math::constants<float>::tau_by_2, +math::constants<float>::tau_by_2};
		Range<float> m_swing_z = {-math::constants<float>::tau_by_2, +math::constants<float>::tau_by_2};
		Range<float> m_twist = {-math::constants<float>::tau_by_2, +math::constants<float>::tau_by_2};
		float m_damping = 0.0f;
	};

	// One typed constraint law between endpoint-local frames. D6 is the default law; only the selected parameter alternative is stored.
	struct ConstraintDesc
	{
		BodyFrame m_frame_a = {};
		BodyFrame m_frame_b = {};
		std::variant<D6ConstraintParameters, SphericalAngularConstraintParameters> m_parameters = {};
		float m_break_force = std::numeric_limits<float>::infinity();
		float m_break_torque = std::numeric_limits<float>::infinity();
		bool m_collide_connected = false;
		bool m_enabled = true;

		// Return the selected coordinate law.
		EConstraintType Type() const;

		// Return D6 parameters, rejecting a descriptor of another kind.
		D6ConstraintParameters& D6();

		// Return D6 parameters, rejecting a descriptor of another kind.
		D6ConstraintParameters const& D6() const;
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
	ConstraintDesc ToD6(BallSocketConstraintDesc const& desc);

	// Convert a hinge descriptor to the general D6 representation.
	ConstraintDesc ToD6(HingeConstraintDesc const& desc);

	// Convert a slider descriptor to the general D6 representation.
	ConstraintDesc ToD6(SliderConstraintDesc const& desc);

	// Convert a weld descriptor to the general D6 representation.
	ConstraintDesc ToD6(WeldConstraintDesc const& desc);
}
