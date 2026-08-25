//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/articulation/articulation.h"

namespace pr::physics::detail
{
	using SpatialInertia = Mat6x8<float, Motion, Force>;

	// Cached topology, state ranges, and kinematic intermediates for one physical link.
	struct ArticulationLinkState
	{
		LinkHandle m_handle = {};
		int m_parent_index = -1;
		int m_position_offset = 0;
		int m_velocity_offset = 0;
		ArticulationLinkDesc m_link = {};
		ArticulationJointDesc m_joint = {};
		m4x4 m_link_to_world = m4x4::Identity();
		m4x4 m_child_to_parent = m4x4::Identity();
		m4x4 m_parent_to_child = m4x4::Identity();
		std::array<v8motion, 6> m_motion_subspace = {};
		v8motion m_joint_velocity = {};
		v8motion m_joint_bias = {};          // Complete velocity-product acceleration c = cJ + v x vJ.
		v8motion m_link_velocity = {};
		v8motion m_link_acceleration = {};
		v8motion m_link_acceleration_start = {};
		v8motion m_response_acceleration = {};
		v8force m_external_force = {};
		v8force m_response_impulse = {};
		SpatialInertia m_articulated_inertia = SpatialInertia::Zero();
		v8force m_articulated_bias = {};
		std::array<v8force, 6> m_u_columns = {};
		std::array<float, 36> m_inverse_joint_inertia = {};
		std::array<float, 6> m_reduced_force = {};
	};

	// Complete CPU-owned topology, generalized arrays, and lazily refreshed link state.
	struct ArticulationState
	{
		ArticulationId m_id = {};
		uint32_t m_link_generation = 0;
		EArticulationRootType m_root_type = EArticulationRootType::Fixed;
		std::vector<ArticulationLinkState> m_links;
		std::vector<float> m_position;
		std::vector<float> m_velocity;
		std::vector<float> m_force;
		std::vector<float> m_acceleration;
		std::vector<float> m_response;
		std::vector<float> m_position_start;
		std::vector<float> m_velocity_start;
		std::vector<float> m_velocity_midpoint;
		std::vector<float> m_acceleration_start;
		bool m_kinematics_dirty = true;
	};

	// Tracks whether a unique root has been added and transfers its state exactly once.
	struct ArticulationBuilderState
	{
		std::unique_ptr<ArticulationState> m_articulation;
		bool m_consumed = false;
	};

	// Detached final generalized state returned by one successful GPU articulation step.
	struct ArticulationIntegrationOutput
	{
		m4x4 m_root_to_world = m4x4::Identity();
		std::span<float const> m_positions;
		std::span<float const> m_velocities;
		std::span<float const> m_accelerations;
		float m_substep_seconds = 0.0f;
	};

	// Return a validated link state or throw without indexing caller-controlled handle data.
	ArticulationLinkState& CheckedLink(ArticulationState& state, LinkHandle link);

	// Return a validated link state or throw without indexing caller-controlled handle data.
	ArticulationLinkState const& CheckedLink(ArticulationState const& state, LinkHandle link);

	// Validate a finite rigid transform used by articulation topology or state.
	void ValidateArticulationTransform(m4x4 const& transform, char const* name);

	// Load six contiguous generalized values as one padded spatial motion vector.
	v8motion LoadSpatialMotion(std::span<float const> values, int offset);

	// Store one padded spatial motion vector into six contiguous generalized values.
	void StoreSpatialMotion(std::span<float> values, int offset, v8motion motion);

	// Run force or impulse ABA into the selected persistent output buffers.
	void SolveArticulationDynamics(ArticulationState& state, bool impulse_response);
}
