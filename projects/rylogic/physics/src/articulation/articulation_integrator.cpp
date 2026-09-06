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
		constexpr int MidpointIterationLimit = 12;
		constexpr float MidpointRelativeTolerance = 2.0e-6f;

		// Advance a floating root by a body-frame midpoint twist while preserving a rigid orthonormal transform.
		m4x4 IntegrateRootTransform(m4x4 const& root_to_world, v8motion midpoint_velocity, float elapsed_seconds)
		{
			auto const angular_displacement = midpoint_velocity.ang * elapsed_seconds;
			auto const maximum_angle_component = std::max({Abs(angular_displacement.x), Abs(angular_displacement.y), Abs(angular_displacement.z)});
			if (!IsFinite(maximum_angle_component) || maximum_angle_component > 1.0e4f)
				throw std::runtime_error("Articulation root angular displacement is too large for a finite integration step");

			auto const half_rotation = m3x3::Rotation(angular_displacement.xyz * 0.5f);
			auto const full_rotation = m3x3::Rotation(angular_displacement.xyz);
			auto const midpoint_to_world = root_to_world.rot * half_rotation;
			auto const rotation = root_to_world.rot * full_rotation;
			auto const position = root_to_world.pos + midpoint_to_world * midpoint_velocity.lin * elapsed_seconds;
			return Orthonorm(m4x4{rotation, position});
		}

		// Return the largest absolute generalized value without allocating reduction state.
		float MaximumMagnitude(std::span<float const> values)
		{
			auto magnitude = 0.0f;
			for (auto value : values)
				magnitude = std::max(magnitude, Abs(value));

			return magnitude;
		}

		// Reconstruct link-frame accelerations at the accepted midpoint from generalized acceleration and refreshed midpoint kinematics.
		void ReconstructMidpointAccelerations(detail::ArticulationState& state)
		{
			auto& root = state.m_links.front();
			switch (state.m_root_type)
			{
				case EArticulationRootType::Fixed:
				{
					root.m_link_acceleration = {};
					break;
				}
				case EArticulationRootType::Floating:
				{
					root.m_link_acceleration = detail::LoadSpatialMotion(state.m_acceleration, 0);
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}

			// Builder order is topological, so each child consumes an already reconstructed parent acceleration.
			for (int link_index = 1; link_index != isize(state.m_links); ++link_index)
			{
				auto& link = state.m_links[link_index];
				auto const& parent = state.m_links[link.m_parent_index];
				auto acceleration = link.m_parent_to_child * parent.m_link_acceleration + link.m_joint_bias;
				for (int axis_index = 0; axis_index != link.m_joint.m_dof_count; ++axis_index)
					acceleration += link.m_motion_subspace[axis_index] * state.m_acceleration[link.m_velocity_offset + axis_index];

				link.m_link_acceleration = acceleration;
			}
		}

		// Restore the accepted state after a failed nonlinear midpoint solve.
		void RestoreState(detail::ArticulationState& state, m4x4 const& root_to_world)
		{
			std::ranges::copy(state.m_position_start, state.m_position.begin());
			std::ranges::copy(state.m_velocity_start, state.m_velocity.begin());
			std::ranges::copy(state.m_acceleration_start, state.m_acceleration.begin());
			for (auto& link : state.m_links)
				link.m_link_acceleration = link.m_link_acceleration_start;
			state.m_links.front().m_link_to_world = root_to_world;
			state.m_kinematics_dirty = true;
		}

		// Restore the last accepted state unless the complete midpoint update is committed.
		struct StateRollback
		{
			detail::ArticulationState& m_state;
			m4x4 const& m_root_to_world;
			bool m_committed = false;

			// Restore state during exception unwinding without allocating or throwing.
			~StateRollback()
			{
				if (!m_committed)
					RestoreState(m_state, m_root_to_world);
			}
		};
	}

	namespace detail
	{
		// Integrate a detached generalized pseudo velocity into articulation coordinates without changing physical momentum.
		void ApplyArticulationPositionCorrection(Articulation& articulation, std::span<float const> generalized_velocity, float timestep)
		{
			if (!articulation.m_state)
				throw std::logic_error("Articulation has been moved from");
			if (!IsFinite(timestep) || timestep <= 0.0f)
				throw std::invalid_argument("Articulation position-correction timestep must be finite and positive");
			if (isize(generalized_velocity) != articulation.DofCount())
				throw std::invalid_argument("Articulation pseudo-velocity dimension does not match the articulation");
			if (!std::ranges::all_of(generalized_velocity, [](float value) { return IsFinite(value); }))
				throw std::invalid_argument("Articulation pseudo velocity must be finite");
			if (std::ranges::all_of(generalized_velocity, [](float value) { return value == 0.0f; }))
				return;

			auto& state = *articulation.m_state;
			articulation.Wake();

			// Reduced-coordinate drift follows the same generalized ordering as physical articulation integration.
			for (auto const& link : state.m_links | std::views::drop(1))
			{
				for (int axis_index = 0; axis_index != link.m_joint.m_dof_count; ++axis_index)
					state.m_position[link.m_position_offset + axis_index] += timestep * generalized_velocity[link.m_velocity_offset + axis_index];
			}

			// A floating root uses its body-frame pseudo twist while a fixed root contributes no generalized root entries.
			switch (state.m_root_type)
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto& root_to_world = state.m_links.front().m_link_to_world;
					root_to_world = IntegrateRootTransform(root_to_world, LoadSpatialMotion(generalized_velocity, 0), timestep);
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}
			state.m_kinematics_dirty = true;
		}

		// Validate a detached GPU integration result without changing its articulation.
		void ValidateArticulationIntegrationOutput(Articulation const& articulation, ArticulationIntegrationOutput const& output)
		{
			if (!articulation.m_state)
				throw std::logic_error("Articulation has been moved from");

			auto const& state = *articulation.m_state;
			if (!IsFinite(output.m_substep_seconds) || output.m_substep_seconds < 0.0f)
				throw std::invalid_argument("GPU articulation substep duration must be finite and non-negative");
			if (output.m_positions.size() != state.m_position.size() ||
				output.m_velocities.size() != state.m_velocity.size() ||
				output.m_accelerations.size() != state.m_acceleration.size())
				throw std::invalid_argument("GPU articulation generalized output dimensions do not match the articulation");

			ValidateArticulationTransform(output.m_root_to_world, "GPU articulation root transform");
			if (!std::ranges::all_of(output.m_positions, [](float value) { return IsFinite(value); }) ||
				!std::ranges::all_of(output.m_velocities, [](float value) { return IsFinite(value); }) ||
				!std::ranges::all_of(output.m_accelerations, [](float value) { return IsFinite(value); }))
				throw std::invalid_argument("GPU articulation generalized output must be finite");
		}

		// Commit one prevalidated GPU integration result and reconstruct its midpoint link accelerations.
		void CommitArticulationIntegrationOutput(Articulation& articulation, ArticulationIntegrationOutput const& output)
		{
			ValidateArticulationIntegrationOutput(articulation, output);
			if (output.m_substep_seconds == 0.0f)
			{
				articulation.ClearForces();
				return;
			}

			auto& state = *articulation.m_state;
			auto const half_dt = 0.5f * output.m_substep_seconds;

			// Stage detached final output in persistent scratch so reconstruction remains safe even if the source spans alias articulation storage.
			std::ranges::copy(output.m_positions, state.m_position_start.begin());
			std::ranges::copy(output.m_velocities, state.m_velocity_start.begin());
			std::ranges::copy(output.m_accelerations, state.m_acceleration_start.begin());

			// The converged midpoint follows directly from final velocity and acceleration, independent of earlier internal substeps.
			for (int index = 0; index != isize(state.m_velocity); ++index)
				state.m_velocity_midpoint[index] = state.m_velocity_start[index] - half_dt * state.m_acceleration_start[index];
			std::ranges::copy(state.m_velocity_midpoint, state.m_velocity.begin());
			std::ranges::copy(state.m_acceleration_start, state.m_acceleration.begin());
			for (auto const& link : state.m_links | std::views::drop(1))
			{
				for (int axis_index = 0; axis_index != link.m_joint.m_dof_count; ++axis_index)
				{
					state.m_position[link.m_position_offset + axis_index] =
						state.m_position_start[link.m_position_offset + axis_index] -
						half_dt * state.m_velocity_midpoint[link.m_velocity_offset + axis_index];
				}
			}
			state.m_kinematics_dirty = true;
			articulation.UpdateKinematics();
			ReconstructMidpointAccelerations(state);

			// Publish only the final generalized state; midpoint kinematic caches remain deliberately dirty just like CPU integration.
			std::ranges::copy(state.m_position_start, state.m_position.begin());
			std::ranges::copy(state.m_velocity_start, state.m_velocity.begin());
			switch (state.m_root_type)
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					state.m_links.front().m_link_to_world = output.m_root_to_world;
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}
			state.m_kinematics_dirty = true;
			articulation.ClearForces();
		}
	}

	// Advance unconstrained state with a bounded implicit-midpoint solve and consume the applied forces.
	void Articulation::Integrate(float elapsed_seconds)
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");
		if (!IsFinite(elapsed_seconds) || elapsed_seconds < 0.0f)
			throw std::invalid_argument("Articulation integration timestep must be finite and non-negative");
		if (Sleeping())
		{
			ClearForces();
			return;
		}
		if (elapsed_seconds == 0.0f)
		{
			ClearForces();
			return;
		}

		auto& state = *m_state;
		auto const root_to_world_start = state.m_links.front().m_link_to_world;
		auto const half_dt = 0.5f * elapsed_seconds;
		std::ranges::copy(state.m_position, state.m_position_start.begin());
		std::ranges::copy(state.m_velocity, state.m_velocity_start.begin());
		std::ranges::copy(state.m_acceleration, state.m_acceleration_start.begin());
		for (auto& link : state.m_links)
			link.m_link_acceleration_start = link.m_link_acceleration;
		auto rollback = StateRollback{state, root_to_world_start};

		// Seed the nonlinear solve with an ABA acceleration at the accepted start state.
		ForwardDynamics();
		for (int index = 0; index != isize(state.m_velocity); ++index)
			state.m_velocity_midpoint[index] = state.m_velocity_start[index] + half_dt * state.m_acceleration[index];

		auto converged = false;
		for (int iteration = 0; iteration != MidpointIterationLimit; ++iteration)
		{
			if (!std::ranges::all_of(state.m_velocity_midpoint, [](float value) { return IsFinite(value); }))
				throw std::runtime_error("Articulation implicit-midpoint velocity became non-finite");

			// Evaluate ABA at the candidate midpoint configuration and velocity.
			std::ranges::copy(state.m_velocity_midpoint, state.m_velocity.begin());
			for (auto const& link : state.m_links | std::views::drop(1))
			{
				for (int axis_index = 0; axis_index != link.m_joint.m_dof_count; ++axis_index)
				{
					state.m_position[link.m_position_offset + axis_index] = state.m_position_start[link.m_position_offset + axis_index] + half_dt * state.m_velocity_midpoint[link.m_velocity_offset + axis_index];
					if (!IsFinite(state.m_position[link.m_position_offset + axis_index]))
						throw std::runtime_error("Articulation implicit-midpoint position became non-finite");
				}
			}
			if (state.m_root_type == EArticulationRootType::Floating)
				state.m_links.front().m_link_to_world = IntegrateRootTransform(root_to_world_start, detail::LoadSpatialMotion(state.m_velocity_midpoint, 0), half_dt);
			state.m_kinematics_dirty = true;
			ForwardDynamics();

			// Fixed-point iteration is bounded; a failed solve restores the last accepted finite state for a smaller caller-selected substep.
			auto residual = 0.0f;
			for (int index = 0; index != isize(state.m_velocity); ++index)
			{
				auto const next_velocity = state.m_velocity_start[index] + half_dt * state.m_acceleration[index];
				if (!IsFinite(next_velocity))
					throw std::runtime_error("Articulation implicit-midpoint velocity became non-finite");

				residual = std::max(residual, Abs(next_velocity - state.m_velocity_midpoint[index]));
				state.m_velocity_midpoint[index] = next_velocity;
			}
			auto const tolerance = MidpointRelativeTolerance * (1.0f + MaximumMagnitude(state.m_velocity_midpoint));
			if (residual <= tolerance)
			{
				converged = true;
				break;
			}
		}

		if (!converged || !std::ranges::all_of(state.m_acceleration, [](float value) { return IsFinite(value); }))
			throw std::runtime_error("Articulation implicit-midpoint integration did not converge; use a smaller substep");

		// Commit the midpoint slope in one update so the method remains time-centred and avoids first-order secular energy gain.
		for (int index = 0; index != isize(state.m_velocity); ++index)
			state.m_velocity[index] = state.m_velocity_start[index] + elapsed_seconds * state.m_acceleration[index];
		for (auto const& link : state.m_links | std::views::drop(1))
		{
			for (int axis_index = 0; axis_index != link.m_joint.m_dof_count; ++axis_index)
				state.m_position[link.m_position_offset + axis_index] = state.m_position_start[link.m_position_offset + axis_index] + elapsed_seconds * state.m_velocity_midpoint[link.m_velocity_offset + axis_index];
		}
		if (state.m_root_type == EArticulationRootType::Floating)
			state.m_links.front().m_link_to_world = IntegrateRootTransform(root_to_world_start, detail::LoadSpatialMotion(state.m_velocity_midpoint, 0), elapsed_seconds);
		state.m_kinematics_dirty = true;
		ClearForces();
		rollback.m_committed = true;
	}
}
