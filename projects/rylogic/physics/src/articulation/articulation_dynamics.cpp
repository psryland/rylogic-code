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
		// Return one generalized scalar from a spatial force using angular-then-linear ordering.
		float SpatialComponent(v8force force, int index)
		{
			if (index < 0 || index >= 6)
				throw std::out_of_range("Spatial force component is out of range");

			return index < 3 ? force.ang[index] : force.lin[index - 3];
		}

		// Accumulate scaled spatial-force columns without constructing a dynamic matrix.
		v8force MultiplyColumns(std::array<v8force, 6> const& columns, std::span<float const> values, int count)
		{
			auto result = v8force{};
			for (int index = 0; index != count; ++index)
				result += columns[index] * values[index];

			return result;
		}

		// Accumulate scaled motion-subspace columns without constructing a dynamic matrix.
		v8motion MultiplyColumns(std::array<v8motion, 6> const& columns, std::span<float const> values, int count)
		{
			auto result = v8motion{};
			for (int index = 0; index != count; ++index)
				result += columns[index] * values[index];

			return result;
		}

		// Multiply a row-major bounded joint matrix by one joint vector.
		std::array<float, 6> MultiplyJointMatrix(std::array<float, 36> const& matrix, std::span<float const> vector, int count)
		{
			auto result = std::array<float, 6>{};
			for (int row = 0; row != count; ++row)
				for (int column = 0; column != count; ++column)
					result[row] += matrix[row * 6 + column] * vector[column];

			return result;
		}

		// Invert a symmetric positive-definite joint inertia using a scale-aware Cholesky factorization.
		std::array<float, 36> InvertJointInertia(std::array<float, 36> matrix, int count)
		{
			auto lower = std::array<float, 36>{};
			auto scale = 1.0f;
			for (int row = 0; row != count; ++row)
			for (int column = 0; column != count; ++column)
			{
				if (!IsFinite(matrix[row * 6 + column]))
					throw std::runtime_error("Articulation joint inertia became non-finite");

				scale = std::max(scale, Abs(matrix[row * 6 + column]));
			}

			// Symmetrize round-off before factorization because the exact S-transpose-I-S operator is symmetric.
			for (int row = 0; row != count; ++row)
			for (int column = row + 1; column != count; ++column)
			{
				auto const value = 0.5f * (matrix[row * 6 + column] + matrix[column * 6 + row]);
				matrix[row * 6 + column] = value;
				matrix[column * 6 + row] = value;
			}

			auto const pivot_tolerance = 64.0f * std::numeric_limits<float>::epsilon() * scale * static_cast<float>(count);
			for (int row = 0; row != count; ++row)
			{
				for (int column = 0; column != row + 1; ++column)
				{
					auto value = matrix[row * 6 + column];
					for (int inner = 0; inner != column; ++inner)
						value -= lower[row * 6 + inner] * lower[column * 6 + inner];

					if (row == column)
					{
						if (value <= pivot_tolerance)
							throw std::runtime_error("Articulation joint inertia is singular or not positive definite");

						lower[row * 6 + column] = Sqrt(value);
					}
					else
					{
						lower[row * 6 + column] = value / lower[column * 6 + column];
					}
				}
			}

			// Solve one unit right-hand side per column so the inverse inherits the factorization's conditioning checks.
			auto inverse = std::array<float, 36>{};
			for (int inverse_column = 0; inverse_column != count; ++inverse_column)
			{
				auto intermediate = std::array<float, 6>{};
				for (int row = 0; row != count; ++row)
				{
					auto value = row == inverse_column ? 1.0f : 0.0f;
					for (int inner = 0; inner != row; ++inner)
						value -= lower[row * 6 + inner] * intermediate[inner];
					intermediate[row] = value / lower[row * 6 + row];
				}

				for (int row = count; row-- != 0;)
				{
					auto value = intermediate[row];
					for (int inner = row + 1; inner != count; ++inner)
						value -= lower[inner * 6 + row] * inverse[inner * 6 + inverse_column];
					inverse[row * 6 + inverse_column] = value / lower[row * 6 + row];
				}
			}
			return inverse;
		}

		// Form U*D^-1*U-transpose as a full spatial inertia in bounded scalar work.
		detail::SpatialInertia JointInertiaReduction(detail::ArticulationLinkState const& link)
		{
			auto reduction = detail::SpatialInertia::Zero();
			auto const count = link.m_joint.m_dof_count;
			for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
			{
				auto dual_projection = std::array<float, 6>{};
				for (int joint_row = 0; joint_row != count; ++joint_row)
					dual_projection[joint_row] = SpatialComponent(link.m_u_columns[joint_row], spatial_column);

				auto coefficients = MultiplyJointMatrix(link.m_inverse_joint_inertia, dual_projection, count);
				reduction.col(spatial_column, MultiplyColumns(link.m_u_columns, coefficients, count));
			}
			return reduction;
		}

		// Return the selected persistent generalized input array.
		std::span<float const> GeneralizedInput(detail::ArticulationState const& state, bool impulse_response)
		{
			return impulse_response
				? std::span<float const>{state.m_response}
				: std::span<float const>{state.m_force};
		}

		// Return the selected link-space external input.
		v8force ExternalInput(detail::ArticulationLinkState const& link, bool impulse_response)
		{
			return impulse_response ? link.m_response_impulse : link.m_external_force;
		}

		// Return the selected persistent generalized output array.
		std::span<float> GeneralizedOutput(detail::ArticulationState& state, bool impulse_response)
		{
			return impulse_response
				? std::span<float>{state.m_response}
				: std::span<float>{state.m_acceleration};
		}

		// Return the selected persistent link acceleration output.
		v8motion& LinkOutput(detail::ArticulationLinkState& link, bool impulse_response)
		{
			return impulse_response ? link.m_response_acceleration : link.m_link_acceleration;
		}

		// Return the selected persistent link acceleration output.
		v8motion const& LinkOutput(detail::ArticulationLinkState const& link, bool impulse_response)
		{
			return impulse_response ? link.m_response_acceleration : link.m_link_acceleration;
		}
	}

	namespace detail
	{
		// Run force or impulse ABA into the selected persistent output buffers.
		void SolveArticulationDynamics(ArticulationState& state, bool impulse_response)
		{
			auto const generalized_input = GeneralizedInput(state, impulse_response);
			auto generalized_output = GeneralizedOutput(state, impulse_response);
			std::ranges::fill(generalized_output, 0.0f);

			// Initialize each link's articulated inertia and bias force from physical inertia, current velocity, and external input.
			for (auto& link : state.m_links)
			{
				link.m_articulated_inertia = link.m_link.m_inertia.To6x6();
				auto const momentum = link.m_articulated_inertia * link.m_link_velocity;
				link.m_articulated_bias = impulse_response
					? -ExternalInput(link, true)
					: Cross(link.m_link_velocity, momentum) - ExternalInput(link, false);
			}

			// Eliminate child joint accelerations from leaves to root and propagate their reduced operators to each parent.
			for (int link_index = isize(state.m_links); link_index-- != 1;)
			{
				auto& link = state.m_links[link_index];
				auto const dof_count = link.m_joint.m_dof_count;
				auto joint_inertia = std::array<float, 36>{};
				for (int row = 0; row != dof_count; ++row)
				{
					link.m_u_columns[row] = link.m_articulated_inertia * link.m_motion_subspace[row];
					link.m_reduced_force[row] = generalized_input[link.m_velocity_offset + row] - Dot(link.m_motion_subspace[row], link.m_articulated_bias);
				}
				for (int row = 0; row != dof_count; ++row)
				for (int column = 0; column != dof_count; ++column)
					joint_inertia[row * 6 + column] = Dot(link.m_motion_subspace[row], link.m_u_columns[column]);
				link.m_inverse_joint_inertia = InvertJointInertia(joint_inertia, dof_count);

				// Remove the joint-space response while retaining its force and kinematic-bias contribution.
				auto const reduced_inertia = link.m_articulated_inertia - JointInertiaReduction(link);
				auto const reduced_coefficients = MultiplyJointMatrix(link.m_inverse_joint_inertia, link.m_reduced_force, dof_count);
				auto reduced_bias = link.m_articulated_bias + MultiplyColumns(link.m_u_columns, reduced_coefficients, dof_count);
				if (!impulse_response)
					reduced_bias += reduced_inertia * link.m_joint_bias;

				auto& parent = state.m_links[link.m_parent_index];
				auto const motion_parent_to_child = math::spatial::Transform<Motion>(link.m_parent_to_child);
				auto const force_child_to_parent = math::spatial::Transform<Force>(link.m_child_to_parent);
				parent.m_articulated_inertia = parent.m_articulated_inertia + force_child_to_parent * reduced_inertia * motion_parent_to_child;
				parent.m_articulated_bias += link.m_child_to_parent * reduced_bias;
			}

			// A floating base solves its complete six-dimensional articulated inertia; a fixed base supplies zero acceleration.
			auto& root = state.m_links.front();
			switch (state.m_root_type)
			{
				case EArticulationRootType::Fixed:
				{
					LinkOutput(root, impulse_response) = {};
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto root_input = LoadSpatialMotion(generalized_input, 0);
					auto root_force = v8force{root_input.ang, root_input.lin};
					auto root_acceleration = Invert(root.m_articulated_inertia) * (root_force - root.m_articulated_bias);
					LinkOutput(root, impulse_response) = root_acceleration;
					StoreSpatialMotion(generalized_output, 0, root_acceleration);
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}

			// Recover link and generalized accelerations from root to leaves using the cached elimination factors.
			for (int link_index = 1; link_index != isize(state.m_links); ++link_index)
			{
				auto& link = state.m_links[link_index];
				auto const& parent = state.m_links[link.m_parent_index];
				auto const dof_count = link.m_joint.m_dof_count;
				auto link_acceleration = link.m_parent_to_child * LinkOutput(parent, impulse_response);
				if (!impulse_response)
					link_acceleration += link.m_joint_bias;

				auto acceleration_rhs = link.m_reduced_force;
				for (int row = 0; row != dof_count; ++row)
					acceleration_rhs[row] -= Dot(link_acceleration, link.m_u_columns[row]);

				auto joint_acceleration = MultiplyJointMatrix(link.m_inverse_joint_inertia, acceleration_rhs, dof_count);
				link_acceleration += MultiplyColumns(link.m_motion_subspace, joint_acceleration, dof_count);
				LinkOutput(link, impulse_response) = link_acceleration;
				for (int row = 0; row != dof_count; ++row)
					generalized_output[link.m_velocity_offset + row] = joint_acceleration[row];
			}
		}
	}

	// Solve unconstrained generalized and link accelerations with Featherstone's force ABA.
	void Articulation::ForwardDynamics()
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");
		if (m_state->m_kinematics_dirty)
			UpdateKinematics();

		detail::SolveArticulationDynamics(*m_state, false);
	}

	// Apply one link-frame impulse through the complete tree response.
	void Articulation::ApplyImpulse(LinkHandle link, v8force impulse)
	{
		auto request = ArticulationImpulse{
			.m_link = link,
			.m_impulse = impulse,
		};
		ApplyImpulses(std::span{&request, 1});
	}

	// Accumulate link-frame impulses and apply them through one complete tree response.
	void Articulation::ApplyImpulses(std::span<ArticulationImpulse const> impulses)
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");
		if (m_state->m_kinematics_dirty)
			UpdateKinematics();

		// Reuse persistent scratch so a batched constraint iteration allocates no temporary tree-sized storage.
		std::ranges::fill(m_state->m_response, 0.0f);
		for (auto& link : m_state->m_links)
			link.m_response_impulse = {};
		for (auto const& request : impulses)
		{
			if (!IsFinite(request.m_impulse.ang) || !IsFinite(request.m_impulse.lin))
				throw std::invalid_argument("Articulation impulses must be finite");

			detail::CheckedLink(*m_state, request.m_link).m_response_impulse += request.m_impulse;
		}

		// Impulse ABA maps accumulated impulses directly to generalized velocity deltas at fixed configuration.
		detail::SolveArticulationDynamics(*m_state, true);
		for (int index = 0; index != isize(m_state->m_velocity); ++index)
			m_state->m_velocity[index] += m_state->m_response[index];
		m_state->m_kinematics_dirty = true;
	}
}
