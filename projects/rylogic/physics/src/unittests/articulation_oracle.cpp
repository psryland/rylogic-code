//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/forward.h"
#include "src/unittests/articulation_oracle.h"

namespace pr::physics::tests::articulation_oracle
{
	namespace
	{
		using DVector = math::Vec4<double>;
		using DTransform = math::Mat4x4<double>;
		using DMotion = math::Vec8<double, Motion>;
		using DForce = math::Vec8<double, Force>;
		using DInertia = math::Mat6x8<double, Motion, Force>;

		// One double-precision link record used by the independent inverse-dynamics recursion.
		struct OracleLink
		{
			int m_parent_index = -1;
			int m_velocity_offset = 0;
			int m_dof_count = 0;
			DTransform m_child_to_parent = DTransform::Identity();
			DTransform m_parent_to_child = DTransform::Identity();
			std::array<DMotion, 6> m_motion_subspace = {};
			DMotion m_velocity = {};
			DMotion m_bias_acceleration = {};
			DInertia m_inertia = DInertia::Zero();
			DForce m_external_force = {};
		};

		// Convert a single-precision vector without changing its homogeneous component.
		DVector ToDouble(v4 vector)
		{
			return DVector{vector.x, vector.y, vector.z, vector.w};
		}

		// Convert a single-precision affine transform column by column.
		DTransform ToDouble(m4x4 const& transform)
		{
			return DTransform{
				ToDouble(transform.x),
				ToDouble(transform.y),
				ToDouble(transform.z),
				ToDouble(transform.w),
			};
		}

		// Convert a motion vector while preserving its spatial vector space.
		DMotion ToDouble(v8motion motion)
		{
			return DMotion{ToDouble(motion.ang), ToDouble(motion.lin)};
		}

		// Convert a force vector while preserving its dual spatial vector space.
		DForce ToDouble(v8force force)
		{
			return DForce{ToDouble(force.ang), ToDouble(force.lin)};
		}

		// Convert the physical spatial inertia into a double-precision matrix.
		DInertia ToDouble(Inertia const& inertia)
		{
			auto const source = inertia.To6x6();
			auto result = DInertia::Zero();
			for (int column = 0; column != 6; ++column)
			{
				auto const value = source.col(column);
				result.col(column, math::Vec8<double, void>{ToDouble(value.ang), ToDouble(value.lin)});
			}
			return result;
		}

		// Return one scalar screw displacement in the same ordered convention as the public joint descriptor.
		DTransform AxisTransform(ArticulationAxisDesc const& axis, double position)
		{
			switch (axis.m_type)
			{
				case EArticulationAxisType::Revolute:
				{
					return DTransform::Transform(Normalise(ToDouble(axis.m_axis)), position, DVector::Origin());
				}
				case EArticulationAxisType::Prismatic:
				{
					return DTransform::Translation(Normalise(ToDouble(axis.m_axis)) * position);
				}
				default:
				{
					throw std::runtime_error("Articulation oracle encountered an invalid joint axis type");
				}
			}
		}

		// Return a unit motion vector for one scalar joint coordinate.
		DMotion AxisMotion(ArticulationAxisDesc const& axis)
		{
			switch (axis.m_type)
			{
				case EArticulationAxisType::Revolute:
				{
					return DMotion{Normalise(ToDouble(axis.m_axis)), DVector{}};
				}
				case EArticulationAxisType::Prismatic:
				{
					return DMotion{DVector{}, Normalise(ToDouble(axis.m_axis))};
				}
				default:
				{
					throw std::runtime_error("Articulation oracle encountered an invalid joint axis type");
				}
			}
		}

		// Evaluate one ordered scalar-axis joint independently in double precision.
		void EvaluateJoint(ArticulationJointDesc const& joint, std::span<float const> position, std::span<float const> velocity, OracleLink& link)
		{
			auto motion_to_parent_joint = DTransform::Identity();
			auto joint_velocity = DMotion{};
			auto joint_bias = DMotion{};

			// Propagate through each scalar frame to retain all internal velocity-product terms.
			for (int axis_index = 0; axis_index != joint.m_dof_count; ++axis_index)
			{
				auto const axis_to_parent = AxisTransform(joint.m_axes[axis_index], position[axis_index]);
				auto const parent_to_axis = InvertOrthonormal(axis_to_parent);
				auto const axis_velocity = AxisMotion(joint.m_axes[axis_index]) * static_cast<double>(velocity[axis_index]);
				motion_to_parent_joint = motion_to_parent_joint * axis_to_parent;
				joint_velocity = parent_to_axis * joint_velocity + axis_velocity;
				joint_bias = parent_to_axis * joint_bias + Cross(joint_velocity, axis_velocity);
			}

			// Express the joint result at the physical child-link origin.
			auto const joint_to_parent = ToDouble(joint.m_joint_to_parent);
			auto const joint_to_child = ToDouble(joint.m_joint_to_child);
			link.m_child_to_parent = joint_to_parent * motion_to_parent_joint * InvertAffine(joint_to_child);
			link.m_parent_to_child = InvertAffine(link.m_child_to_parent);
			auto const child_joint_velocity = joint_to_child * joint_velocity;
			auto const child_joint_bias = joint_to_child * joint_bias;

			// Form each body Jacobian column by probing a unit speed through the ordered scalar frames.
			for (int column = 0; column != joint.m_dof_count; ++column)
			{
				auto unit_velocity = DMotion{};
				for (int axis_index = 0; axis_index != joint.m_dof_count; ++axis_index)
				{
					auto const parent_to_axis = InvertOrthonormal(AxisTransform(joint.m_axes[axis_index], position[axis_index]));
					unit_velocity = parent_to_axis * unit_velocity;
					if (axis_index == column)
						unit_velocity += AxisMotion(joint.m_axes[axis_index]);
				}
				link.m_motion_subspace[column] = joint_to_child * unit_velocity;
			}

			// Return joint-local terms; the caller adds transformed parent state once the complete transform is known.
			link.m_velocity = child_joint_velocity;
			link.m_bias_acceleration = child_joint_bias;
		}

		// Return a generalized vector assembled in the articulation's stable velocity order.
		std::vector<double> GeneralizedForce(Articulation const& articulation)
		{
			auto result = std::vector<double>(articulation.DofCount(), 0.0);
			auto offset = 0;
			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto const force = articulation.RootForce();
					result[0] = force.ang.x;
					result[1] = force.ang.y;
					result[2] = force.ang.z;
					result[3] = force.lin.x;
					result[4] = force.lin.y;
					result[5] = force.lin.z;
					offset = 6;
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation oracle encountered an invalid root type");
				}
			}

			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const force = articulation.JointForce(articulation.LinkAt(link_index));
				for (auto value : force)
					result[offset++] = value;
			}
			return result;
		}

		// Build double-precision topology, kinematics, and physical properties from the public articulation contract.
		std::vector<OracleLink> BuildModel(Articulation const& articulation, bool include_external_force)
		{
			auto links = std::vector<OracleLink>(articulation.LinkCount());
			auto handles = std::vector<LinkHandle>(articulation.LinkCount());
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				handles[link_index] = articulation.LinkAt(link_index);

			// The root contributes a six-dimensional identity Jacobian only when it is floating.
			auto& root = links.front();
			root.m_inertia = ToDouble(articulation.LinkDescription(handles.front()).m_inertia);
			root.m_external_force = include_external_force ? ToDouble(articulation.ExternalForce(handles.front())) : DForce{};
			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					root.m_dof_count = 6;
					root.m_velocity = ToDouble(articulation.RootVelocity());
					for (int column = 0; column != 6; ++column)
					{
						auto basis = math::Vec8<double, void>{};
						if (column < 3)
							basis.ang[column] = 1.0;
						else
							basis.lin[column - 3] = 1.0;
						root.m_motion_subspace[column] = static_cast<DMotion const&>(basis);
					}
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation oracle encountered an invalid root type");
				}
			}

			// Public link order is topological, so each parent's kinematics are ready before its children.
			auto velocity_offset = root.m_dof_count;
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto& link = links[link_index];
				auto const handle = handles[link_index];
				auto const parent = articulation.Parent(handle);
				auto const parent_iter = std::ranges::find(handles, parent);
				if (parent_iter == handles.end())
					throw std::runtime_error("Articulation oracle could not resolve a parent link");

				link.m_parent_index = s_cast<int>(parent_iter - handles.begin());
				link.m_velocity_offset = velocity_offset;
				link.m_dof_count = articulation.JointDofCount(handle);
				link.m_inertia = ToDouble(articulation.LinkDescription(handle).m_inertia);
				link.m_external_force = include_external_force ? ToDouble(articulation.ExternalForce(handle)) : DForce{};
				// Transform parent state before evaluating child-local joint terms.
				auto const& joint = articulation.JointDescription(handle);
				auto const position = articulation.JointPosition(handle);
				auto const velocity = articulation.JointVelocity(handle);
				EvaluateJoint(joint, position, velocity, link);
				auto const joint_velocity = link.m_velocity;
				auto const joint_bias = link.m_bias_acceleration;
				link.m_velocity = link.m_parent_to_child * links[link.m_parent_index].m_velocity + joint_velocity;
				link.m_bias_acceleration = joint_bias + Cross(link.m_velocity, joint_velocity);
				velocity_offset += link.m_dof_count;
			}
			return links;
		}

		// Return generalized inverse dynamics for one acceleration probe using recursive Newton-Euler accumulation.
		std::vector<double> InverseDynamics(std::span<OracleLink const> links, EArticulationRootType root_type, std::span<double const> acceleration)
		{
			auto link_acceleration = std::vector<DMotion>(links.size());
			auto link_force = std::vector<DForce>(links.size());
			auto generalized_force = std::vector<double>(acceleration.size(), 0.0);

			// Propagate candidate generalized accelerations and evaluate each link's inertial wrench.
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				auto const& link = links[link_index];
				auto value = link.m_bias_acceleration;
				if (link.m_parent_index != -1)
					value += link.m_parent_to_child * link_acceleration[link.m_parent_index];
				for (int axis_index = 0; axis_index != link.m_dof_count; ++axis_index)
					value += link.m_motion_subspace[axis_index] * acceleration[link.m_velocity_offset + axis_index];

				link_acceleration[link_index] = value;
				auto const momentum = link.m_inertia * link.m_velocity;
				link_force[link_index] = link.m_inertia * value + Cross(link.m_velocity, momentum) - link.m_external_force;
			}

			// Accumulate child wrenches toward the root and project each joint wrench onto its motion subspace.
			for (int link_index = isize(links); link_index-- != 0;)
			{
				auto const& link = links[link_index];
				for (int axis_index = 0; axis_index != link.m_dof_count; ++axis_index)
					generalized_force[link.m_velocity_offset + axis_index] = Dot(link.m_motion_subspace[axis_index], link_force[link_index]);
				if (link.m_parent_index != -1)
					link_force[link.m_parent_index] += link.m_child_to_parent * link_force[link_index];
			}

			if (root_type == EArticulationRootType::Floating)
			{
				auto const& force = link_force.front();
				generalized_force[0] = force.ang.x;
				generalized_force[1] = force.ang.y;
				generalized_force[2] = force.ang.z;
				generalized_force[3] = force.lin.x;
				generalized_force[4] = force.lin.y;
				generalized_force[5] = force.lin.z;
			}
			return generalized_force;
		}

		// Assemble H and h from independent inverse-dynamics acceleration probes.
		std::pair<constraint_oracle::DenseMatrix, std::vector<double>> EquationOfMotion(Articulation const& articulation, bool include_external_force)
		{
			auto const dof_count = articulation.DofCount();
			auto links = BuildModel(articulation, include_external_force);
			auto zero_acceleration = std::vector<double>(dof_count, 0.0);
			auto bias = InverseDynamics(links, articulation.RootType(), zero_acceleration);
			auto mass = constraint_oracle::DenseMatrix(dof_count, dof_count);

			// Subtract the zero-acceleration result from each probe so velocity bias and external forces cannot contaminate H.
			for (int column = 0; column != dof_count; ++column)
			{
				auto unit_acceleration = zero_acceleration;
				unit_acceleration[column] = 1.0;
				auto const force = InverseDynamics(links, articulation.RootType(), unit_acceleration);
				for (int row = 0; row != dof_count; ++row)
					mass(row, column) = force[row] - bias[row];
			}
			return {std::move(mass), std::move(bias)};
		}

		// Solve a small dense system with partial pivoting to remain independent of the production Cholesky solves.
		std::vector<double> SolveLinear(constraint_oracle::DenseMatrix matrix, std::span<double const> rhs)
		{
			auto const dimension = matrix.RowCount();
			if (dimension == 0 || matrix.ColumnCount() != dimension || isize(rhs) != dimension)
				throw std::invalid_argument("Articulation oracle linear-system dimensions do not agree");

			auto solution = std::vector<double>(rhs.begin(), rhs.end());
			auto scale = 1.0;
			for (auto value : matrix.m_data)
				scale = std::max(scale, std::abs(value));
			auto const pivot_tolerance = 128.0 * std::numeric_limits<double>::epsilon() * scale * dimension;

			// Partial pivoting keeps the dense reference reliable for mixed rotational and translational scales.
			for (int pivot = 0; pivot != dimension; ++pivot)
			{
				auto pivot_row = pivot;
				auto pivot_size = std::abs(matrix(pivot, pivot));
				for (int row = pivot + 1; row != dimension; ++row)
				{
					auto const candidate_size = std::abs(matrix(row, pivot));
					if (candidate_size > pivot_size)
					{
						pivot_row = row;
						pivot_size = candidate_size;
					}
				}
				if (pivot_size <= pivot_tolerance)
					throw std::runtime_error("Articulation oracle mass matrix is singular");

				if (pivot_row != pivot)
				{
					for (int column = pivot; column != dimension; ++column)
						std::swap(matrix(pivot, column), matrix(pivot_row, column));
					std::swap(solution[pivot], solution[pivot_row]);
				}
				for (int row = pivot + 1; row != dimension; ++row)
				{
					auto const factor = matrix(row, pivot) / matrix(pivot, pivot);
					for (int column = pivot + 1; column != dimension; ++column)
						matrix(row, column) -= factor * matrix(pivot, column);
					solution[row] -= factor * solution[pivot];
				}
			}

			for (int row = dimension; row-- != 0;)
			{
				for (int column = row + 1; column != dimension; ++column)
					solution[row] -= matrix(row, column) * solution[column];
				solution[row] /= matrix(row, row);
			}
			return solution;
		}

		// Replace external force fields with the requested simultaneous impulses.
		std::vector<OracleLink> BuildImpulseModel(Articulation const& articulation, std::span<ArticulationImpulse const> impulses)
		{
			auto links = BuildModel(articulation, false);
			for (auto const& impulse : impulses)
			{
				auto found = false;
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				{
					if (articulation.LinkAt(link_index) != impulse.m_link)
						continue;

					links[link_index].m_external_force += ToDouble(impulse.m_impulse);
					found = true;
					break;
				}
				if (!found)
					throw std::invalid_argument("Articulation impulse references a foreign or stale link");
			}
			return links;
		}
	}

	// Solve forward dynamics through double-precision recursive Newton-Euler probes and a dense pivoted solve.
	DynamicsSolution SolveForwardDynamics(Articulation const& articulation)
	{
		auto [mass, bias] = EquationOfMotion(articulation, true);
		auto rhs = GeneralizedForce(articulation);
		for (int index = 0; index != isize(rhs); ++index)
			rhs[index] -= bias[index];

		return DynamicsSolution{
			.m_mass = mass,
			.m_bias = std::move(bias),
			.m_acceleration = SolveLinear(std::move(mass), rhs),
		};
	}

	// Solve the generalized velocity change caused by simultaneous link-frame impulses.
	std::vector<double> SolveImpulseResponse(Articulation const& articulation, std::span<ArticulationImpulse const> impulses)
	{
		auto [mass, unused_bias] = EquationOfMotion(articulation, false);
		auto baseline_links = BuildModel(articulation, false);
		auto links = BuildImpulseModel(articulation, impulses);
		auto zero_acceleration = std::vector<double>(articulation.DofCount(), 0.0);
		auto const baseline_force = InverseDynamics(baseline_links, articulation.RootType(), zero_acceleration);
		auto impulse_force = InverseDynamics(links, articulation.RootType(), zero_acceleration);
		for (int index = 0; index != isize(impulse_force); ++index)
			impulse_force[index] = baseline_force[index] - impulse_force[index];

		return SolveLinear(std::move(mass), impulse_force);
	}
}
