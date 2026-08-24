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
		using DMobility = math::Mat6x8<double, Force, Motion>;

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

		// Convert angular-then-linear scalar components into a double-precision spatial force.
		DForce ToDouble(std::array<double, 6> const& force)
		{
			return DForce{
				DVector{force[0], force[1], force[2], 0.0},
				DVector{force[3], force[4], force[5], 0.0},
			};
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

		// Propagate generalized acceleration into each link frame independently of the production outward ABA pass.
		std::vector<DMotion> LinkAccelerations(std::span<OracleLink const> links, std::span<double const> acceleration)
		{
			auto result = std::vector<DMotion>(links.size());
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				auto const& link = links[link_index];
				auto value = link.m_bias_acceleration;
				if (link.m_parent_index != -1)
					value += link.m_parent_to_child * result[link.m_parent_index];
				for (int axis_index = 0; axis_index != link.m_dof_count; ++axis_index)
					value += link.m_motion_subspace[axis_index] * acceleration[link.m_velocity_offset + axis_index];

				result[link_index] = value;
			}
			return result;
		}

		// Propagate one generalized velocity delta into every link without adding configuration-velocity bias.
		std::vector<DMotion> LinkVelocityDeltas(std::span<OracleLink const> links, std::span<double const> velocity)
		{
			auto result = std::vector<DMotion>(links.size());
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				auto const& link = links[link_index];
				auto value = link.m_parent_index != -1
					? link.m_parent_to_child * result[link.m_parent_index]
					: DMotion{};
				for (int axis_index = 0; axis_index != link.m_dof_count; ++axis_index)
					value += link.m_motion_subspace[axis_index] * velocity[link.m_velocity_offset + axis_index];

				result[link_index] = value;
			}
			return result;
		}

		// Resolve one public link handle into the stable topological index used by the oracle.
		int LinkIndex(Articulation const& articulation, LinkHandle link)
		{
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				if (articulation.LinkAt(link_index) == link)
					return link_index;

			throw std::invalid_argument("Articulation constraint row references a foreign or stale link");
		}

		// Return generalized inverse dynamics for one acceleration probe using recursive Newton-Euler accumulation.
		std::vector<double> InverseDynamics(std::span<OracleLink const> links, EArticulationRootType root_type, std::span<double const> acceleration)
		{
			auto const link_acceleration = LinkAccelerations(links, acceleration);
			auto link_force = std::vector<DForce>(links.size());
			auto generalized_force = std::vector<double>(acceleration.size(), 0.0);

			// Propagate candidate generalized accelerations and evaluate each link's inertial wrench.
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				auto const& link = links[link_index];
				auto const momentum = link.m_inertia * link.m_velocity;
				link_force[link_index] = link.m_inertia * link_acceleration[link_index] + Cross(link.m_velocity, momentum) - link.m_external_force;
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

		// Return one angular-then-linear component of a double-precision spatial force.
		double SpatialComponent(DForce force, int index)
		{
			if (index < 0 || index >= 6)
				throw std::out_of_range("Spatial force component is out of range");

			return index < 3 ? force.ang[index] : force.lin[index - 3];
		}

		// Return one angular-then-linear motion basis vector.
		DMotion MotionBasis(int index)
		{
			if (index < 0 || index >= 6)
				throw std::out_of_range("Spatial motion basis index is out of range");

			auto result = DMotion{};
			if (index < 3)
				result.ang[index] = 1.0;
			else
				result.lin[index - 3] = 1.0;
			return result;
		}

		// Return one angular-then-linear force basis vector.
		DForce ForceBasis(int index)
		{
			if (index < 0 || index >= 6)
				throw std::out_of_range("Spatial force basis index is out of range");

			auto result = DForce{};
			if (index < 3)
				result.ang[index] = 1.0;
			else
				result.lin[index - 3] = 1.0;
			return result;
		}

		// Multiply one bounded dense joint matrix by a joint vector.
		std::array<double, 6> MultiplyJointMatrix(constraint_oracle::DenseMatrix const& matrix, std::span<double const> vector, int count)
		{
			auto result = std::array<double, 6>{};
			for (int row = 0; row != count; ++row)
				for (int column = 0; column != count; ++column)
					result[row] += matrix(row, column) * vector[column];

			return result;
		}

		// Accumulate scaled spatial-force columns without constructing an intermediate matrix.
		DForce MultiplyColumns(std::array<DForce, 6> const& columns, std::span<double const> values, int count)
		{
			auto result = DForce{};
			for (int index = 0; index != count; ++index)
				result += columns[index] * values[index];

			return result;
		}

		// Accumulate scaled motion-subspace columns without constructing an intermediate matrix.
		DMotion MultiplyColumns(std::array<DMotion, 6> const& columns, std::span<double const> values, int count)
		{
			auto result = DMotion{};
			for (int index = 0; index != count; ++index)
				result += columns[index] * values[index];

			return result;
		}

		// Invert one positive-definite reduced joint matrix through independent pivoted solves.
		constraint_oracle::DenseMatrix InvertJointMatrix(constraint_oracle::DenseMatrix const& matrix)
		{
			auto inverse = constraint_oracle::DenseMatrix(matrix.RowCount(), matrix.ColumnCount());
			auto unit = std::vector<double>(matrix.RowCount(), 0.0);
			for (int column = 0; column != matrix.ColumnCount(); ++column)
			{
				unit[column] = 1.0;
				auto const solution = SolveLinear(matrix, unit);
				unit[column] = 0.0;
				for (int row = 0; row != matrix.RowCount(); ++row)
					inverse(row, column) = solution[row];
			}
			return inverse;
		}

		// One configuration-only ABA factor used to prove the linear-time self-link mobility recurrence.
		struct MobilityFactor
		{
			DInertia m_articulated_inertia = DInertia::Zero();
			std::array<DForce, 6> m_u_columns = {};
			constraint_oracle::DenseMatrix m_inverse_joint_inertia = {};
		};

		// Return U*D^-1*U-transpose as a double-precision spatial inertia.
		DInertia JointInertiaReduction(MobilityFactor const& factor, int dof_count)
		{
			auto reduction = DInertia::Zero();
			for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
			{
				auto dual_projection = std::array<double, 6>{};
				for (int joint_row = 0; joint_row != dof_count; ++joint_row)
					dual_projection[joint_row] = SpatialComponent(factor.m_u_columns[joint_row], spatial_column);

				auto const coefficients = MultiplyJointMatrix(factor.m_inverse_joint_inertia, dual_projection, dof_count);
				reduction.col(spatial_column, MultiplyColumns(factor.m_u_columns, coefficients, dof_count));
			}
			return reduction;
		}

		// Return S*D^-1*S-transpose as the joint's direct spatial mobility.
		DMobility JointMobility(OracleLink const& link, MobilityFactor const& factor)
		{
			auto mobility = DMobility::Zero();
			for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
			{
				auto dual_projection = std::array<double, 6>{};
				auto const force_basis = ForceBasis(spatial_column);
				for (int joint_row = 0; joint_row != link.m_dof_count; ++joint_row)
					dual_projection[joint_row] = Dot(link.m_motion_subspace[joint_row], force_basis);

				auto const coefficients = MultiplyJointMatrix(factor.m_inverse_joint_inertia, dual_projection, link.m_dof_count);
				mobility.col(spatial_column, MultiplyColumns(link.m_motion_subspace, coefficients, link.m_dof_count));
			}
			return mobility;
		}

		// Return I-S*D^-1*U-transpose, which propagates parent acceleration into the child after joint elimination.
		math::Mat6x8<double, Motion, Motion> MotionProjection(OracleLink const& link, MobilityFactor const& factor)
		{
			auto projection = math::Mat6x8<double, Motion, Motion>::Zero();
			for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
			{
				auto const basis = MotionBasis(spatial_column);
				auto dual_projection = std::array<double, 6>{};
				for (int joint_row = 0; joint_row != link.m_dof_count; ++joint_row)
					dual_projection[joint_row] = Dot(basis, factor.m_u_columns[joint_row]);

				auto const coefficients = MultiplyJointMatrix(factor.m_inverse_joint_inertia, dual_projection, link.m_dof_count);
				projection.col(spatial_column, basis - MultiplyColumns(link.m_motion_subspace, coefficients, link.m_dof_count));
			}
			return projection;
		}

		// Convert one spatial mobility to the row-major dense representation used by the test oracle.
		constraint_oracle::DenseMatrix ToDense(DMobility const& mobility)
		{
			auto result = constraint_oracle::DenseMatrix(6, 6);
			for (int column = 0; column != 6; ++column)
			{
				auto const value = mobility.col(column);
				result(0, column) = value.ang.x;
				result(1, column) = value.ang.y;
				result(2, column) = value.ang.z;
				result(3, column) = value.lin.x;
				result(4, column) = value.lin.y;
				result(5, column) = value.lin.z;
			}
			return result;
		}

		// Compute all exact self-link mobilities with one inward factorization and one outward recurrence.
		std::vector<constraint_oracle::DenseMatrix> RecursiveLinkMobilities(std::span<OracleLink const> links, EArticulationRootType root_type, int64_t& work_count)
		{
			auto factors = std::vector<MobilityFactor>(links.size());
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				factors[link_index].m_articulated_inertia = links[link_index].m_inertia;
				++work_count;
			}

			// Eliminate each child joint once and accumulate its reduced articulated inertia into the parent.
			for (int link_index = isize(links); link_index-- != 1;)
			{
				++work_count;
				auto const& link = links[link_index];
				auto& factor = factors[link_index];
				auto joint_inertia = constraint_oracle::DenseMatrix(link.m_dof_count, link.m_dof_count);
				for (int row = 0; row != link.m_dof_count; ++row)
					factor.m_u_columns[row] = factor.m_articulated_inertia * link.m_motion_subspace[row];
				for (int row = 0; row != link.m_dof_count; ++row)
					for (int column = 0; column != link.m_dof_count; ++column)
						joint_inertia(row, column) = Dot(link.m_motion_subspace[row], factor.m_u_columns[column]);
				if (link.m_dof_count != 0)
					factor.m_inverse_joint_inertia = InvertJointMatrix(joint_inertia);

				auto const reduced_inertia = factor.m_articulated_inertia - JointInertiaReduction(factor, link.m_dof_count);
				auto const motion_parent_to_child = math::spatial::Transform<Motion>(link.m_parent_to_child);
				auto const force_child_to_parent = math::spatial::Transform<Force>(link.m_child_to_parent);
				factors[link.m_parent_index].m_articulated_inertia =
					factors[link.m_parent_index].m_articulated_inertia +
					force_child_to_parent * reduced_inertia * motion_parent_to_child;
			}

			auto mobility = std::vector<DMobility>(links.size(), DMobility::Zero());
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					mobility.front() = Invert(factors.front().m_articulated_inertia);
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation oracle encountered an invalid root type");
				}
			}

			// Propagate parent mobility through the joint and add the joint's direct D-inverse response.
			for (int link_index = 1; link_index != isize(links); ++link_index)
			{
				++work_count;
				auto const& link = links[link_index];
				auto const projection = MotionProjection(link, factors[link_index]);
				auto const parent_to_child = math::spatial::Transform<Motion>(link.m_parent_to_child);
				auto const propagation = projection * parent_to_child;
				auto propagation_transpose = Transpose(propagation);
				auto const& dual_propagation = static_cast<math::Mat6x8<double, Force, Force> const&>(propagation_transpose);
				auto const joint_mobility = JointMobility(link, factors[link_index]);
				for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
				{
					auto const force = ForceBasis(spatial_column);
					auto const parent_force = dual_propagation * force;
					auto const parent_motion = mobility[link.m_parent_index] * parent_force;
					mobility[link_index].col(spatial_column, propagation * parent_motion + joint_mobility * force);
				}
			}

			auto result = std::vector<constraint_oracle::DenseMatrix>{};
			result.reserve(mobility.size());
			for (auto const& value : mobility)
			{
				result.push_back(ToDense(value));
				++work_count;
			}
			return result;
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

		// Preserve double precision through the independent acceleration propagation before exposing plain scalar records to tests.
		auto solve_mass = mass;
		auto acceleration = SolveLinear(std::move(solve_mass), rhs);
		auto const links = BuildModel(articulation, true);
		auto const spatial_acceleration = LinkAccelerations(links, acceleration);
		auto link_acceleration = std::vector<std::array<double, 6>>(spatial_acceleration.size());
		for (int link_index = 0; link_index != isize(spatial_acceleration); ++link_index)
		{
			auto const value = spatial_acceleration[link_index];
			link_acceleration[link_index] = {
				value.ang.x, value.ang.y, value.ang.z,
				value.lin.x, value.lin.y, value.lin.z,
			};
		}

		return DynamicsSolution{
			.m_mass = std::move(mass),
			.m_bias = std::move(bias),
			.m_acceleration = std::move(acceleration),
			.m_link_acceleration = std::move(link_acceleration),
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

	// Form J and A=J*H^-1*J-transpose independently in double precision for a tiny articulation constraint system.
	ConstraintSystem BuildConstraintSystem(Articulation const& articulation, std::span<ConstraintJacobianRow const> rows)
	{
		if (rows.empty())
			throw std::invalid_argument("Articulation constraint system requires at least one row");

		auto [mass, unused_bias] = EquationOfMotion(articulation, false);
		auto const links = BuildModel(articulation, false);
		auto resolved_links = std::vector<std::array<int, 2>>(rows.size());
		for (int row_index = 0; row_index != isize(rows); ++row_index)
		{
			auto const& row = rows[row_index];
			if (row.m_term_count < 1 || row.m_term_count > isize(row.m_terms))
				throw std::invalid_argument("Articulation constraint row must contain one or two endpoints");

			for (int term_index = 0; term_index != row.m_term_count; ++term_index)
			{
				for (auto component : row.m_terms[term_index].m_wrench)
					if (!std::isfinite(component))
						throw std::invalid_argument("Articulation constraint row contains a non-finite wrench");

				resolved_links[row_index][term_index] = LinkIndex(articulation, row.m_terms[term_index].m_link);
			}
		}

		// Probe generalized unit velocities to form both the requested rows and every link's spatial Jacobian.
		auto jacobian = constraint_oracle::DenseMatrix(isize(rows), articulation.DofCount());
		auto link_jacobian = std::vector<constraint_oracle::DenseMatrix>{};
		link_jacobian.reserve(links.size());
		for (int link_index = 0; link_index != isize(links); ++link_index)
			link_jacobian.emplace_back(6, articulation.DofCount());
		auto generalized_velocity = std::vector<double>(articulation.DofCount(), 0.0);
		for (int column = 0; column != articulation.DofCount(); ++column)
		{
			generalized_velocity[column] = 1.0;
			auto const link_velocity = LinkVelocityDeltas(links, generalized_velocity);
			generalized_velocity[column] = 0.0;
			for (int link_index = 0; link_index != isize(links); ++link_index)
			{
				auto const velocity = link_velocity[link_index];
				link_jacobian[link_index](0, column) = velocity.ang.x;
				link_jacobian[link_index](1, column) = velocity.ang.y;
				link_jacobian[link_index](2, column) = velocity.ang.z;
				link_jacobian[link_index](3, column) = velocity.lin.x;
				link_jacobian[link_index](4, column) = velocity.lin.y;
				link_jacobian[link_index](5, column) = velocity.lin.z;
			}
			for (int row_index = 0; row_index != isize(rows); ++row_index)
			{
				auto const& row = rows[row_index];
				for (int term_index = 0; term_index != row.m_term_count; ++term_index)
					jacobian(row_index, column) += Dot(ToDouble(row.m_terms[term_index].m_wrench), link_velocity[resolved_links[row_index][term_index]]);
			}
		}

		// Apply H^-1 to every Jacobian row so the exact Delassus matrix remains independent of production impulse ABA.
		auto response = constraint_oracle::DenseMatrix(isize(rows), isize(rows));
		auto generalized_impulse = std::vector<double>(articulation.DofCount(), 0.0);
		for (int response_column = 0; response_column != isize(rows); ++response_column)
		{
			for (int dof_index = 0; dof_index != articulation.DofCount(); ++dof_index)
				generalized_impulse[dof_index] = jacobian(response_column, dof_index);

			auto const velocity_delta = SolveLinear(mass, generalized_impulse);
			for (int response_row = 0; response_row != isize(rows); ++response_row)
				for (int dof_index = 0; dof_index != articulation.DofCount(); ++dof_index)
					response(response_row, response_column) += jacobian(response_row, dof_index) * velocity_delta[dof_index];
		}

		// Exact self-link mobility matrices provide the reference for the linear-time articulated-inertia recurrence selected by the gate.
		auto link_response = std::vector<constraint_oracle::DenseMatrix>{};
		link_response.reserve(links.size());
		for (int link_index = 0; link_index != isize(links); ++link_index)
		{
			auto mobility = constraint_oracle::DenseMatrix(6, 6);
			for (int response_column = 0; response_column != 6; ++response_column)
			{
				for (int dof_index = 0; dof_index != articulation.DofCount(); ++dof_index)
					generalized_impulse[dof_index] = link_jacobian[link_index](response_column, dof_index);

				auto const velocity_delta = SolveLinear(mass, generalized_impulse);
				for (int response_row = 0; response_row != 6; ++response_row)
					for (int dof_index = 0; dof_index != articulation.DofCount(); ++dof_index)
						mobility(response_row, response_column) += link_jacobian[link_index](response_row, dof_index) * velocity_delta[dof_index];
			}
			link_response.push_back(std::move(mobility));
		}

		auto recursive_work = int64_t{};
		auto recursive_link_response = RecursiveLinkMobilities(links, articulation.RootType(), recursive_work);
		return ConstraintSystem{
			.m_mass = std::move(mass),
			.m_jacobian = std::move(jacobian),
			.m_response = std::move(response),
			.m_link_response = std::move(link_response),
			.m_recursive_link_response = std::move(recursive_link_response),
			.m_recursive_work = recursive_work,
		};
	}
}
