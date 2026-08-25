//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_internal.h"
#include "src/unittests/articulation_oracle.h"

namespace pr::physics::tests
{
	namespace
	{
		using constraint_oracle::DenseMatrix;
		using articulation_oracle::ConstraintJacobianRow;
		using articulation_oracle::ConstraintJacobianTerm;

		// A contiguous row range projected and preconditioned as one solver block.
		struct ConstraintBlock
		{
			int m_row_begin;
			int m_row_count;
		};

		// An articulation and its stable topological link handles for constructing test rows.
		struct TestTree
		{
			Articulation m_articulation;
			std::vector<LinkHandle> m_links;
		};

		// Spectral properties that determine a preconditioned simultaneous iteration's stable step and rate.
		struct SpectralMetrics
		{
			double m_response_minimum;
			double m_response_condition;
			double m_preconditioned_maximum;
			double m_preconditioned_condition;
			double m_majorizer_minimum;
		};

		// Error reduction and line-search work measured over one fixed iteration budget.
		struct IterationMetrics
		{
			double m_error_at_16;
			double m_error_at_64;
			int m_backtrack_count;
		};

		// Return asymmetric positive mass properties so all spatial-inertia couplings remain observable.
		ArticulationLinkDesc EffectiveMassLink(int seed, float mass)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.19f + 0.011f * scale, 0.27f + 0.007f * scale, 0.34f + 0.009f * scale, 0},
					mass,
					v4{0.013f * scale, -0.009f * scale, 0.006f * scale, 0}),
			};
		}

		// Build a deterministic chain whose varied joint axes and attachment offsets exercise full tree coupling.
		TestTree BuildTestTree(EArticulationRootType root_type, int link_count, float mass_scale = 1.0f)
		{
			if (link_count < 2)
				throw std::invalid_argument("Effective-mass test tree requires at least two links");

			auto builder = ArticulationBuilder{};
			auto links = std::vector<LinkHandle>{};
			links.reserve(link_count);
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					links.push_back(builder.AddFixedRoot(EffectiveMassLink(0, 2.7f * mass_scale)));
					break;
				}
				case EArticulationRootType::Floating:
				{
					links.push_back(builder.AddFloatingRoot(EffectiveMassLink(0, 2.7f * mass_scale)));
					break;
				}
				default:
				{
					throw std::invalid_argument("Effective-mass test root type is invalid");
				}
			}

			auto const axes = std::array{v4::XAxis(), v4::YAxis(), v4::ZAxis()};
			for (int link_index = 1; link_index != link_count; ++link_index)
			{
				auto joint = ArticulationJointDesc::Revolute(
					axes[link_index % isize(axes)],
					m4x4::Translation(0.31f, 0.05f * static_cast<float>(link_index & 1), -0.04f),
					m4x4::Translation(-0.18f, 0.03f, 0.06f));
				joint.m_initial_position[0] = 0.07f * static_cast<float>(link_index);
				links.push_back(builder.AddLink(
					links.back(),
					joint,
					EffectiveMassLink(link_index, mass_scale * (0.6f + 0.17f * static_cast<float>(link_index)))));
			}
			return TestTree{
				.m_articulation = builder.Build(),
				.m_links = std::move(links),
			};
		}

		// Return an angular link-frame wrench along a unit direction.
		std::array<double, 6> AngularWrench(v4 axis)
		{
			axis = Normalise(axis);
			return {axis.x, axis.y, axis.z, 0.0, 0.0, 0.0};
		}

		// Return a linear link-frame wrench along a unit direction.
		std::array<double, 6> LinearWrench(v4 axis)
		{
			axis = Normalise(axis);
			return {0.0, 0.0, 0.0, axis.x, axis.y, axis.z};
		}

		// Reverse a wrench contribution without changing its frame.
		std::array<double, 6> Negated(std::array<double, 6> wrench)
		{
			for (auto& component : wrench)
				component = -component;

			return wrench;
		}

		// Return a scalar row with one articulation endpoint.
		ConstraintJacobianRow OneEndpointRow(LinkHandle link, std::array<double, 6> wrench)
		{
			return ConstraintJacobianRow{
				.m_terms = {ConstraintJacobianTerm{.m_link = link, .m_wrench = wrench}},
				.m_term_count = 1,
			};
		}

		// Return a scalar row coupling two links of one articulation.
		ConstraintJacobianRow TwoEndpointRow(LinkHandle link_a, std::array<double, 6> wrench_a, LinkHandle link_b, std::array<double, 6> wrench_b)
		{
			return ConstraintJacobianRow{
				.m_terms = {
					ConstraintJacobianTerm{.m_link = link_a, .m_wrench = wrench_a},
					ConstraintJacobianTerm{.m_link = link_b, .m_wrench = wrench_b},
				},
				.m_term_count = 2,
			};
		}

		// Convert angular-then-linear scalar components to the production spatial-force type.
		v8force SpatialForce(std::array<double, 6> const& wrench)
		{
			return v8force{
				v4{static_cast<float>(wrench[0]), static_cast<float>(wrench[1]), static_cast<float>(wrench[2]), 0},
				v4{static_cast<float>(wrench[3]), static_cast<float>(wrench[4]), static_cast<float>(wrench[5]), 0},
			};
		}

		// Return a symmetric copy so round-off asymmetry cannot contaminate spectral diagnostics.
		DenseMatrix Symmetrized(DenseMatrix matrix)
		{
			if (matrix.RowCount() != matrix.ColumnCount())
				throw std::invalid_argument("Only square matrices can be symmetrized");

			for (int row = 0; row != matrix.RowCount(); ++row)
			for (int column = row + 1; column != matrix.ColumnCount(); ++column)
			{
				auto const value = 0.5 * (matrix(row, column) + matrix(column, row));
				matrix(row, column) = value;
				matrix(column, row) = value;
			}
			return matrix;
		}

		// Return the largest absolute matrix element.
		double MaxAbs(DenseMatrix const& matrix)
		{
			auto result = 0.0;
			for (auto value : matrix.m_data)
				result = std::max(result, std::abs(value));

			return result;
		}

		// Add one matrix into another without constructing an intermediate allocation.
		void Add(DenseMatrix& lhs, DenseMatrix const& rhs)
		{
			if (lhs.RowCount() != rhs.RowCount() || lhs.ColumnCount() != rhs.ColumnCount())
				throw std::invalid_argument("Dense matrix dimensions do not agree");

			for (int index = 0; index != isize(lhs.m_data); ++index)
				lhs.m_data[index] += rhs.m_data[index];
		}

		// Add one strictly positive proximal diagonal to both the physical response and its preconditioner.
		double Regularize(DenseMatrix& response, DenseMatrix& preconditioner)
		{
			auto diagonal_scale = 0.0;
			for (int index = 0; index != response.RowCount(); ++index)
				diagonal_scale = std::max(diagonal_scale, std::abs(response(index, index)));

			auto const regularization = std::max(1.0e-10, 1.0e-6 * diagonal_scale);
			for (int index = 0; index != response.RowCount(); ++index)
			{
				response(index, index) += regularization;
				preconditioner(index, index) += regularization;
			}
			return regularization;
		}

		// Form the O(R) block approximation from linear-time self-link mobility matrices while omitting cross-link and cross-block response.
		DenseMatrix BuildLocalPreconditioner(
			Articulation const& articulation,
			articulation_oracle::ConstraintSystem const& system,
			std::span<ConstraintJacobianRow const> rows,
			std::span<ConstraintBlock const> blocks,
			double articulation_split = 1.0,
			int64_t* work_count = nullptr)
		{
			if (isize(system.m_recursive_link_response) != articulation.LinkCount())
				throw std::invalid_argument("Self-link response count does not match articulation topology");

			auto preconditioner = DenseMatrix(isize(rows), isize(rows));
			auto covered_rows = 0;
			for (auto const& block : blocks)
			{
				if (block.m_row_begin != covered_rows || block.m_row_count < 1 || block.m_row_begin + block.m_row_count > isize(rows))
					throw std::invalid_argument("Constraint blocks must cover every row once in contiguous order");

				for (int local_row = 0; local_row != block.m_row_count; ++local_row)
				for (int local_column = 0; local_column != block.m_row_count; ++local_column)
				{
					auto const row_index = block.m_row_begin + local_row;
					auto const column_index = block.m_row_begin + local_column;
					auto const& row = rows[row_index];
					auto const& column = rows[column_index];
					for (int row_term = 0; row_term != row.m_term_count; ++row_term)
					for (int column_term = 0; column_term != column.m_term_count; ++column_term)
					{
						if (row.m_terms[row_term].m_link != column.m_terms[column_term].m_link)
							continue;

						auto link_index = 0;
						for (; link_index != articulation.LinkCount(); ++link_index)
							if (articulation.LinkAt(link_index) == row.m_terms[row_term].m_link)
								break;
						if (link_index == articulation.LinkCount())
							throw std::invalid_argument("Preconditioner row references a foreign or stale articulation link");

						auto const& mobility = system.m_recursive_link_response[link_index];
						for (int spatial_row = 0; spatial_row != 6; ++spatial_row)
						for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
						{
							preconditioner(row_index, column_index) +=
								articulation_split *
								row.m_terms[row_term].m_wrench[spatial_row] *
								mobility(spatial_row, spatial_column) *
								column.m_terms[column_term].m_wrench[spatial_column];
							if (work_count != nullptr)
								++*work_count;
						}
					}
				}
				covered_rows += block.m_row_count;
			}
			if (covered_rows != isize(rows))
				throw std::invalid_argument("Constraint blocks do not cover every row");

			return Symmetrized(std::move(preconditioner));
		}

		// Form a conservative free-link block operator whose degree-scaled form is available when backtracking repeatedly rejects the faster approximation.
		DenseMatrix BuildFreeLinkPreconditioner(
			Articulation const& articulation,
			std::span<ConstraintJacobianRow const> rows,
			std::span<ConstraintBlock const> blocks,
			double articulation_split)
		{
			auto preconditioner = DenseMatrix(isize(rows), isize(rows));
			auto covered_rows = 0;
			for (auto const& block : blocks)
			{
				if (block.m_row_begin != covered_rows || block.m_row_count < 1 || block.m_row_begin + block.m_row_count > isize(rows))
					throw std::invalid_argument("Constraint blocks must cover every row once in contiguous order");

				for (int local_row = 0; local_row != block.m_row_count; ++local_row)
				for (int local_column = 0; local_column != block.m_row_count; ++local_column)
				{
					auto const row_index = block.m_row_begin + local_row;
					auto const column_index = block.m_row_begin + local_column;
					auto const& row = rows[row_index];
					auto const& column = rows[column_index];
					for (int row_term = 0; row_term != row.m_term_count; ++row_term)
					for (int column_term = 0; column_term != column.m_term_count; ++column_term)
					{
						if (row.m_terms[row_term].m_link != column.m_terms[column_term].m_link)
							continue;

						auto const inverse_inertia = Invert(articulation.LinkDescription(row.m_terms[row_term].m_link).m_inertia.To6x6());
						auto const response = inverse_inertia * SpatialForce(column.m_terms[column_term].m_wrench);
						preconditioner(row_index, column_index) += articulation_split * Dot(SpatialForce(row.m_terms[row_term].m_wrench), response);
					}
				}
				covered_rows += block.m_row_count;
			}
			if (covered_rows != isize(rows))
				throw std::invalid_argument("Constraint blocks do not cover every row");

			return Symmetrized(std::move(preconditioner));
		}

		// Factor a symmetric positive-definite matrix into L*L-transpose.
		DenseMatrix Cholesky(DenseMatrix const& matrix)
		{
			if (matrix.RowCount() == 0 || matrix.RowCount() != matrix.ColumnCount())
				throw std::invalid_argument("Cholesky factorization requires a non-empty square matrix");

			auto const dimension = matrix.RowCount();
			auto const tolerance = 256.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, MaxAbs(matrix)) * dimension;
			auto lower = DenseMatrix(dimension, dimension);
			for (int row = 0; row != dimension; ++row)
			for (int column = 0; column != row + 1; ++column)
			{
				auto value = matrix(row, column);
				for (int inner = 0; inner != column; ++inner)
					value -= lower(row, inner) * lower(column, inner);

				if (row == column)
				{
					if (value <= tolerance)
						throw std::runtime_error("Effective-mass matrix is not positive definite");

					lower(row, column) = std::sqrt(value);
				}
				else
				{
					lower(row, column) = value / lower(column, column);
				}
			}
			return lower;
		}

		// Solve L*x=rhs for a lower-triangular matrix.
		std::vector<double> SolveLower(DenseMatrix const& lower, std::span<double const> rhs)
		{
			if (lower.RowCount() != lower.ColumnCount() || lower.RowCount() != isize(rhs))
				throw std::invalid_argument("Lower-triangular solve dimensions do not agree");

			auto result = std::vector<double>(rhs.begin(), rhs.end());
			for (int row = 0; row != lower.RowCount(); ++row)
			{
				for (int column = 0; column != row; ++column)
					result[row] -= lower(row, column) * result[column];
				result[row] /= lower(row, row);
			}
			return result;
		}

		// Solve L-transpose*x=rhs for a lower-triangular matrix.
		std::vector<double> SolveLowerTranspose(DenseMatrix const& lower, std::span<double const> rhs)
		{
			if (lower.RowCount() != lower.ColumnCount() || lower.RowCount() != isize(rhs))
				throw std::invalid_argument("Upper-triangular solve dimensions do not agree");

			auto result = std::vector<double>(rhs.begin(), rhs.end());
			for (int row = lower.RowCount(); row-- != 0;)
			{
				for (int column = row + 1; column != lower.ColumnCount(); ++column)
					result[row] -= lower(column, row) * result[column];
				result[row] /= lower(row, row);
			}
			return result;
		}

		// Solve one symmetric positive-definite system by Cholesky factorization.
		std::vector<double> SolvePositiveDefinite(DenseMatrix const& matrix, std::span<double const> rhs)
		{
			auto const lower = Cholesky(matrix);
			auto const intermediate = SolveLower(lower, rhs);
			return SolveLowerTranspose(lower, intermediate);
		}

		// Return all eigenvalues of a small symmetric matrix using convergent Jacobi rotations.
		std::vector<double> SymmetricEigenvalues(DenseMatrix matrix)
		{
			if (matrix.RowCount() == 0 || matrix.RowCount() != matrix.ColumnCount())
				throw std::invalid_argument("Symmetric eigensolve requires a non-empty square matrix");

			matrix = Symmetrized(std::move(matrix));
			auto const dimension = matrix.RowCount();
			auto const tolerance = 512.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, MaxAbs(matrix));
			auto const iteration_limit = 100 * dimension * dimension;
			for (int iteration = 0; iteration != iteration_limit; ++iteration)
			{
				auto pivot_row = 0;
				auto pivot_column = 0;
				auto pivot_size = 0.0;
				for (int row = 0; row != dimension; ++row)
				for (int column = row + 1; column != dimension; ++column)
				{
					auto const candidate = std::abs(matrix(row, column));
					if (candidate <= pivot_size)
						continue;

					pivot_row = row;
					pivot_column = column;
					pivot_size = candidate;
				}
				if (pivot_size <= tolerance)
					break;
				if (iteration + 1 == iteration_limit)
					throw std::runtime_error("Symmetric eigensolve did not converge");

				// An orthogonal plane rotation annihilates the selected off-diagonal pair while preserving symmetry.
				auto const diagonal_row = matrix(pivot_row, pivot_row);
				auto const diagonal_column = matrix(pivot_column, pivot_column);
				auto const off_diagonal = matrix(pivot_row, pivot_column);
				auto const tau = (diagonal_column - diagonal_row) / (2.0 * off_diagonal);
				auto const tangent = std::copysign(1.0, tau) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
				auto const cosine = 1.0 / std::sqrt(1.0 + tangent * tangent);
				auto const sine = tangent * cosine;
				for (int index = 0; index != dimension; ++index)
				{
					if (index == pivot_row || index == pivot_column)
						continue;

					auto const row_value = matrix(index, pivot_row);
					auto const column_value = matrix(index, pivot_column);
					matrix(index, pivot_row) = cosine * row_value - sine * column_value;
					matrix(pivot_row, index) = matrix(index, pivot_row);
					matrix(index, pivot_column) = sine * row_value + cosine * column_value;
					matrix(pivot_column, index) = matrix(index, pivot_column);
				}
				matrix(pivot_row, pivot_row) = cosine * cosine * diagonal_row - 2.0 * sine * cosine * off_diagonal + sine * sine * diagonal_column;
				matrix(pivot_column, pivot_column) = sine * sine * diagonal_row + 2.0 * sine * cosine * off_diagonal + cosine * cosine * diagonal_column;
				matrix(pivot_row, pivot_column) = 0.0;
				matrix(pivot_column, pivot_row) = 0.0;
			}

			auto eigenvalues = std::vector<double>(dimension);
			for (int index = 0; index != dimension; ++index)
				eigenvalues[index] = matrix(index, index);
			std::ranges::sort(eigenvalues);
			return eigenvalues;
		}

		// Transform A by P^-1/2 so a symmetric eigensolve yields the generalized spectrum of A*x=lambda*P*x.
		DenseMatrix SymmetricPreconditionedOperator(DenseMatrix const& response, DenseMatrix const& preconditioner)
		{
			if (response.RowCount() != preconditioner.RowCount() || response.ColumnCount() != preconditioner.ColumnCount())
				throw std::invalid_argument("Response and preconditioner dimensions do not agree");

			auto const lower = Cholesky(preconditioner);
			auto left_solved = DenseMatrix(response.RowCount(), response.ColumnCount());
			auto column = std::vector<double>(response.RowCount());
			for (int column_index = 0; column_index != response.ColumnCount(); ++column_index)
			{
				for (int row_index = 0; row_index != response.RowCount(); ++row_index)
					column[row_index] = response(row_index, column_index);
				auto const solved = SolveLower(lower, column);
				for (int row_index = 0; row_index != response.RowCount(); ++row_index)
					left_solved(row_index, column_index) = solved[row_index];
			}

			auto result = DenseMatrix(response.RowCount(), response.ColumnCount());
			auto row = std::vector<double>(response.ColumnCount());
			for (int row_index = 0; row_index != response.RowCount(); ++row_index)
			{
				for (int column_index = 0; column_index != response.ColumnCount(); ++column_index)
					row[column_index] = left_solved(row_index, column_index);
				auto const solved = SolveLower(lower, row);
				for (int column_index = 0; column_index != response.ColumnCount(); ++column_index)
					result(row_index, column_index) = solved[column_index];
			}
			return Symmetrized(std::move(result));
		}

		// Return the ratio between the largest and smallest positive eigenvalues.
		double ConditionNumber(std::span<double const> eigenvalues)
		{
			if (eigenvalues.empty() || eigenvalues.front() <= 0.0)
				throw std::runtime_error("Condition number requires strictly positive eigenvalues");

			return eigenvalues.back() / eigenvalues.front();
		}

		// Measure response conditioning, preconditioned conditioning, and whether P globally majorizes A.
		SpectralMetrics MeasureSpectrum(DenseMatrix const& response, DenseMatrix const& preconditioner)
		{
			auto response_eigenvalues = SymmetricEigenvalues(response);
			auto preconditioned_eigenvalues = SymmetricEigenvalues(SymmetricPreconditionedOperator(response, preconditioner));
			auto majorizer = preconditioner;
			for (int index = 0; index != isize(majorizer.m_data); ++index)
				majorizer.m_data[index] -= response.m_data[index];
			auto majorizer_eigenvalues = SymmetricEigenvalues(std::move(majorizer));
			return SpectralMetrics{
				.m_response_minimum = response_eigenvalues.front(),
				.m_response_condition = ConditionNumber(response_eigenvalues),
				.m_preconditioned_maximum = preconditioned_eigenvalues.back(),
				.m_preconditioned_condition = ConditionNumber(preconditioned_eigenvalues),
				.m_majorizer_minimum = majorizer_eigenvalues.front(),
			};
		}

		// Multiply a dense matrix by one vector.
		std::vector<double> Multiply(DenseMatrix const& matrix, std::span<double const> vector)
		{
			if (matrix.ColumnCount() != isize(vector))
				throw std::invalid_argument("Dense matrix/vector dimensions do not agree");

			auto result = std::vector<double>(matrix.RowCount(), 0.0);
			for (int row = 0; row != matrix.RowCount(); ++row)
				for (int column = 0; column != matrix.ColumnCount(); ++column)
					result[row] += matrix(row, column) * vector[column];
			return result;
		}

		// Return the quadratic merit 0.5*lambda-transpose*A*lambda-rhs-transpose*lambda.
		double Objective(DenseMatrix const& response, std::span<double const> rhs, std::span<double const> impulse)
		{
			auto const response_impulse = Multiply(response, impulse);
			auto result = 0.0;
			for (int index = 0; index != isize(impulse); ++index)
				result += 0.5 * impulse[index] * response_impulse[index] - rhs[index] * impulse[index];
			return result;
		}

		// Return the response-metric norm of one impulse error.
		double EnergyNorm(DenseMatrix const& response, std::span<double const> error)
		{
			auto const response_error = Multiply(response, error);
			auto norm_squared = 0.0;
			for (int index = 0; index != isize(error); ++index)
				norm_squared += error[index] * response_error[index];
			return std::sqrt(std::max(0.0, norm_squared));
		}

		// Run simultaneous block-preconditioned descent with bounded monotone objective backtracking.
		IterationMetrics RunMonotoneIteration(
			DenseMatrix const& response,
			DenseMatrix const& preconditioner,
			std::span<ConstraintBlock const> blocks)
		{
			auto rhs = std::vector<double>(response.RowCount());
			for (int index = 0; index != isize(rhs); ++index)
				rhs[index] = (index & 1 ? -1.0 : +1.0) * (0.37 + 0.11 * static_cast<double>(index));

			auto const exact = SolvePositiveDefinite(response, rhs);
			auto impulse = std::vector<double>(response.RowCount(), 0.0);
			auto direction = std::vector<double>(response.RowCount(), 0.0);
			auto const initial_error = EnergyNorm(response, exact);
			auto metrics = IterationMetrics{
				.m_error_at_16 = std::numeric_limits<double>::infinity(),
				.m_error_at_64 = std::numeric_limits<double>::infinity(),
				.m_backtrack_count = 0,
			};

			for (int iteration = 0; iteration != 64; ++iteration)
			{
				auto gradient = Multiply(response, impulse);
				for (int index = 0; index != isize(gradient); ++index)
					gradient[index] -= rhs[index];

				// Solve every local block from the same residual so the harness matches the simultaneous coupled lane.
				for (auto const& block : blocks)
				{
					auto local_preconditioner = DenseMatrix(block.m_row_count, block.m_row_count);
					auto local_gradient = std::vector<double>(block.m_row_count);
					for (int row = 0; row != block.m_row_count; ++row)
					{
						local_gradient[row] = gradient[block.m_row_begin + row];
						for (int column = 0; column != block.m_row_count; ++column)
							local_preconditioner(row, column) = preconditioner(block.m_row_begin + row, block.m_row_begin + column);
					}
					auto const local_direction = SolvePositiveDefinite(local_preconditioner, local_gradient);
					for (int row = 0; row != block.m_row_count; ++row)
						direction[block.m_row_begin + row] = -local_direction[row];
				}

				// Backtracking prevents an approximate non-majorizing block operator from increasing the island merit.
				auto const objective = Objective(response, rhs, impulse);
				auto candidate = impulse;
				auto relaxation = 0.9;
				auto accepted = false;
				for (int retry = 0; retry != 12; ++retry)
				{
					for (int index = 0; index != isize(candidate); ++index)
						candidate[index] = impulse[index] + relaxation * direction[index];
					auto const candidate_objective = Objective(response, rhs, candidate);
					auto const tolerance = 1.0e-13 * std::max({1.0, std::abs(objective), std::abs(candidate_objective)});
					if (candidate_objective <= objective + tolerance)
					{
						accepted = true;
						break;
					}

					relaxation *= 0.5;
					++metrics.m_backtrack_count;
				}
				if (!accepted)
					throw std::runtime_error("Monotone effective-mass iteration exhausted its backtracking budget");

				impulse = std::move(candidate);
				auto error = impulse;
				for (int index = 0; index != isize(error); ++index)
					error[index] -= exact[index];
				auto const relative_error = EnergyNorm(response, error) / initial_error;
				if (iteration == 15)
					metrics.m_error_at_16 = relative_error;
				if (iteration == 63)
					metrics.m_error_at_64 = relative_error;
			}
			return metrics;
		}

		// Add one diagonal rigid-body mobility that is exact in both the response and local preconditioner.
		void AddRigidTranslationResponse(DenseMatrix& response, DenseMatrix& preconditioner, int row_begin, int row_count, double inverse_mass)
		{
			for (int row = 0; row != row_count; ++row)
			{
				response(row_begin + row, row_begin + row) += inverse_mass;
				preconditioner(row_begin + row, row_begin + row) += inverse_mass;
			}
		}

		// Require a canonical system to remain SPD and converge under the selected local approximation.
		void ExpectGatePass(
			std::string_view label,
			DenseMatrix response,
			DenseMatrix preconditioner,
			std::span<ConstraintBlock const> blocks,
			bool expect_non_majorizing)
		{
			response = Symmetrized(std::move(response));
			preconditioner = Symmetrized(std::move(preconditioner));
			Regularize(response, preconditioner);
			auto const spectrum = MeasureSpectrum(response, preconditioner);
			auto const iteration = RunMonotoneIteration(response, preconditioner, blocks);

			// Emit the measured evidence so the documented gate values can be regenerated from the standard test run.
			pr::unittests::TestFramework::out() << std::format(
				"  [effective-mass] {}: kappa(A)={:.6g}, kappa(P^-1 A)={:.6g}, lambda_max={:.6g}, min_eigenvalue(P-A)={:.6g}, error16={:.6g}, error64={:.6g}, backtracks={}\n",
				label,
				spectrum.m_response_condition,
				spectrum.m_preconditioned_condition,
				spectrum.m_preconditioned_maximum,
				spectrum.m_majorizer_minimum,
				iteration.m_error_at_16,
				iteration.m_error_at_64,
				iteration.m_backtrack_count);

			PR_EXPECT(spectrum.m_response_minimum > 0.0);
			PR_EXPECT(spectrum.m_response_condition < 128.0);
			PR_EXPECT(spectrum.m_preconditioned_maximum < 2.0 / 0.9);
			PR_EXPECT(spectrum.m_preconditioned_condition < 32.0);
			if (expect_non_majorizing)
				PR_EXPECT(spectrum.m_majorizer_minimum < 0.0);
			else
				PR_EXPECT(spectrum.m_majorizer_minimum > -1.0e-10 * std::max(1.0, MaxAbs(response)));
			PR_EXPECT(iteration.m_error_at_16 < 0.1);
			PR_EXPECT(iteration.m_error_at_64 < 0.01);
			PR_EXPECT(iteration.m_backtrack_count == 0);
		}

		// Require every recursively computed self-link mobility to match the independent dense H-inverse result.
		void ExpectRecursiveMobilitiesNear(articulation_oracle::ConstraintSystem const& system)
		{
			PR_EXPECT(system.m_link_response.size() == system.m_recursive_link_response.size());
			for (int link_index = 0; link_index != isize(system.m_link_response); ++link_index)
			{
				for (int row = 0; row != 6; ++row)
				for (int column = 0; column != 6; ++column)
				{
					auto const dense = system.m_link_response[link_index](row, column);
					auto const recursive = system.m_recursive_link_response[link_index](row, column);
					auto const scale = std::max({1.0, std::abs(dense), std::abs(recursive)});
					PR_EXPECT(std::abs(dense - recursive) < 2.0e-10 * scale);
				}
			}
		}
	}

	PRUnitTestClass(ArticulationEffectiveMassTests)
	{
		// Match the dense double-precision Delassus matrix against production impulse ABA column probes.
		PRUnitTestMethod(ExactResponseMatchesImpulseAba, Quick)
		{
			auto tree = BuildTestTree(EArticulationRootType::Floating, 6);
			auto const rows = std::array{
				OneEndpointRow(tree.m_links.back(), LinearWrench(v4::XAxis())),
				OneEndpointRow(tree.m_links.back(), LinearWrench(v4::YAxis())),
				OneEndpointRow(tree.m_links.back(), LinearWrench(v4::ZAxis())),
				OneEndpointRow(tree.m_links[3], AngularWrench(v4::XAxis())),
				OneEndpointRow(tree.m_links[3], AngularWrench(v4::YAxis())),
				TwoEndpointRow(tree.m_links[2], LinearWrench(Normalise(v4{1, 1, 0, 0})), tree.m_links.back(), Negated(LinearWrench(Normalise(v4{1, 1, 0, 0})))),
			};
			auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
			ExpectRecursiveMobilitiesNear(system);

			for (int response_column = 0; response_column != isize(rows); ++response_column)
			{
				auto impulses = std::array<ArticulationImpulse, 2>{};
				auto impulse_count = 0;
				for (int term_index = 0; term_index != rows[response_column].m_term_count; ++term_index)
				{
					auto const& term = rows[response_column].m_terms[term_index];
					impulses[impulse_count++] = ArticulationImpulse{.m_link = term.m_link, .m_impulse = SpatialForce(term.m_wrench)};
				}
				tree.m_articulation.ApplyImpulses(std::span{impulses}.first(impulse_count));

				for (int response_row = 0; response_row != isize(rows); ++response_row)
				{
					auto actual = 0.0;
					for (int term_index = 0; term_index != rows[response_row].m_term_count; ++term_index)
					{
						auto const& term = rows[response_row].m_terms[term_index];
						actual += Dot(SpatialForce(term.m_wrench), tree.m_articulation.LinkVelocity(term.m_link));
					}
					auto const scale = std::max({1.0, std::abs(actual), std::abs(system.m_response(response_row, response_column))});
					PR_EXPECT(std::abs(actual - system.m_response(response_row, response_column)) < 2.0e-3 * scale);
				}

				// Restore the zero-velocity probe state without changing configuration or stable handles.
				tree.m_articulation.RootVelocity({});
				for (int link_index = 1; link_index != tree.m_articulation.LinkCount(); ++link_index)
					tree.m_articulation.JointVelocity(tree.m_links[link_index], std::array{0.0f});
			}

			// The fixed-root zero-mobility seed follows a distinct recurrence branch and requires its own dense comparison.
			auto fixed_tree = BuildTestTree(EArticulationRootType::Fixed, 6);
			auto const fixed_rows = std::array{OneEndpointRow(fixed_tree.m_links.back(), LinearWrench(v4::XAxis()))};
			ExpectRecursiveMobilitiesNear(articulation_oracle::BuildConstraintSystem(fixed_tree.m_articulation, fixed_rows));
		}

		// Match the production linear-time self-link recurrence against the independent double-precision construction.
		PRUnitTestMethod(ProductionLinkMobilitiesMatchOracle, Quick)
		{
			for (auto const root_type : {EArticulationRootType::Fixed, EArticulationRootType::Floating})
			{
				auto tree = BuildTestTree(root_type, 8);
				auto const rows = std::array{OneEndpointRow(tree.m_links.back(), LinearWrench(v4::XAxis()))};
				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto mobilities = std::vector<detail::SpatialMobility>(tree.m_articulation.LinkCount());
				detail::ComputeArticulationLinkMobilities(tree.m_articulation, mobilities);

				for (int link_index = 0; link_index != tree.m_articulation.LinkCount(); ++link_index)
				for (int column = 0; column != 6; ++column)
				{
					auto const actual = mobilities[link_index].col(column);
					auto const actual_components = std::array{actual.ang.x, actual.ang.y, actual.ang.z, actual.lin.x, actual.lin.y, actual.lin.z};
					for (int row = 0; row != 6; ++row)
					{
						auto const expected = system.m_recursive_link_response[link_index](row, column);
						auto const scale = std::max({1.0, std::abs(expected), std::abs(static_cast<double>(actual_components[row]))});
						PR_EXPECT(std::abs(static_cast<double>(actual_components[row]) - expected) < 3.0e-4 * scale);
					}
				}
			}
		}

		// Select the O(R) free-link block approximation only if canonical fixed, floating, loop, and mixed systems converge monotonically.
		PRUnitTestMethod(LocalApproximationPassesCanonicalGate, Quick)
		{
			{
				auto tree = BuildTestTree(EArticulationRootType::Floating, 7);
				auto const rows = std::array{
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::XAxis())),
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::YAxis())),
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::ZAxis())),
					OneEndpointRow(tree.m_links[3], AngularWrench(v4::XAxis())),
					OneEndpointRow(tree.m_links[3], AngularWrench(v4::YAxis())),
					TwoEndpointRow(tree.m_links[2], LinearWrench(Normalise(v4{1, 0, 1, 0})), tree.m_links[5], Negated(LinearWrench(Normalise(v4{1, 0, 1, 0})))),
				};
				auto const blocks = std::array{
					ConstraintBlock{.m_row_begin = 0, .m_row_count = 3},
					ConstraintBlock{.m_row_begin = 3, .m_row_count = 2},
					ConstraintBlock{.m_row_begin = 5, .m_row_count = 1},
				};
				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto const preconditioner = BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks);
				ExpectGatePass("floating tree", system.m_response, preconditioner, blocks, true);
			}
			{
				auto tree = BuildTestTree(EArticulationRootType::Fixed, 8);
				auto const rows = std::array{
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::XAxis())),
					OneEndpointRow(tree.m_links[6], LinearWrench(v4::YAxis())),
					OneEndpointRow(tree.m_links[5], AngularWrench(Normalise(v4{1, 1, 1, 0}))),
				};
				auto const blocks = std::array{
					ConstraintBlock{.m_row_begin = 0, .m_row_count = 1},
					ConstraintBlock{.m_row_begin = 1, .m_row_count = 1},
					ConstraintBlock{.m_row_begin = 2, .m_row_count = 1},
				};
				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto const preconditioner = BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks);
				ExpectGatePass("fixed tree", system.m_response, preconditioner, blocks, true);
			}
			{
				auto tree = BuildTestTree(EArticulationRootType::Floating, 6);
				auto const rows = std::array{
					TwoEndpointRow(tree.m_links[2], LinearWrench(v4::XAxis()), tree.m_links.back(), Negated(LinearWrench(v4::XAxis()))),
					TwoEndpointRow(tree.m_links[2], LinearWrench(v4::YAxis()), tree.m_links.back(), Negated(LinearWrench(v4::YAxis()))),
					OneEndpointRow(tree.m_links[4], AngularWrench(v4::ZAxis())),
				};
				auto const blocks = std::array{
					ConstraintBlock{.m_row_begin = 0, .m_row_count = 2},
					ConstraintBlock{.m_row_begin = 2, .m_row_count = 1},
				};
				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto const preconditioner = BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks);
				ExpectGatePass("same-articulation loop", system.m_response, preconditioner, blocks, true);
			}
			{
				auto tree = BuildTestTree(EArticulationRootType::Floating, 5, 1.0e-2f);
				auto const rows = std::array{
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::XAxis())),
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::YAxis())),
					OneEndpointRow(tree.m_links.back(), LinearWrench(v4::ZAxis())),
				};
				auto const blocks = std::array{ConstraintBlock{.m_row_begin = 0, .m_row_count = 3}};
				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto preconditioner = BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks);
				auto response = system.m_response;
				AddRigidTranslationResponse(response, preconditioner, 0, 3, 1.0 / 100.0);
				ExpectGatePass("mixed rigid-articulation", std::move(response), std::move(preconditioner), blocks, false);
			}
			{
				auto tree_a = BuildTestTree(EArticulationRootType::Floating, 5, 0.7f);
				auto tree_b = BuildTestTree(EArticulationRootType::Floating, 4, 1.8f);
				auto const rows_a = std::array{
					OneEndpointRow(tree_a.m_links.back(), LinearWrench(v4::XAxis())),
					OneEndpointRow(tree_a.m_links.back(), LinearWrench(v4::YAxis())),
					OneEndpointRow(tree_a.m_links.back(), LinearWrench(v4::ZAxis())),
				};
				auto const rows_b = std::array{
					OneEndpointRow(tree_b.m_links.back(), Negated(LinearWrench(v4::XAxis()))),
					OneEndpointRow(tree_b.m_links.back(), Negated(LinearWrench(v4::YAxis()))),
					OneEndpointRow(tree_b.m_links.back(), Negated(LinearWrench(v4::ZAxis()))),
				};
				auto const blocks = std::array{ConstraintBlock{.m_row_begin = 0, .m_row_count = 3}};
				auto const system_a = articulation_oracle::BuildConstraintSystem(tree_a.m_articulation, rows_a);
				auto const system_b = articulation_oracle::BuildConstraintSystem(tree_b.m_articulation, rows_b);
				auto response = system_a.m_response;
				auto preconditioner = BuildLocalPreconditioner(tree_a.m_articulation, system_a, rows_a, blocks);
				Add(response, system_b.m_response);
				Add(preconditioner, BuildLocalPreconditioner(tree_b.m_articulation, system_b, rows_b, blocks));
				ExpectGatePass("two articulations", std::move(response), std::move(preconditioner), blocks, false);
			}
		}

		// A split by active articulation degree restores a global majorizer for a repeated-block hub where the unsplit local operator cannot.
		PRUnitTestMethod(DegreeSplitProvidesSafeFallback, Quick)
		{
			auto tree = BuildTestTree(EArticulationRootType::Floating, 5);
			auto const repeated_wrench = LinearWrench(Normalise(v4{1, 2, -1, 0}));
			auto const rows = std::array{
				OneEndpointRow(tree.m_links.back(), repeated_wrench),
				OneEndpointRow(tree.m_links.back(), repeated_wrench),
				OneEndpointRow(tree.m_links.back(), repeated_wrench),
				OneEndpointRow(tree.m_links.back(), repeated_wrench),
			};
			auto const blocks = std::array{
				ConstraintBlock{.m_row_begin = 0, .m_row_count = 1},
				ConstraintBlock{.m_row_begin = 1, .m_row_count = 1},
				ConstraintBlock{.m_row_begin = 2, .m_row_count = 1},
				ConstraintBlock{.m_row_begin = 3, .m_row_count = 1},
			};
			auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
			auto response = Symmetrized(system.m_response);
			auto local = BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks);
			auto split = BuildFreeLinkPreconditioner(tree.m_articulation, rows, blocks, static_cast<double>(blocks.size()));
			auto local_regularized = local;
			auto split_regularized = split;
			Regularize(response, local_regularized);
			for (int index = 0; index != response.RowCount(); ++index)
				split_regularized(index, index) += local_regularized(index, index) - local(index, index);

			auto const local_spectrum = MeasureSpectrum(response, local_regularized);
			auto const split_spectrum = MeasureSpectrum(response, split_regularized);
			PR_EXPECT(local_spectrum.m_majorizer_minimum < 0.0);
			PR_EXPECT(split_spectrum.m_majorizer_minimum > -1.0e-10 * std::max(1.0, MaxAbs(response)));
			PR_EXPECT(split_spectrum.m_preconditioned_maximum <= 1.0 + 1.0e-8);
		}

		// Actual recurrence and bounded-block work counters must grow linearly when links and rows are doubled together.
		PRUnitTestMethod(SelectedSetupWorkScalesLinearly, Quick)
		{
			auto measure = [](int link_count, int row_count)
			{
				auto tree = BuildTestTree(EArticulationRootType::Floating, link_count);
				auto rows = std::vector<ConstraintJacobianRow>{};
				auto blocks = std::vector<ConstraintBlock>{};
				rows.reserve(row_count);
				blocks.reserve(row_count);
				for (int row_index = 0; row_index != row_count; ++row_index)
				{
					auto const axes = std::array{v4::XAxis(), v4::YAxis(), v4::ZAxis()};
					rows.push_back(OneEndpointRow(tree.m_links.back(), LinearWrench(axes[row_index % isize(axes)])));
					blocks.push_back(ConstraintBlock{.m_row_begin = row_index, .m_row_count = 1});
				}

				auto const system = articulation_oracle::BuildConstraintSystem(tree.m_articulation, rows);
				auto block_work = int64_t{};
				BuildLocalPreconditioner(tree.m_articulation, system, rows, blocks, 1.0, &block_work);
				return std::pair{system.m_recursive_work, block_work};
			};

			auto const small_metrics = measure(8, 8);
			auto const large_metrics = measure(16, 16);
			auto const recurrence_ratio = static_cast<double>(large_metrics.first) / small_metrics.first;
			auto const block_ratio = static_cast<double>(large_metrics.second) / small_metrics.second;
			PR_EXPECT(recurrence_ratio > 1.8 && recurrence_ratio < 2.2);
			PR_EXPECT(block_ratio > 1.99 && block_ratio < 2.01);
		}
	};
}
#endif
