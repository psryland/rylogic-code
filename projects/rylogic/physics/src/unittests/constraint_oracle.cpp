//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/forward.h"
#include "src/unittests/constraint_oracle.h"

namespace pr::physics::tests::constraint_oracle
{
	namespace
	{
		// Return a safe allocation size after validating matrix dimensions.
		size_t CheckedElementCount(int rows, int columns)
		{
			if (rows < 0 || columns < 0)
				throw std::invalid_argument("Dense matrix dimensions cannot be negative");

			return static_cast<size_t>(rows) * static_cast<size_t>(columns);
		}

		// Return the largest absolute matrix element for scale-aware validation.
		double MaxAbs(DenseMatrix const& matrix)
		{
			auto maximum = 0.0;
			for (auto value : matrix.m_data)
				maximum = std::max(maximum, std::abs(value));

			return maximum;
		}

		// Reject non-finite matrix data before numerical work can disguise invalid input.
		void ValidateFinite(DenseMatrix const& matrix, char const* name)
		{
			for (auto value : matrix.m_data)
			{
				if (!std::isfinite(value))
					throw std::invalid_argument(std::format("{} contains a non-finite value", name));
			}
		}

		// Reject non-finite vector data before numerical work can disguise invalid input.
		void ValidateFinite(std::span<double const> vector, char const* name)
		{
			for (auto value : vector)
			{
				if (!std::isfinite(value))
					throw std::invalid_argument(std::format("{} contains a non-finite value", name));
			}
		}

		// Require a finite symmetric positive-definite matrix so each reference optimization has a unique solution.
		void ValidatePositiveDefinite(DenseMatrix const& matrix, char const* name)
		{
			if (matrix.RowCount() == 0 || matrix.RowCount() != matrix.ColumnCount())
				throw std::invalid_argument(std::format("{} must be a non-empty square matrix", name));

			ValidateFinite(matrix, name);
			auto const dimension = matrix.RowCount();
			auto const scale = std::max(1.0, MaxAbs(matrix));
			auto const symmetry_tolerance = 64.0 * std::numeric_limits<double>::epsilon() * scale;
			for (int row = 0; row != dimension; ++row)
			{
				for (int column = row + 1; column != dimension; ++column)
				if (std::abs(matrix(row, column) - matrix(column, row)) > symmetry_tolerance)
					throw std::invalid_argument(std::format("{} must be symmetric", name));
			}

			// Cholesky factorization is used only as a strict convexity check, not as the reference solver.
			auto lower = DenseMatrix(dimension, dimension);
			auto const pivot_tolerance = 128.0 * std::numeric_limits<double>::epsilon() * scale * dimension;
			for (int row = 0; row != dimension; ++row)
			{
				for (int column = 0; column != row + 1; ++column)
				{
					auto value = matrix(row, column);
					for (int inner = 0; inner != column; ++inner)
						value -= lower(row, inner) * lower(column, inner);

					if (row == column)
					{
						if (value <= pivot_tolerance)
							throw std::invalid_argument(std::format("{} must be positive definite", name));

						lower(row, column) = std::sqrt(value);
					}
					else
					{
						lower(row, column) = value / lower(column, column);
					}
				}
			}
		}

		// Solve a tiny dense system using partial-pivot Gaussian elimination.
		std::vector<double> SolveLinear(DenseMatrix matrix, std::span<double const> rhs)
		{
			if (matrix.RowCount() == 0 || matrix.RowCount() != matrix.ColumnCount() || matrix.RowCount() != isize(rhs))
				throw std::invalid_argument("Dense linear system dimensions do not agree");

			ValidateFinite(matrix, "Dense linear system");
			ValidateFinite(rhs, "Dense linear right-hand side");
			auto solution = std::vector<double>(rhs.begin(), rhs.end());
			auto const dimension = matrix.RowCount();
			auto const scale = std::max(1.0, MaxAbs(matrix));
			auto const pivot_tolerance = 128.0 * std::numeric_limits<double>::epsilon() * scale * dimension;

			// Pivot each column before elimination so the oracle remains reliable for indefinite KKT matrices.
			for (int pivot = 0; pivot != dimension; ++pivot)
			{
				auto pivot_row = pivot;
				auto pivot_size = std::abs(matrix(pivot, pivot));
				for (int row = pivot + 1; row != dimension; ++row)
				{
					auto const candidate_size = std::abs(matrix(row, pivot));
					if (candidate_size <= pivot_size)
						continue;

					pivot_row = row;
					pivot_size = candidate_size;
				}
				if (pivot_size <= pivot_tolerance)
					throw std::runtime_error("Dense linear system is singular or numerically rank deficient");

				if (pivot_row != pivot)
				{
					for (int column = pivot; column != dimension; ++column)
						std::swap(matrix(pivot, column), matrix(pivot_row, column));
					std::swap(solution[pivot], solution[pivot_row]);
				}

				for (int row = pivot + 1; row != dimension; ++row)
				{
					auto const factor = matrix(row, pivot) / matrix(pivot, pivot);
					matrix(row, pivot) = 0.0;
					for (int column = pivot + 1; column != dimension; ++column)
						matrix(row, column) -= factor * matrix(pivot, column);
					solution[row] -= factor * solution[pivot];
				}
			}

			// Back substitution recovers the solution after the triangular reduction.
			for (int row = dimension; row-- != 0;)
			{
				for (int column = row + 1; column != dimension; ++column)
					solution[row] -= matrix(row, column) * solution[column];
				solution[row] /= matrix(row, row);
			}
			return solution;
		}

		// Multiply a dense matrix by a vector.
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

		// Return the infinity norm of a vector.
		double InfinityNorm(std::span<double const> vector)
		{
			auto norm = 0.0;
			for (auto value : vector)
				norm = std::max(norm, std::abs(value));

			return norm;
		}

		// Return the objective gradient W*lambda-rhs for an impulse-space quadratic program.
		std::vector<double> Gradient(DenseMatrix const& response, std::span<double const> rhs, std::span<double const> impulse)
		{
			auto gradient = Multiply(response, impulse);
			for (int index = 0; index != isize(gradient); ++index)
				gradient[index] -= rhs[index];

			return gradient;
		}

		// Return the scalar quadratic objective used to choose among numerically equivalent active sets.
		double Objective(DenseMatrix const& response, std::span<double const> rhs, std::span<double const> impulse)
		{
			auto response_impulse = Multiply(response, impulse);
			auto objective = 0.0;
			for (int index = 0; index != isize(impulse); ++index)
				objective += 0.5 * impulse[index] * response_impulse[index] - rhs[index] * impulse[index];

			return objective;
		}

		// Validate common dimensions and strict convexity for impulse-space reference solves.
		void ValidateQuadraticProblem(DenseMatrix const& response, std::span<double const> rhs, std::span<ImpulseBounds const> bounds)
		{
			ValidatePositiveDefinite(response, "Constraint response matrix");
			if (response.RowCount() != isize(rhs))
				throw std::invalid_argument("Constraint response and right-hand side dimensions do not agree");
			if (!bounds.empty() && response.RowCount() != isize(bounds))
				throw std::invalid_argument("Constraint response and bound dimensions do not agree");

			ValidateFinite(rhs, "Constraint right-hand side");
			for (auto const& bound : bounds)
				if (std::isnan(bound.m_lower) || std::isnan(bound.m_upper) || bound.m_lower > bound.m_upper)
					throw std::invalid_argument("Constraint impulse bounds are invalid");
		}

		// Return explicit unbounded limits when a caller omits scalar bounds.
		std::vector<ImpulseBounds> MaterialiseBounds(int count, std::span<ImpulseBounds const> bounds)
		{
			if (!bounds.empty())
				return std::vector<ImpulseBounds>(bounds.begin(), bounds.end());

			return std::vector<ImpulseBounds>(
				count,
				ImpulseBounds{
					.m_lower = -std::numeric_limits<double>::infinity(),
					.m_upper = +std::numeric_limits<double>::infinity(),
				});
		}

		enum class EBoundState
		{
			Free,
			Lower,
			Upper,
		};

		// Solve and test one complete bound active-set assignment.
		bool EvaluateActiveSet(
			DenseMatrix const& response,
			std::span<double const> rhs,
			std::span<ImpulseBounds const> bounds,
			std::span<EBoundState const> state,
			double tolerance,
			std::vector<double>& impulse,
			std::vector<double>& gradient)
		{
			auto free_indices = std::vector<int>{};
			auto fixed_indices = std::vector<int>{};
			impulse.assign(response.RowCount(), 0.0);
			for (int index = 0; index != response.RowCount(); ++index)
			{
				switch (state[index])
				{
					case EBoundState::Free:
					{
						free_indices.push_back(index);
						break;
					}
					case EBoundState::Lower:
					{
						if (!std::isfinite(bounds[index].m_lower))
							return false;

						fixed_indices.push_back(index);
						impulse[index] = bounds[index].m_lower;
						break;
					}
					case EBoundState::Upper:
					{
						if (!std::isfinite(bounds[index].m_upper))
							return false;

						fixed_indices.push_back(index);
						impulse[index] = bounds[index].m_upper;
						break;
					}
					default:
					{
						throw std::invalid_argument("Unknown bound active-set state");
					}
				}
			}

			// Free variables satisfy their stationarity equations after substituting all fixed variables.
			if (!free_indices.empty())
			{
				auto reduced_response = DenseMatrix(isize(free_indices), isize(free_indices));
				auto reduced_rhs = std::vector<double>(free_indices.size(), 0.0);
				for (int reduced_row = 0; reduced_row != isize(free_indices); ++reduced_row)
				{
					auto const row = free_indices[reduced_row];
					reduced_rhs[reduced_row] = rhs[row];
					for (auto fixed : fixed_indices)
						reduced_rhs[reduced_row] -= response(row, fixed) * impulse[fixed];
					for (int reduced_column = 0; reduced_column != isize(free_indices); ++reduced_column)
						reduced_response(reduced_row, reduced_column) = response(row, free_indices[reduced_column]);
				}

				auto const reduced_impulse = SolveLinear(std::move(reduced_response), reduced_rhs);
				for (int reduced_index = 0; reduced_index != isize(free_indices); ++reduced_index)
					impulse[free_indices[reduced_index]] = reduced_impulse[reduced_index];
			}

			// Primal bounds and complementary gradient signs are the KKT acceptance test for this assignment.
			gradient = Gradient(response, rhs, impulse);
			auto const scale = std::max({1.0, InfinityNorm(rhs), InfinityNorm(impulse), MaxAbs(response)});
			auto const threshold = tolerance * scale;
			for (int index = 0; index != response.RowCount(); ++index)
			{
				if (impulse[index] < bounds[index].m_lower - threshold || impulse[index] > bounds[index].m_upper + threshold)
					return false;

				switch (state[index])
				{
					case EBoundState::Free:
					{
						if (std::abs(gradient[index]) > threshold)
							return false;
						break;
					}
					case EBoundState::Lower:
					{
						if (gradient[index] < -threshold)
							return false;
						break;
					}
					case EBoundState::Upper:
					{
						if (gradient[index] > threshold)
							return false;
						break;
					}
					default:
					{
						throw std::invalid_argument("Unknown bound active-set state");
					}
				}
			}
			return true;
		}

		// Project a vector onto the Cartesian product of scalar intervals and disjoint friction cones.
		void ProjectFeasibleSet(std::vector<double>& impulse, std::span<ImpulseBounds const> bounds, std::span<FrictionCone const> cones)
		{
			for (int index = 0; index != isize(impulse); ++index)
				impulse[index] = std::clamp(impulse[index], bounds[index].m_lower, bounds[index].m_upper);

			for (auto const& cone : cones)
			{
				auto const projected = ProjectFrictionCone(
					{impulse[cone.m_normal], impulse[cone.m_tangent_0], impulse[cone.m_tangent_1]},
					cone.m_friction);
				impulse[cone.m_normal] = projected[0];
				impulse[cone.m_tangent_0] = projected[1];
				impulse[cone.m_tangent_1] = projected[2];
			}
		}

		// Validate that cone projections are independent factors of the feasible set.
		void ValidateCones(int dimension, std::span<ImpulseBounds const> bounds, std::span<FrictionCone const> cones)
		{
			auto claimed = std::vector<bool>(dimension, false);
			for (auto const& cone : cones)
			{
				if (cone.m_normal < 0 || cone.m_normal >= dimension ||
					cone.m_tangent_0 < 0 || cone.m_tangent_0 >= dimension ||
					cone.m_tangent_1 < 0 || cone.m_tangent_1 >= dimension)
					throw std::invalid_argument("Friction cone references an out-of-range impulse component");
				if (cone.m_normal == cone.m_tangent_0 || cone.m_normal == cone.m_tangent_1 || cone.m_tangent_0 == cone.m_tangent_1)
					throw std::invalid_argument("Friction cone impulse components must be distinct");
				if (!std::isfinite(cone.m_friction) || cone.m_friction < 0.0)
					throw std::invalid_argument("Friction coefficient must be finite and non-negative");

				for (auto index : {cone.m_normal, cone.m_tangent_0, cone.m_tangent_1})
				{
					if (claimed[index])
						throw std::invalid_argument("Friction cones must not share impulse components");
					if (std::isfinite(bounds[index].m_lower) || std::isfinite(bounds[index].m_upper))
						throw std::invalid_argument("Friction cone components cannot also have scalar bounds");

					claimed[index] = true;
				}
			}
		}
	}

	// Construct an empty matrix.
	DenseMatrix::DenseMatrix()
		: m_rows()
		, m_columns()
		, m_data()
	{
	}

	// Construct a zero-filled matrix with the requested dimensions.
	DenseMatrix::DenseMatrix(int rows, int columns)
		: m_rows(rows)
		, m_columns(columns)
		, m_data(CheckedElementCount(rows, columns), 0.0)
	{
	}

	// Construct a matrix from row-major values.
	DenseMatrix::DenseMatrix(int rows, int columns, std::initializer_list<double> values)
		: DenseMatrix(rows, columns)
	{
		if (isize(values) != rows * columns)
			throw std::invalid_argument("Dense matrix value count does not match its dimensions");

		std::copy(values.begin(), values.end(), m_data.begin());
	}

	// Return a mutable matrix element.
	double& DenseMatrix::operator()(int row, int column)
	{
		if (row < 0 || row >= m_rows || column < 0 || column >= m_columns)
			throw std::out_of_range("Dense matrix index is out of range");

		return m_data[static_cast<size_t>(row) * static_cast<size_t>(m_columns) + static_cast<size_t>(column)];
	}

	// Return a matrix element.
	double DenseMatrix::operator()(int row, int column) const
	{
		if (row < 0 || row >= m_rows || column < 0 || column >= m_columns)
			throw std::out_of_range("Dense matrix index is out of range");

		return m_data[static_cast<size_t>(row) * static_cast<size_t>(m_columns) + static_cast<size_t>(column)];
	}

	// Return the number of rows.
	int DenseMatrix::RowCount() const
	{
		return m_rows;
	}

	// Return the number of columns.
	int DenseMatrix::ColumnCount() const
	{
		return m_columns;
	}

	// Return a square identity matrix.
	DenseMatrix DenseMatrix::Identity(int dimension)
	{
		auto identity = DenseMatrix(dimension, dimension);
		for (int index = 0; index != dimension; ++index)
			identity(index, index) = 1.0;

		return identity;
	}

	// Solve the saddle-point system for minimum kinetic energy subject to exact velocity constraints.
	EqualitySolution SolveBilateralKkt(DenseMatrix const& mass, DenseMatrix const& jacobian, std::span<double const> momentum, std::span<double const> target)
	{
		ValidatePositiveDefinite(mass, "Generalized mass matrix");
		ValidateFinite(jacobian, "Constraint Jacobian");
		ValidateFinite(momentum, "Generalized momentum");
		ValidateFinite(target, "Constraint velocity target");
		if (mass.RowCount() != jacobian.ColumnCount() || mass.RowCount() != isize(momentum) || jacobian.RowCount() != isize(target))
			throw std::invalid_argument("Bilateral KKT dimensions do not agree");
		if (jacobian.RowCount() == 0)
			throw std::invalid_argument("Bilateral KKT solve requires at least one constraint");

		// The multiplier sign is chosen so J^T*lambda is the generalized constraint impulse.
		auto const velocity_count = mass.RowCount();
		auto const constraint_count = jacobian.RowCount();
		auto const dimension = velocity_count + constraint_count;
		auto kkt = DenseMatrix(dimension, dimension);
		auto kkt_rhs = std::vector<double>(dimension, 0.0);
		for (int row = 0; row != velocity_count; ++row)
		{
			kkt_rhs[row] = momentum[row];
			for (int column = 0; column != velocity_count; ++column)
				kkt(row, column) = mass(row, column);
			for (int constraint = 0; constraint != constraint_count; ++constraint)
				kkt(row, velocity_count + constraint) = -jacobian(constraint, row);
		}
		for (int constraint = 0; constraint != constraint_count; ++constraint)
		{
			kkt_rhs[velocity_count + constraint] = target[constraint];
			for (int column = 0; column != velocity_count; ++column)
				kkt(velocity_count + constraint, column) = jacobian(constraint, column);
		}

		// A rank-deficient Jacobian makes the ideal KKT multipliers non-unique and is rejected explicitly.
		auto const solution = SolveLinear(std::move(kkt), kkt_rhs);
		auto result = EqualitySolution{};
		result.m_velocity.assign(solution.begin(), solution.begin() + velocity_count);
		result.m_impulse.assign(solution.begin() + velocity_count, solution.end());

		auto const constrained_velocity = Multiply(jacobian, result.m_velocity);
		auto residual = 0.0;
		for (int constraint = 0; constraint != constraint_count; ++constraint)
			residual = std::max(residual, std::abs(constrained_velocity[constraint] - target[constraint]));
		result.m_residual = residual;
		return result;
	}

	// Solve a strictly convex impulse-space quadratic program by enumerating every feasible bound active set.
	ImpulseSolution SolveBoundedQp(DenseMatrix const& response, std::span<double const> rhs, std::span<ImpulseBounds const> bounds, double tolerance)
	{
		ValidateQuadraticProblem(response, rhs, bounds);
		if (!std::isfinite(tolerance) || tolerance <= 0.0)
			throw std::invalid_argument("Bounded QP tolerance must be finite and positive");
		if (response.RowCount() > 12)
			throw std::invalid_argument("Exact active-set oracle is limited to twelve impulses");

		auto const materialised_bounds = MaterialiseBounds(response.RowCount(), bounds);
		auto state = std::vector<EBoundState>(response.RowCount(), EBoundState::Free);
		auto best_impulse = std::vector<double>{};
		auto best_gradient = std::vector<double>{};
		auto best_objective = std::numeric_limits<double>::infinity();
		auto candidate_impulse = std::vector<double>{};
		auto candidate_gradient = std::vector<double>{};
		auto candidate_count = 0;

		// Base-three enumeration is intentionally exhaustive so the oracle never inherits production active-set heuristics.
		auto const assignment_count = [&]
		{
			auto count = uint64_t{1};
			for (int index = 0; index != response.RowCount(); ++index)
				count *= 3;
			return count;
		}();
		for (uint64_t assignment = 0; assignment != assignment_count; ++assignment)
		{
			auto encoded = assignment;
			for (int index = 0; index != response.RowCount(); ++index)
			{
				switch (encoded % 3)
				{
					case 0:
					{
						state[index] = EBoundState::Free;
						break;
					}
					case 1:
					{
						state[index] = EBoundState::Lower;
						break;
					}
					case 2:
					{
						state[index] = EBoundState::Upper;
						break;
					}
					default:
					{
						throw std::logic_error("Invalid base-three active-set digit");
					}
				}
				encoded /= 3;
			}

			++candidate_count;
			if (!EvaluateActiveSet(response, rhs, materialised_bounds, state, tolerance, candidate_impulse, candidate_gradient))
				continue;

			auto const objective = Objective(response, rhs, candidate_impulse);
			if (objective >= best_objective)
				continue;

			best_objective = objective;
			best_impulse = candidate_impulse;
			best_gradient = candidate_gradient;
		}
		if (best_impulse.empty())
			throw std::runtime_error("No KKT-feasible bound active set was found");

		// Report the largest complementary stationarity violation after the exact active-set selection.
		auto optimality_residual = 0.0;
		for (int index = 0; index != response.RowCount(); ++index)
		{
			auto violation = std::abs(best_gradient[index]);
			if (best_impulse[index] <= materialised_bounds[index].m_lower + tolerance)
				violation = std::max(0.0, -best_gradient[index]);
			if (best_impulse[index] >= materialised_bounds[index].m_upper - tolerance)
				violation = std::max(0.0, +best_gradient[index]);
			optimality_residual = std::max(optimality_residual, violation);
		}
		return ImpulseSolution{
			.m_impulse = std::move(best_impulse),
			.m_gradient = std::move(best_gradient),
			.m_optimality_residual = optimality_residual,
			.m_iterations = candidate_count,
		};
	}

	// Solve a strictly convex impulse-space quadratic program over disjoint Coulomb cones and scalar bounds.
	ImpulseSolution SolveFrictionQp(
		DenseMatrix const& response,
		std::span<double const> rhs,
		std::span<ImpulseBounds const> bounds,
		std::span<FrictionCone const> cones,
		double tolerance,
		int max_iterations)
	{
		ValidateQuadraticProblem(response, rhs, bounds);
		if (!std::isfinite(tolerance) || tolerance <= 0.0)
			throw std::invalid_argument("Friction QP tolerance must be finite and positive");
		if (max_iterations <= 0)
			throw std::invalid_argument("Friction QP iteration limit must be positive");

		auto const materialised_bounds = MaterialiseBounds(response.RowCount(), bounds);
		ValidateCones(response.RowCount(), materialised_bounds, cones);

		// A Gershgorin upper bound is a conservative Lipschitz constant for the quadratic gradient.
		auto lipschitz = 0.0;
		for (int row = 0; row != response.RowCount(); ++row)
		{
			auto row_sum = 0.0;
			for (int column = 0; column != response.ColumnCount(); ++column)
				row_sum += std::abs(response(row, column));
			lipschitz = std::max(lipschitz, row_sum);
		}
		auto const step = 1.0 / lipschitz;
		auto impulse = std::vector<double>(response.RowCount(), 0.0);
		ProjectFeasibleSet(impulse, materialised_bounds, cones);

		// Projected gradient converges independently of production PGS ordering and uses exact product-set projections.
		for (int iteration = 1; iteration != max_iterations + 1; ++iteration)
		{
			auto const gradient = Gradient(response, rhs, impulse);
			auto next_impulse = impulse;
			for (int index = 0; index != isize(next_impulse); ++index)
				next_impulse[index] -= step * gradient[index];
			ProjectFeasibleSet(next_impulse, materialised_bounds, cones);

			auto projected_gradient_residual = 0.0;
			for (int index = 0; index != isize(impulse); ++index)
				projected_gradient_residual = std::max(projected_gradient_residual, std::abs(next_impulse[index] - impulse[index]) / step);

			auto const scale = std::max({1.0, InfinityNorm(rhs), InfinityNorm(impulse)});
			impulse = std::move(next_impulse);
			if (projected_gradient_residual > tolerance * scale)
				continue;

			return ImpulseSolution{
				.m_impulse = impulse,
				.m_gradient = Gradient(response, rhs, impulse),
				.m_optimality_residual = projected_gradient_residual,
				.m_iterations = iteration,
			};
		}
		throw std::runtime_error("Friction cone oracle did not converge within its iteration limit");
	}

	// Project one normal/tangent triplet onto an exact circular Coulomb cone in Euclidean impulse space.
	std::array<double, 3> ProjectFrictionCone(std::array<double, 3> impulse, double friction)
	{
		if (!std::isfinite(friction) || friction < 0.0)
			throw std::invalid_argument("Friction coefficient must be finite and non-negative");
		for (auto component : impulse)
			if (!std::isfinite(component))
				throw std::invalid_argument("Friction impulse must be finite");

		auto const tangent_length = std::hypot(impulse[1], impulse[2]);
		if (impulse[0] >= 0.0 && tangent_length <= friction * impulse[0])
			return impulse;
		if (impulse[0] + friction * tangent_length <= 0.0)
			return {0.0, 0.0, 0.0};

		// The closest boundary point has the input tangent direction and an analytically minimized normal magnitude.
		auto const normal = (impulse[0] + friction * tangent_length) / (1.0 + friction * friction);
		auto const tangent_scale = tangent_length != 0.0 ? friction * normal / tangent_length : 0.0;
		return {normal, impulse[1] * tangent_scale, impulse[2] * tangent_scale};
	}
}
