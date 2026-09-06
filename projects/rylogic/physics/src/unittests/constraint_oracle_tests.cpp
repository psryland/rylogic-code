//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "src/unittests/constraint_oracle.h"

namespace pr::physics::tests
{
	namespace
	{
		using namespace constraint_oracle;

		// A deterministic generator keeps randomized oracle coverage exactly reproducible.
		struct RandomSource
		{
			uint64_t m_state;

			// Return a deterministic value in [-1,+1).
			double NextSigned()
			{
				m_state ^= m_state << 13;
				m_state ^= m_state >> 7;
				m_state ^= m_state << 17;
				auto const unit = static_cast<double>(m_state >> 11) * (1.0 / 9007199254740992.0);
				return 2.0 * unit - 1.0;
			}
		};

		// Require a scalar result to agree with its double-precision expectation.
		void ExpectNear(double actual, double expected, double tolerance)
		{
			PR_EXPECT(std::abs(actual - expected) <= tolerance);
		}

		// Require two vectors to agree component by component.
		void ExpectNear(std::span<double const> actual, std::span<double const> expected, double tolerance)
		{
			PR_EXPECT(actual.size() == expected.size());
			for (int index = 0; index != isize(actual); ++index)
				ExpectNear(actual[index], expected[index], tolerance);
		}

		// Multiply a test matrix by a vector without relying on oracle implementation helpers.
		std::vector<double> TestMultiply(DenseMatrix const& matrix, std::span<double const> vector)
		{
			auto result = std::vector<double>(matrix.RowCount(), 0.0);
			for (int row = 0; row != matrix.RowCount(); ++row)
				for (int column = 0; column != matrix.ColumnCount(); ++column)
					result[row] += matrix(row, column) * vector[column];

			return result;
		}

		// Multiply a transposed test matrix by a vector without relying on oracle implementation helpers.
		std::vector<double> TestTransposeMultiply(DenseMatrix const& matrix, std::span<double const> vector)
		{
			auto result = std::vector<double>(matrix.ColumnCount(), 0.0);
			for (int row = 0; row != matrix.RowCount(); ++row)
				for (int column = 0; column != matrix.ColumnCount(); ++column)
					result[column] += matrix(row, column) * vector[row];

			return result;
		}

		// Build a well-conditioned symmetric positive-definite matrix from a deterministic dense factor.
		DenseMatrix RandomPositiveDefinite(int dimension, RandomSource& random)
		{
			auto factor = DenseMatrix(dimension, dimension);
			for (int row = 0; row != dimension; ++row)
				for (int column = 0; column != dimension; ++column)
					factor(row, column) = 0.35 * random.NextSigned();

			auto matrix = DenseMatrix(dimension, dimension);
			for (int row = 0; row != dimension; ++row)
			{
				for (int column = 0; column != dimension; ++column)
				for (int inner = 0; inner != dimension; ++inner)
					matrix(row, column) += factor(inner, row) * factor(inner, column);
				matrix(row, row) += 1.0;
			}
			return matrix;
		}

		// Return unbounded scalar limits for cone-only test problems.
		std::vector<ImpulseBounds> Unbounded(int dimension)
		{
			return std::vector<ImpulseBounds>(
				dimension,
				ImpulseBounds{
					.m_lower = -std::numeric_limits<double>::infinity(),
					.m_upper = +std::numeric_limits<double>::infinity(),
				});
		}
	}

	PRUnitTestClass(ConstraintOracleTests)
	{
		// Prove the KKT sign convention and exact velocity enforcement with a closed-form two-body collision.
		PRUnitTestMethod(BilateralKktMatchesClosedForm, Quick)
		{
			auto const mass = DenseMatrix(2, 2, {
				2.0, 0.0,
				0.0, 3.0,
			});
			auto const jacobian = DenseMatrix(1, 2, {-1.0, +1.0});
			auto const momentum = std::array{4.0, -3.0};
			auto const target = std::array{0.0};

			auto const solution = SolveBilateralKkt(mass, jacobian, momentum, target);

			ExpectNear(solution.m_velocity, std::array{0.2, 0.2}, 1.0e-14);
			ExpectNear(solution.m_impulse, std::array{3.6}, 1.0e-14);
			ExpectNear(solution.m_residual, 0.0, 1.0e-14);
		}

		// Recover planted velocities and impulses from randomized full-rank KKT systems.
		PRUnitTestMethod(BilateralKktRecoversRandomSolutions, Quick)
		{
			auto random = RandomSource{0x9E3779B97F4A7C15ull};
			for (int trial = 0; trial != 64; ++trial)
			{
				auto const velocity_count = 2 + trial % 7;
				auto const constraint_count = 1 + trial % velocity_count;
				auto const mass = RandomPositiveDefinite(velocity_count, random);
				auto jacobian = DenseMatrix(constraint_count, velocity_count);
				for (int row = 0; row != constraint_count; ++row)
				{
					for (int column = 0; column != velocity_count; ++column)
						jacobian(row, column) = 0.15 * random.NextSigned();
					jacobian(row, row) += 1.0;
				}

				auto expected_velocity = std::vector<double>(velocity_count);
				auto expected_impulse = std::vector<double>(constraint_count);
				for (auto& value : expected_velocity)
					value = random.NextSigned();
				for (auto& value : expected_impulse)
					value = random.NextSigned();

				auto momentum = TestMultiply(mass, expected_velocity);
				auto const jacobian_impulse = TestTransposeMultiply(jacobian, expected_impulse);
				for (int index = 0; index != velocity_count; ++index)
					momentum[index] -= jacobian_impulse[index];
				auto const target = TestMultiply(jacobian, expected_velocity);

				auto const solution = SolveBilateralKkt(mass, jacobian, momentum, target);

				ExpectNear(solution.m_velocity, expected_velocity, 2.0e-12);
				ExpectNear(solution.m_impulse, expected_impulse, 2.0e-12);
				PR_EXPECT(solution.m_residual <= 2.0e-12);
			}
		}

		// Select the analytically correct unilateral active set in a coupled two-row problem.
		PRUnitTestMethod(BoundedQpMatchesClosedForm, Quick)
		{
			auto const response = DenseMatrix(2, 2, {
				2.0, 1.0,
				1.0, 2.0,
			});
			auto const rhs = std::array{3.0, -1.0};
			auto const bounds = std::array{
				ImpulseBounds{.m_lower = 0.0, .m_upper = +std::numeric_limits<double>::infinity()},
				ImpulseBounds{.m_lower = 0.0, .m_upper = +std::numeric_limits<double>::infinity()},
			};

			auto const solution = SolveBoundedQp(response, rhs, bounds);

			ExpectNear(solution.m_impulse, std::array{1.5, 0.0}, 1.0e-14);
			ExpectNear(solution.m_gradient, std::array{0.0, 2.5}, 1.0e-14);
			ExpectNear(solution.m_optimality_residual, 0.0, 1.0e-14);
		}

		// Recover planted free, lower-active, and upper-active solutions from randomized convex systems.
		PRUnitTestMethod(BoundedQpRecoversRandomKktSolutions, Quick)
		{
			auto random = RandomSource{0xD1B54A32D192ED03ull};
			for (int trial = 0; trial != 40; ++trial)
			{
				auto const dimension = 1 + trial % 7;
				auto const response = RandomPositiveDefinite(dimension, random);
				auto const bounds = std::vector<ImpulseBounds>(
					dimension,
					ImpulseBounds{.m_lower = -1.0, .m_upper = +1.0});
				auto expected_impulse = std::vector<double>(dimension);
				auto expected_gradient = std::vector<double>(dimension);
				for (int index = 0; index != dimension; ++index)
				{
					switch ((index + trial) % 3)
					{
						case 0:
						{
							expected_impulse[index] = 0.5 * random.NextSigned();
							expected_gradient[index] = 0.0;
							break;
						}
						case 1:
						{
							expected_impulse[index] = bounds[index].m_lower;
							expected_gradient[index] = 0.25 + std::abs(random.NextSigned());
							break;
						}
						case 2:
						{
							expected_impulse[index] = bounds[index].m_upper;
							expected_gradient[index] = -0.25 - std::abs(random.NextSigned());
							break;
						}
						default:
						{
							throw std::logic_error("Invalid randomized active-set state");
						}
					}
				}

				auto rhs = TestMultiply(response, expected_impulse);
				for (int index = 0; index != dimension; ++index)
					rhs[index] -= expected_gradient[index];

				auto const solution = SolveBoundedQp(response, rhs, bounds);

				ExpectNear(solution.m_impulse, expected_impulse, 2.0e-11);
				ExpectNear(solution.m_gradient, expected_gradient, 2.0e-11);
				PR_EXPECT(solution.m_optimality_residual <= 2.0e-11);
			}
		}

		// Match all three analytic cone-projection regions: interior, apex, and sliding boundary.
		PRUnitTestMethod(FrictionQpMatchesAnalyticProjection, Quick)
		{
			auto const response = DenseMatrix::Identity(3);
			auto const bounds = Unbounded(3);
			auto const cones = std::array{
				FrictionCone{
					.m_normal = 0,
					.m_tangent_0 = 1,
					.m_tangent_1 = 2,
					.m_friction = 0.5,
				},
			};

			auto const inside = SolveFrictionQp(response, std::array{2.0, 0.5, 0.25}, bounds, cones);
			auto const apex = SolveFrictionQp(response, std::array{-2.0, 0.5, 0.0}, bounds, cones);
			auto const sliding = SolveFrictionQp(response, std::array{0.5, 2.0, 0.0}, bounds, cones);

			ExpectNear(inside.m_impulse, std::array{2.0, 0.5, 0.25}, 1.0e-14);
			ExpectNear(apex.m_impulse, std::array{0.0, 0.0, 0.0}, 1.0e-14);
			ExpectNear(sliding.m_impulse, std::array{1.2, 0.6, 0.0}, 1.0e-14);
			PR_EXPECT(inside.m_optimality_residual <= 1.0e-13);
			PR_EXPECT(apex.m_optimality_residual <= 1.0e-13);
			PR_EXPECT(sliding.m_optimality_residual <= 1.0e-13);
		}

		// Converge to a planted interior solution with two friction cones coupled by a dense response matrix.
		PRUnitTestMethod(FrictionQpSolvesCoupledCones, Quick)
		{
			auto random = RandomSource{0x94D049BB133111EBull};
			auto const response = RandomPositiveDefinite(6, random);
			auto const bounds = Unbounded(6);
			auto const cones = std::array{
				FrictionCone{.m_normal = 0, .m_tangent_0 = 1, .m_tangent_1 = 2, .m_friction = 0.8},
				FrictionCone{.m_normal = 3, .m_tangent_0 = 4, .m_tangent_1 = 5, .m_friction = 0.6},
			};
			auto const expected_impulse = std::array{
				2.0, 0.2, -0.1,
				1.5, -0.15, 0.1,
			};
			auto const rhs = TestMultiply(response, expected_impulse);

			auto const solution = SolveFrictionQp(response, rhs, bounds, cones);

			ExpectNear(solution.m_impulse, expected_impulse, 2.0e-11);
			PR_EXPECT(solution.m_optimality_residual <= 2.0e-11);
			PR_EXPECT(solution.m_iterations < 1000);
		}

		// Reject rank-deficient or ambiguously constrained reference problems instead of returning plausible garbage.
		PRUnitTestMethod(InvalidProblemsAreRejected, Quick)
		{
			auto const singular_mass = DenseMatrix(2, 2, {
				1.0, 0.0,
				0.0, 0.0,
			});
			auto const identity = DenseMatrix::Identity(3);
			auto const dependent_jacobian = DenseMatrix(2, 3, {
				1.0, 0.0, 0.0,
				1.0, 0.0, 0.0,
			});
			auto const invalid_bounds = std::array{
				ImpulseBounds{.m_lower = 1.0, .m_upper = -1.0},
				ImpulseBounds{.m_lower = 0.0, .m_upper = 1.0},
				ImpulseBounds{.m_lower = 0.0, .m_upper = 1.0},
			};
			auto const unbounded = Unbounded(3);
			auto const overlapping_cones = std::array{
				FrictionCone{.m_normal = 0, .m_tangent_0 = 1, .m_tangent_1 = 2, .m_friction = 0.5},
				FrictionCone{.m_normal = 0, .m_tangent_0 = 1, .m_tangent_1 = 2, .m_friction = 0.5},
			};

			PR_THROWS(SolveBilateralKkt(singular_mass, DenseMatrix(1, 2, {1.0, 0.0}), std::array{0.0, 0.0}, std::array{0.0}), std::exception);
			PR_THROWS(SolveBilateralKkt(identity, dependent_jacobian, std::array{0.0, 0.0, 0.0}, std::array{0.0, 0.0}), std::exception);
			PR_THROWS(SolveBoundedQp(identity, std::array{0.0, 0.0, 0.0}, invalid_bounds), std::exception);
			PR_THROWS(SolveFrictionQp(identity, std::array{0.0, 0.0, 0.0}, unbounded, overlapping_cones), std::exception);
		}
	};
}
#endif
