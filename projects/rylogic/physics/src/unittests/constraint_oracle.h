//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics::tests::constraint_oracle
{
	// A small row-major matrix used only by the independent double-precision constraint oracle.
	struct DenseMatrix
	{
		int m_rows;
		int m_columns;
		std::vector<double> m_data;

		// Construct an empty matrix.
		DenseMatrix();

		// Construct a zero-filled matrix with the requested dimensions.
		DenseMatrix(int rows, int columns);

		// Construct a matrix from row-major values.
		DenseMatrix(int rows, int columns, std::initializer_list<double> values);

		// Return a mutable matrix element.
		double& operator()(int row, int column);

		// Return a matrix element.
		double operator()(int row, int column) const;

		// Return the number of rows.
		int RowCount() const;

		// Return the number of columns.
		int ColumnCount() const;

		// Return a square identity matrix.
		static DenseMatrix Identity(int dimension);
	};

	// The dense equality solve result contains generalized velocities and the impulses that enforce the target constraint velocities.
	struct EqualitySolution
	{
		std::vector<double> m_velocity;
		std::vector<double> m_impulse;
		double m_residual;
	};

	// Independent lower and upper limits for one scalar impulse.
	struct ImpulseBounds
	{
		double m_lower;
		double m_upper;
	};

	// A three-dimensional Coulomb cone containing one normal and two tangent impulse components.
	struct FrictionCone
	{
		int m_normal;
		int m_tangent_0;
		int m_tangent_1;
		double m_friction;
	};

	// The dense inequality solve result includes the objective gradient and a first-order optimality residual.
	struct ImpulseSolution
	{
		std::vector<double> m_impulse;
		std::vector<double> m_gradient;
		double m_optimality_residual;
		int m_iterations;
	};

	// Solve the saddle-point system for minimum kinetic energy subject to exact velocity constraints.
	EqualitySolution SolveBilateralKkt(DenseMatrix const& mass, DenseMatrix const& jacobian, std::span<double const> momentum, std::span<double const> target);

	// Solve a strictly convex impulse-space quadratic program by enumerating every feasible bound active set.
	ImpulseSolution SolveBoundedQp(DenseMatrix const& response, std::span<double const> rhs, std::span<ImpulseBounds const> bounds, double tolerance = 1.0e-12);

	// Solve a strictly convex impulse-space quadratic program over disjoint Coulomb cones and scalar bounds.
	ImpulseSolution SolveFrictionQp(
		DenseMatrix const& response,
		std::span<double const> rhs,
		std::span<ImpulseBounds const> bounds,
		std::span<FrictionCone const> cones,
		double tolerance = 1.0e-12,
		int max_iterations = 100000);

	// Project one normal/tangent triplet onto an exact circular Coulomb cone in Euclidean impulse space.
	std::array<double, 3> ProjectFrictionCone(std::array<double, 3> impulse, double friction);
}
