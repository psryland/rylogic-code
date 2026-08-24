//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/articulation/articulation.h"
#include "src/unittests/constraint_oracle.h"

namespace pr::physics::tests::articulation_oracle
{
	// Dense generalized acceleration and the independently assembled equation H*qdd + h = tau.
	struct DynamicsSolution
	{
		constraint_oracle::DenseMatrix m_mass;
		std::vector<double> m_bias;
		std::vector<double> m_acceleration;
		std::vector<std::array<double, 6>> m_link_acceleration;
	};

	// One link-frame wrench contribution to an articulation constraint Jacobian row.
	struct ConstraintJacobianTerm
	{
		LinkHandle m_link = {};
		std::array<double, 6> m_wrench = {};
	};

	// One scalar constraint row containing at most two endpoints on the same articulation.
	struct ConstraintJacobianRow
	{
		std::array<ConstraintJacobianTerm, 2> m_terms = {};
		int m_term_count = 0;
	};

	// Dense generalized mass, link Jacobian, and exact Delassus response for a tiny constraint system.
	struct ConstraintSystem
	{
		constraint_oracle::DenseMatrix m_mass;
		constraint_oracle::DenseMatrix m_jacobian;
		constraint_oracle::DenseMatrix m_response;
		std::vector<constraint_oracle::DenseMatrix> m_link_response;
		std::vector<constraint_oracle::DenseMatrix> m_recursive_link_response;
		int64_t m_recursive_work;
	};

	// Solve forward dynamics through double-precision recursive Newton-Euler probes and a dense pivoted solve.
	DynamicsSolution SolveForwardDynamics(Articulation const& articulation);

	// Solve the generalized velocity change caused by simultaneous link-frame impulses.
	std::vector<double> SolveImpulseResponse(Articulation const& articulation, std::span<ArticulationImpulse const> impulses);

	// Form J and A=J*H^-1*J-transpose independently in double precision for a tiny articulation constraint system.
	ConstraintSystem BuildConstraintSystem(Articulation const& articulation, std::span<ConstraintJacobianRow const> rows);
}
