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
	};

	// Solve forward dynamics through double-precision recursive Newton-Euler probes and a dense pivoted solve.
	DynamicsSolution SolveForwardDynamics(Articulation const& articulation);

	// Solve the generalized velocity change caused by simultaneous link-frame impulses.
	std::vector<double> SolveImpulseResponse(Articulation const& articulation, std::span<ArticulationImpulse const> impulses);
}
