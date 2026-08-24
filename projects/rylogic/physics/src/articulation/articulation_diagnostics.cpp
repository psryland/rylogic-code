//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/articulation/articulation.h"
#include "src/articulation/articulation_internal.h"

namespace pr::physics
{
	// Return the most recently solved link-frame spatial acceleration.
	v8motion Articulation::LinkAcceleration(LinkHandle link) const
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");

		auto const& state = *m_state;
		return detail::CheckedLink(state, link).m_link_acceleration;
	}

	// Return the articulation's total world-space momentum about the world origin.
	v8force Articulation::MomentumWS() const
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");

		auto& state = *m_state;
		if (state.m_kinematics_dirty)
			const_cast<Articulation*>(this)->UpdateKinematics();

		// Transform every link momentum as a spatial force so translation contributes its orbital angular momentum.
		auto momentum = v8force{};
		for (auto const& link : state.m_links)
		{
			auto const link_momentum = link.m_link.m_inertia * link.m_link_velocity;
			momentum += link.m_link_to_world * link_momentum;
		}
		return momentum;
	}

	// Return the sum of the physical links' kinetic energies.
	float Articulation::KineticEnergy() const
	{
		if (!m_state)
			throw std::logic_error("Articulation has been moved from");

		auto& state = *m_state;
		if (state.m_kinematics_dirty)
			const_cast<Articulation*>(this)->UpdateKinematics();

		// Spatial inertia includes link mass and centre-of-mass offset, so each dot product is measured about the link origin.
		auto energy = 0.0f;
		for (auto const& link : state.m_links)
			energy += 0.5f * Dot(link.m_link_velocity, link.m_link.m_inertia * link.m_link_velocity);

		return energy;
	}
}
