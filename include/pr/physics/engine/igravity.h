//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_IGRAVITY_H
#define PR_PHYSICS_IGRAVITY_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		struct IGravity
		{
			virtual ~IGravity() {}

			// This method should return the 'acceleration' due to gravity at 'position'
			virtual v4 GravityField(v4 position) const = 0;

			// This method returns the potential energy of 'position' in the gravity field
			virtual float GravityPotential(v4 position) const = 0;
		};

		// Assign the gravity field interface to use.
		// This must remain in scope for the lifetime of the physics engine
		void RegisterGravityField(ph::IGravity const* gravity_interface);
		
		// Return the gravitational acceleration experienced at 'position'
		v4 GetGravitationalAcceleration(v4 position);

		// Returns the potential energy of a position in the gravity field
		float GetGravitationalPotential(v4 position);

		// Some default helper implementations
		struct NoGravity : IGravity
		{
			v4    GravityField    (v4) const { return v4::Zero(); }
			float GravityPotential(v4) const { return 0.0f; }
		};
		struct DirectionalGravity : IGravity
		{
			v4 m_grav;
			DirectionalGravity(v4 grav = v4(0.0f, -9.8f, 0.0f, 0.0f)) : m_grav(grav) {}
			v4    GravityField    (v4) const		{ return m_grav; }
			float GravityPotential(v4 pos) const { return -Dot3(m_grav, pos); }
		};
		struct InverseSqrGravity : IGravity
		{
			v4 m_centre;
			float m_strength;
			float m_min_dist;
			InverseSqrGravity(v4 centre, float strength, float min_dist) : m_centre(centre), m_strength(strength), m_min_dist(min_dist) {}
			v4 GravityField(v4 position) const
			{
				auto diff = m_centre - position;
				auto r = Max(Length(diff), m_min_dist);
				return diff * (m_strength / (r*r*r)); // first 'r' is to normalise 'diff', then it's r^2
			}
			float GravityPotential(v4 pos) const
			{
				return Length(GravityField(pos));
			}
		};
	}
}

#endif
