//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_CONTACT_H
#define PR_PHYSICS_CONTACT_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		struct Contact
		{
			v4						m_pointA;			// The world space contact point on objectA (see note)
			v4						m_pointB;			// The world space contact point on objectB (see note)
			v4						m_normal;			// The contact normal in world space (from objectB to objectA, or the direction objectA needs to move to stop penetration, or pointing away from objectB)
			float					m_depth;			// The depth of penetration. > 0.0f indicates contact
			uint32_t				m_material_idA;		// The material id for pointA
			uint32_t				m_material_idB;		// The material id for pointB

			Contact() : m_depth(-limits<float>::max())	{}
			void FlipResults()
			{
				std::swap(m_pointA, m_pointB);
				m_normal *= -1.0f;
				std::swap(m_material_idA, m_material_idB);
			}

			//// This is not needed with the new solver system
			//void Clear()							{ memset(this, 0, sizeof(*this)); }
			//bool IsIncreasingPenetration() const	{ return m_rel_norm_speed > 0.0f; }
			//void CalculateExtraContactInfo(Rigidbody const& objA, Rigidbody const& objB);

			//// Post-detection data
			//Rigidbody const*		m_objectA;			// objectA
			//Rigidbody const*		m_objectB;			// objectB
			//v4						m_relative_velocity;// The velocity of pointB from pointA's perspective
			//v4						m_tangent;			// The tangent to the point of contact in world space in the direction of the tangential relative velocity.
			//float					m_resting_speed;
			//float					m_rel_norm_speed;	// The magnitude of the normal component of the relative velocity
			//float					m_rel_tang_speed;	// The magnitude of the tangential component of the relative velocity
			//v4						m_ws_impulse;		// The world space impulse applied to object A (-ve to object B)
			//float					m_static_friction;	// The static friction used to resolve the collision
			//float					m_dynamic_friction;	// The dynamic friction used to resolve the collision
			//float					m_rolling_friction;	// The rolling friction used to resolve the collision
			//float					m_elasticity_n;		// The normal component of the elasticity used to resolve the collision
			//float					m_elasticity_t;		// The tangential component of the elasticity used to resolve the collision
			//float					m_elasticity_tor;	// The tortional component of the elasticity used to resolve the collision
		};

		// Note: about contact points
		//	m_pointA and m_pointB are in absolute world space. They were relative to
		//	m_objectA and m_objectB but doing this makes collision detection between
		//	composite types more difficult (e.g. Array vs ???)

	}
}

#endif
