//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_SUPPORT_H
#define PR_PHYSICS_SUPPORT_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		namespace support
		{
			struct Leg
			{
				v4  m_point;			// The 2d point around the CoM
				int m_support_number;	// The support number this is
				int	m_count;			// A tally of the number of times we've seen this support
				Leg *m_next, *m_prev;	// Used for chaining these objects together
			};
		}

		// Supports for sleeping objects.
		// Each rb is chained to the objects it is supporting. Every object can be supported by
		// up to three objects. Therefore, each object has 3 chain links that can be part of 
		// 3 different chains. The length of the chains is the number of objects that rest on a particular
		// object.
		struct Support
		{
			support::Leg	m_on_me;			// The head of a chain of objects resting on 'me'
			support::Leg	m_leg[3];			// Up to three objects that we are resting on.
			uint32_t		m_num_supports;		// The number of objects we're resting on.
			mutable int		m_active;			// A down counter used to detect streams of micro collisions
			bool			m_supported;		// True if we think the object is supported

			enum { DecayTime = 5, RepeatCount = 2 };
			void Construct();
			
			// Called when this support is no longer providing support. Wakes up the objects that are resting on 'rb'
			void Clear();

			// Add a support point. Attempt to add 'point' as a support of the object that owns this struct
			void Add(Rigidbody& on_obj, v4 gravity, v4 point);

			// Returns true if this struct appears to be supported
			bool IsSupported() const			{ m_active -= (m_active > 1); return m_supported; }
		};

		// Consider 'contact' to see if the collision is a micro collision that would occur
		// if the objects where settling on a support.
		void LookForSupports(Contact const& contact, Rigidbody& objectA, Rigidbody& objectB);

		// Return the rigidbody for a support
		Rigidbody const& GetRBFromSupport(Support const& support);
		Rigidbody&		 GetRBFromSupport(Support& support);
		
		// Return the support that contains 'leg'
		Support const&	GetSupportFromLeg(support::Leg const& leg);
		Support&		GetSupportFromLeg(support::Leg& leg);

	}
}

#endif

