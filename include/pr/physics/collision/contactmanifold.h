//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PH_CONTACT_MANIFOLD_AGENT_H
#define PR_PH_CONTACT_MANIFOLD_AGENT_H

#include "pr/physics/types/forward.h"
#include "pr/physics/collision/contact.h"

namespace pr
{
	namespace ph
	{
		// This struct is used to represent the area of contact
		// If 'm_type' == EType_Point, m_lower = m_upper = the point,
		// If 'm_type' == EType_Line, line from m_lower to m_upper,
		// If 'm_type' == EType_Area, bounding box from m_lower to m_upper
		struct Manifold
		{
			enum EType { EType_Point = 0, EType_Line = 1, EType_Area = 2};
			EType	m_type;
			v4		m_lower;
			v4		m_upper;
		};

		// This object is used to collect contact points during collision detection
		// Contacts are collected, magically processed and returned as a single contact point
		// This abstraction is so that one day I might do something better than just find the
		// deepest penetration.
		class ContactManifold
		{
			enum { MaxContacts = 10 };
			Contact m_contact[MaxContacts];	// The points of contact
			uint32_t m_num_contacts;			// The number of contacts in 'm_contact'
			bool m_flip;					// True if contacts should be flipped before adding them to the manifold

		public:
			ContactManifold()
			{
				Reset();
			}
			void Reset()
			{
				m_num_contacts = 0;
				m_flip = false;
			}			
			void Add(Contact const& contact)
			{
				if( contact.m_depth <= 0.0f ) return;
				if( m_num_contacts == MaxContacts ) return;
				m_contact[m_num_contacts] = contact;
				if( m_flip ) m_contact[m_num_contacts].FlipResults();
				++m_num_contacts;
			}
			void Flip()
			{
				m_flip = !m_flip;
			}
			bool IsOverlap() const
			{
				return m_num_contacts != 0;
			}
			v4 ContactCentre() const
			{
				v4 pos = v4::Zero();
				for( uint32_t i = 0; i != m_num_contacts; ++i ) pos += m_contact[i].m_pointA;
				pos /= float(m_num_contacts);
				pos.w = 1.0f;
				return pos;
			}
			uint32_t Size() const
			{
				return m_num_contacts;
			}
			Contact const& operator[](uint32_t i) const
			{
				return m_contact[i];
			}
			Contact& operator[](uint32_t i)
			{
				return m_contact[i];
			}
		};
	}
}

#endif
