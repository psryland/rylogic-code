//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/rigidbody/support.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/collision/contact.h"
#include "physics/utility/ldrhelper_private.h"
#include "physics/utility/profile.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;
using namespace pr::ph::support;
using namespace pr::geometry;

// Return the rigid body that contains 'support'
inline Rigidbody& GetRB(Support& support)
{
	Rigidbody& rb = *reinterpret_cast<Rigidbody*>(reinterpret_cast<uint8_t*>(&support) - offsetof(Rigidbody, m_support));
	PR_ASSERT(PR_DBG_PHYSICS, &rb.m_support == &support, "");
	return rb; 
}

// Return the support that contains 'leg'
inline Support& GetSupport(Leg& leg)
{
	Support& support = *reinterpret_cast<Support*>(&leg - leg.m_support_number - 1);
	PR_ASSERT(PR_DBG_PHYSICS, &support.m_leg[leg.m_support_number] == &leg, "");
	return support;
}

// Return the rigid body for a support
Rigidbody const& pr::ph::GetRBFromSupport(Support const& support)	{ return GetRB(const_cast<Support&>(support)); }
Rigidbody&		 pr::ph::GetRBFromSupport(Support&		 support)	{ return GetRB(support); }

// Return the support that contains 'leg'
Support const&	pr::ph::GetSupportFromLeg(Leg const& leg)			{ return GetSupport(const_cast<Leg&>(leg)); }
Support&		pr::ph::GetSupportFromLeg(Leg&		 leg)			{ return GetSupport(leg); }

// 'Construct' the support structure
void Support::Construct()
{
	m_active = 0;
	m_supported = false;
	m_num_supports = 0;
	pr::chain::Init(m_on_me);
	pr::chain::Init(m_leg[0]); m_leg[0].m_support_number = 0;
	pr::chain::Init(m_leg[1]); m_leg[1].m_support_number = 1;
	pr::chain::Init(m_leg[2]); m_leg[2].m_support_number = 2;

	// Mark the 'on_me' support so we can find it in the chain
	PR_EXPAND(PR_DBG_PHYSICS, m_on_me.m_support_number = -1;)
}

// Called when this support is no longer providing support. Wakes up the objects that are resting on us
void Support::Clear()
{
	pr::chain::Remove(m_leg[0]);
	pr::chain::Remove(m_leg[1]);
	pr::chain::Remove(m_leg[2]);
	m_num_supports = 0;
	m_supported = false;
	m_active = 0;
	
	// Every object that is resting on me needs waking up.
	while (!pr::chain::Empty(m_on_me))
	{
		Rigidbody& obj = GetRB(GetSupport(m_on_me));
		PR_ASSERT(PR_DBG_PHYSICS, &obj != &GetRB(*this), "");
		obj.SetSleepState(false); // Clears the support object for 'obj'
	}
}

// Insert 'leg' into the chain of objects supported by 'on_obj'
inline void AddSupport(v4 point, support::Leg& leg, Rigidbody& on_obj)
{
	leg.m_point = point;
	leg.m_count = 0;
	pr::chain::Insert(on_obj.m_support.m_on_me, leg);
}

// Add a support point. Attempt to add 'point' as a support of the object that owns this struct
void Support::Add(Rigidbody& on_obj, v4 gravity, v4 point)
{
	PR_DECLARE_PROFILE(PR_PROFILE_SLEEPING, phSleepAddSupport);
	PR_PROFILE_SCOPE(PR_PROFILE_SLEEPING, phSleepAddSupport);

	// Get the rigid body that owns this support
	Rigidbody& rb = GetRB(*this);
	PR_ASSERT(PR_DBG_PHYSICS, &rb != &on_obj, "Objects cannot support themselves");
	PR_ASSERT(PR_DBG_PHYSICS, !FEql(gravity,pr::v4::Zero()), "This object has no gravity and therefore can't come to rest");
	PR_ASSERT(PR_DBG_PHYSICS, rb.HasMicroVelocity() && on_obj.HasMicroVelocity(), "One of these objects has a velocity above the threshold");
	PR_EXPAND(PR_LDR_SLEEPING, ldr::PhSupport(*this, "sleeping_support");)

	// If the last 'Add' was too long ago, clear the support
	if( m_active == 1 )
		Clear();

	// Get the point relative to our centre of mass
	v4 radius = point - rb.Position();
	v4 radius2d = radius - (Dot3(radius, gravity) / LengthSq(gravity)) * gravity;

	// Reject points that are too close to the centre of mass
	float thres = 0.05f * LengthSq(rb.BBoxOS().Radius());
	if( LengthSq(radius2d) < thres )
		return;

	// Test if this is a repeat of a support we've already seen
	uint32_t pt = 0;
	for( ; pt != m_num_supports; ++pt )
	{
		if( !FEqlRelative(m_leg[pt].m_point, radius2d, thres) ) continue;
		++m_leg[pt].m_count;
		break; // It's a repeat
	}

	// If not a repeat, see if we can add a support
	if( pt == m_num_supports )
	{
		switch( m_num_supports )
		{
		case 0:
			AddSupport(radius2d, m_leg[0], on_obj);
			++m_num_supports;
			break;
		case 1:
			{
				float align = Sqr(Dot3(radius2d, m_leg[0].m_point)) / (LengthSq(radius2d) * LengthSq(m_leg[0].m_point));
				if( Abs(align) > 0.8f ) break;
				AddSupport(radius2d, m_leg[1], on_obj);
				++m_num_supports;
			}break;
		case 2:
			{
				v4 L0xG = Cross(m_leg[0].m_point, gravity);
				v4 L1xG = Cross(m_leg[1].m_point, gravity);
				if( Dot3(m_leg[1].m_point, L0xG) > 0.0f )
				{
					if( Dot3(radius2d, L0xG) > -thres ||
						Dot3(radius2d, L1xG) < thres )
						break;
				}
				else
				{
					if( Dot3(radius2d, L0xG) < thres ||
						Dot3(radius2d, L1xG) > -thres )
						break;
				}
				AddSupport(radius2d, m_leg[2], on_obj);
				++m_num_supports;

				PR_ASSERT(PR_DBG_PHYSICS, PointWithinTriangle2(v4::Zero(), m_leg[0].m_point, m_leg[1].m_point, m_leg[2].m_point, 0.01f), "");
			}break;
		default:
			break;
		}
	}

	m_active	=	DecayTime;
	m_supported =	m_num_supports == 3 &&
					m_leg[0].m_count > RepeatCount &&
					m_leg[1].m_count > RepeatCount &&
					m_leg[2].m_count > RepeatCount;

	PR_EXPAND(PR_LDR_SLEEPING, ldr::PhSupport(*this, "sleeping_support");)
}

// Consider 'contact' to see if the collision is a micro collision
// that would occur if the objects where settling on a support.
void pr::ph::LookForSupports(Contact const& contact, Rigidbody& objectA, Rigidbody& objectB)
{
//return; // DISABLED SLEEPING
	// Decide whether the collision is suitable as a micro collision.
	// Note: sleeping uses absolute velocity, not relative velocity because
	// objects that are asleep are not stepped and therefore won't move.
	if( !(objectA.HasMicroVelocity() && objectB.HasMicroVelocity()) )
		return;

	// Get the acceleration due to gravity at the locations of each object
	v4 gravity_at_A = objectA.Gravity();
	v4 gravity_at_B = objectB.Gravity();

	// Hmm, look at this,... not sure it's valid
	if( objectA.m_motion_type == EMotion_Dynamic &&
		// The radius (CoM to collision point) dotted with the gravity vector should be positive
		Dot3(gravity_at_A, contact.m_pointA - objectA.Position()) > 0.0f &&
		// And the collision normal should oppose the gravity vector
		Dot3(gravity_at_A, contact.m_normal) < 0.0f )
	{
		// Attempt to add this point as a support for the other object
		objectA.m_support.Add(objectB, gravity_at_A, contact.m_pointA);
	}

	if( objectB.m_motion_type == EMotion_Dynamic &&
		// The radius (CoM to collision point) dotted with the gravity vector should be positive
		Dot3(gravity_at_B, contact.m_pointB - objectB.Position()) > 0.0f &&
		// And the collision normal should oppose the gravity vector
		Dot3(gravity_at_B, contact.m_normal) > 0.0f )
	{
		// Attempt to add this point as a support for the other object
		objectB.m_support.Add(objectA, gravity_at_B, contact.m_pointB);
	}
}
