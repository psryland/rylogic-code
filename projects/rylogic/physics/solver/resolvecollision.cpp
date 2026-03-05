//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/solver/resolvecollision.h"
#include "pr/physics/collision/contact.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/rigidbody/support.h"
#include "pr/physics/material/imaterial.h"
#include "physics/utility/assert.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;

#if 0
void pr::ph::ResolveCollision(Rigidbody& objectA, Rigidbody& objectB, ContactManifold& manifold)
{
	PR_DECLARE_PROFILE(PR_PROFILE_RESOLVE_COLLISION, phResolveCol);
	PR_PROFILE_SCOPE  (PR_PROFILE_RESOLVE_COLLISION, phResolveCol);
	PR_EXPAND(PR_LOG_RB, Log(objectA, FmtS("ResolveCollision %p", &objectA));)
	PR_EXPAND(PR_LOG_RB, Log(objectB, FmtS("ResolveCollision %p", &objectB));)

	Contact& contact = manifold.GetContact();
	PR_ASSERT(PR_DBG_PHYSICS, contact.IsIncreasingPenetration());

	// Check the contact points are in world space
	// How to debug: set the PC to the end of this function then step out. Then set the PC
	// to the agent.DoCollision() call to do the collision again
	//PR_ASSERT(PR_DBG_PHYSICS, IsWithin(GetWSBBox(objectA), contact.m_pointA, 0.1f));
	//PR_ASSERT(PR_DBG_PHYSICS, IsWithin(GetWSBBox(objectB), contact.m_pointB, 0.1f));
	v4 dir_to_A = contact.m_pointA - objectA.Position();
	v4 dir_to_B = contact.m_pointB - objectB.Position();

	// Get the material properties for the collision, sets them in 'contact'
	SetMaterialProperties(contact);

	// Impulse response
	ImpulseResponse(contact, contact.m_ws_impulse);
	objectA.ApplyWSImpulse( contact.m_ws_impulse, dir_to_A);
	objectB.ApplyWSImpulse(-contact.m_ws_impulse, dir_to_B);

	// THIS DOESN'T WORK - cause it adds energy...
	//// Rolling friction
	//v4 rf_impulse, rf_twist;
	//RollingFriction(contact, rf_impulse, rf_twist);
	//ApplyWSImpulse(objectA,  rf_impulse);
	//ApplyWSImpulse(objectB, -rf_impulse);
	//ApplyWSTwist(objectA,  rf_twist);
	//ApplyWSTwist(objectB, -rf_twist);
}

// Sets the material properties for the collision
void pr::ph::SetMaterialProperties(Contact& contact)
{
	// Determine the surface properties of the two objects
	const ph::Material& materialA = ph::GetMaterial(contact.m_material_idA);
	const ph::Material& materialB = ph::GetMaterial(contact.m_material_idB);

	contact.m_static_friction	= materialA.m_static_friction  * materialB.m_static_friction;
	contact.m_dynamic_friction	= materialA.m_dynamic_friction * materialB.m_dynamic_friction;
	contact.m_rolling_friction	= materialA.m_rolling_friction * materialB.m_rolling_friction;
	contact.m_dynamic_friction	= Minimum(contact.m_static_friction, contact.m_dynamic_friction);
	contact.m_elasticity_n		= (materialA.m_elasticity            + materialB.m_elasticity) * 0.5f;
	contact.m_elasticity_t		= (materialA.m_tangential_elasticity + materialB.m_tangential_elasticity) * 0.5f;
	contact.m_elasticity_tor	= (materialA.m_tortional_elasticity  + materialB.m_tortional_elasticity) * 0.5f;
	
	// Adjust the friction values so that 0->1 corresponds to 0->inf
	contact.m_static_friction	= contact.m_static_friction  / (1.000001f - contact.m_static_friction);
	contact.m_dynamic_friction	= contact.m_dynamic_friction / (1.000001f - contact.m_dynamic_friction);

	// Adjust elasticity for micro collisions
	if( contact.m_objectA->HasMicroVelocity() &&
		contact.m_objectB->HasMicroVelocity() )
	{
		float momA_sq = contact.m_objectA->Momentum().LengthSq();
		float momB_sq = contact.m_objectB->Momentum().LengthSq();		
		float micro_momA_sq = contact.m_objectA->m_micro_mom_sq + tiny<float>;
		float micro_momB_sq = contact.m_objectB->m_micro_mom_sq + tiny<float>;
		float elasticityA = materialA.m_elasticity * Maximum(0.0f, momA_sq/micro_momA_sq - 0.5f*micro_momA_sq);
		float elasticityB = materialB.m_elasticity * Maximum(0.0f, momB_sq/micro_momB_sq - 0.5f*micro_momB_sq);
		contact.m_elasticity_n = (elasticityA + elasticityB) * 0.5f;
	}
}

// Returns the impulse that would resolve a collision between two objects.
// 'ws_impulse' is a world space impulse that should be applied to object A
// (-ws_impulse should be applied to object B)
void pr::ph::ImpulseResponse(Contact& contact, v4& ws_impulse)
{
	Rigidbody const& objA = *contact.m_objectA;
	Rigidbody const& objB = *contact.m_objectB;

	// "mass" is a matrix defined as impulse = mass * drelative_velocity.
	// "inv_mass" is also called the 'K' matrix and is equal to:
	//	[(1/MassA + 1/MassB)*Identity - (CrossProductMatrix(pointA)*InvMassTensorWS()A*CrossProductMatrix(pointA) + CrossProductMatrix(pointB)*InvMassTensorWS()B*CrossProductMatrix(pointB))]
	// Say "inv_mass" = "inv_mass1" + "inv_mass2" then
	// "inv_mass1" = [(1/MassA)*Identity - (pointA.CrossProductMatrix()*InvMassTensorWS()A*pointA.CrossProductMatrix())] and
	// "inv_mass2" = [(1/MassB)*Identity - (pointB.CrossProductMatrix()*InvMassTensorWS()B*pointB.CrossProductMatrix())]
	m3x4 cpmA = CrossProductMatrix3x3(contact.m_pointA - objA.Position());
	m3x4 cpmB = CrossProductMatrix3x3(contact.m_pointB - objB.Position());
	m3x4 inv_mass1	= objA.m_inv_mass * (m3x4::Identity() - (cpmA * objA.m_ws_inv_inertia_tensor * cpmA));
	m3x4 inv_mass2	= objB.m_inv_mass * (m3x4::Identity() - (cpmB * objB.m_ws_inv_inertia_tensor * cpmB));
	m3x4 inv_mass	= inv_mass1 + inv_mass2;
	m3x4 mass		= inv_mass.Invert();

	// Pi is the impulse required to reduce the normal component of rel_velocity to zero.
	// Pii is the impulse to reduce rel_velocity to zero
	// See article: A New Algebraic Rigid Body Collision Law Based On Impulse Space Considerations
	v4 Pi			= (contact.m_rel_norm_speed / Dot3(contact.m_normal, inv_mass * contact.m_normal)) * contact.m_normal;
	v4 Pii			= mass * contact.m_relative_velocity;
	v4 Pdiff		= Pii - Pi;
	ws_impulse		= (1.0f + contact.m_elasticity_n) * Pi + (1.0f + contact.m_elasticity_t) * Pdiff;

	// Clip this impulse to the friction cone
	float impulse_n = Dot3(contact.m_normal,  ws_impulse);
	v4 ws_impulse_t	= impulse_n * contact.m_normal - ws_impulse;
	float impulse_t = ws_impulse_t.Length();
	if( Abs(impulse_t) > Abs(impulse_n * contact.m_static_friction) )
	{
		float kappa = contact.m_dynamic_friction * (1.0f + contact.m_elasticity_n) * Dot3(contact.m_normal, Pi) /
						(impulse_t - contact.m_dynamic_friction * Dot3(Pdiff, contact.m_normal));

		ws_impulse = (1.0f + contact.m_elasticity_n) * Pi + kappa * Pdiff;
	}
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(ws_impulse, ph::OverflowValue));
}

// Calculate rolling friction impulses
void pr::ph::RollingFriction(Contact& contact, v4& rf_impulse, v4& rf_twist)
{
	// Rolling friction reduces the relative tangential linear velocity
	// and the relative normal angular velocity of the objects.
	// Note: this is really just a hack
	v4 rel_lin_momentum		 = contact.m_objectB->Momentum() - contact.m_objectA->Momentum();
	v4 rel_lin_momentum_tang = rel_lin_momentum - Dot3(rel_lin_momentum, contact.m_normal) * contact.m_normal;
	rf_impulse				 = contact.m_rolling_friction * rel_lin_momentum_tang;

	v4 rel_ang_momentum		 = contact.m_objectB->AngMomentum() - contact.m_objectA->AngMomentum();
	v4 rel_ang_momentum_norm = Dot3(rel_ang_momentum, contact.m_normal) * contact.m_normal;
	rf_twist				 = contact.m_rolling_friction * rel_ang_momentum_norm;
}

// Separate two objects by the penetration distance
void pr::ph::PushOut(Rigidbody& objA, Rigidbody& objB, Contact& contact)
{
	float push_dist = contact.m_depth;//Minimum<float>(contact.m_depth, 0.01f);//

	PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_after.pr_script");EndFile();)
	PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_pushA.pr_script");EndFile();)
	PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_pushB.pr_script");EndFile();)
	PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_before.pr_script");)
	PR_EXPAND(PR_LDR_PUSHOUT, ldr::PhRigidbody("objA_before", "80FF0000", objA, 0);)
	PR_EXPAND(PR_LDR_PUSHOUT, ldr::PhRigidbody("objB_before", "80FF0000", objB, 0);)
	PR_EXPAND(PR_LDR_PUSHOUT, EndFile();)

	static float pushout_nrg_scale = 1.0f;
	if( objA.m_inv_mass != 0.0f )
	{
		v4 push = contact.m_normal * push_dist * (objB.m_mass / (objA.m_mass + objB.m_mass));
		PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_pushA.pr_script");)
		PR_EXPAND(PR_LDR_PUSHOUT, ldr::LineD("Push", "FFFFFF00", contact.m_pointA, push); EndFile();)

		// Simple version
		objA.m_object_to_world.pos += push;

		//// Decompose the push into a shift of the CoM and a rotation
		//v4 radius = contact.m_pointA - objA.m_object_to_world.pos;
		//v4 rot    = Cross(radius, push)         / radius.LengthSq();
		//v4 trans  = Dot3(radius, push) * radius  / radius.LengthSq();
		//objA.m_object_to_world     += CrossProductMatrix3x3(rot) * objA.m_object_to_world.Getm3x4();
		//objA.m_object_to_world.pos += trans;
		//if( rot.LengthSq() > 0.01f ) objA.m_object_to_world.Orthonormalise();

		// Remove energy gained by pushing. E ~= v^2
		//float nrg_gain = -Dot3(objA.m_gravity, push) * pushout_nrg_scale;
		//if( nrg_gain > 0.0f )
		{
			//v4 vel = objA.Velocity();
			//float vel_sq = vel.LengthSq();
			//if( nrg_gain > vel_sq )	objA.SetVelocity(v4::Zero());
			//else					objA.SetVelocity(vel * Sqrt((vel_sq - nrg_gain) / vel_sq));
		}
	}
	if( objB.m_inv_mass != 0.0f )
	{
		v4 push = contact.m_normal * -push_dist * (objA.m_mass / (objA.m_mass + objB.m_mass));
		PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_pushB.pr_script");)
		PR_EXPAND(PR_LDR_PUSHOUT, ldr::LineD("Push", "FFFFFF00", contact.m_pointB, push); EndFile();)

		// Simple version
		objB.m_object_to_world.pos += push;

		//// Decompose the push into a shift of the CoM and a rotation
		//v4 radius = contact.m_pointB - objB.m_object_to_world.pos;
		//v4 rot    = Cross(radius, push)         / radius.LengthSq();
		//v4 trans  = Dot3(radius, push) * radius  / radius.LengthSq();
		//objB.m_object_to_world     += CrossProductMatrix3x3(rot) * objB.m_object_to_world.Getm3x4();
		//objB.m_object_to_world.pos += trans;
		//if( rot.LengthSq() > 0.01f ) objB.m_object_to_world.Orthonormalise();

		// Remove energy gained by pushing. E ~= v^2
		//float nrg_gain = -Dot3(objB.m_gravity, push) * pushout_nrg_scale;
		//if( nrg_gain > 0.0f )
		{
			//v4 vel = objB.Velocity();
			//float vel_sq = vel.LengthSq();
			//if( nrg_gain > vel_sq )	objB.SetVelocity(v4::Zero());
			//else					objB.SetVelocity(vel * Sqrt((vel_sq - nrg_gain) / vel_sq));
		}
	}

	PR_EXPAND(PR_LDR_PUSHOUT, StartFile("C:/Deleteme/pushout_after.pr_script");)
	PR_EXPAND(PR_LDR_PUSHOUT, ldr::PhRigidbody("objA_after", "800000FF", objA, 0);)
	PR_EXPAND(PR_LDR_PUSHOUT, ldr::PhRigidbody("objB_after", "800000FF", objB, 0);)
	PR_EXPAND(PR_LDR_PUSHOUT, EndFile();)
}

#endif//0