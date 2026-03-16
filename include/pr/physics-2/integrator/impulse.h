//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"

namespace pr::physics
{
	// Impulse Calculations:
	//'  Two objects; A and B, collide at 'p'
	//'  rA  = vector from A origin to 'p'
	//'  rB  = vector from B origin to 'p'
	//'  Va¯ = Velocity at 'p' before collision = VA + WA x rA = body A linear + angular velocity
	//'  Vb¯ = Velocity at 'p' before collision = VB + WB x rB = body B linear + angular velocity
	//'  Va† = Velocity at 'p' after collision = -J(1/ma + rA²/Ia) - Va¯    (in 3D rA²/Ia = -rA x Ia¯ x rA)
	//'  Vb† = Velocity at 'p' after collision = +J(1/mb + rB²/Ib) - Vb¯    (ma,mb = mass, Ia,Ib = inertia)
	//'  V¯  = Relative velocity at 'p' before collision = Vb¯ - Va¯
	//'  V†  = Relative velocity at 'p' after collision = Vb† - Va† = eV¯   (e = elasticity)
	//'      = J(1/mb + rB²/Ib) - Vb¯ + J(1/ma + rA²/Ia) + Va¯              (J = impulse)
	//'      = J(1/ma + 1/mb + rA²/Ia + rB²/Ib) - V¯= eV¯
	//'      = J(1/ma + 1/mb + rA²/Ia + rB²/Ib) = eV¯ + V¯ = (e + 1)V¯
	//'  J   = (e + 1) * (1/ma + 1/mb + rA²/Ia + rB²/Ib)¯¹ * V¯

	// Elasticity and friction:
	//  Elasticity is how bouncy a material is in the normal direction.
	//  Friction is how sticky a material is in the tangential direction.
	//  The normal and torsion components of the outbound velocity are controlled by elasticity.
	//  Friction is used to limit the size of the tangential component of the impulse which effects the
	//  outbound tangential velocity.
	//  See comments in the implementation below

	// Two equal, but opposite, impulses in object space, measured at the object model origin
	struct ImpulsePair
	{
		v8force m_os_impulse_objA;
		v8force m_os_impulse_objB;
		RbContact const* m_contact;
	};

	// Calculate the impulse that will resolve the collision between two objects.
	ImpulsePair RestitutionImpulse(RbContact const& c);
}

