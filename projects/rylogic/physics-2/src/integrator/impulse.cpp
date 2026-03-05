//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/integrator/impulse.h"

namespace pr::physics
{
	// Calculate the impulse that will resolve the collision between two objects.
	ImpulsePair RestitutionImpulse(Contact const& c)
	{
		// Calculate the effective inertia at 'c.m_point'. This is not the sum of inertias
		// because, even though the bodies are in contact at 'c.m_point', the point has a
		// different velocity on each body.
		// Let:
		//'   +p, -p = the restitution impulse for each object (equal but opposite)
		//' dVa, dVb = the change in velocities for the objects
		//' Ia¯, Ib¯ = the inverse inertia for each object expressed at the collision point (in objA space) '
		//'  impulse = change in momentum; p = dH = I.dV '
		//'    Vdiff = dVb - dVa          '
		//'      dVa = -Ia¯.p             '
		//'      dVb = +Ib¯.p             '
		//'    Vdiff = (Ib¯.p + Ia¯.p)    '
		//'    Vdiff = (Ib¯ + Ia¯).p      '
		//' => p = (Ib¯ + Ia¯)¯.Vdiff     '

		// Debugging Tips:
		//  - Check the impulse for each object assuming the other object has infinite mass
		//    i.e. set one of Ia¯ or Ib¯ to zero.

		auto& objA = *c.m_objA;
		auto& objB = *c.m_objB;
		auto& pt = c.m_point_at_t;

		// Check the relative velocity is into the collision
		#if PR_DBG
		auto rel_normal_velocity = Dot(c.m_velocity.LinAt(pt), c.m_axis);
		assert("Point of contact is moving out of collision" && rel_normal_velocity <= 0);
		//Dump(c);
		#endif

		// rA = vector from objA origin to 'p'
		auto rA = pt - v4::Origin();

		// rB = vector from objB origin to 'p'
		auto rB = pt - c.m_b2a.pos;

		// V¯ = Relative velocity at 'p' before collision = Vb¯ - Va¯
		auto V_inv  = c.m_velocity.LinAt(pt);

		// Vn¯ = Relative normal velocity at 'p' before the collision
		auto Vn_inv = Dot(V_inv, c.m_axis) * c.m_axis;

		// The collision inertia contribution by each object
		auto col_Ia_inv = (1/objA.Mass()) * m3x4::Identity() - CPM(rA) * objA.InertiaInvOS().To3x3() * CPM(rA);
		auto col_Ib_inv = (1/objB.Mass()) * m3x4::Identity() - CPM(rB) * objB.InertiaInvOS(c.m_b2a).To3x3() * CPM(rB);
		auto col_I_inv = col_Ia_inv + col_Ib_inv;
		auto col_I = Invert(col_I_inv);
		
		// Get the impulse that would change the relative velocity at 'pt' to zero
		auto impulse0 = -(col_I * V_inv);

		// Get the impulse that would reduce the normal component of the relative velocity at 'pt' to zero
		// Check denominator to avoid division by zero for degenerate collision configurations
		auto denom = Dot(c.m_axis, col_I_inv * c.m_axis);
		auto impulseN = Abs(denom) > maths::tiny<float> 
			? -(Dot(c.m_axis, V_inv) / denom) * c.m_axis 
			: v4{};

		// The difference is the impulse that would reduce the tangential component of the relative velocity at 'pt' to zero
		auto impulseT = impulse0 - impulseN;

		#if PR_DBG
		//{
		//	// Assert that impulse0 would kill the relative velocity
		//	auto imp = Shift(v8force{v4{}, impulse0}, v4::Origin() - pt);
		//	auto velA_ws = (objA.InertiaInvWS() * (objA.O2W().rot * (objA.MomentumOS() +                       -imp)));
		//	auto velB_ws = (objB.InertiaInvWS() * (objB.O2W().rot * (objB.MomentumOS() + InvertAffine(c.m_b2a) * +imp)));
		//	
		//	auto pt_ws = objA.O2W() * pt;
		//	auto velA_at_pt_ws = velA_ws.LinAt(pt_ws - objA.O2W().pos);
		//	auto velB_at_pt_ws = velB_ws.LinAt(pt_ws - objB.O2W().pos);
		//	auto dvel_ws = (velB_at_pt_ws - velA_at_pt_ws);
		//	assert(FEqlRelative(Length(dvel_ws), 0, 0.0001f));
		//}
		#endif
		// TODO optimise...

		// Calculate the restitution impulse
		// Apply elasticity to the normal component only (tangential handled by friction)
		auto impulse4 = (1 + c.m_mat.m_elasticity_norm) * impulseN + impulseT;

		// Limit the normal and tangential components of 'impulse' to the friction cone.
		{
			// Scale 0->1 to 0->inf, 0.5->1.0f. Clamp friction to avoid division by zero.
			auto clamped_friction = Min(c.m_mat.m_friction_static, 0.9999f);
			auto static_friction = clamped_friction / (1.000001f - clamped_friction);

			// If '|Jt|/|Jn|' (the ratio of tangential to normal magnitudes) is greater than static friction
			// then the contact 'slips' and the impulse is reduced in the tangential direction.
			auto Jn = Dot(impulse4, c.m_axis);
			
			// Clamp to avoid sqrt of negative value due to floating point errors
			auto Jt = Sqrt(Max(0.0f, LengthSq(impulse4) - Sqr(Jn)));
			if (Jt > static_friction * Abs(Jn))
			{
				// Reduce the tangential component of the impulse
				Jt = static_friction * Abs(Jn);
				
				// Only normalize if impulseT has non-zero length
				auto impulseT_lenSq = LengthSq(impulseT);
				if (impulseT_lenSq > Sqr(maths::tiny<float>))
					impulseT = Jt * (impulseT / Sqrt(impulseT_lenSq));
				
				impulse4 = (1 + c.m_mat.m_elasticity_norm) * impulseN + impulseT;
			}
		}

		auto impulse = Shift(v8force{v4{}, impulse4}, v4::Origin() - pt);

		auto impulse_pair = ImpulsePair{};
		impulse_pair.m_os_impulse_objA = -impulse;
		impulse_pair.m_os_impulse_objB = InvertAffine(c.m_b2a) * +impulse;
		impulse_pair.m_contact = &c;
		return impulse_pair;
	}
}
