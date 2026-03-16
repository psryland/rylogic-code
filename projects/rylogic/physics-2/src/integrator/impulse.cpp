//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/integrator/impulse.h"
#include "pr/physics-2/collision/contact.h"
#include "pr/physics-2/rigid_body/rigid_body.h"

namespace pr::physics
{
	// Calculate the restitution impulse that resolves a collision between two rigid bodies.
	//
	// Overview:
	//   Given a contact between objA and objB (all data in objA's frame), this function
	//   computes the impulse needed to prevent interpenetration and apply restitution.
	//   The result is a pair of spatial force wrenches — one for each body — expressed
	//   at each body's model origin in its own object space.
	//
	// The approach:
	//   1. Compute the relative velocity at the contact point.
	//   2. Build the "collision mass matrix" that relates an impulse at the contact point
	//      to the resulting velocity change, accounting for both translational and rotational
	//      degrees of freedom.
	//   3. Decompose the impulse into normal and tangential components.
	//   4. Scale the normal component by (1 + elasticity) for restitution.
	//   5. Clamp the tangential component to the friction cone.
	//   6. Convert the point impulse to a spatial wrench at each body's origin.
	//
	ImpulsePair RestitutionImpulse(RbContact const& c)
	{
		auto& objA = *c.m_objA;
		auto& objB = *c.m_objB;
		auto& pt = c.m_point_at_t;

		// Sanity check: the contact should be approaching, not separating
		#if PR_DBG
		auto rel_normal_velocity = Dot(c.m_velocity.LinAt(pt), c.m_axis);
		assert("Point of contact is moving out of collision" && rel_normal_velocity <= 0);
		#endif

		// Lever arms from each body's CoM to the contact point (in A's frame).
		// Momentum/forces are stored at the CoM, so lever arms must be from CoM.
		auto com_A_in_A = v4::Origin() + objA.CentreOfMassOS();
		auto com_B_in_A = c.m_b2a.pos + c.m_b2a.rot * objB.CentreOfMassOS();
		auto rA = pt - com_A_in_A;
		auto rB = pt - com_B_in_A;

		// Relative velocity at the contact point.
		auto V_inv  = c.m_velocity.LinAt(pt);
		auto Vn_inv = Dot(V_inv, c.m_axis) * c.m_axis;

		// Inverse inertia tensors at each body's CoM, in A's frame.
		// Since InertiaInvOS().CoM() == 0, To3x3() returns Ic3x3() (the CoM inertia).
		auto Ia_inv_3x3 = objA.InertiaInvOS().To3x3();
		auto Ib_inv_3x3 = objB.InertiaInvOS(c.m_b2a.rot).To3x3();

		// Collision inverse-mass matrix.
		auto col_Ia_inv = (1/objA.Mass()) * m3x4::Identity() - CPM<m3x4>(rA) * Ia_inv_3x3 * CPM<m3x4>(rA);
		auto col_Ib_inv = (1/objB.Mass()) * m3x4::Identity() - CPM<m3x4>(rB) * Ib_inv_3x3 * CPM<m3x4>(rB);
		auto col_I_inv = col_Ia_inv + col_Ib_inv;

		// The collision mass matrix.
		auto col_I = Invert(col_I_inv);

		// Decomposethe impulse into normal and tangential components.
		// impulse0: the impulse needed to kill ALL relative velocity (zero restitution).
		// impulseN: the normal component only (for zero restitution).
		// impulseT: the tangential (friction) component.
		auto impulse0 = -(col_I * V_inv);
		auto denom = Dot(c.m_axis, col_I_inv * c.m_axis);
		auto impulseN = Abs(denom) > math::tiny<float> 
			? -(Dot(c.m_axis, V_inv) / denom) * c.m_axis 
			: v4{};
		auto impulseT = impulse0 - impulseN;

		// Apply restitution to the normal component.
		// For elasticity e: the post-collision normal velocity is -e times the pre-collision value.
		// This requires scaling the normal impulse by (1 + e). e=0 is perfectly inelastic
		// (objects stick together), e=1 is perfectly elastic (kinetic energy conserved).
		auto impulse4 = (1 + c.m_mat.m_elasticity_norm) * impulseN + impulseT;

		// Coulomb friction cone: the tangential impulse cannot exceed μ · |Jn|.
		// If it does, we clamp it, which models sliding friction. When friction is zero,
		// this effectively removes all tangential impulse, making the collision frictionless.
		{
			auto clamped_friction = pr::Min(c.m_mat.m_friction_static, 0.9999f);
			auto static_friction = clamped_friction / (1.000001f - clamped_friction);
			auto Jn = Dot(impulse4, c.m_axis);
			auto Jt = Sqrt(pr::Max(0.0f, LengthSq(impulse4) - Sqr(Jn)));
			if (Jt > static_friction * Abs(Jn))
			{
				Jt = static_friction * Abs(Jn);
				auto impulseT_lenSq = LengthSq(impulseT);
				if (impulseT_lenSq > Sqr(math::tiny<float>))
					impulseT = Jt * (impulseT / Sqrt(impulseT_lenSq));
				impulse4 = (1 + c.m_mat.m_elasticity_norm) * impulseN + impulseT;
			}
		}

		// Convert the linear impulse at the contact point into spatial force wrenches
		// at each body's CoM. The pure force at the contact point creates a torque = r × F
		// about the CoM where r is the lever arm from CoM to contact point.
		auto impulse_at_pt = v8force{v4{}, impulse4};

		// For A: shift impulse from contact point to A's CoM (in A's frame)
		auto impulseA = Shift(impulse_at_pt, com_A_in_A - pt);

		// For B: shift impulse from contact point to B's CoM (in A's frame),
		// then rotate to B's frame (pure rotation, the wrench is already at B's CoM)
		auto impulseB_in_A = Shift(impulse_at_pt, com_B_in_A - pt);
		auto a2b_rot = InvertOrthonormal(c.m_b2a.rot);

		// Build the impulse pair: equal and opposite wrenches for each body.
		auto impulse_pair = ImpulsePair{};
		impulse_pair.m_os_impulse_objA = -impulseA;
		impulse_pair.m_os_impulse_objB = a2b_rot * impulseB_in_A;
		impulse_pair.m_contact = &c;
		return impulse_pair;
	}
}

