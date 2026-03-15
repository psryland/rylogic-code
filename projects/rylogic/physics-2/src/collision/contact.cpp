//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/collision/contact.h"
#include "pr/physics-2/utility/ldraw.h"
#include "src/collision/gpu_collision_types.h"

namespace pr::physics
{
	RbContact::RbContact()
		:m_b2a()
		, m_velocity()
		, m_point_at_t()
		, m_objA()
		, m_objB()
		, m_mat()
		, m_time()
	{
	}
	RbContact::RbContact(RigidBody const& objA, RigidBody const& objB)
		:RbContact()
	{
		m_objA = &objA;
		m_objB = &objB;
		Update(0);
	}
	RbContact::RbContact(RigidBody const& objA, RigidBody const& objB, GpuContact const& contact)
		:RbContact(objA, objB)
	{
		// Copy geometric data from GPU contact (already in objA's space)
		m_axis = contact.axis;
		m_point = contact.pt;
		m_depth = contact.depth;
		m_mat_idA = contact.mat_id_a;
		m_mat_idB = contact.mat_id_b;
	}

	// Adjust the collision data to the given sub-step time.
	void RbContact::Update(float dt_sub)
	{
		// 'm_b2a' is the position/orientation of objB in objA space at 'time'
		// 'm_velocity' is value of objB's velocity vector field sampled at objA's origin.
		// 'm_point_at_t' is adjusted by half 'dt' because it is the average of the overlap.
		m_b2a = InvertOrthonormal(m_objA->O2W(dt_sub)) * m_objB->O2W(dt_sub);

		// VelocityOS() returns the spatial velocity at the CoM (because momentum and
		// inertia are stored at the CoM). The collision code expects velocity at the
		// model origin so that LinAt(pt) gives the correct velocity at contact points
		// measured from the model origin. Shift each body's velocity from CoM to origin.
		auto va = Shift(m_objA->VelocityOS(), -m_objA->CentreOfMassOS());
		auto vb = Shift(m_objB->VelocityOS(), -m_objB->CentreOfMassOS());
		m_velocity = m_b2a * vb - va;

		m_point_at_t = m_point + 0.5f * dt_sub * m_velocity.LinAt(m_point);
		m_time = dt_sub;
	}

	// Dump the collision scene to LDraw script (best-effort, won't throw)
	void Dump(RbContact const& c)
	{
		(void)c;
		#if 0 //TODO
		try
		{
			using namespace pr::ldraw;

			Builder builder;
			builder.Add<LdrRigidBody>("ObjA", 0x80FF0000).rigid_body(*c.m_objA).flags(ERigidBodyFlags::None);
			builder.Add<LdrRigidBody>("ObjB", 0x8000FF00).rigid_body(*c.m_objB).flags(ERigidBodyFlags::None).o2w(c.m_b2a);
			ldr::VectorField(str, "Velocity", 0xFFFFFF00, (v8)c.m_velocity * 0.1f, v4::Origin(), 2, 0.25f);
			ldr::Arrow(str, "Normal", 0xFFFFFFFF, ldr::EArrowType::Fwd, c.m_point_at_t, c.m_axis * 0.1f, 5);
			builder.Box("Contact", 0xFFFFFF00).box(0.005f).pos(c.m_point_at_t.w1());
			builder.Save(L"dump\\collision.ldr");
	}
		catch (...) {}
		#endif
}
}
