//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/collision/contact.h"
#include "pr/physics/utility/ldraw.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	namespace
	{
		// Copy the geometric and material fields shared by transient resolver contacts and compact public events.
		template <typename Contact> void AssignGpuContact(RbContact& output, Contact const& contact)
		{
			output.m_axis = contact.axis;
			output.m_manifold = {};
			output.m_feature = static_cast<collision::EFeature>(contact.feature);
			for (int i = 0, iend = output.Count(); i != iend; ++i)
				output.m_manifold[i] = contact.manifold[i];
			if (output.Count() == 0)
				output.SetPoint(contact.contact_point);
			output.m_depth = contact.depth;
			output.m_mat_idA = contact.mat_id_a;
			output.m_mat_idB = contact.mat_id_b;
			output.m_child_idA = contact.child_idx_a;
			output.m_child_idB = contact.child_idx_b;
		}
	}

	RbContact::RbContact()
		: m_b2a()
		, m_velocity()
		, m_point_at_t()
		, m_objA()
		, m_objB()
		, m_mat()
		, m_child_idA()
		, m_child_idB()
		, m_substep_index()
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
	RbContact::RbContact(RigidBody const& objA, RigidBody const& objB, GpuResolveContact const& contact)
		:RbContact(objA, objB)
	{
		AssignGpuContact(*this, contact);
		Update(contact.collision_time);
	}

	// Construct a public contact from the compact frame event stream.
	RbContact::RbContact(RigidBody const& objA, RigidBody const& objB, GpuCollisionEvent const& collision_event)
		:RbContact()
	{
		// Retain the exact substep state because caller-owned bodies are not updated until after collision publication.
		m_objA = &objA;
		m_objB = &objB;
		AssignGpuContact(*this, collision_event);
		m_b2a = collision_event.b2a;
		m_velocity = v8motion{collision_event.relative_velocity_ang, collision_event.relative_velocity_lin};
		m_time = collision_event.collision_time;
		auto const point = Point();
		m_point_at_t = point + 0.5f * m_time * m_velocity.LinAt(point);
		m_substep_index = collision_event.substep_index;
	}

	// Adjust the collision data to the given sub-step time.
	void RbContact::Update(float dt_sub)
	{
		// 'm_b2a' is the position/orientation of objB in objA space at 'time'
		// 'm_velocity' is value of objB's velocity vector field sampled at objA's origin.
		// 'm_point_at_t' is adjusted by half 'dt' because it is the average of the overlap.
		m_b2a = InvertOrthonormal(m_objA->O2W(dt_sub)) * m_objB->O2W(dt_sub);

		// VelocityOS() returns the spatial velocity at the CoM (because momentum and inertia are stored at the CoM).
		// The collision code expects velocity at the model origin so that LinAt(pt) gives the correct velocity at contact points
		// measured from the model origin. Shift each body's velocity from CoM to origin.
		auto va = Shift(m_objA->VelocityOS(), -m_objA->CentreOfMassOS());
		auto vb = Shift(m_objB->VelocityOS(), -m_objB->CentreOfMassOS());
		m_velocity = m_b2a * vb - va;

		auto point = Point();
		m_point_at_t = point + 0.5f * dt_sub * m_velocity.LinAt(point);
		m_time = dt_sub;
	}

	// Reverse the sense of the contact information
	void Flip(RbContact& c)
	{
		// Transform from old A space to old B space (which becomes new A space after the swap)
		auto a2b = InvertOrthonormal(c.m_b2a);

		// Reverse the collision normal and transform to new A space
		c.m_axis = a2b * (-c.m_axis);

		// Transform the contact manifold to new A space
		for (auto& point : std::span{ c.m_manifold }.subspan(0, c.Count()))
			point = a2b * point;
		if (auto count = c.Count(); count > 1)
			std::reverse(c.m_manifold.begin(), c.m_manifold.begin() + count);

		// Depth is sign-symmetric — positive means overlap regardless of A/B assignment
		// c.m_depth unchanged

		// Swap material IDs, child IDs, and object pointers
		std::swap(c.m_mat_idA, c.m_mat_idB);
		std::swap(c.m_child_idA, c.m_child_idB);
		std::swap(c.m_objA, c.m_objB);

		// Transform the retained relative state so flipping a published event does not consult caller bodies that have not been unpacked yet.
		c.m_b2a = a2b;
		c.m_velocity = -(a2b * c.m_velocity);
		auto const point = c.Point();
		c.m_point_at_t = point + 0.5f * c.m_time * c.m_velocity.LinAt(point);
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
