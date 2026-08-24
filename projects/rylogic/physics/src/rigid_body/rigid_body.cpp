//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/shape_mass.h"
#include "pr/physics/utility/misc.h"

namespace pr::physics
{
	namespace
	{
		// Allocate a process-local nonzero identity for a newly constructed rigid body.
		BodyId NextBodyId()
		{
			static auto s_next_id = std::atomic_uint64_t{1};
			auto value = s_next_id.load(std::memory_order_relaxed);

			// Reserve one identity without allowing counter wrap to reissue an earlier value.
			for (;;)
			{
				if (value == std::numeric_limits<uint64_t>::max())
					throw std::overflow_error("Rigid-body identity space exhausted");

				if (s_next_id.compare_exchange_weak(value, value + 1, std::memory_order_relaxed))
					return BodyId{.m_value = value};
			}
		}
	}

	// Construct the rigid body with a collision shape
	// Inertia is not automatically derived from the collision shape, that is left to the caller.
	RigidBody::RigidBody(collision::Shape const* shape, m4x4 const& o2w, Inertia const& inertia)
		:m_o2w(o2w)
		,m_id(NextBodyId())
		,m_os_com()
		,m_ws_momentum()
		,m_ws_force()
		,m_ws_gravity()
		,m_os_inertia_inv()
		,m_shape(collision::shape_cast(shape))
		,m_state_flags(ERigidBodyStateFlags::None)
		,m_sleep()
	{
		SetMassProperties(inertia);
	}

	// Return the stable identity used by persistent constraints.
	BodyId RigidBody::Id() const
	{
		return m_id;
	}

	// Get/Set the collision shape for the rigid body
	template <ShapeType TShape> TShape const& RigidBody::Shape() const
	{
		return shape_cast<TShape>(Shape());
	}
	collision::Shape const& RigidBody::Shape() const
	{
		return m_shape ? *m_shape : NoShape();
	}
	bool RigidBody::HasShape() const
	{
		return m_shape != nullptr;
	}
		
	// Set the shape only, leave the mass properties unchanged
	void RigidBody::Shape(collision::Shape const* shape)
	{
		if (shape != m_shape)
			Wake();

		ShapeChange(*this, ChangeEventArgs<collision::Shape const*>(m_shape, true));
		m_shape = shape;
		ShapeChange(*this, ChangeEventArgs<collision::Shape const*>(m_shape, false));
	}

	// Set the shape and derive mass properties from the shape.
	void RigidBody::Shape(collision::Shape const* shape, float mass, bool mass_is_actually_density)
	{
		// Set the shape
		Shape(shape);

		if (mass > 0 && mass < InfiniteMass)
		{
			// Derive the mass properties from the shape
			auto mp = CalcMassProperties(*m_shape, mass_is_actually_density ? mass : 1.0f);
			if (!mass_is_actually_density) mp.m_mass = mass;
			SetMassProperties(Inertia{ mp }, mp.m_centre_of_mass);
		}
		else
		{
			// Set the mass properties to be immovable
			SetMassProperties(Inertia::Infinite());
			m_os_com = v4{};
		}
	}

	// Set the shape and mass properties explicitly
	void RigidBody::Shape(collision::Shape const* shape, Inertia inertia, v4 com)
	{
		// Set the shape
		Shape(shape);

		// Set the mass properties explicitly
		SetMassProperties(inertia, com);
	}

	// Get/Set the body object to world transform
	m4x4 const& RigidBody::O2W() const
	{
		return m_o2w;
	}
	m4x4 RigidBody::W2O() const
	{
		return InvertOrthonormal(O2W());
	}
	void RigidBody::O2W(m4x4 const& o2w)
	{
		assert(IsOrthonormal(o2w));
		m_o2w = o2w;
		Wake(); // For performance, assume any o2w change is a wake-up signal as well
	}

	// Extrapolate the position based on the current momentum and forces
	m4x4 RigidBody::O2W(float dt) const
	{
		return Abs(dt) > math::tiny<float>
			? ExtrapolateO2W(O2W(), CentreOfMassOS(), MomentumWS(), ForceWS(), InertiaInvWS(), dt)
			: O2W();
	}

	// Return the world space bounding box for this object
	BBox RigidBody::BBoxWS() const
	{
		auto const& shape = Shape();
		return O2W() * (shape.m_s2r * shape.m_bbox);
	}

	// The mass of the rigid body
	float RigidBody::Mass() const
	{
		return InertiaInvOS().Mass();
	}
	void RigidBody::Mass(float mass)
	{
		if (!FEql(m_os_inertia_inv.Mass(), mass))
			Wake();

		m_os_inertia_inv.Mass(mass);
	}
	float RigidBody::InvMass() const
	{
		return InertiaInvOS().InvMass();
	}
	void RigidBody::InvMass(float invmass)
	{
		if (!FEql(m_os_inertia_inv.InvMass(), invmass))
			Wake();

		m_os_inertia_inv.InvMass(invmass);
	}

	// Return the model-origin-to-centre-of-mass offset in object space.
	v4 RigidBody::CentreOfMassOS() const
	{
		return m_os_com;
	}

	// Return the model-origin-to-centre-of-mass offset in world orientation.
	v4 RigidBody::CentreOfMassOffsetWS() const
	{
		return (O2W().rot * CentreOfMassOS()).w0();
	}

	// Return the absolute world-space position of the centre of mass.
	v4 RigidBody::CentreOfMassPositionWS() const
	{
		return (O2W().pos + CentreOfMassOffsetWS()).w1();
	}

	// InertiaInv (use 'SetMassProperties' to change)
	InertiaInv RigidBody::InertiaInvOS() const
	{
		return m_os_inertia_inv;
	}
	InertiaInv RigidBody::InertiaInvWS() const
	{
		return Rotate(InertiaInvOS(), O2W().rot);
	}
	Inertia RigidBody::InertiaOS() const
	{
		return Invert(InertiaInvOS());
	}
	Inertia RigidBody::InertiaWS() const
	{
		return Invert(InertiaInvWS());
	}

	// Return the inertia rotated from object space to 'A' space
	// 'com' is the position of this object's CoM in 'A' space
	Inertia RigidBody::InertiaOS(m3x3 const& o2a, v4 com) const
	{
		auto inertia = InertiaOS();
		inertia = Rotate(inertia, o2a);
		inertia.CoM(com);
		return inertia;
	}
	InertiaInv RigidBody::InertiaInvOS(m3x3 const& o2a, v4 com) const
	{
		auto inertia_inv = InertiaInvOS();
		inertia_inv = Rotate(inertia_inv, o2a);
		inertia_inv.CoM(com);
		return inertia_inv;
	}
	Inertia RigidBody::InertiaOS(m4x4 const& o2a) const
	{
		return InertiaOS(o2a.rot, o2a.pos);
	}
	InertiaInv RigidBody::InertiaInvOS(m4x4 const& o2a) const
	{
		return InertiaInvOS(o2a.rot, o2a.pos);
	}

	// Get/Set the momentum of the rigid body
	v8force RigidBody::MomentumWS() const
	{
		return m_ws_momentum;
	}
	v8force RigidBody::MomentumOS() const
	{
		return W2O().rot * MomentumWS();
	}
	void RigidBody::MomentumWS(v8force const& ws_momentum)
	{
		if (!FEql(m_ws_momentum, ws_momentum))
			Wake();

		m_ws_momentum = ws_momentum;
	}
	void RigidBody::MomentumOS(v8force const& os_momentum)
	{
		auto ws_momentum = O2W().rot * os_momentum;
		MomentumWS(ws_momentum);
	}

	// Get/Set the velocity
	v8motion RigidBody::VelocityWS() const
	{
		auto ws_velocity = InertiaInvWS() * MomentumWS();
		return ws_velocity;
	}
	v8motion RigidBody::VelocityOS() const
	{
		return W2O().rot * VelocityWS();
	}
	void RigidBody::VelocityWS(v8motion const& ws_velocity)
	{
		auto ws_momentum = InertiaWS() * ws_velocity;
		MomentumWS(ws_momentum);
	}
	void RigidBody::VelocityOS(v8motion const& os_velocity)
	{
		auto ws_velocity = O2W().rot * os_velocity;
		VelocityWS(ws_velocity);
	}
	void RigidBody::VelocityWS(v4 ws_ang, v4 ws_lin, v4 ws_at)
	{
		// 'ws_ang' and 'ws_lin' describe velocity at 'ws_at' (offset from model origin).
		// Shift to the centre of mass.
		auto ws_com = O2W().rot * m_os_com;
		auto spatial_velocity = v8motion{ws_ang, ws_lin};
		spatial_velocity = Shift(spatial_velocity, ws_com - ws_at);
		VelocityWS(spatial_velocity);
	}
	void RigidBody::VelocityOS(v4 os_ang, v4 os_lin, v4 os_at)
	{
		auto ws_ang = O2W() * os_ang;
		auto ws_lin = O2W() * os_lin;
		auto ws_at  = O2W() * os_at;
		VelocityWS(ws_ang, ws_lin, ws_at);
	}

	// Reset the state of the body
	void RigidBody::ZeroForces()
	{
		m_ws_force = v8force{};
	}
	void RigidBody::ZeroMomentum()
	{
		MomentumWS(v8force{});
	}

	// Apply gravity to the body. This should be called each frame to apply the gravity force
	// to the body, even for static bodies in order to define the "down" direction for the body.
	v4 RigidBody::GravityWS() const
	{
		return m_ws_gravity;
	}
	void RigidBody::GravityWS(v4 ws_gravity)
	{
		m_ws_gravity = ws_gravity;
		if (Sleeping())
			return;

		auto mass = Mass();
		if (mass < InfiniteMass * 0.5f)
		{
			auto ws_com = O2W().rot * m_os_com;
			auto ws_at = O2W().rot * CentreOfMassOS();
			auto spatial_force = v8force{v4::Zero(), m_ws_gravity * mass};
			spatial_force = Shift(spatial_force, ws_com - ws_at);
			AccumulateForceWS(spatial_force);
		}
	}

	// Return the body's state flags
	ERigidBodyStateFlags RigidBody::StateFlags() const
	{
		return m_state_flags;
	}

	// True if the body is flagged as asleep
	bool RigidBody::Sleeping() const
	{
		return AllSet(m_state_flags, ERigidBodyStateFlags::Sleeping);
	}
	void RigidBody::Sleeping(bool sleeping)
	{
		if (sleeping)
			Sleep();
		else
			Wake();
	}
	
	// Put the body to sleep immediately, or wake it up immediately.
	void RigidBody::Sleep()
	{
		assert(!NeverSleep());
		if (NeverSleep())
			return;

		InvalidateSleepIsland();
		m_state_flags = SetBits(m_state_flags, ERigidBodyStateFlags::Sleeping, true);
		m_ws_momentum = v8force{};
		m_ws_force = v8force{};
		m_sleep.m_timer_s = 0.0f;
	}
	void RigidBody::Wake()
	{
		InvalidateSleepIsland();
		m_state_flags = SetBits(m_state_flags, ERigidBodyStateFlags::Sleeping, false);
		m_sleep.m_timer_s = 0.0f;
	}

	// True if the body is immune to automatic sleeping
	bool RigidBody::NeverSleep() const
	{
		return AllSet(m_state_flags, ERigidBodyStateFlags::NeverSleep);
	}
	void RigidBody::NeverSleep(bool never_sleep)
	{
		if (never_sleep)
			Wake();

		m_state_flags = SetBits(m_state_flags, ERigidBodyStateFlags::NeverSleep, never_sleep);
	}

	// Invalidate cached sleep island membership.
	void RigidBody::InvalidateSleepIsland()
	{
		m_sleep.m_island_id = -1;
		m_sleep.m_generation += 1;
		m_sleep.m_flags = 0;
	}
	
	// Add force without changing sleep state.
	void RigidBody::AccumulateForceWS(v8force const& ws_force)
	{
		m_ws_force += ws_force;
	}

	// Get/Set the current forces applied to this body (measured at the centre of mass).
	v8force RigidBody::ForceWS() const
	{
		return m_ws_force;
	}
	void RigidBody::ForceWS(v8force const& ws_force)
	{
		m_ws_force = ws_force;
	}
	v8force RigidBody::ForceOS() const
	{
		return W2O().rot * ForceWS();
	}

	// Add a force acting on the rigid body at position 'ws_at' (world space, model origin relative).
	// The force is shifted from the application point to the centre of mass before accumulation.
	// For gravity: pass ws_at = O2W().rot * CentreOfMassOS() so gravity produces no torque about CoM.
	void RigidBody::ApplyForceWS(v4 ws_force, v4 ws_torque, v4 ws_at)
	{
		assert("'at' should be an offset (in world space) from the object origin" && ws_at.w == 0);

		// Shift the spatial force from the application point to the centre of mass.
		// ws_at is relative to the model origin, ws_com is the CoM relative to the model origin.
		auto ws_com = O2W().rot * m_os_com;
		auto spatial_force = v8force{ws_torque, ws_force};
		spatial_force = Shift(spatial_force, ws_com - ws_at);
		ApplyForceWS(spatial_force);
	}
	void RigidBody::ApplyForceWS(v8force ws_force)
	{
		if (!FEql(ws_force, v8force{}))
			Wake();

		AccumulateForceWS(ws_force);
	}

	// Add a force acting on the rigid body at position 'os_at' (object space, model origin relative)
	void RigidBody::ApplyForceOS(v4 os_force, v4 os_torque, v4 os_at)
	{
		assert("'at' should be an offset (in object space) from the object origin" && os_at.w == 0);
		auto o2w = O2W();
		auto ws_force  = o2w * os_force;
		auto ws_torque = o2w * os_torque;
		auto ws_at     = o2w * os_at;
		ApplyForceWS(ws_force, ws_torque, ws_at);
	}
	void RigidBody::ApplyForceOS(v8force const& os_force)
	{
		auto ws_force = O2W().rot * os_force;
		ApplyForceWS(ws_force);
	}

	// Set the mass properties of the body.
	// 'os_inertia' is the inertia for the body, measured at the model origin (not CoM) (in object space)
	// 'os_model_to_com' is the vector from the model origin to the body's centre of mass (in object space)
	void RigidBody::SetMassProperties(Inertia const& os_inertia, v4 os_model_to_com)
	{
		// Notes:
		//  - When CoM is offset from the model origin, we translate the inertia from
		//    the model origin to the CoM using the parallel axis theorem. The stored
		//    inverse inertia is always in the CoM frame with CoM() == 0 (block-diagonal).
		//  - Momentum and forces are stored about the CoM, so the inertia multiply is
		//    simple: omega = Ic_inv * h_ang, v = h_lin / m (no coupling terms).
		//  - The CoM offset is stored separately in m_os_com for position updates
		//    (converting CoM velocity to model-origin position changes).
		//  - For future Featherstone articulated body support, the inertia can be shifted
		//    to joint frames on demand using Translate()/To6x6().
		assert("'os_model_to_com' should be an offset (in object space) from the object origin" && os_model_to_com.w == 0);
			
		// Translate the inertia from the model origin to the CoM.
		// Do NOT set inertia.CoM() — we want it zero so operator*(InertiaInv, v8force)
		// takes the block-diagonal path (no angular-linear coupling).
		auto inertia = os_inertia;
		if (LengthSq(os_model_to_com) != 0)
			inertia = Translate(inertia, os_model_to_com, ETranslateInertia::TowardCoM);

		// Object space inverse inertia, measured at the CoM
		auto inertia_inv = Invert(inertia);
		auto changed = !FEql(m_os_inertia_inv, inertia_inv) || !FEql(m_os_com, os_model_to_com);
		if (changed)
			Wake();

		m_os_inertia_inv = inertia_inv;

		// Position of the centre of mass (in object space)
		m_os_com = os_model_to_com;

		// Set state flags
		m_state_flags = SetBits(m_state_flags, ERigidBodyStateFlags::Static, inertia.InvMass() == 0.0f);
	}

	// Return the kinetic energy of the body
	float RigidBody::KineticEnergy() const
	{
		// KE = 0.5 v.h = 0.5 v.Iv
		auto ke = 0.5f * Dot(VelocityWS(), MomentumWS());
		return ke;
	}
}
