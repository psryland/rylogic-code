//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/shape/inertia.h"
#include "pr/physics-2/shape/shape_mass.h"
#include "pr/physics-2/utility/misc.h"

namespace pr::physics
{
	struct RigidBody
	{
	protected:

		// Notes:
		//  - Object space is the space that the collision model is given in. It has the model origin
		//    at (0,0,0), the coordinate frame equal to the root object in the collision shape, and
		//    the centre of mass at 'm_os_com'.
		//  - Dynamics state is stored in world space but relative to the model origin. If world space
		//    spatial vectors were relative to the world origin then floating point accuracy would be
		//    an issue.
		//  - Careful with spatial vectors, transforming a spatial vector does not move it, it describes
		//    it from a new position/orientation. Changing 'o2w' does move the spatial vectors though.

		// World space position/orientation of the rigid body
		// This is the position of the model origin in world space (not the CoM)
		m4x4 m_o2w;

		// Offset from the model origin to the CoM (in object space). 
		v4 m_os_com;

		// World space spatial momentum, measured at the model origin (not CoM)
		v8force m_ws_momentum;

		// The external forces and torques applied to this body (in world space), measured at the model origin (not CoM).
		// This value is an accumulator and is reset to zero after each physics step so forces that should
		// be constant need to be applied each frame.
		v8force m_ws_force;

		// Inertia, measured at the model origin (not CoM). Currently this is just simple 3x3 inertia. Articulated bodies will need 6x6 inertia.
		InertiaInv m_os_inertia_inv;

		// Collision shape
		Shape const* m_shape;

	public:

		// Construct the rigid body with a collision shape
		// Inertia is not automatically derived from the collision shape, that is left to the caller.
		template <ShapeType TShape>
		explicit RigidBody(TShape const* shape, m4x4 const& o2w = m4x4::Identity(), Inertia const& inertia = {})
			:RigidBody(shape_cast(shape), o2w, inertia)
		{}
		explicit RigidBody(Shape const* shape = nullptr, m4x4 const& o2w = m4x4::Identity(), Inertia const& inertia = {})
			:m_o2w(o2w)
			,m_os_com()
			,m_ws_momentum()
			,m_ws_force()
			,m_os_inertia_inv()
			,m_shape(collision::shape_cast(shape))
		{
			SetMassProperties(inertia);
		}

		// Raised after the collision shape changes.
		EventHandler<RigidBody&, ChangeEventArgs<collision::Shape const*>> ShapeChange;

		// Get/Set the collision shape for the rigid body
		template <ShapeType TShape> TShape const& Shape() const
		{
			return shape_cast<TShape>(Shape());
		}
		collision::Shape const& Shape() const
		{
			return *m_shape;
		}
		bool HasShape() const
		{
			return m_shape != nullptr;
		}
		
		// Set the shape only, leave the mass properties unchanged
		void Shape(collision::Shape const* shape)
		{
			ShapeChange(*this, ChangeEventArgs<collision::Shape const*>(m_shape, true));
			m_shape = shape;
			ShapeChange(*this, ChangeEventArgs<collision::Shape const*>(m_shape, false));
		}

		// Set the shape and derive mass properties from the shape.
		void Shape(collision::Shape const* shape, float mass, bool mass_is_actually_density = false)
		{
			// Set the shape
			Shape(shape);

			// Derive the mass properties from the shape
			auto mp = CalcMassProperties(*m_shape, mass_is_actually_density ? mass : 1.0f);
			if (!mass_is_actually_density) mp.m_mass = mass;
			SetMassProperties(Inertia{mp}, mp.m_centre_of_mass);
		}

		// Set the shape and mass properties explicitly
		void Shape(collision::Shape const* shape, Inertia inertia, v4 com = v4{})
		{
			// Set the shape
			Shape(shape);

			// Set the mass properties explicitly
			SetMassProperties(inertia, com);
		}

		// Get/Set the body object to world transform
		m4x4 const& O2W() const
		{
			return m_o2w;
		}
		m4x4 W2O() const
		{
			return InvertAffine(O2W());
		}
		void O2W(m4x4 const& o2w)
		{
			assert(IsOrthonormal(o2w));
			m_o2w = o2w;
		}

		// Extrapolate the position based on the current momentum and forces
		m4x4 O2W(float dt) const
		{
			return Abs(dt) > math::tiny<float>
				? ExtrapolateO2W(O2W(), MomentumWS(), ForceWS(), InertiaInvWS(), dt)
				: O2W();
		}

		// Return the world space bounding box for this object
		BBox BBoxWS() const
		{
			return O2W() * Shape().m_bbox;
		}

		// The mass of the rigid body
		float Mass() const
		{
			return InertiaInvOS().Mass();
		}
		void Mass(float mass)
		{
			m_os_inertia_inv.Mass(mass);
		}
		float InvMass() const
		{
			return InertiaInvOS().InvMass();
		}
		void InvMass(float invmass)
		{
			return m_os_inertia_inv.InvMass(invmass);
		}

		// Offset to the centre of mass (w = 0) (Object relative)
		v4 CentreOfMassOS() const
		{
			return m_os_com;
		}
		v4 CentreOfMassWS() const
		{
			return O2W() * CentreOfMassOS();
		}

		// InertiaInv (use 'SetMassProperties' to change)
		InertiaInv InertiaInvOS() const
		{
			return m_os_inertia_inv;
		}
		InertiaInv InertiaInvWS() const
		{
			return Rotate(InertiaInvOS(), O2W().rot);
		}
		Inertia InertiaOS() const
		{
			return Invert(InertiaInvOS());
		}
		Inertia InertiaWS() const
		{
			return Invert(InertiaInvWS());
		}

		// Return the inertia rotated from object space to 'A' space
		// 'com' is the position of this object's CoM in 'A' space
		Inertia InertiaOS(m3x4 const& o2a, v4 com = v4{}) const
		{
			auto inertia = InertiaOS();
			inertia = Rotate(inertia, o2a);
			inertia.CoM(com);
			return inertia;
		}
		InertiaInv InertiaInvOS(m3x4 const& o2a, v4 com = v4{}) const
		{
			auto inertia_inv = InertiaInvOS();
			inertia_inv = Rotate(inertia_inv, o2a);
			inertia_inv.CoM(com);
			return inertia_inv;
		}
		Inertia InertiaOS(m4x4 const& o2a) const
		{
			return InertiaOS(o2a.rot, o2a.pos);
		}
		InertiaInv InertiaInvOS(m4x4 const& o2a) const
		{
			return InertiaInvOS(o2a.rot, o2a.pos);
		}

		// Get/Set the velocity
		v8motion VelocityWS() const
		{
			auto ws_velocity = InertiaInvWS() * MomentumWS();
			return ws_velocity;
		}
		v8motion VelocityOS() const
		{
			return W2O().rot * VelocityWS();
		}
		void VelocityWS(v8motion const& ws_velocity)
		{
			auto ws_momentum = InertiaWS() * ws_velocity;
			MomentumWS(ws_momentum);
		}
		void VelocityOS(v8motion const& os_velocity)
		{
			auto ws_velocity = O2W().rot * os_velocity;
			VelocityWS(ws_velocity);
		}
		void VelocityWS(v4 ws_ang, v4 ws_lin, v4 ws_at = v4{})
		{
			// 'ws_ang' and 'ws_lin' describe velocity at 'ws_at'. Shift to model origin.
			auto spatial_velocity = v8motion{ws_ang, ws_lin};
			spatial_velocity = Shift(spatial_velocity, -ws_at);
			VelocityWS(spatial_velocity);
		}
		void VelocityOS(v4 os_ang, v4 os_lin, v4 os_at = v4{})
		{
			auto ws_ang = O2W() * os_ang;
			auto ws_lin = O2W() * os_lin;
			auto ws_at  = O2W() * os_at;
			VelocityWS(ws_ang, ws_lin, ws_at);
		}

		// Get/Set the momentum of the rigid body
		v8force MomentumWS() const
		{
			return m_ws_momentum;
		}
		v8force MomentumOS() const
		{
			return W2O().rot * MomentumWS();
		}
		void MomentumWS(v8force const& ws_momentum)
		{
			m_ws_momentum = ws_momentum;
		}
		void MomentumOS(v8force const& os_momentum)
		{
			auto ws_momentum = O2W().rot * os_momentum;
			MomentumWS(ws_momentum);
		}

		// Reset the state of the body
		void ZeroForces()
		{
			m_ws_force = v8force{};
		}
		void ZeroMomentum()
		{
			m_ws_momentum = v8force{};
		}

		// Get/Set the current forces applied to this body.
		v8force ForceWS() const
		{
			return m_ws_force;
		}
		v8force ForceOS() const
		{
			return W2O().rot * ForceWS();
		}

		// Add a force acting on the rigid body at position 'at' (world space, object origin relative, not CoM relative)
		void ApplyForceWS(v4 ws_force, v4 ws_torque, v4 ws_at = v4::Zero())
		{
			assert("'at' should be an offset (in world space) from the object origin" && ws_at.w == 0);
			auto spatial_force = v8force{ws_torque, ws_force};
			spatial_force = Shift(spatial_force, -ws_at);
			ApplyForceWS(spatial_force);
		}
		void ApplyForceWS(v8force ws_force)
		{
			m_ws_force += ws_force;
		}

		// Add a force acting on the rigid body at position 'at' (object space, not CoM relative)
		void ApplyForceOS(v4 os_force, v4 os_torque, v4 os_at = v4::Zero())
		{
			assert("'at' should be an offset (in object space) from the object origin" && os_at.w == 0);
			auto o2w = O2W();
			auto ws_force  = o2w * os_force;
			auto ws_torque = o2w * os_torque;
			auto ws_at     = o2w * os_at;
			ApplyForceWS(ws_force, ws_torque, ws_at);
		}
		void ApplyForceOS(v8force const& os_force)
		{
			auto ws_force = O2W().rot * os_force;
			ApplyForceWS(ws_force);
		}

		// Set the mass properties of the body.
		// 'os_inertia' is the inertia for the body, measured at the model origin (not CoM) (in object space)
		// 'os_model_to_com' is the vector from the model origin to the body's centre of mass (in object space)
		void SetMassProperties(Inertia const& os_inertia, v4 os_model_to_com = v4{})
		{
			// Notes:
			//  - os_inertia.CoM() vs. os_model_to_com:
			//    See comments for 'Inertia', but you probably want 'os_inertia.CoM()' to be zero. It is really only
			//    used with spatial vectors. 'os_model_to_com' is the more common case where the inertia has been
			//    measured at a point that isn't the CoM (typically the model origin). This is recorded so that
			//    callers can apply forces to the CoM.
			//  - When CoM is offset from the model origin, we translate the inertia back to the CoM and record
			//    the offset. This enables the full 6x6 spatial inertia math (with coupling between angular and
			//    linear components) in the operator* overloads.
			assert("'os_model_to_com' should be an offset (in object space) from the object origin" && os_model_to_com.w == 0);
			
			// Translate the inertia to the CoM and set the CoM offset so that the full
			// 6x6 spatial inertia is used for operator*(Inertia, v8motion) etc.
			auto inertia = os_inertia;
			if (os_model_to_com != v4{})
			{
				inertia = Translate(inertia, os_model_to_com, ETranslateInertia::TowardCoM);
				inertia.CoM(os_model_to_com);
			}

			// Object space inertia inverse
			m_os_inertia_inv = Invert(inertia);

			// Position of the centre of mass (in object space)
			m_os_com = os_model_to_com;
		}

		// Return the kinetic energy of the body
		float KineticEnergy() const
		{
			// KE = 0.5 v.h = 0.5 v.Iv
			auto ke = 0.5f * Dot(VelocityWS(), MomentumWS());
			return ke;
		}
	};

	// Return the world space bounding box for 'rb'
	inline BBox BBoxWS(RigidBody const& rb)
	{
		return rb.BBoxWS();
	}
}

