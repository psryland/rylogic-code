//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/constraint/constraint_ids.h"
#include "pr/physics/shape/inertia.h"
#include "pr/physics/rigid_body/sleep_data.h"
#include "pr/physics/rigid_body/state_flags.h"

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
		//  - Inertia is not automatically derived from the collision shape, that is left to the caller.

		// World space position/orientation of the rigid body
		// This is the position of the model origin in world space (not the CoM)
		m4x4 m_o2w;

		// Stable identity copied with the body so ordinary value relocation cannot silently retarget persistent constraints.
		// Two simultaneously submitted copies are rejected as duplicate identities during endpoint remapping.
		BodyId m_id;

		// Offset from the model origin to the CoM (in object space). 
		v4 m_os_com;

		// World space spatial momentum, measured at the centre of mass.
		// For bodies with CoM at the model origin, this is equivalent to model-origin momentum.
		v8force m_ws_momentum;

		// The external forces and torques applied to this body (in world space), measured at the centre of mass.
		// This value is an accumulator and is reset to zero after each physics step so forces that should
		// be constant need to be applied each frame.
		v8force m_ws_force;

		// Each body has its own gravity vector to define local "down" for this object. This value is updated
		// via the Gravity methods and should be called each frame to apply the gravity force to the body (even static bodies).
		v4 m_ws_gravity;

		// Inverse inertia, measured at the centre of mass (CoM() == 0, block-diagonal).
		// For articulated bodies, the inertia can be shifted to joint frames using Translate()/To6x6().
		InertiaInv m_os_inertia_inv;

		// Collision shape
		collision::Shape const* m_shape;

		// Rigid body state flags
		ERigidBodyStateFlags m_state_flags;

		// Sleep state. Island membership is implementation-owned and caller-invisible.
		SleepData m_sleep;

		friend struct Engine;
		friend struct EngineBufferCache;
		friend struct BodyHistory;
		friend GpuRigidBody PackDynamics(RigidBody const& rb, int shape_id);
		friend void UnpackDynamics(GpuRigidBody const& dyn, RigidBody& rb);

		// Invalidate cached sleep island membership.
		void InvalidateSleepIsland();

		// Add force without changing sleep state.
		void AccumulateForceWS(v8force const& ws_force);

	public:

		// Construct the rigid body with a collision shape
		// Inertia is not automatically derived from the collision shape, that is left to the caller.
		template <ShapeType TShape>
		explicit RigidBody(TShape const* shape, m4x4 const& o2w = m4x4::Identity(), Inertia const& inertia = {})
			:RigidBody(shape_cast(shape), o2w, inertia)
		{}
		explicit RigidBody(collision::Shape const* shape = nullptr, m4x4 const& o2w = m4x4::Identity(), Inertia const& inertia = {});

		// Return the stable identity used by persistent constraints.
		BodyId Id() const;

		// Raised after the collision shape changes.
		EventHandler<RigidBody&, ChangeEventArgs<collision::Shape const*>, true> ShapeChange;

		// Get/Set the collision shape for the rigid body
		template <ShapeType TShape> TShape const& Shape() const;
		collision::Shape const& Shape() const;
		bool HasShape() const;
		
		// Set the shape only, leave the mass properties unchanged
		void Shape(collision::Shape const* shape);

		// Set the shape and derive mass properties from the shape.
		void Shape(collision::Shape const* shape, float mass, bool mass_is_actually_density = false);

		// Set the shape and mass properties explicitly
		void Shape(collision::Shape const* shape, Inertia inertia, v4 com = v4{});

		// Get/Set the body object to world transform
		m4x4 const& O2W() const;
		m4x4 W2O() const;
		void O2W(m4x4 const& o2w);

		// Extrapolate the position based on the current momentum and forces
		m4x4 O2W(float dt) const;

		// Return the world space bounding box for this object
		BBox BBoxWS() const;

		// The mass of the rigid body
		float Mass() const;
		void Mass(float mass);
		float InvMass() const;
		void InvMass(float invmass);

		// Return the model-origin-to-centre-of-mass offset in object space.
		v4 CentreOfMassOS() const;

		// Return the model-origin-to-centre-of-mass offset in world orientation.
		v4 CentreOfMassWS() const;

		// Return the model-origin-to-centre-of-mass offset in world orientation.
		v4 CentreOfMassOffsetWS() const;

		// Return the absolute world-space position of the centre of mass.
		v4 CentreOfMassPositionWS() const;

		// InertiaInv (use 'SetMassProperties' to change)
		InertiaInv InertiaInvOS() const;
		InertiaInv InertiaInvWS() const;
		Inertia InertiaOS() const;
		Inertia InertiaWS() const;

		// Return the inertia rotated from object space to 'A' space
		// 'com' is the position of this object's CoM in 'A' space
		Inertia InertiaOS(m3x3 const& o2a, v4 com = v4{}) const;
		InertiaInv InertiaInvOS(m3x3 const& o2a, v4 com = v4{}) const;
		Inertia InertiaOS(m4x4 const& o2a) const;
		InertiaInv InertiaInvOS(m4x4 const& o2a) const;

		// Get/Set the momentum of the rigid body
		v8force MomentumWS() const;
		v8force MomentumOS() const;
		void MomentumWS(v8force const& ws_momentum);
		void MomentumOS(v8force const& os_momentum);

		// Get/Set the velocity
		v8motion VelocityWS() const;
		v8motion VelocityOS() const;
		void VelocityWS(v8motion const& ws_velocity);
		void VelocityOS(v8motion const& os_velocity);
		void VelocityWS(v4 ws_ang, v4 ws_lin, v4 ws_at = v4{});
		void VelocityOS(v4 os_ang, v4 os_lin, v4 os_at = v4{});

		// Reset the state of the body
		void ZeroForces();
		void ZeroMomentum();

		// Apply gravity to the body. This should be called each frame to apply the gravity force
		// to the body, even for static bodies in order to define the "down" direction for the body.
		v4 GravityWS() const;
		void GravityWS(v4 ws_gravity);

		// Return the body's state flags
		ERigidBodyStateFlags StateFlags() const;

		// True if the body is flagged as asleep
		bool Sleeping() const;
		void Sleeping(bool sleeping);

		// Put the body to sleep immediately, or wake it up immediately.
		void Sleep();
		void Wake();

		// True if the body is immune to automatic sleeping
		bool NeverSleep() const;
		void NeverSleep(bool never_sleep);

		// Get/Set the current forces applied to this body (measured at the centre of mass).
		v8force ForceWS() const;
		v8force ForceOS() const;
		void ForceWS(v8force const& ws_force);

		// Add a force acting on the rigid body at position 'ws_at' (world space, model origin relative).
		// The force is shifted from the application point to the centre of mass before accumulation.
		// For gravity: pass ws_at = CentreOfMassOffsetWS() so gravity produces no torque about CoM.
		void ApplyForceWS(v4 ws_force, v4 ws_torque, v4 ws_at = v4::Zero());
		void ApplyForceWS(v8force ws_force);

		// Add a force acting on the rigid body at position 'os_at' (object space, model origin relative)
		void ApplyForceOS(v4 os_force, v4 os_torque, v4 os_at = v4::Zero());
		void ApplyForceOS(v8force const& os_force);

		// Set the mass properties of the body.
		// 'os_inertia' is the inertia for the body, measured at the model origin (not CoM) (in object space)
		// 'os_model_to_com' is the vector from the model origin to the body's centre of mass (in object space)
		void SetMassProperties(Inertia const& os_inertia, v4 os_model_to_com = v4{});

		// Return the kinetic energy of the body
		float KineticEnergy() const;
	};
}
