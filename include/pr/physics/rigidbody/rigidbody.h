//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once

#include "pr/physics/types/forward.h"
#include "pr/physics/shape/shape.h"
#include "pr/physics/broadphase/bpentity.h"
#include "pr/physics/rigidbody/support.h"
#include "pr/physics/engine/igravity.h"

#ifndef PR_EXPAND
#   define PR_EXPAND_DEFINED
#   define PR_EXPAND(grp, exp)
#endif
#ifndef PR_LOG_RB
#   define PR_LOG_RB 0
#endif

namespace pr
{
	namespace ph
	{
		enum ERigidbody
		{
			ERigidbody_Dynamic,
			ERigidbody_Static,
			ERigidbody_Terrain
		};

		enum ERBFlags
		{
			ERBFlags_None   =      0,
			ERBFlags_PreCol = 1 << 0,   // Notify for pre collision
			ERBFlags_PstCol = 1 << 1,   // Notify for post collision
		};

		// Data used to initialise a rigid body
		struct RigidbodySettings
		{
			m4x4            m_object_to_world;          //
			Shape*          m_shape;                    // The collision shape of the rigid body
			ERigidbody      m_type;                     // The type of rigidbody to create
			MassProperties  m_mass_properties;          //
			EMotion         m_motion_type;              //
			bool            m_initially_sleeping;       //
			v4              m_lin_velocity;             //
			v4              m_ang_velocity;             //
			v4              m_force;                    //
			v4              m_torque;                   //
			void*           m_user_data;                //
			uint32_t        m_flags;                    // Bitwise OR of ERBFlags
			const char*     m_name;

			RigidbodySettings()
			{
				m_object_to_world                       = m4x4::Identity();
				m_shape                                 = GetDummyShape();
				m_type                                  = ERigidbody_Dynamic;
				m_mass_properties.m_mass                = 10.0f;
				m_mass_properties.m_centre_of_mass      = pr::v4::Origin();
				m_mass_properties.m_os_inertia_tensor   = m3x4::Identity();
				m_motion_type                           = EMotion_Dynamic;
				m_initially_sleeping                    = false;
				m_lin_velocity                          = pr::v4::Zero();
				m_ang_velocity                          = pr::v4::Zero();
				m_force                                 = pr::v4::Zero();
				m_torque                                = pr::v4::Zero();
				m_user_data                             = 0;
				m_flags                                 = ERBFlags_None;
				m_name                                  = "";
			}
		};

		// A rigid body
		struct Rigidbody
		{
			typedef pr::chain::Link<Rigidbody> Link;

			Rigidbody(RigidbodySettings const& settings = RigidbodySettings());
			Rigidbody(Rigidbody const& copy);
			Rigidbody& operator = (Rigidbody const& copy);
			RigidbodySettings GetSettings() const;
			void Create(RigidbodySettings const& settings);
			~Rigidbody();

			// Read Access Functions ******************************
			ERigidbody      Type() const                            { return m_type; }
			m4x4 const&     ObjectToWorld() const                   { return m_object_to_world; }
			v4       Position() const                        { return m_object_to_world.pos; }
			m3x4 const&     Orientation() const                     { return m_object_to_world.rot; }
			Shape const*    GetShape() const                        { return m_shape; }
			float           Mass() const                            { return m_mass; }
			EMotion         MotionType() const                      { return m_motion_type; }
			v4              Momentum() const                        { return m_lin_momentum; }
			v4              AngMomentum() const                     { return m_ang_momentum; }
			v4              Velocity() const                        { return m_inv_mass * Momentum(); }
			v4              AngVelocity() const                     { return m_inv_mass * (m_ws_inv_inertia_tensor * AngMomentum()); }
			v4              VelocityAt(v4 ws_offset) const   { return Velocity() + Cross(AngVelocity(), ws_offset); }
			BBox            BBoxWS() const                          { return m_ws_bbox; }
			BBox            BBoxOS() const                          { return GetShape()->m_bbox; }
			m3x4            InertiaOS() const                       { return m_os_inertia_tensor; }
			void*           UserData() const                        { return m_user_data; }
			MassProperties  GetMassProperties() const               { MassProperties mp = {m_os_inertia_tensor, v4::Zero(), Mass()}; return mp; }
			v4              Gravity() const                         { return GetGravitationalAcceleration(m_object_to_world.pos); }
			float           Energy() const                          { return PotentialEnergy() + KineticEnergy(); }             // mgh + 0.5mv^2 + 0.5wIw
			float           PotentialEnergy() const                 { return -Dot3(Gravity(), Position()); }                    // mgh
			float           KineticEnergy() const                   { return LinearKineticEnergy() + AngularKineticEnergy(); }  // 0.5mv^2 + 0.5wIw
			float           LinearKineticEnergy() const             { return 0.5f * Mass() * LengthSq(Velocity()); }           // 0.5mv^2
			float           AngularKineticEnergy() const            { return 0.5f * Dot3(AngVelocity(), AngMomentum()); }       // 0.5wIw
			bool            SleepState() const                      { return m_sleeping; }
			bool            HasMicroVelocity() const                { return m_motion_type == EMotion_Static || (LengthSq(Momentum()) < m_micro_mom_sq && LengthSq(AngMomentum()) < m_micro_mom_sq); }
			void            RestingContacts(v4* contacts, uint32_t& count) const;

			// Write Access Functions ******************************
			void            SetObjectToWorld(m4x4 const& o2w);
			void            SetPosition(v4 position);
			void            SetOrientation(m3x4 const& ori);
			void            SetMass(float mass);
			void            SetMassProperties(MassProperties const& mp);
			void            SetMotionType(EMotion motion_type);
			void            SetCollisionShape(Shape* shape, m4x4 const& o2w);
			void            SetCollisionShape(Shape* shape, m4x4 const& o2w, MassProperties const& mp);
			void            SetVelocity(v4 velocity);
			void            SetAngVelocity(v4 ang_velocity);
			void            SetMomentum(v4 momentum);
			void            SetAngMomentum(v4 ang_momentum);
			void            SetForce(v4 force);
			void            SetTorque(v4 torque);
			void            SetSleepState(bool asleep);
			void            SetName(char const* name);

			// Impulse functions ******************************
			void            ApplyWSImpulse(v4 ws_impulse);
			void            ApplyWSTwist(v4 ws_twist);
			void            ApplyWSImpulse(v4 ws_impulse, v4 point);

#ifndef PR_PH_BUILD
		private:
#endif//PR_PH_BUILD

			// Impulse accumulator ******************************
			v4              AccMomentum() const                     { return m_lin_momentum + m_acc_impulse; }
			v4              AccAngMomentum() const                  { return m_ang_momentum + m_acc_twist; }
			v4              AccVelocity() const                     { return m_inv_mass * AccMomentum(); }
			v4              AccAngVelocity() const                  { return m_inv_mass * (m_ws_inv_inertia_tensor * AccAngMomentum()); }
			v4              AccVelocityAt(v4 ws_offset) const { return AccVelocity() + Cross(AccAngVelocity(), ws_offset); }
			void            AccClearImpulse();
			void            AccAddWSImpulse(v4 ws_impulse, v4 point);
			void            AccApplyWSImpulse();

			// DO NOT USE THESE MEMBERS DIRECTLY, use the access functions/methods
			m4x4            m_object_to_world;          // Object to world transform
			Shape*          m_shape;                    // The shape of this model.
			ERigidbody      m_type;                     // Physics object type
			BPEntity        m_bp_entity;                // So that this object can be added to a broadphase
			Link            m_engine_ref;               // Used to chain these together within the physics engine
			Support         m_support;                  // Support data

			// Bounds
			//BBox   m_ms_bbox;  (get from shape now)// The model space bounding box for this object.
			BBox     m_ws_bbox;                  // World space bounding box. This is continuously updated for dynamic objects

			// Mass properties
			m3x4            m_os_inertia_tensor;        // The object space inertia tensor
			m3x4            m_os_inv_inertia_tensor;    // The object space inverse inertia tensor
			m3x4            m_ws_inv_inertia_tensor;    // The world space inverse inertia tensor. Calculated per step
			float           m_mass;                     // The mass of the object
			float           m_inv_mass;                 // The inverse mass

			// Dynamics
			EMotion         m_motion_type;              // Static, Keyframed, or Dynamic
			v4              m_lin_momentum;             // The linear momentum of the object in world space
			v4              m_ang_momentum;             // The angular momentum of the object in world space
			v4              m_force;                    // Constant force applied to this object
			v4              m_torque;                   // Constant torque applied to this object
			v4              m_acc_impulse;              // Accumulative impulses calculated during collision resolution
			v4              m_acc_twist;                // Accumulative twists calculated during collision resolution
			bool            m_sleeping;                 // True when this object is asleep
			float           m_micro_mom_sq;             // Micro momentum (sq) threshold

			// Miscellaneous
			void*           m_user_data;                // User data
			unsigned int    m_flags;                    // Flags
			uint8_t         m_constraint_set;           // An id for the constraint set this object belongs to

			// Debugging
			char            m_name[64];                 // A name for the physics object

			// todo: move this out of the interface
			PR_EXPAND(PR_LOG_RB, char m_log_buf[512];)  // A buffer for the log data
			PR_EXPAND(PR_LOG_RB, char* m_log;)          // A pointer into the log buffer (grows from the end)
		};
	}
}

#ifdef PR_EXPAND_DEFINED
#   undef PR_EXPAND_DEFINED
#   undef PR_EXPAND
#endif
