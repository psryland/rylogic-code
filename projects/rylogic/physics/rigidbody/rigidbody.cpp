//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/utility/globalfunctions.h"
#include "pr/physics/utility/event_types.h"
#include "pr/physics/utility/events.h"
#include "physics/utility/assert.h"
#include "physics/utility/debug.h"

using namespace pr;
using namespace pr::ph;

// Constructor
Rigidbody::Rigidbody(RigidbodySettings const& settings)
{
	Create(settings);
}

// Copy constructor
Rigidbody::Rigidbody(Rigidbody const& copy)
{
	new(this) Rigidbody(copy.GetSettings());
}

// Assignment
Rigidbody& Rigidbody::operator = (Rigidbody const& copy)
{
	Rigidbody::~Rigidbody();
	return *new(this) Rigidbody(copy);
}

// Returns creation settings for this rigidbody in it's current state
RigidbodySettings Rigidbody::GetSettings() const
{
	RigidbodySettings settings;
	settings.m_object_to_world                      = m_object_to_world;
	settings.m_shape                                = m_shape;
	settings.m_type                                 = m_type;
	settings.m_mass_properties                      .set(m_os_inertia_tensor, v4::Zero(), m_mass);
	settings.m_motion_type                          = m_motion_type;
	settings.m_initially_sleeping                   = m_sleeping;
	settings.m_lin_velocity                         = Velocity();
	settings.m_ang_velocity                         = AngVelocity();
	settings.m_user_data                            = m_user_data;
	settings.m_flags                                = m_flags;
	PR_EXPAND(PR_DBG_PHYSICS, settings.m_name       = m_name;)
	return settings;
}

// Initialise
void Rigidbody::Create(RigidbodySettings const& settings)
{
	PR_EXPAND(PR_LOG_RB, memset(m_log_buf, ' ', sizeof(m_log_buf));)
	PR_EXPAND(PR_LOG_RB, m_log = &m_log_buf[sizeof(m_log_buf) - 1];)
	PR_EXPAND(PR_LOG_RB, m_log[0] = 0;)
	m_object_to_world           = settings.m_object_to_world;
	m_shape                     = settings.m_shape;
	m_type                      = settings.m_type;
	m_bp_entity                 .init(*this, m_ws_bbox);
	m_engine_ref                .init(this);
	m_support                   .Construct();
	m_ws_bbox                   = settings.m_object_to_world * settings.m_shape->m_bbox; // World space bounding box. This is continuously updated for dynamic objects
	SetMassProperties(settings.m_mass_properties);
	SetMotionType(settings.m_motion_type);
	SetVelocity(settings.m_lin_velocity);
	SetAngVelocity(settings.m_ang_velocity);
	SetForce(settings.m_force);
	SetTorque(settings.m_torque);
	SetSleepState(settings.m_initially_sleeping);
	m_acc_impulse               = pr::v4::Zero();
	m_acc_twist                 = pr::v4::Zero();
	m_micro_mom_sq              = 0.0f;
	m_user_data                 = settings.m_user_data;
	m_flags                     = settings.m_flags;
	m_constraint_set            = NoConstraintSet;
	SetName(settings.m_name);
}

// Destructor
Rigidbody::~Rigidbody()
{
	m_support.Clear();
	if (m_bp_entity.m_broadphase) m_bp_entity.m_broadphase->Remove(m_bp_entity);
	pr::chain::Remove(m_engine_ref);
}

// Read accessor functions ************************

// Return the current resting contact points.
// 'contacts' should point to at least 3 v4's
void Rigidbody::RestingContacts(v4* contacts, uint32_t& count) const
{
	for (count = 0; count != m_support.m_num_supports; ++count)
		contacts[count] = m_support.m_leg[count].m_point + m_object_to_world.pos;
}

// Write accessor functions ************************

// Set the object to world transform for the object
void Rigidbody::SetObjectToWorld(m4x4 const& o2w)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetObjectToWorld");)
	m_object_to_world       = o2w;
	m_ws_bbox               = ObjectToWorld() * BBoxOS();
	m_ws_inv_inertia_tensor = InvInertiaTensorWS(Orientation(), m_os_inv_inertia_tensor);
	m_bp_entity.Update();
}

// Set the position of the object
void Rigidbody::SetPosition(v4 position)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetPosition");)
	v4 diff                 = position - m_object_to_world.pos;
	m_object_to_world.pos   = position;
	m_ws_bbox.m_centre      += diff;
	m_bp_entity.Update();
}

// Set the orientation of the object
void Rigidbody::SetOrientation(m3x4 const& ori)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetOrientation");)
	m_object_to_world.rot = ori;
	m_ws_bbox               = ObjectToWorld() * BBoxOS();
	m_ws_inv_inertia_tensor = InvInertiaTensorWS(Orientation(), m_os_inv_inertia_tensor);
	m_bp_entity.Update();
}

// Set the mass of a rigid body
void Rigidbody::SetMass(float mass)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetMass");)
	PR_ASSERT(PR_DBG_PHYSICS, mass > 0.0f, "");
	m_mass                  = mass;
	m_inv_mass              = (1.0f/mass > maths::tiny<float>) ? 1.0f/mass : 0.0f;
}

// Set the mass properties of a rigidbody
void Rigidbody::SetMassProperties(MassProperties const& mp)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetMassProperties");)
	SetMass(mp.m_mass);
	m_os_inertia_tensor     = mp.m_os_inertia_tensor;
	m_os_inv_inertia_tensor = Invert(mp.m_os_inertia_tensor);
	m_ws_inv_inertia_tensor = InvInertiaTensorWS(Orientation(), m_os_inv_inertia_tensor);
}

// Set the motion type of a rigidbody
void Rigidbody::SetMotionType(EMotion motion_type)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetMotionType");)
	m_motion_type = motion_type;
}

// Update the shape for a rigid body but use the old mass properties
// Also updates the transform for 'rb' since the new collision shape
// does not necessarily have the same orientation as the previous one
//  Transforms needed to maintain the same orientation:
//  m4x4 oldinertial_2_world        = ObjectToWorld();
//  m4x4 model_2_oldinertial        = from old collision model mass properties
//  m4x4 newinertial_2_model        = from new collision model mass properties
//  m4x4 newinertial_2_oldinertial  = model_2_oldinertial * newinertial_2_model;
//  m4x4 newinsertial_2_world       = oldinertial_2_world * newinertial_2_oldinertial;
void Rigidbody::SetCollisionShape(Shape* shape, m4x4 const& o2w)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetCollisionShape(1)");)
	SetObjectToWorld(o2w);
	m_shape         = shape;
	m_ws_bbox       = ObjectToWorld() * BBoxOS();
	m_bp_entity     .Update();
	RBEvent e = {this, RBEvent::EType_ShapeChanged};
	events::Send(e);
}

// Update the shape for a rigid body
// Also updates the transform for 'rb' since the new collision shape
// does not necessarily have the same orientation as the previous one
void Rigidbody::SetCollisionShape(Shape* shape, m4x4 const& o2w, MassProperties const& mp)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetCollisionShape(2)");)
	SetObjectToWorld(o2w);
	SetMassProperties(mp);
	m_shape             = shape;
	m_ws_bbox           = ObjectToWorld() * BBoxOS();
	m_bp_entity         .Update();
	RBEvent e = {this, RBEvent::EType_ShapeChanged};
	events::Send(e);
}

// Set a constant force for a rigidbody
void Rigidbody::SetForce(v4 force)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetForce");)
	m_force = force;
}

// Set a constant torque for a rigidbody
void Rigidbody::SetTorque(v4 torque)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetTorque");)
	m_torque = torque;
}

// Set the velocity of a rigidbody
void Rigidbody::SetVelocity(v4 velocity)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetVelocity");)
	m_lin_momentum = m_mass * velocity;
}

// Set the angular velocity of a rigidbody
void Rigidbody::SetAngVelocity(v4 ang_velocity)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetAngVelocity");)
	m_ang_momentum = m_mass * (InertiaTensorWS(Orientation(), m_os_inertia_tensor) * ang_velocity);
}

// Set the linear momentum of a rigidbody
void Rigidbody::SetMomentum(v4 momentum)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetMomentum");)
	m_lin_momentum = momentum;
}

// Set the angular momentum of a rigidbody
void Rigidbody::SetAngMomentum(v4 ang_momentum)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "SetAngMomentum");)
	m_ang_momentum = ang_momentum;
}

// Set the sleep status of an object.
void Rigidbody::SetSleepState(bool asleep)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, FmtS("SetSleepState<%d>", asleep));)
	m_sleeping = asleep;
	if (!m_sleeping) m_support.Clear();
}

// Set a debugger friendly name for the physics object
void Rigidbody::SetName(char const* PR_EXPAND(PR_DBG_PHYSICS, name))
{
	PR_EXPAND(PR_DBG_PHYSICS, int i = 0;)
	PR_EXPAND(PR_DBG_PHYSICS, for (; i != sizeof(m_name) - 1 && name[i]; ++i) m_name[i] = name[i];)
		PR_EXPAND(PR_DBG_PHYSICS, m_name[i] = 0;)
	}

// Impulse functions **************************************

// Apply a world space impulse
void Rigidbody::ApplyWSImpulse(v4 ws_impulse)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "ApplyWSImpulse(1)");)
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(ws_impulse, ph::OverflowValue), "");
	PR_ASSERT(PR_DBG_PHYSICS, ws_impulse.w == 0.0f, "");
	if (m_motion_type != EMotion_Dynamic) return;
	m_lin_momentum += ws_impulse;
	// If the object is asleep only wake it up once the velocity is above the micro velocity
	if (m_sleeping)
	{
		if (!HasMicroVelocity())    SetSleepState(false);
		//else                      m_lin_momentum.Zero();
	}
}

// Apply a world space twist
void Rigidbody::ApplyWSTwist(v4 ws_twist)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "ApplyWSTwist");)
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(ws_twist, ph::OverflowValue), "");
	PR_ASSERT(PR_DBG_PHYSICS, ws_twist.w == 0.0f, "");
	if (m_motion_type != EMotion_Dynamic) return;
	m_ang_momentum += ws_twist;
	// If the object is asleep only wake it up once the velocity is above the micro velocity
	if (m_sleeping)
	{
		if (!HasMicroVelocity())    SetSleepState(false);
		//else                      m_ang_momentum.Zero();
	}
}

// Apply an off-CoG impulse/twist to 'rb'
void Rigidbody::ApplyWSImpulse(v4 ws_impulse, v4 point)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "ApplyWSImpulse(2)");)
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(point, ph::OverflowValue) && IsFinite(ws_impulse, ph::OverflowValue), "");
	if (m_motion_type != EMotion_Dynamic) return;
	m_lin_momentum += ws_impulse;
	m_ang_momentum += Cross(point, ws_impulse);
	// If the object is asleep only wake it up once the velocity is above the micro velocity
	if (m_sleeping)
	{
		if (!HasMicroVelocity())    { SetSleepState(false); }
		//else                      { m_lin_momentum.Zero(); m_ang_momentum.Zero(); }
	}
}

// Impulse accumulator **************************************

// Clear the impulse accumulator
void Rigidbody::AccClearImpulse()
{
	m_acc_impulse = pr::v4::Zero();
	m_acc_twist = pr::v4::Zero();
}

// Add an impulse to the accumulator members
void Rigidbody::AccAddWSImpulse(v4 ws_impulse, v4 point)
{
	PR_EXPAND(PR_LOG_RB, Log(*this, "AddWSImpulse");)
	PR_ASSERT(PR_DBG_PHYSICS, IsFinite(point, ph::OverflowValue) && IsFinite(ws_impulse, ph::OverflowValue), "");
	PR_ASSERT(PR_DBG_PHYSICS, ws_impulse.w == 0.0f, "");
	if (m_motion_type != EMotion_Dynamic) return;
	m_acc_impulse += ws_impulse;
	m_acc_twist   += Cross(point, ws_impulse);
}

// Apply the accumulated impulses to the velocity
void Rigidbody::AccApplyWSImpulse()
{
	ApplyWSImpulse(m_acc_impulse);
	ApplyWSTwist(m_acc_twist);
	AccClearImpulse();
}
