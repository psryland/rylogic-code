//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#pragma once
#ifndef PR_PHYSICS_INTEGRATE_H
#define PR_PHYSICS_INTEGRATE_H

#include "pr/physics/types/forward.h"

namespace pr
{
	namespace ph
	{
		// Evolve a rigid body forward in time. This applies the accumulated external
		// forces and torques for 'elapsed_seconds' then resets these members
		void Evolve(Rigidbody& rb, float elapsed_seconds);

		//struct Instance : pr::chain::link<Instance, ManagedInstance>
		//{
		//	Instance();
		//	Instance(Rigidbody* rigid_body, m4x4* object_to_world);
		//
		//	// Accessors
		//	const BBox&	BBox() const						{ return m_rigid_body->m_bbox; }
		//	const BBox&	BBoxWS() const						{ return m_ws_bbox; }
		//	float				Mass() const						{ return m_rigid_body->m_mass; }
		//	const m4x4&			InvMassTensorWS() const				{ return m_ws_inv_inertia_tensor; }
		//	uint32_t				CollisionGroup() const				{ return m_collision_group; }
		//	std::size_t			NumPrimitives() const				{ return m_rigid_body->m_num_primitives; }
		//	const Model&		GetModel() const					{ return *m_rigid_body; }
		//	const Primitive*	prim_begin() const					{ return m_rigid_body->prim_begin(); }
		//	const Primitive*	prim_end() const					{ return m_rigid_body->prim_end(); }
		//	const m4x4&			ObjectToWorld() const				{ return *m_object_to_world; }
		//	v4			Velocity() const					{ return m_velocity; }
		//	v4			AngVelocity() const					{ return m_ang_velocity; }
		//	v4					VelocityAt(v4 where) const	{ return m_velocity + Cross(m_ang_velocity, where); }
		//	float				GetEnergy(v4 gravity) const;

		//	// Set methods
		//	void SetCollisionGroup(uint32_t group)						{ m_collision_group = group; }
		//	void SetRigidBody(Rigidbody* rigid_body)				{ m_rigid_body = rigid_body; }
		//	void SetObjectToWorld(m4x4* object_to_world)			{ m_object_to_world = object_to_world; }
		//	void SetVelocity(v4 velocity)					{ m_velocity = velocity; }
		//	void SetAngVelocity(v4 ang_vel);

		//	void PushOut(v4 push_distance);

		//private:
		//	void StepOrder1(float elapsed_seconds);
		//	void StepOrder2(float elapsed_seconds);
		//	void StepOrder4(float elapsed_seconds);

		//private:
		//	Rigidbody*	m_rigid_body;				// The collision shape and mass properties
		//	uint32_t		m_collision_group;			// The collision group that this instance belongs to
		//	broadphase::Entity m_bp_entity;			// Allow these objects to be added to the broadphase

		//	// State variables
		//	m4x4*		m_object_to_world;			// The transform from physics object space into world space
		//	v4			m_lin_velocity;				// The velocity of the object in world space
		//	v4			m_ang_velocity;				// The angular velocity of the object in world space
		//	v4			m_force;					// The accumulation of world space forces within a step
		//	v4			m_torque;					// The accumulation of world space torques within a step

		//	// Derived variables
		//	v4			m_ang_momentum;				// The angular momentum of the object in world space
		//	m4x4		m_ws_inv_inertia_tensor;		// The world space inverse mass tensor
		//	BBox	m_ws_bbox;					// The world space bounding box for this object. Calculated per step
		//};

		//// Implementation ***********************************************
		//
		////*****
		//// Apply an impulse at the centre of mass
		//inline void Instance::ApplyWorldImpulse(v4 force)
		//{
		//	m_force += force;
		//}

		////*****
		//// Apply a moment to the centre of mass
		//inline void Instance::ApplyWorldMoment(v4 torque)
		//{
		//	m_torque += torque;
		//}

		////*****
		//// Apply an impulse at a location relative to the centre of mass.
		//// 'impulse' and 'where' are in world space although 'where' is relative
		//// to the object centre of mass.
		//inline void	Instance::ApplyWorldImpulseAt(v4 force, v4 where)
		//{
		//	m_force	 += force;
		//	m_torque += Cross(where, force);
		//}

		////*****
		//// Apply a collision impulse. These zero the current components of force
		//// and torque in the direction of the force and torque we're about to apply
		//inline void Instance::ApplyWorldCollisionImpulseAt(v4 force, v4 where)
		//{
		//	float inward_force = Dot3(force, m_force);
		//	if( inward_force < 0.0f ) { m_force -= (inward_force / force.LengthSq()) * force; }
		//	m_force += force;

		//	v4 torque = Cross(where, force);
		//	float inward_torque = Dot3(torque, m_torque);
		//	if( inward_torque < 0.0f ) { m_torque -= (inward_torque / torque.LengthSq()) * torque; }
		//	m_torque += torque;
		//}
	}
}

#endif
