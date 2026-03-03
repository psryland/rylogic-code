//*********************************************
// Physics engine
//  Copyright (c) Rylogic Ltd 2006
//*********************************************

#include "physics/utility/stdafx.h"
#include "pr/physics/engine/engine.h"
#include "pr/physics/rigidbody/rigidbody.h"
#include "pr/physics/rigidbody/integrate.h"
#include "pr/physics/collision/contactmanifold.h"
#include "pr/physics/collision/collider.h"
#include "pr/physics/collision/icollisionobserver.h"
#include "pr/physics/terrain/iterrain.h"
#include "pr/physics/solver/resolvecollision.h"
#include "pr/physics/ray/raycast.h"
#include "pr/physics/ray/raycastresult.h"
#include "physics/broadphase/bppair.h"
#include "physics/engine/collisionagent.h"
#include "physics/engine/agentcache.h"
#include "physics/utility/assert.h"
#include "physics/utility/debug.h"
#include "physics/utility/profile.h"

using namespace pr;
using namespace pr::ph;

// An object to use when no broadphase is provided
struct NoBroadphase : IBroadphase
{
	void Add   (BPEntity&)		{}
	void Remove(BPEntity&)		{}
	void Update(BPEntity&)		{}
	void RemoveAll()			{}
	void EnumPairs(EnumPairsFunc, void*)					{}
	void EnumPairs(EnumPairsFunc, BPEntity const&, void*)	{}
	void EnumPairs(EnumPairsFunc, Ray const&, void*)		{}
} g_no_broadphase; // Dummy object to use when no broadphase is provided

// Constructors
Engine::Engine(ph::Settings const& settings)
:m_settings(settings)
,m_constraints(*this, settings.m_Allocate, settings.m_Deallocate)
{
	ConstructCommon();
}
Engine::Engine(
	IBroadphase*			broadphase,
	ITerrain*				terrain,
	IPreCollisionObserver*	pre_col,
	IPstCollisionObserver*	pst_col,
	std::size_t				constraint_buffer_size,
	std::size_t				collision_cache_size,
	pr::AllocFunction		allocate,
	pr::DeallocFunction		deallocate)
:m_constraints(*this, allocate, deallocate)
{
	m_settings.m_broadphase				= broadphase;
	m_settings.m_terrain				= terrain;
	m_settings.m_constraint_buffer_size	= constraint_buffer_size;
	m_settings.m_collision_cache_size	= collision_cache_size;
	m_settings.m_pre_col_observer		= pre_col;
	m_settings.m_pst_col_observer		= pst_col;
	m_settings.m_Allocate				= allocate;
	m_settings.m_Deallocate				= deallocate;
	ConstructCommon();
}

// Common constructor called from main constructors
void Engine::ConstructCommon()
{
	// If no broadphase system is provided, use the dummy one.
	if( !m_settings.m_broadphase )
		m_settings.m_broadphase = &g_no_broadphase;

	// Initialise the rigidbody chain
	m_rigid_bodies.init(0);

	// Allocate the constraint buffer
	m_constraints.SetBufferSize(m_settings.m_constraint_buffer_size);

	// Setup the terrain object
	RigidbodySettings terrain_settings;
	terrain_settings.m_object_to_world        = m4x4::Identity();
	terrain_settings.m_shape                  = &m_terrain_shape.set(m_settings.m_terrain, m4x4::Identity(), 0, EShapeFlags_None).m_base;
	terrain_settings.m_type                   = ERigidbody_Terrain;
	terrain_settings.m_motion_type            = EMotion_Static;
	terrain_settings.m_mass_properties.m_mass = maths::float_max;
	m_terrain_object.Create(terrain_settings);
	m_terrain_object.m_ws_bbox.m_radius      = v4Max;
	m_terrain_object.m_os_inv_inertia_tensor = m3x4::Zero();
	m_terrain_object.m_os_inertia_tensor     = m3x4Identity * maths::float_max;
	m_terrain_object.m_ws_inv_inertia_tensor = m3x4::Zero();

	m_stepping     = false;
	m_frame_number = 0;

	PR_EXPAND(PR_DBG_PHYSICS, m_time = 0.0f;)
}

// Add a rigid body to the engine
void Engine::Register(ph::Rigidbody& rigid_body)
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_stepping, "Do not modify the state of the engine during a step");

	// Add the rigid body to those we know about
	pr::chain::Insert(m_rigid_bodies, rigid_body.m_engine_ref);

	// Add the rigid body to the broad phase
	m_settings.m_broadphase->Add(rigid_body.m_bp_entity);
}

// Remove a rigid body from the engine
void Engine::Unregister(ph::Rigidbody& rigid_body)
{
	PR_ASSERT(PR_DBG_PHYSICS, !m_stepping, "Do not modify the state of the engine during a step");

	// Remove the rigid body from our list
	pr::chain::Remove(rigid_body.m_engine_ref);

	m_settings.m_broadphase->Remove(rigid_body.m_bp_entity);
}

// Return a chain link to the registered objects
Rigidbody::Link const& Engine::GetRegisteredObjects() const
{
	return m_rigid_bodies;
}

// Step all of the physics objects
void Engine::Step(float elapsed_seconds)
{{
	PR_DECLARE_PROFILE(PR_PROFILE_ENGINE_STEP, phEngineStep);
	PR_PROFILE_SCOPE(PR_PROFILE_ENGINE_STEP, phEngineStep);

	PR_EXPAND(PR_DBG_PHYSICS, m_time += elapsed_seconds);
	//PR_EXPAND(PR_DBG_PHYSICS, DebugOutput(0,0, Fmt("Engine time: %f", m_time).c_str());)

	++m_frame_number;
	m_stepping = true;

	// Integrate velocities / positions before doing collision detection
	// This will move the objects to their next position and update the bbox's in the broadphase
	for (Rigidbody::Link* s = m_rigid_bodies.begin(), *s_end = m_rigid_bodies.end(); s != s_end; s = s->m_next)
	{
		ph::Rigidbody& rb = *s->m_owner;

		// Check the sleeping status of the objects
		if( rb.m_sleeping )
		{
			if( rb.HasMicroVelocity() ) continue;
			rb.SetSleepState(false);
		}
		else if( rb.m_support.IsSupported() )
		{
			rb.SetVelocity(v4Zero);
			rb.SetAngVelocity(v4Zero);
			rb.SetSleepState(true);
			continue;
		}

		// Integrate
		Evolve(rb, elapsed_seconds);
	}

	// Initialise the constraint solver for the frame
	m_constraints.BeginFrame(elapsed_seconds);

	// Add object to object constraints
	m_settings.m_broadphase->EnumPairs(ObjectVsObjectConstraints, this);

	// Add object vs terrain constraints
	ObjectVsTerrainConstraints();

	// Add joint constraints
	JointConstraints();

	// Solve all constraints
	m_constraints.Solve();

	m_stepping = false;
}PR_PROFILE_FRAME;PR_PROFILE_OUTPUT(120);}

// Perform collision detection between two objects (ordered by their shape type)
// and add constraints to the solver for them if they are in collision
void Engine::CollisionDetection(Rigidbody const& objA, Rigidbody const& objB)
{
	PR_EXPAND(PR_DBG_COLLISION, std::string str);
	PR_EXPAND(PR_DBG_COLLISION, ldr::PhCollisionScene("scene", "FFFFFFFF", *objA.m_shape, objA.m_object_to_world, *objB.m_shape, objB.m_object_to_world, str));
	PR_EXPAND(PR_DBG_COLLISION, StringToFile(str, "C:/deleteme/collision_scene.pr_script"));

	// Do narrow phase collision detection
	ContactManifold manifold;
	Collide(*objA.m_shape, objA.m_object_to_world, *objB.m_shape, objB.m_object_to_world, manifold, &m_collision_cache);
	if( !manifold.IsOverlap() )
		return;

	// These casts are the transition from the const operation of collision
	// detection to the non-const operation of resolving collisions.
	Rigidbody& rbA = const_cast<Rigidbody&>(objA);
	Rigidbody& rbB = const_cast<Rigidbody&>(objB);

	// Notify observers about this detected collision and allow it to be ignored
	if( !NotifyPreCollision(rbA, rbB, manifold) )
		return;

	// These objects are overlapping, record a constraint for them
	m_constraints.AddContact(rbA, rbB, manifold);
}

// Add constraints for object vs. object collisions.
void Engine::ObjectVsObjectConstraints(BPPair const& pair, void* context)
{
	PR_DECLARE_PROFILE(PR_PROFILE_NARROW_PHASE, phObjVsObjConstraints);
	PR_PROFILE_SCOPE(PR_PROFILE_NARROW_PHASE, phObjVsObjConstraints);

	Engine* This = static_cast<Engine*>(context);
	Rigidbody const& ownerA = pair.m_objectA->owner<Rigidbody>();
	Rigidbody const& ownerB = pair.m_objectB->owner<Rigidbody>();
	if( ownerA.m_shape->m_type <= ownerB.m_shape->m_type )
		This->CollisionDetection(ownerA, ownerB);
	else
		This->CollisionDetection(ownerB, ownerA);
}

// Bounding sphere terrain intercept test
bool BoundingSphereTest(const terrain::Result&, void* context)
{
	*static_cast<bool*>(context) = true;
	return false;
}

// Add constraints for object verses terrain collisions
void Engine::ObjectVsTerrainConstraints()
{
	PR_DECLARE_PROFILE(PR_PROFILE_TERR_COLLISION, phObjVsTerrConstraints);
	PR_PROFILE_SCOPE(PR_PROFILE_TERR_COLLISION, phObjVsTerrConstraints);

	// No terrain system provided
	if( !m_settings.m_terrain ) return;

	// Test each rigidbody against the terrain system
	for (Rigidbody::Link *s = m_rigid_bodies.begin(), *s_end = m_rigid_bodies.end(); s != s_end; s = s->m_next)
	{
		ph::Rigidbody& rb = *s->m_owner;

		// Do a bounding sphere test first
		terrain::Sample point;
		point.m_point  = rb.m_ws_bbox.Centre();
		point.m_radius = Length(rb.m_ws_bbox.Radius());
		bool bounds_contact = false;
		m_settings.m_terrain->CollideSpheres(&point, 1, BoundingSphereTest, &bounds_contact);
		if( !bounds_contact ) continue;

		if( rb.m_shape->m_type <= m_terrain_object.m_shape->m_type )
			CollisionDetection(rb, m_terrain_object);
		else
			CollisionDetection(m_terrain_object, rb);
	}
}

// Add constraints for joints between objects
void Engine::JointConstraints()
{
	// ToDo.
}

// Collision observers
bool Engine::NotifyPreCollision(ph::Rigidbody const& rbA, ph::Rigidbody const& rbB, ph::ContactManifold& manifold)
{
	return !m_settings.m_pre_col_observer ||
			((rbA.m_flags & ERBFlags_PreCol) == 0 && (rbB.m_flags & ERBFlags_PreCol) == 0) ||
			m_settings.m_pre_col_observer->NotifyPreCollision(rbA, rbB, manifold);
}
void Engine::NotifyPstCollision(ph::Rigidbody const& rbA, ph::Rigidbody const& rbB, ph::ContactManifold const& manifold)
{
	!m_settings.m_pst_col_observer ||
	((rbA.m_flags & ERBFlags_PstCol) == 0 && (rbB.m_flags & ERBFlags_PstCol) == 0) ||
	(m_settings.m_pst_col_observer->NotifyPstCollision(rbA, rbB, manifold), true);
}

// Cast a ray into the physics world. Returns true if the ray hits something
bool Engine::RayCast(Ray const& ray, RayVsWorldResult& result)
{
	result.m_intercept = 1.0f;
	m_settings.m_broadphase->EnumPairs(RayCastCollisionDetection, ray, &result);
	return result.m_intercept != 1.0f;
}

// Test a ray against objects that overlap the ray returned from the broadphase
void Engine::RayCastCollisionDetection(BPPair const& pair, void* context)
{
	RayVsWorldResult*	result	= static_cast<RayVsWorldResult*>(context);
	Rigidbody const&	objA	= pair.m_objectA->owner<Rigidbody>();
	Ray const&			ray		= *static_cast<Ray const*>(pair.m_objB_void);

	// Test the ray against the shape of objA
	RayCastResult cast;
	if( RayCastWS(ray, *objA.m_shape, objA.ObjectToWorld(), cast) )
	{
		if( cast.m_t0 < result->m_intercept )
		{
			result->m_intercept		= cast.m_t0;
			result->m_normal		= cast.m_normal;
			result->m_object		= &objA;
			result->m_shape			= cast.m_shape;
		}
	}
}

// Event handling
void Engine::OnEvent(RBEvent const& e)
{
	switch( e.m_type )
	{
	case RBEvent::EType_ShapeChanged:
		break;
	default:
		PR_ASSERT(PR_DBG_PHYSICS, false, "Unhandled rigid body event type");
	}
}
//
//// Perform narrow phase collision detection on pairs returned from the broadphase
//void Engine::NarrowPhaseCollisionDetection(BPPair const& pair, void* context)
//{
//	PR_DECLARE_PROFILE(PR_PROFILE_NARROW_PHASE, phNarrowPhase);
//	PR_PROFILE_SCOPE(PR_PROFILE_NARROW_PHASE, phNarrowPhase);
//
//	Engine* This = static_cast<Engine*>(context);
//
//	Rigidbody const& objA = pair.m_objectA->owner<Rigidbody>();
//	Rigidbody const& objB = pair.m_objectB->owner<Rigidbody>();
//
//	// Get a collision agent for this collision
//	// and perform narrow phase collision detection
//	CollisionAgent& agent = This->m_agent_cache.GetAgent(objA, objB, This->m_frame_number, &This->m_collision_cache);
//	agent.DetectCollision();
//	if( agent.m_manifold.IsOverlap() )
//	{
//		// These casts are the transition from the const operation of collision
//		// detection to the non-const operation of resolving collisions.
//		Rigidbody& rbA = *const_cast<Rigidbody*>(agent.m_objectA);
//		Rigidbody& rbB = *const_cast<Rigidbody*>(agent.m_objectB);
//
//		// Check for micro collisions
//		LookForSupports(agent.m_manifold.GetContact(), rbA, rbB);
//
//		if( agent.m_manifold.IsContact() )
//		{
//			// Notify observers about this detected collision.
//			// Allow the collision to be ignored
//			if( !This->NotifyPreCollision(rbA, rbB, agent.m_manifold) )
//				return;
//
//			// If a collision was detected, resolve it
//			ResolveCollision(rbA, rbB, agent.m_manifold);
//
//			// Notify observers about a resolved collision.
//			This->NotifyPstCollision(rbA, rbB, agent.m_manifold);
//		}
//
//		// Use dodgy push out for now until the constraint solver works
//		PushOut(rbA, rbB, agent.m_manifold.GetContact());
//	}
//}
//
//// Test the objects against the terrain
//void Engine::TerrainCollisionDetection()
//{
//	PR_DECLARE_PROFILE(PR_PROFILE_TERR_COLLISION, phTerrainColDet);
//	PR_PROFILE_SCOPE(PR_PROFILE_TERR_COLLISION, phTerrainColDet);
//
//	if( !m_settings.m_terrain ) return;
//
//	for( pod_chain::link *s = m_rigid_bodies.begin(), *s_end = m_rigid_bodies.end(); s != s_end; s = s->m_next )
//	{
//		ph::Rigidbody& rb = s->owner<ph::Rigidbody>();
//
//		// Do a bounding sphere test first
//		terrain::Sample point;
//		point.m_point  = rb.m_ws_bbox.Centre();
//		point.m_radius = rb.m_ws_bbox.Radius().Length();
//		bool bounds_contact = false;
//		m_settings.m_terrain->CollideSpheres(&point, 1, BoundingSphereTest, &bounds_contact);
//		if( bounds_contact )
//		{
//			// Get an agent for object vs. terrain collision and do collision detection
//			CollisionAgent& agent = m_agent_cache.GetAgent(rb, m_terrain_object, m_frame_number, &m_collision_cache);
//			agent.DetectCollision();
//			if( agent.m_manifold.IsOverlap() )
//			{
//				// These casts are the transition from the const operation of collision
//				// detection to the non-const operation of resolving collisions.
//				Rigidbody& rbA = *const_cast<Rigidbody*>(agent.m_objectA);
//				Rigidbody& rbB = *const_cast<Rigidbody*>(agent.m_objectB);
//
//				// Check for micro collisions
//				LookForSupports(agent.m_manifold.GetContact(), rbA, rbB);
//
//				if( agent.m_manifold.IsContact() )
//				{
//					// Notify observers about this detected collision.
//					// Allow the collision to be ignored
//					if( !NotifyPreCollision(rb, m_terrain_object, agent.m_manifold) )
//						return;
//
//					// If a collision was detected, resolve it
//					ResolveCollision(rbA, rbB, agent.m_manifold);
//
//					// Notify observers about a resolved collision.
//					NotifyPstCollision(rb, m_terrain_object, agent.m_manifold);
//				}
//
//				// Use dodgy push out for now until the constraint solver works
//				PushOut(rbA, rbB, agent.m_manifold.GetContact());
//			}
//		}
//	}
//}

//{
//
//	// Don't step multiple times within this function as it doesn't allow the client
//	// to apply impulses between each step. This would cause bouncing for objects that
//	// should receive a continuous stream of impulses.
//
//	m_broadphase->Step();
//
//	// Ask the client to generate the potentially colliding pairs. This should
//	// include potential object vs. terrain pairs as well
//	m_settings.m_BroadphaseCollisionDetection(m_instances);
//
//	// Perform narrow phase detection on each pair
//	CollisionPair* pair;
//	CollisionPair* collision_list = 0;
//	while( m_settings.m_GetCollisionPair(pair) )
//	{
//		bool is_collision;
//		if( pair->m_objB )	{ is_collision = IsCollision       (*(pair->m_objA) ,*(pair->m_objB) ,pair->m_contact); }
//		else				{ is_collision = IsTerrainCollision(*(pair->m_objA)                  ,pair->m_contact); }
//		if( is_collision && pair->CalculatePostDetectionData() )
//		{
//			pair->m_next_collision = collision_list;
//			collision_list = pair;
//		}
//	}
//
//	// Resolve collisions
//	for( ; collision_list; collision_list = collision_list->m_next_collision )
//	{
//		if( collision_list->m_objB )	{ ResolveCollision(*(collision_list->m_objA) ,*(collision_list->m_objB) ,collision_list->m_contact); }
//		else							{ ResolveCollision(*(collision_list->m_objA)                            ,collision_list->m_contact); }
//	}
//
//	// Evolve the instances forward in time
//	for( ph::TInstanceChain::iterator i = m_instances.begin(), i_end = m_instances.end(); i != i_end; ++i )
//	{
//		i->Step(elapsed_seconds);
//	}
//}

////*****
//// Detect collisions between physics objects.
//bool Engine::IsCollision(ph::Instance& objectA, ph::Instance& objectB, ph::Contact& contact) const
//{
//	contact.clear();
//
//	// Test each primitive against every other primitive
//	for( const Primitive *pa = objectA.prim_begin(), *pa_end = objectA.prim_end(); pa != pa_end; ++pa )
//	{
//		for( const Primitive *pb = objectB.prim_begin(), *pb_end = objectB.prim_end(); pb != pb_end; ++pb )
//		{
//			PrimitiveCollision(*pa, objectA.ObjectToWorld(), *pb, objectB.ObjectToWorld(), contact);
//		}
//	}
//
//	return contact.m_depth > 0.0f;
//}
//
////*****
//// Detect collision between an object and the terrain system.
//bool Engine::IsTerrainCollision(ph::Instance& object, ph::Contact& contact) const
//{
//	contact.clear();
//
//	// Test each primitive against the terrain and find the contact points.
//	for( const Primitive *pa = object.prim_begin(), *pa_end = object.prim_end(); pa != pa_end; ++pa )
//	{
//		switch( pa->m_type )
//		{
//		case EPrimitive_Box:		TerrainCollisionBox		(*pa, object.ObjectToWorld(), contact); break;
//		case EPrimitive_Cylinder:	TerrainCollisionCylinder(*pa, object.ObjectToWorld(), contact); break;
//		case EPrimitive_Sphere:		TerrainCollisionSphere	(*pa, object.ObjectToWorld(), contact); break;
//		default: PR_ASSERT(PR_DBG_PHYSICS, false, "Unknown primitive type");
//		}
//	}
//
//	return contact.m_depth > 0.0f;
//}
//
//
////*****
//// Resolve a collision between objectA and objectB using 'contact'. If 'objectB' is 0 then it is
//// assumed to be an infinite mass object. The collision is resolved by _setting_ the impulses
//// in objectA and objectB so that they will move out of collision. Note: CalculatePostDetectionData()
//// must have been called to fill out the extra data in 'contact' before passing it to this function
//void Engine::ResolveCollision(ph::Instance* objectA, ph::Instance* objectB, ph::Contact& contact) const
//{
//	// "mass" is a matrix defined as impulse = mass * drelative_velocity.
//	// "inv_mass" is also called the 'K' matrix and is equal to:
//	//	[(1/MassA + 1/MassB)*Identity - (pointA.CrossProductMatrix()*InvMassTensorWS()A*pointA.CrossProductMatrix() + pointB.CrossProductMatrix()*InvMassTensorWS()B*pointB.CrossProductMatrix())]
//	// Say "inv_mass" = "inv_mass1" + "inv_mass2" then
//	// "inv_mass1" = [(1/MassA)*Identity - (pointA.CrossProductMatrix()*InvMassTensorWS()A*pointA.CrossProductMatrix())] and
//	// "inv_mass2" = [(1/MassB)*Identity - (pointB.CrossProductMatrix()*InvMassTensorWS()B*pointB.CrossProductMatrix())]
//	m4x4 pointAcross = contact.m_pointA.CrossProductMatrix();
//	m4x4 inv_mass1 = (1.0f / objectA->Mass()) * m4x4::Identity() - (pointAcross * objectA->InvMassTensorWS() * pointAcross);
//	m4x4 inv_mass2;
//	if( objectB )
//	{
//		m4x4 pointBcross = contact.m_pointB.CrossProductMatrix();
//		inv_mass2 = (1.0f / objectB->Mass()) * m4x4::Identity() - (pointBcross * objectB->InvMassTensorWS() * pointBcross);
//	}
//	else
//	{
//		inv_mass2 = m4x4::Zero();
//	}
//
//	m4x4 inv_mass = inv_mass1 + inv_mass2; inv_mass[3][3] = 1.0f;
//	m4x4 mass = inv_mass.Invert();
//
//	const ph::Material& materialA = PhysicsMaterial(contact.m_material_indexA);
//	const ph::Material& materialB = PhysicsMaterial(contact.m_material_indexB);
//
//	float elasticity_n		= Minimum<float>(materialA.m_elasticity,			materialB.m_elasticity);
//	float elasticity_t		= Minimum<float>(materialA.m_tangential_elasticity,	materialB.m_tangential_elasticity);
//	float static_friction	= Maximum<float>(materialA.m_static_friction,		materialB.m_static_friction);
//	float dynamic_friction	= Maximum<float>(materialA.m_dynamic_friction,		materialB.m_dynamic_friction);
//	bool norm_resting_contact = contact.m_rel_norm_speed < m_settings.m_max_resting_speed;
//	bool tang_resting_contact = contact.m_rel_tang_speed < m_settings.m_max_resting_speed;
//
//	float rel_velocity_n = Dot3(contact.m_normal,  contact.m_relative_velocity);
//	float rel_velocity_t = Dot3(contact.m_tangent, contact.m_relative_velocity);
//	if( norm_resting_contact )
//	{
//		elasticity_n = 1.0f - (elasticity_n - 1.0f) * rel_velocity_n / m_settings.m_max_resting_speed;
//	}
//	if( tang_resting_contact )
//	{
//		elasticity_t = -1.0f + (elasticity_t + 1.0f) * rel_velocity_t / m_settings.m_max_resting_speed;
//	}
//
//	// Pi is the impulse required to reduce the normal component of rel_velocity to zero.
//	// Pii is the impulse to reduce rel_velocity to zero
//	// See article: A New Algebraic Rigid Body Collision Law Based On Impulse Space Considerations
//	v4 Pi = -(rel_velocity_n / Dot3(contact.m_normal, inv_mass * contact.m_normal)) * contact.m_normal;
//	v4 Pii = -(mass * contact.m_relative_velocity);
//	v4 Pdiff = Pii - Pi;
//
//	v4 impulse = (1.0f + elasticity_n) * Pi + (1.0f + elasticity_t) * Pdiff;
//
//	// Clip this impulse to the friction cone
//	float impulse_n = Dot3(contact.m_normal,  impulse);
//	float impulse_t = Dot3(contact.m_tangent, impulse);
//	if( Abs(impulse_t) > static_friction * impulse_n )
//	{
//		float kappa = dynamic_friction * (1.0f + elasticity_n) * Dot3(contact.m_normal, Pi) /
//			(Abs(Dot3(contact.m_tangent, Pii)) - dynamic_friction * Dot3(contact.m_normal, Pdiff));
//
//		impulse = (1.0f + elasticity_n) * Pi + kappa * Pdiff;
//	}
//
//	// Apply the collision impulses
//	objectA->ApplyWorldCollisionImpulseAt(-m_inv_time_step * impulse, contact.m_pointA);
//	if( objectB ) objectB->ApplyWorldCollisionImpulseAt(m_inv_time_step * impulse, contact.m_pointB);
//
//	// Push the objects out of penetration
//	if( norm_resting_contact )
//	{
//		float dist = Minimum<float>(contact.m_depth, m_settings.m_max_push_out_distance);
//
//		float fracA = 1.0f;
//		float fracB = 0.0f;
//		if( objectB )
//		{
//			float total_mass = objectA->Mass() + objectB->Mass();
//			fracA = objectB->Mass() / total_mass;
//			fracB = objectA->Mass() / total_mass;
//		}
//
//		v4 distA = -dist * fracA * contact.m_normal;
//		objectA->PushOut(distA);
//
//		if( objectB )
//		{
//			v4 distB = dist * fracB * contact.m_normal;
//			objectB->PushOut(distB);
//		}
//
//
//		// Reduce the angular momentum in the direction of the normal
////PSR...		float ang_mom_dot_normal = contact.m_normal.Dot3(objectA->m_ang_momentum);
////PSR...		if( ang_mom_dot_normal > 0.0f )
////PSR...		{
////PSR...			objectA->m_ang_momentum -= 0.1f * ang_mom_dot_normal * contact.m_normal;
////PSR...		}
////PSR...		else
////PSR...		{
////PSR...			objectA->m_ang_momentum += 0.1f * ang_mom_dot_normal * contact.m_normal;
////PSR...		}
//	}
//
//	// Test for sleeping condition
//	if( norm_resting_contact && tang_resting_contact )
//	{
//		// Add support and test for sleeping
//	}
//}
//
///*
////*****
//// Resolve a resting contact. This is done by zeroing the components of force/torque
//// in the ...
//void Engine::ResolveRestingContact(CollisionData& data)
//{
//	data;
////REMEMBER CONTACT FORCES...
//// If a contact is not moving toward the contact normal but is accelerating toward the contact normal
//// then this is a contact force.
//}
//*/
