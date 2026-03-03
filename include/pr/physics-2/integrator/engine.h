//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once

#include "pr/physics-2/forward.h"
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/integrator/contact.h"
#include "pr/physics-2/integrator/integrator.h"
#include "pr/physics-2/integrator/impulse.h"
#include "pr/physics-2/material/material_map.h"
#include "pr/physics-2/utility/ldraw.h"

namespace pr::physics
{
	// ToDo:
	//  - Make use of sub step collision time
	//  - Stop Evolve adding energy to the system (higher order integrating?)
	//  - Use spatial vectors for impulse restitution
	//  - Optimise the impulse restitution function

	// A container object that groups the parts of a physics system together
	template <typename TBroadphase, typename TMaterials = MaterialMap>
	struct Engine
	{
		TBroadphase m_broadphase;
		TMaterials m_materials;

		// Raised after collision detection, but before resolution.
		// The collisions list can be modified by event handlers.
		EventHandler<Engine&, std::vector<Contact>&> PostCollisionDetection;

		// Evolve the physics objects forward in time and resolve any collisions
		template <typename TRigidBodyCont>
		void Step(float dt, TRigidBodyCont& bodies)
		{
			// Before here, callers should have set forces on the rigid bodies (including gravity).
			// Todo: A lot of this could be done in parallel/pipelined...

			// Advance all bodies to 't + dt'
			for (auto& body : bodies)
				Evolve(body, dt);

			// Perform collision detection
			std::vector<Contact> collision_queue;
			m_broadphase.EnumOverlappingPairs([&](void*, RigidBody const& objA, RigidBody const& objB)
			{
				// Narrow phase collision detection
				auto c = Contact{objA, objB};
				if (NarrowPhaseCollision(dt, c))
				{
					Dump(c);
					collision_queue.push_back(c);
				}
			});

			// Sort the collisions by time
			// Todo: for parallel processing, collision queue should be broken up into islands of objects that affect each other.
			// Assign an increasing 'island' number to each Contact. If a preceding contact involves one of the same objects, use the lowest number
			std::sort(std::begin(collision_queue), std::end(collision_queue), [](auto& lhs, auto& rhs){ return lhs.m_time < rhs.m_time; });

			// Notify of detected collisions, and allow updates/additions
			PostCollisionDetection(*this, collision_queue);

			// Resolve collisions
			for (auto& c : collision_queue)
				ResolveCollision(c);
		}

		// Narrow phase collision detection
		// Returns true if 'objA' and 'objB' are in contact
		// 'c' is only valid if the function returns true.
		bool NarrowPhaseCollision(float dt, Contact& c)
		{
			// Notes:
			//  t0 = 't', t1 = 't + dt'. Objects are currently at t1.
			
			auto& objA = *c.m_objA;
			auto& objB = *c.m_objB;

			// Perform the narrow phase collision detection in 'objA' space to reduce floating point errors
			if (!collision::Collide(objA.Shape(), m4x4::Identity(), objB.Shape(), c.m_b2a, c))
				return false;

			// If the collision point is moving out of collision, ignore the collision.
			auto rel_vel_at_point = c.m_velocity.LinAt(c.m_point);
			if (Dot(rel_vel_at_point, c.m_axis) > 0)
				return false;

			// Get the combined material properties of the contact
			// Todo: in the previous implementation, micro velocity collisions changed the material properties.
			c.m_mat = m_materials(c.m_mat_idA, c.m_mat_idB);

			// Determine the parametric value for the time of the collision by estimating the A-space position
			// of 'c.m_point' at t0 assuming linear velocity (because it's faster and easier)
			auto point_at_t0 = c.m_point - dt * c.m_velocity.LinAt(c.m_point);
			
			// The distance from 'point_at_t0' to 'point_at_t1' in the direction of 'c.m_axis'
			auto distance = Abs(Dot(c.m_point - point_at_t0, c.m_axis));

			// The collision time is a negative value so that position_at_collision = position_now + time * velocity_now
			// If 'c.m_depth' is zero then the point in only just in collision.
			// If 'c.m_depth' = 'distance' then the point was in collision in the last frame.
			auto sub_step = distance > c.m_depth ? -c.m_depth / distance : 0.0f;

			// Adjust the collision point and relative transform to the collision time
			c.update(sub_step * dt);
			return true;
		}

		// Calculate and apply forces to resolve the contact between the objects in 'pair'
		// Contact values in 'pair' are expected to be in objA space.
		void ResolveCollision(Contact const& c)
		{
			// Algorithm:
			//  - Extrapolate back (using the current dynamics) to the time of collision
			//  - Resolve the collision with impulses
			//  - Extrapolate to the original 't + dt'
			// Notes:
			//  An impulse is an instantaneous change in momentum.

			// Const cast is needed here because collision detection is a read-only process
			// but now when want to change the objects that were detected as in collision.
			auto& objA = const_cast<RigidBody&>(*c.m_objA);
			auto& objB = const_cast<RigidBody&>(*c.m_objB);

			#if PR_DBG
			//auto vel_beforeA = objA.VelocityWS();
			//auto vel_beforeB = objB.VelocityWS();
			//auto ke_beforeA = objA.KineticEnergy();
			//auto ke_beforeB = objB.KineticEnergy();
			//auto h_before = objA.MomentumWS() + objB.MomentumWS();
			#endif

			// Calculate the world space restitution impulse
			auto impulse_pair = RestitutionImpulse(c);

			// Apply the impulse to the objects
			objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);

			{
				auto c2 = c;
				c2.update(0);

				using namespace pr::rdr12::ldraw;
				Builder builder;
				builder._<LdrRigidBody>("body0", 0x80FF0000).rigid_body(objA).flags(ERigidBodyFlags::None);
				builder._<LdrRigidBody>("body1", 0x8000FF00).rigid_body(objB).flags(ERigidBodyFlags::None).o2w(c.m_b2a);
				#if 0 //TODO
				builder.Arrow(str, "Normal", 0xFFFFFFFF, ldr::EArrowType::Fwd, c.m_point_at_t, c.m_axis * 0.1f, 5);
				builder.VectorField(str, "VelocityBefore", 0xFFFFFF00, (v8)c.m_velocity * 0.1f, v4Origin, 2, 0.25f);
				ldr::VectorField(str, "VelocityAfter", 0xFF00FFFF, (v8)c2.m_velocity * 0.1f, v4Origin, 2, 0.25f);
				#endif
				//ldr::VectorField(str, "ImpulseField", 0xFF007F00, (v8)impulse * 0.1f, v4Origin, 5, 0.25f);
				//ldr::Arrow(str, "Impulse", 0xFF0000FF, ldr::EArrowType::Fwd, c.m_point - impulse.lin, impulse.lin, 3);
				//ldr::Arrow(str, "Twist", 0xFF000080, ldr::EArrowType::Fwd, c.m_point - impulse.ang, impulse.ang, 3);
				builder.Save(L"\\dump\\collision.ldr");
			}

			// Collisions should not add energy to the system and momentum should be conserved.
			#if PR_DBG
		//	auto vel_afterA = objA.VelocityWS();
		//	auto vel_afterB = objB.VelocityWS();
		//	auto ke_afterA  = objA.KineticEnergy();
		//	auto ke_afterB  = objB.KineticEnergy();
		//	auto ke_diff    = (ke_afterA + ke_afterB) - (ke_beforeA + ke_beforeB);
		//	auto h_after    = objA.MomentumWS() + objB.MomentumWS();
		//	auto h_ang_diff = Length(h_after.ang) - Length(h_before.ang);
		//	auto h_lin_diff = Length(h_after.lin) - Length(h_before.lin);
		//	assert("Collision caused an increase in angular momentum" && h_ang_diff <= 0);
		//	assert("Collision caused an increase in linear momentum" && h_lin_diff <= 0);
		//	assert("Collision caused an increase in K.E." && ke_diff <= 0);
			#endif
		}
	};
}
