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

	// A container object that groups the parts of a physics system together.
	// TBroadphase provides spatial overlap queries (e.g. brute-force, sweep-and-prune).
	// TMaterials maps material ID pairs to combined material properties (friction, elasticity).
	template <typename TBroadphase, typename TMaterials = MaterialMap>
	struct Engine
	{
		TBroadphase m_broadphase;
		TMaterials m_materials;

		// Raised after collision detection, but before resolution.
		// Subscribers can inspect, modify, add, or remove contacts before impulses are applied.
		EventHandler<Engine&, std::vector<Contact>&> PostCollisionDetection;

		// Evolve the physics objects forward in time and resolve any collisions.
		// The simulation pipeline is: Evolve → Broad Phase → Narrow Phase → Resolve.
		template <typename TRigidBodyCont>
		void Step(float dt, TRigidBodyCont& bodies)
		{
			// Before here, callers should have set forces on the rigid bodies (including gravity).
			// Todo: A lot of this could be done in parallel/pipelined...

			// Advance all bodies to 't + dt' using semi-implicit Euler integration.
			// After this, body positions reflect the new time but may overlap.
			// Static bodies (infinite mass) are skipped — they have no forces, no
			// velocity, and evolving them would only accumulate numerical drift.
			for (auto& body : bodies)
			{
				if (body.Mass() >= InfiniteMass * 0.5f) continue;
				Evolve(body, dt);
			}

			// Broad phase: find pairs of bodies whose bounding volumes overlap.
			// Narrow phase: test each pair for actual geometric contact.
			std::vector<Contact> collision_queue;
			m_broadphase.EnumOverlappingPairs([&](void*, RigidBody const& objA, RigidBody const& objB)
			{
				auto c = Contact{objA, objB};
				if (NarrowPhaseCollision(dt, c))
				{
					#ifdef PR_PHYSICS_DUMP_CONTACTS
					Dump(c);
					#endif
					collision_queue.push_back(c);
				}
			});

			// Sort the collisions by estimated time of impact so earlier collisions are resolved first.
			// Todo: for parallel processing, collision queue should be broken up into islands of objects that affect each other.
			// Assign an increasing 'island' number to each Contact. If a preceding contact involves one of the same objects, use the lowest number
			std::sort(std::begin(collision_queue), std::end(collision_queue), [](auto& lhs, auto& rhs){ return lhs.m_time < rhs.m_time; });

			// Notify of detected collisions, and allow updates/additions
			PostCollisionDetection(*this, collision_queue);

			// Apply restitution impulses to resolve each collision
			for (auto& c : collision_queue)
				ResolveCollision(c);
		}

		// Narrow phase collision detection.
		// Tests whether 'objA' and 'objB' are geometrically in contact using GJK/SAT.
		// All collision data (point, axis, depth) is computed in objA's object space to
		// minimise floating-point error. Returns true if the objects are in contact and
		// the contact is approaching (not separating).
		bool NarrowPhaseCollision(float dt, Contact& c)
		{
			auto& objA = *c.m_objA;
			auto& objB = *c.m_objB;

			// Collision detection in objA space: objA is at identity, objB is at c.m_b2a.
			if (!collision::Collide(objA.Shape(), m4x4::Identity(), objB.Shape(), c.m_b2a, c))
				return false;

			// If the collision point is moving out of collision, ignore the collision.
			// This prevents re-resolving contacts that are already separating.
			auto rel_vel_at_point = c.m_velocity.LinAt(c.m_point);
			if (Dot(rel_vel_at_point, c.m_axis) > 0)
				return false;

			// Look up the combined material properties for this contact pair
			c.m_mat = m_materials(c.m_mat_idA, c.m_mat_idB);

			// Estimate the sub-step time when the collision actually occurred.
			// The bodies have already been evolved past the collision point, so we
			// need to estimate how far back in time the actual contact was. We project
			// the contact point backward along the relative velocity to find the
			// pre-overlap position, then compute sub_step as the fraction of dt to
			// rewind. This gives a more accurate contact point and lever arms for
			// the impulse calculation.
			auto point_at_t0 = c.m_point - dt * c.m_velocity.LinAt(c.m_point);
			auto distance = Abs(Dot(c.m_point - point_at_t0, c.m_axis));
			auto sub_step = distance > c.m_depth ? -c.m_depth / distance : 0.0f;

			// Recompute contact data (b2a, velocity, contact point) at the estimated collision time.
			// Note: this is an approximation — the bodies aren't actually moved back in time,
			// so there will be a small angular momentum error proportional to penetration depth.
			c.update(sub_step * dt);

			return true;
		}

		// Calculate and apply the restitution impulse to resolve a collision.
		// The impulse is computed in objA's space (where all contact data lives),
		// then transformed to each body's own object space before being applied.
		//
		// Important: When multiple collisions are resolved in a single time step,
		// earlier resolutions change body momenta. We must recompute the relative
		// velocity using CURRENT momenta before computing each impulse, otherwise
		// stale velocity data causes catastrophic energy injection. For example:
		// if body A bounces off the ground (velocity reversed from -v to +v), then
		// a subsequent collision (A, B) using the old velocity (-v) would compute
		// an impulse as if A is still approaching B, doubling the energy.
		void ResolveCollision(Contact& c)
		{
			auto& objA = const_cast<RigidBody&>(*c.m_objA);
			auto& objB = const_cast<RigidBody&>(*c.m_objB);

			// Recompute relative velocity using current momenta.
			// The geometric data (contact point, axis, depth) is still valid because
			// only momenta changed, not positions. But the velocity field is stale.
			c.m_velocity = c.m_b2a * objB.VelocityOS() - objA.VelocityOS();

			// Re-check the separating condition with updated velocities.
			// A previous impulse in this step may have already resolved this contact.
			auto rel_vel_at_point = c.m_velocity.LinAt(c.m_point_at_t);
			auto sep_dot = Dot(rel_vel_at_point, c.m_axis);
			if (sep_dot > 0)
				return;

			// Compute the equal-and-opposite impulse pair as spatial force wrenches.
			// Each wrench is expressed at the body's own model origin, in its own frame.
			auto impulse_pair = RestitutionImpulse(c);

			// Apply the impulses to each body's momentum (stored as spatial force at model origin).
			// The impulse changes both linear momentum (causing velocity change) and angular
			// momentum (causing spin change proportional to the lever arm from CoM to contact).
			objA.MomentumOS(objA.MomentumOS() + impulse_pair.m_os_impulse_objA);
			objB.MomentumOS(objB.MomentumOS() + impulse_pair.m_os_impulse_objB);
		}
	};
}
