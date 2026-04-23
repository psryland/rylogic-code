//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/integrator/engine.h"
#include "pr/physics/integrator/integrator.h"
#include "pr/physics/integrator/impulse.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/collision/contact.h"
#include "pr/physics/shape/inertia.h"
#include "pr/physics/diagnostics/body_history.h"
#include "src/compute/physics_types.h"
#include "src/compute/integrate_gpu.h"
#include "src/compute/sweep_gpu.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/resolve_gpu.h"
#include "src/integrator/engine_buffer_cache.h"
#include "src/collision/shape_cache.h"
#include "src/materials/material_map.h"
#include "src/diagnostics/physics_log.h"
#include "src/diagnostics/dbg_physics.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuBuffers
	{
		ReadbackAlloc rb_bodies;
		ReadbackAlloc rb_counters;
		ReadbackAlloc rb_contacts;
		ReadbackAlloc rb_intg_diag;
		ReadbackAlloc rb_pair_diag;
	};

	Engine::Engine(EngineConfig const& config, ID3D12Device4* existing_device)
		: m_config(config)
		, m_gpu(new Gpu(existing_device))
		, m_gpu_integrator(new GpuIntegrator(*m_gpu, config))
		, m_gpu_sort_and_sweep(new GpuSortAndSweep(*m_gpu, config))
		, m_gpu_collision_detector(new GpuCollisionDetector(*m_gpu, config))
		, m_gpu_resolver(new GpuResolver(*m_gpu, config))
		, m_materials(new MaterialMap)
		, m_cache(new EngineBufferCache())
	{
	}

	// Access the physics material properties for a given material ID.
	physics::Material Engine::Material(int id) const
	{
		return (*m_materials)[id];
	}
	void Engine::Material(physics::Material mat)
	{
		(*m_materials).Set(mat);
	}

	// Evolve the physics objects forward in time and resolve any collisions.
	void Engine::Step(float dt, std::span<RigidBody*> rigid_bodies)
	{
		// Notes:
		//  - There is a limitation on the number of collision pairs that can be generated per frame.
		//    If this limit becomes a problem, the options are increase the max number of collision pairs
		//    or run Engine::Step() multiple times on "islands" of physics objects
		if (rigid_bodies.empty())
			return;

		#if PR_PIX_ENABLED
		static bool capture = false;
		rdr12::pix::CaptureScope pix_capture("E:/Dump/PIXCaptures/Physics.wpix", capture);
		capture = false;
		#endif

		GpuBuffers buffers;
		m_cache->NewFrame(rigid_bodies, m_config.max_collision_pairs);

		// Pack all bodies into a GPU-friendly format
		Pack(rigid_bodies);

		// Integrate -> Updates dynamics, generates AABBs, debug data
		Integrate(dt);

		// Broadphase -> uses AABBs from integrate -> generates collision pairs
		BroadPhase();

		// Narrow phase -> uses collision pairs -> generates contacts
		Collide();

		// Resolve -> uses contacts -> applies impulses to bodies
		Resolve(dt);

		// Read buffers back to CPU memory
		Readback(buffers);
		
		// Run the GPU queue and wait for completion before using the results.
		m_gpu->m_job.Run();

		// Readback dynamics from GPU and unpack into bodies
		Unpack(buffers, rigid_bodies);
	}

	// Pack the body data into GPU buffers for the current frame.
	void Engine::Pack(std::span<RigidBody*> rigid_bodies)
	{
		for (auto body : rigid_bodies)
		{
			// Clear flags from the previous frame
			body->m_state_flags = SetBits(body->m_state_flags, ERigidBodyStateFlags::Collided, false);

			// Populate the shape cache
			auto shape_id = m_cache->m_shape_cache.GetOrAdd(body->Shape());

			// Copy the body data into the GPU staging buffer
			m_cache->m_rb_dynamics.push_back(PackDynamics(*body, shape_id));
		}
	}

	// Apply forces, evolve body dynamics forward in time, and generate AABBs for broadphase.
	void Engine::Integrate(float dt)
	{
		m_gpu_integrator->Integrate(m_gpu->m_job, m_cache->m_rb_dynamics, dt);

		#if PR_DBG_PHYSICS
		//auto body_count = static_cast<int>(m_cache->m_rb_dynamics.size());
		//DbgPhysics(*this).ReadbackIntegrate(body_count);
		#endif
	}

	// Broadphase collision detection to generate potential collision pairs.
	void Engine::BroadPhase()
	{
		auto body_count = static_cast<int>(m_cache->m_rb_dynamics.size());

		// GPU broadphase is only useful when GPU detect will consume the pairs
		auto counters = m_gpu_integrator->Counters();
		auto aabb = m_gpu_integrator->AABBAxisX(); // Todo: Should be choosing based on largest axis variance
		auto aabb_idx = m_gpu_integrator->AABBBodyIndices();
		auto bodies = m_gpu_integrator->Bodies();
		m_gpu_sort_and_sweep->Sort(m_gpu->m_job, body_count, aabb, aabb_idx);
		m_gpu_sort_and_sweep->Sweep(m_gpu->m_job, body_count, m_config.max_collision_pairs, counters, aabb_idx, bodies);

		#if PR_DBG_PHYSICS
		//DbgPhysics(*this).ReadbackSweep(counters);
		#endif
	}

	// Narrow phase collision detection to generate contact points.
	void Engine::Collide()
	{
		auto counters = m_gpu_integrator->Counters();
		auto dispatch = m_gpu_sort_and_sweep->CDDispatchArgs();
		auto col_pairs = m_gpu_sort_and_sweep->CollisionPairs();
		m_gpu_collision_detector->DetectCollisions(m_gpu->m_job, m_config.max_collision_pairs, dispatch, col_pairs, counters, m_cache->m_shape_cache);

		#if PR_DBG_PHYSICS
		//DbgPhysics(*this).ReadbackCollide(counters);
		#endif
	}

	// Apply impulses to resolve collisions and update body dynamics.
	void Engine::Resolve(float dt)
	{
		auto body_count = static_cast<int>(m_cache->m_rb_dynamics.size());

		auto counters = m_gpu_integrator->Counters();
		auto dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		auto contacts = m_gpu_collision_detector->Contacts();
		auto bodies = m_gpu_integrator->Bodies();
		m_gpu_resolver->Resolve(m_gpu->m_job, dt, body_count, m_config.max_collision_pairs, dispatch, counters, contacts, bodies, m_materials->span());

		#if PR_DBG_PHYSICS
		//DbgPhysics(*this).ReadbackResolve(body_count, bodies);
		#endif
	}

	// Read buffers back to CPU memory
	void Engine::Readback(GpuBuffers& buffers)
	{
		auto body_count = static_cast<int>(m_cache->m_rb_dynamics.size());
		auto contacts_count = static_cast<int>(m_cache->m_contacts.size());
		auto bodies = m_gpu_integrator->Bodies();
		auto counters = m_gpu_integrator->Counters();
		auto contacts = m_gpu_collision_detector->Contacts();
		auto intg_diags = m_gpu_integrator->Diagnostics();
		auto pair_diags = m_gpu_collision_detector->Diagnostics();

		{
			m_gpu->m_job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			m_gpu->m_job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			m_gpu->m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			#if PR_COLLISION_DIAGNOSTICS
			m_gpu->m_job.m_barriers.Transition(intg_diags.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			m_gpu->m_job.m_barriers.Transition(pair_diags.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			#endif
			m_gpu->m_job.m_barriers.Commit();
		}
		{
			buffers.rb_bodies = m_gpu->m_job.m_readback.template Alloc<GpuRigidBody>(body_count);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_bodies, bodies.get(), 0);

			buffers.rb_counters = m_gpu->m_job.m_readback.template Alloc<GpuCollisionCounters>(1);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_counters, counters.get(), 0);

			buffers.rb_contacts = m_gpu->m_job.m_readback.template Alloc<GpuResolveContact>(contacts_count);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_contacts, contacts.get(), 0);

			#if PR_COLLISION_DIAGNOSTICS
			buffers.rb_intg_diag = m_gpu->m_job.m_readback.template Alloc<GpuIntegrateDiag>(body_count);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_intg_diag, intg_diags.get(), 0);

			buffers.rb_pair_diag = m_gpu->m_job.m_readback.template Alloc<GpuPairDiag>(contacts_count);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_pair_diag, pair_diags.get(), 0);
			#endif
		}
		{
			m_gpu->m_job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			m_gpu->m_job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			m_gpu->m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#if PR_COLLISION_DIAGNOSTICS
			m_gpu->m_job.m_barriers.Transition(intg_diags.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			m_gpu->m_job.m_barriers.Transition(pair_diags.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			#endif
			m_gpu->m_job.m_barriers.Commit();
		}
	}

	// Update rigid bodies with results from the step
	void Engine::Unpack(GpuBuffers const& buffers, std::span<RigidBody*> rigid_bodies)
	{
		auto body_count = static_cast<int>(m_cache->m_rb_dynamics.size());
		auto max_contacts = static_cast<int>(m_cache->m_contacts.size());

		// Read the results back to the CPU
		auto const& counts = *buffers.rb_counters.ptr<GpuCollisionCounters>();
		auto contact_count = std::min(counts.contact_count, max_contacts);
		std::memcpy(m_cache->m_rb_dynamics.data(), buffers.rb_bodies.ptr<GpuRigidBody>(), body_count * sizeof(GpuRigidBody));
		std::memcpy(m_cache->m_contacts.data(), buffers.rb_contacts.ptr<GpuResolveContact>(), contact_count * sizeof(GpuResolveContact));
		#if PR_COLLISION_DIAGNOSTICS
		// TODO: read the diags
		#endif

		// Before updating the bodies with new dynamics, raise the collision events
		if (contact_count != 0)
		{
			m_cache->m_contacts_cpu.resize(0);
			m_cache->m_contacts_cpu.reserve(contact_count);
			for (auto const& c : m_cache->m_contacts)
				m_cache->m_contacts_cpu.push_back(RbContact(*rigid_bodies[c.body_idx_a], *rigid_bodies[c.body_idx_b], c));

			Collisions(*this, m_cache->m_contacts_cpu);
		}

		// Unpack the GPU results into the RigidBody objects
		for (auto [body, i] : with_index(rigid_bodies))
		{
			UnpackDynamics(m_cache->m_rb_dynamics[i], *body);
		}

		#if PR_DBG_PHYSICS
		{
			// Look for anomolies in the dynamics and log them.
			m_cache->m_history.EndFrame(rigid_bodies, [](RigidBody const& rb0, RigidBody const& rb1)
			{
				// Return true if rb0 was below ground and rb1 is worse
				float ground_z = -0.5f;
				if (rb0.O2W().pos.z < ground_z && rb1.O2W().pos.z < rb0.O2W().pos.z)
					return true;

				return false;
			});
		}
		#endif
	}

	// Narrow phase collision detection.
	// Tests whether 'objA' and 'objB' are geometrically in contact using GJK/SAT.
	// All collision data (point, axis, depth) is computed in objA's object space to
	// minimise floating-point error. Returns true if the objects are in contact and
	// the contact is approaching (not separating).
	bool Engine::NarrowPhaseCollision(float dt, RbContact& c)
	{
		// This is the CPU reference implementation. Keep.
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
		c.m_mat = (*m_materials)(c.m_mat_idA, c.m_mat_idB);

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
		c.Update(sub_step * dt);

		return true;
	}

	// Calculate and apply the restitution impulse to resolve a collision.
	// The impulse is computed in objA's space (where all contact data lives),
	// then transformed to each body's own object space before being applied.
	//
	// Important: When multiple collisions are resolved in a single time step,
	// earlier resolutions change body momenta. We must recompute the relative
	// velocity using CURRENT momenta before computing each impulse, otherwise
	// stale velocity data causes catastrophic energy injection.
	void Engine::ResolveCollision(RbContact& c)
	{
		// This is the CPU reference implementation. Keep.
		auto& objA = const_cast<RigidBody&>(*c.m_objA);
		auto& objB = const_cast<RigidBody&>(*c.m_objB);

		// Recompute relative velocity using current momenta.
		// The geometric data (contact point, axis, depth) is still valid because
		// only momenta changed, not positions. But the velocity field is stale.
		auto va = Shift(objA.VelocityOS(), -objA.CentreOfMassOS());
		auto vb = Shift(objB.VelocityOS(), -objB.CentreOfMassOS());
		c.m_velocity = c.m_b2a * vb - va;

		// Re-check the separating condition with updated velocities.
		// A previous impulse in this step may have already resolved this contact.
		auto rel_vel_at_point = c.m_velocity.LinAt(c.m_point_at_t);
		auto sep_dot = Dot(rel_vel_at_point, c.m_axis);
		if (sep_dot > 0)
			return;

		// Measure pre-collision kinetic energy of the pair
		auto ke_before = objA.KineticEnergy() + objB.KineticEnergy();

		// Compute the equal-and-opposite impulse pair as spatial force wrenches.
		// Each wrench is expressed at the body's own model origin, in its own frame.
		auto impulse_pair = RestitutionImpulse(c);
		auto ja = impulse_pair.m_os_impulse_objA;
		auto jb = impulse_pair.m_os_impulse_objB;

		// Pre-compute the "impulse kinetic energy" term: the KE that the impulse
		// alone would create if applied to stationary bodies. This is the coefficient
		// of the α² term in KE(α) = KE₀ + αB + α²A.
		auto va_j = objA.InertiaInvOS() * ja;
		auto vb_j = objB.InertiaInvOS() * jb;
		auto A = 0.5f * (Dot(va_j, ja) + Dot(vb_j, jb));

		// Apply the impulses to each body's momentum (stored as spatial force at CoM).
		// The impulse changes both linear momentum (causing velocity change) and angular
		// momentum (causing spin change proportional to the lever arm from CoM to contact).
		objA.MomentumOS(objA.MomentumOS() + ja);
		objB.MomentumOS(objB.MomentumOS() + jb);

		// Energy conservation guard: if the impulse injected energy, scale it back.
		// For elastic collisions (e=1), KE should be conserved exactly. For inelastic (e<1),
		// KE must decrease. Numerical errors in contact geometry, sub-step approximation,
		// or the collision mass matrix can cause small energy gains that compound over
		// many collisions, leading to objects "exploding" apart.
		//
		// KE(α) = KE₀ + αB + α²A is quadratic in the impulse scale factor α.
		// At α=1: KE₁ = KE₀ + B + A, so B = (KE₁ - KE₀) - A = δ - A.
		// For KE(α) = KE₀: α²A + αB = 0 → α = -B/A = (A - δ)/A.
		auto ke_after = objA.KineticEnergy() + objB.KineticEnergy();
		auto delta = ke_after - ke_before;
		if (delta > 0 && A > math::tiny<float>)
		{
			auto alpha = Clamp((A - delta) / A, 0.0f, 1.0f);
			auto correction = 1.0f - alpha;
			objA.MomentumOS(objA.MomentumOS() - correction * ja);
			objB.MomentumOS(objB.MomentumOS() - correction * jb);

			#if PR_DBG
			{
				auto ke_clamped = objA.KineticEnergy() + objB.KineticEnergy();
				if (delta > 0.1f || alpha < 0.5f)
				{
					char buf[256];
					snprintf(buf, sizeof(buf),
						"[CLAMP] ke_before=%.4f delta=%.4f A=%.4f alpha=%.4f ke_clamped=%.4f\n",
						ke_before, delta, A, alpha, ke_clamped);
					auto f = fopen("dump\\clamp.log", "a");
					if (f) { fputs(buf, f); fclose(f); }
				}
			}
			#endif
		}
	}

	// Deleter implementations
	void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}
}
