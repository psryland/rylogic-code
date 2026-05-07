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
#include "src/compute/sleep_gpu.h"
#include "src/compute/sweep_gpu.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/selective_gpu.h"
#include "src/integrator/engine_buffer_cache.h"
#include "src/collision/shape_cache.h"
#include "src/materials/material_map.h"
#include "src/diagnostics/physics_log.h"
#include "src/diagnostics/dbg_physics.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		double ElapsedMs(Clock::time_point beg, Clock::time_point end)
		{
			return std::chrono::duration<double, std::milli>(end - beg).count();
		}
		template <bool Enabled, auto Field> struct StepProfileScope;
		template <auto Field> struct StepProfileScope<true, Field>
		{
			Engine::StepProfile& m_profile;
			Clock::time_point m_beg;

			explicit StepProfileScope(Engine::StepProfile& profile)
				: m_profile(profile)
				, m_beg(Clock::now())
			{
			}
			~StepProfileScope()
			{
				m_profile.*Field = ElapsedMs(m_beg, Clock::now());
			}
		};
		template <auto Field> struct StepProfileScope<false, Field>
		{
			explicit StepProfileScope(Engine::StepProfile&)
			{
			}
		};
		template <auto Field> auto ProfileScope(Engine::StepProfile& profile)
		{
			return StepProfileScope<PR_PHYSICS_PROFILE != 0, Field>(profile);
		}
		void CheckCollisionCapacity(Engine::CollisionStats const& stats)
		{
			if (stats.PairLimitReached())
			{
				throw std::runtime_error(std::format(
					"GPU collision pair buffer capacity reached: {} potential pairs generated for {} slots. Increase EngineConfig::max_collision_pairs.",
					stats.m_pair_count,
					stats.m_max_pairs));
			}
			if (stats.ContactLimitReached())
			{
				throw std::runtime_error(std::format(
					"GPU collision contact buffer capacity reached: {} contacts generated for {} slots. Increase EngineConfig::max_collision_pairs.",
					stats.m_contact_count,
					stats.m_max_contacts));
			}
		}
	}

	struct GpuBuffers
	{
		ReadbackAlloc rb_bodies;
		ReadbackAlloc rb_counters;
		ReadbackAlloc rb_contacts;
		bool emit_collisions = true;
		bool read_contacts = false;
	};

	Engine::Engine(EngineConfig const& config, ID3D12Device4* existing_device)
		: m_config(config)
		, m_gpu(new Gpu(existing_device))
		, m_gpu_integrator(new GpuIntegrator(*m_gpu, m_config))
		, m_gpu_sleep_manager(new GpuSleepManager(*m_gpu, m_config))
		, m_gpu_sort_and_sweep(new GpuSortAndSweep(*m_gpu, m_config))
		, m_gpu_collision_detector(new GpuCollisionDetector(*m_gpu, m_config))
		, m_gpu_resolver(new GpuResolver(*m_gpu, m_config))
		, m_gpu_selective_resolver(new GpuResolver(*m_gpu, m_config))
		, m_gpu_selective_refresher(new GpuSelectiveRefresher(*m_gpu, m_config))
		, m_materials(new MaterialMap)
		, m_cache(new EngineBufferCache())
		, m_body_ptrs()
		, m_last_step_profile()
		, m_last_collision_stats()
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

	// Update runtime-tunable engine configuration.
	EngineConfig const& Engine::Config() const
	{
		return m_config;
	}
	void Engine::Config(EngineConfig const& config)
	{
		m_config = config;
	}

	// Drop all internally-cached references to caller-supplied data.
	void Engine::ResetCaches()
	{
		m_cache->Reset();
	}

	// Evolve the physics objects forward in time and resolve any collisions.
	void Engine::Step(float dt, std::span<RigidBody*> rigid_bodies)
	{
		// Notes:
		//  - There is a limitation on the number of collision pairs that can be generated per frame.
		//    If this limit becomes a problem, the options are increase the max number of collision pairs
		//    or run Engine::Step() multiple times on "islands" of physics objects
		if (rigid_bodies.empty())
		{
			m_last_step_profile = {};
			m_last_collision_stats = {};
			return;
		}

		#if PR_PIX_ENABLED
		static bool capture = false;
		rdr12::pix::CaptureScope pix_capture("E:/Dump/PIXCaptures/Physics.wpix", capture);
		capture = false;
		#endif

		m_last_step_profile = {};
		m_last_collision_stats = {};

		GpuBuffers buffers;
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_new_frame_ms>(m_last_step_profile);
			m_cache->NewFrame(rigid_bodies, m_config.max_collision_pairs);
		}

		// Pack all bodies into a GPU-friendly format
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_pack_ms>(m_last_step_profile);
			Pack(rigid_bodies);
		}

		// If nothing dynamic is awake then no GPU stage can change the world state.
		if (m_config.sleeping_enabled && m_cache->AwakeDynamicCount() == 0)
			return;

		// Upload -> transfers staged body dynamics and resets GPU counters
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_upload_ms>(m_last_step_profile);
			Upload();
		}

		// Integrate -> Updates dynamics, generates AABBs, debug data
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_integrate_ms>(m_last_step_profile);
			Integrate(dt);
		}

		// SleepWake -> marks sleeping islands disturbed by awake body AABBs
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleepwake_ms>(m_last_step_profile);
			SleepWake();
		}

		// Broadphase -> uses AABBs from integrate -> generates collision pairs
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_broadphase_ms>(m_last_step_profile);
			BroadPhase(m_config.sleeping_enabled);
		}

		// Narrow phase -> uses collision pairs -> generates contacts
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_collide_ms>(m_last_step_profile);
			Collide();
		}

		// Resolve -> uses contacts -> applies impulses to bodies
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_resolve_ms>(m_last_step_profile);
			Resolve(dt);
		}

		// SelectiveRefresh -> retries only problematic contacts with refreshed manifolds
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_selective_ms>(m_last_step_profile);
			SelectiveRefresh(dt);
		}

		// SleepUpdate -> persists wake-ups caused by resolver impulses
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleepupdate_ms>(m_last_step_profile);
			SleepUpdate(dt);
		}

		// ReadBody -> read back body dynamics and contact data
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_readback_ms>(m_last_step_profile);
			Readback(buffers);
		}

		// Run the GPU jobs for all stages up to this point
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_gpu_run_ms>(m_last_step_profile);
			m_gpu->m_job.Run();
		}

		// Unpack the results back into the caller-owned bodies
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_unpack_ms>(m_last_step_profile);
			Unpack(buffers, rigid_bodies);
		}
	}

	// Explicitly initialise missing sleep islands for newly-created sleeping bodies.
	void Engine::UpdateSleepIslands(std::span<RigidBody*> rigid_bodies)
	{
		if (rigid_bodies.empty())
			return;

		m_last_step_profile = {};
		m_last_collision_stats = {};

		GpuBuffers buffers;
		buffers.emit_collisions = false;

		{
			// Rebuild the staging state from the caller-owned bodies. Bodies that are already sleeping but have no island id are deliberately
			// left out of the uploaded island list; the GPU contact graph below creates their first transient ids.
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_new_frame_ms>(m_last_step_profile);
			m_cache->NewFrame(rigid_bodies, m_config.max_collision_pairs);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_pack_ms>(m_last_step_profile);
			Pack(rigid_bodies);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_upload_ms>(m_last_step_profile);
			Upload();
		}
		{
			// A zero-length integrate pass updates the GPU AABBs from the current transforms without advancing the simulation.
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_integrate_ms>(m_last_step_profile);
			Integrate(0.0f);
		}
		{
			// Run the same broadphase, narrowphase, and sleep-update graph as a normal frame, but with
			// sleeping-pair filtering disabled so sleeping/sleeping contacts are visible while the initial islands are being built.
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleepupdate_ms>(m_last_step_profile);
			BroadPhase(false);
			Collide();
			SleepUpdate(0.0f);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_readback_ms>(m_last_step_profile);
			Readback(buffers);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_gpu_run_ms>(m_last_step_profile);
			m_gpu->m_job.Run();
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_unpack_ms>(m_last_step_profile);
			Unpack(buffers, rigid_bodies);
		}
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
			auto dyn = PackDynamics(*body, shape_id);
			m_cache->CountAwakeDynamic(*body);
			m_cache->PackSleepIsland(*body, dyn);
			m_cache->m_rb_dynamics.push_back(dyn);
		}
	}

	// Upload staged body data into GPU buffers for the current frame.
	void Engine::Upload()
	{
		m_gpu_integrator->Upload(m_gpu->m_job, m_cache->m_rb_dynamics);
		m_gpu_sleep_manager->Upload(m_gpu->m_job, m_cache->m_sleep_islands);
	}

	// Apply forces, evolve body dynamics forward in time, and generate AABBs for broadphase.
	void Engine::Integrate(float dt)
	{
		auto body_count = m_cache->RigidBodyCount();
		m_gpu_integrator->Integrate(m_gpu->m_job, body_count, dt);

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
		{
			//DbgPhysics(*this).ReadbackIntegrate(body_count);
		}
	}

	// Mark sleeping islands disturbed by awake bodies before broadphase filtering.
	void Engine::SleepWake()
	{
		auto body_count = m_cache->RigidBodyCount();
		auto island_count = m_cache->SleepIslandCount();
		auto bodies = m_gpu_integrator->Bodies();
		m_gpu_sleep_manager->SleepWake(m_gpu->m_job, body_count, island_count, bodies);
	}

	// Broadphase collision detection to generate potential collision pairs.
	void Engine::BroadPhase(bool sleeping_enabled)
	{
		auto body_count = m_cache->RigidBodyCount();

		// GPU broadphase is only useful when GPU detect will consume the pairs
		auto counters = m_gpu_integrator->Counters();
		auto aabb = m_gpu_integrator->AABBAxisX(); // Todo: Should be choosing based on largest axis variance
		auto aabb_idx = m_gpu_integrator->AABBBodyIndices();
		auto bodies = m_gpu_integrator->Bodies();
		auto sleep_islands = m_gpu_sleep_manager->SleepIslands();
		auto sleep_island_count = m_cache->SleepIslandCount();
		m_gpu_sort_and_sweep->Sort(m_gpu->m_job, body_count, aabb, aabb_idx);
		m_gpu_sort_and_sweep->Sweep(m_gpu->m_job, body_count, m_config.max_collision_pairs, counters, aabb_idx, bodies, sleep_island_count, sleep_islands, sleeping_enabled);

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
		{
			//DbgPhysics(*this).ReadbackSweep(counters);
		}
	}

	// Narrow phase collision detection to generate contact points.
	void Engine::Collide()
	{
		auto counters = m_gpu_integrator->Counters();
		auto dispatch = m_gpu_sort_and_sweep->CDDispatchArgs();
		auto col_pairs = m_gpu_sort_and_sweep->CollisionPairs();
		m_gpu_collision_detector->DetectCollisions(m_gpu->m_job, m_config.max_collision_pairs, m_config.max_collision_pairs, dispatch, col_pairs, counters, m_cache->m_shape_cache);

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
		{
			//DbgPhysics(*this).ReadbackCollide(counters);
		}
	}

	// Apply impulses to resolve collisions and update body dynamics.
	void Engine::Resolve(float dt)
	{
		auto body_count = m_cache->RigidBodyCount();

		auto counters = m_gpu_integrator->Counters();
		auto dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		auto contacts = m_gpu_collision_detector->Contacts();
		auto bodies = m_gpu_integrator->Bodies();
		m_gpu_resolver->Resolve(m_gpu->m_job, dt, body_count, m_config.max_collision_pairs, dispatch, counters, contacts, bodies, m_materials->span());

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
		{
			//DbgPhysics(*this).ReadbackResolve(body_count, bodies);
		}
	}

	// Extra narrowphase/resolve passes over problematic contacts.
	void Engine::SelectiveRefresh(float dt)
	{
		auto const pass_count = std::max(0, m_config.selective_refresh_passes);
		if (pass_count == 0)
			return;

		auto const body_count = m_cache->RigidBodyCount();
		auto const full_max_pairs = m_config.max_collision_pairs;
		auto const max_pairs = Clamp(m_config.selective_refresh_max_pairs, 1, full_max_pairs);
		auto const max_contacts = max_pairs;
		auto full_counters = m_gpu_integrator->Counters();
		auto full_pairs = m_gpu_sort_and_sweep->CollisionPairs();
		auto bodies = m_gpu_integrator->Bodies();
		auto source_counters = full_counters;
		auto source_contacts = m_gpu_collision_detector->Contacts();
		auto source_dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		auto source_max_contacts = full_max_pairs;

		for (int pass = 0; pass != pass_count; ++pass)
		{
			// Each pass scores the contacts produced by the previous resolve, then refreshes
			// narrowphase only for nearby pairs that share those problem bodies.
			auto& work_set = m_gpu_selective_refresher->BuildWorkSet(
				m_gpu->m_job,
				pass,
				body_count,
				max_pairs,
				max_contacts,
				source_max_contacts,
				full_max_pairs,
				source_counters,
				source_contacts,
				source_dispatch,
				full_counters,
				full_pairs,
				bodies);

			m_gpu_collision_detector->DetectCollisions(
				m_gpu->m_job,
				work_set.m_max_contacts,
				work_set.m_max_pairs,
				work_set.m_cd_dispatch,
				work_set.m_pairs,
				work_set.m_counters,
				work_set.m_contacts,
				work_set.m_resolve_dispatch,
				m_cache->m_shape_cache);

			m_gpu_selective_resolver->Resolve(
				m_gpu->m_job,
				dt,
				body_count,
				work_set.m_max_contacts,
				work_set.m_resolve_dispatch,
				work_set.m_counters,
				work_set.m_contacts,
				bodies,
				m_materials->span(),
				m_config.selective_refresh_bias_scale,
				m_config.selective_refresh_solver_iterations,
				m_config.selective_refresh_position_iterations,
				m_config.selective_refresh_restitution_scale,
				m_config.selective_refresh_resolve_support_only);

			source_counters = work_set.m_counters;
			source_contacts = work_set.m_contacts;
			source_dispatch = work_set.m_resolve_dispatch;
			source_max_contacts = work_set.m_max_contacts;
		}
	}

	// Persist wake-ups and update sleep state after collision resolution.
	void Engine::SleepUpdate(float dt)
	{
		auto body_count = m_cache->RigidBodyCount();
		auto island_count = m_cache->SleepIslandCount();
		auto bodies = m_gpu_integrator->Bodies();
		auto counters = m_gpu_integrator->Counters();
		auto contacts = m_gpu_collision_detector->Contacts();
		m_gpu_sleep_manager->SleepUpdate(m_gpu->m_job, dt, body_count, island_count, m_config.max_collision_pairs, counters, contacts, bodies);
	}

	// Read buffers back to CPU memory
	void Engine::Readback(GpuBuffers& buffers)
	{
		auto body_count = m_cache->RigidBodyCount();
		auto contacts_count = m_cache->MaxContactsCount();
		auto bodies = m_gpu_integrator->Bodies();
		auto counters = m_gpu_integrator->Counters();
		auto contacts = m_gpu_collision_detector->Contacts();
		buffers.read_contacts = buffers.emit_collisions && static_cast<bool>(Collisions);

		{
			m_gpu->m_job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			m_gpu->m_job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			if (buffers.read_contacts)
			{
				m_gpu->m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			}
			m_gpu->m_job.m_barriers.Commit();
		}
		{
			buffers.rb_bodies = m_gpu->m_job.m_readback.template Alloc<GpuRigidBody>(body_count);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_bodies, bodies.get(), 0);

			buffers.rb_counters = m_gpu->m_job.m_readback.template Alloc<GpuCollisionCounters>(1);
			m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_counters, counters.get(), 0);

			if (buffers.read_contacts)
			{
				buffers.rb_contacts = m_gpu->m_job.m_readback.template Alloc<GpuResolveContact>(contacts_count);
				m_gpu->m_job.m_cmd_list.CopyBufferRegion(buffers.rb_contacts, contacts.get(), 0);
			}
		}
		{
			m_gpu->m_job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			m_gpu->m_job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			if (buffers.read_contacts)
			{
				m_gpu->m_job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			}
			m_gpu->m_job.m_barriers.Commit();
		}
	}

	// Update rigid bodies with results from the step
	void Engine::Unpack(GpuBuffers const& buffers, std::span<RigidBody*> rigid_bodies)
	{
		auto body_count = m_cache->RigidBodyCount();
		auto max_contacts = m_cache->MaxContactsCount();

		auto const& counts = *buffers.rb_counters.ptr<GpuCollisionCounters>();
		m_last_collision_stats = Engine::CollisionStats{
			.m_pair_count = counts.pair_count,
			.m_contact_count = counts.contact_count,
			.m_max_pairs = m_config.max_collision_pairs,
			.m_max_contacts = max_contacts,
		};
		CheckCollisionCapacity(m_last_collision_stats);

		auto contact_count = std::min(counts.contact_count, max_contacts);
		std::memcpy(m_cache->m_rb_dynamics.data(), buffers.rb_bodies.ptr<GpuRigidBody>(), body_count * sizeof(GpuRigidBody));
		if (buffers.read_contacts)
			std::memcpy(m_cache->m_contacts.data(), buffers.rb_contacts.ptr<GpuResolveContact>(), contact_count * sizeof(GpuResolveContact));

		// Before updating the bodies with new dynamics, raise the collision events
		if (contact_count != 0 && buffers.read_contacts)
		{
			m_cache->m_contacts_cpu.resize(0);
			m_cache->m_contacts_cpu.reserve(contact_count);
			for (auto const& c : std::span{ m_cache->m_contacts }.subspan(0, contact_count))
				m_cache->m_contacts_cpu.push_back(RbContact(*rigid_bodies[c.body_idx_a], *rigid_bodies[c.body_idx_b], c));

			Collisions(*this, m_cache->m_contacts_cpu);
		}

		// Unpack the GPU results into the RigidBody objects
		for (auto [body, i] : with_index(rigid_bodies))
		{
			m_cache->UnpackSleepIsland(m_cache->m_rb_dynamics[i]);
			UnpackDynamics(m_cache->m_rb_dynamics[i], *body);
		}

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
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
	}

	// Narrow phase collision detection.
	bool Engine::NarrowPhaseCollision(float dt, RbContact& c)
	{
		// Notes:
		//  - This is the CPU reference implementation. Keep.
		//  - Tests whether 'objA' and 'objB' are geometrically in contact using GJK/SAT.
		//    All collision data (point, axis, depth) is computed in objA's object space to
		//    minimise floating-point error. Returns true if the objects are in contact and
		//    the contact is approaching (not separating).

		auto& objA = *c.m_objA;
		auto& objB = *c.m_objB;

		// Collision detection in objA space: objA is at identity, objB is at c.m_b2a.
		if (!collision::Collide(objA.Shape(), m4x4::Identity(), objB.Shape(), c.m_b2a, c))
			return false;

		// If the collision point is moving out of collision, ignore the collision.
		// This prevents re-resolving contacts that are already separating.
		auto point = c.Point();
		auto rel_vel_at_point = c.m_velocity.LinAt(point);
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
		auto point_at_t0 = point - dt * c.m_velocity.LinAt(point);
		auto distance = Abs(Dot(point - point_at_t0, c.m_axis));
		auto sub_step = distance > c.m_depth ? -c.m_depth / distance : 0.0f;

		// Recompute contact data (b2a, velocity, contact point) at the estimated collision time.
		c.Update(sub_step * dt);

		return true;
	}

	// Calculate and apply the restitution impulse to resolve a collision.
	void Engine::ResolveCollision(RbContact& c)
	{
		// Notes:
		//  - This is the CPU reference implementation. Keep.
		//  - The impulse is computed in objA's space (where all contact data lives),
		//    then transformed to each body's own object space before being applied.
		//
		//  Important: When multiple collisions are resolved in a single time step,
		//  earlier resolutions change body momenta. We must recompute the relative
		//  velocity using CURRENT momenta before computing each impulse, otherwise
		//  stale velocity data causes catastrophic energy injection.

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
		}
	}

	// Deleter implementations
	void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}
}
