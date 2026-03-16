//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics-2/integrator/engine.h"
#include "pr/physics-2/integrator/integrator.h"
#include "pr/physics-2/integrator/impulse.h"
#include "pr/physics-2/rigid_body/rigid_body.h"
#include "pr/physics-2/rigid_body/rigid_body_dynamics.h"
#include "pr/physics-2/collision/ibroadphase.h"
#include "pr/physics-2/collision/contact.h"
#include "pr/physics-2/materials/imaterials.h"
#include "src/integrator/gpu_integrator.h"
#include "src/collision/collision_shape_cache.h"
#include "src/collision/gpu_sort_and_sweep.h"
#include "src/collision/gpu_collision_detector.h"
#include "src/collision/gpu_collision_types.h"
#include "src/collision/gpu_resolver.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct BodyPair
	{
		m4x4 b2a;
		RigidBody const* objA;
		RigidBody const* objB;
	};

	struct EngineBufferCache
	{
		// Persists across frames
		CollisionShapeCache m_shape_cache;

		// Per-frame buffers
		std::vector<GpuCollisionPair> m_col_pairs;
		std::vector<BodyPair> m_body_pairs;
		std::vector<RbContact> m_collision_queue;
		std::vector<GpuContact> m_gpu_contacts;

		// Reset per-frame buffers
		void BeginFrame()
		{
			m_shape_cache.BeginFrame();
			m_col_pairs.resize(0);
			m_body_pairs.resize(0);
			m_collision_queue.resize(0);
			m_gpu_contacts.resize(0);
		}
	};

	Engine::Engine(IMaterials& mats, ID3D12Device4* existing_device)
		: m_gpu(new Gpu(existing_device))
		, m_gpu_integrator(new GpuIntegrator(*m_gpu))
		, m_gpu_sort_and_sweep(new GpuSortAndSweep(*m_gpu))
		, m_gpu_collision_detector(new GpuCollisionDetector(*m_gpu))
		, m_gpu_resolver(new GpuResolver(*m_gpu))
		, m_materials(mats)
		, m_cache(new EngineBufferCache)
		, m_rb_dynamics()
		, m_integrate_output()
		, m_integrate_aabbs()
		, m_gpu_resolve(true)
		, PostCollisionDetection()
	{
	}

	// Get/Set whether the GPU is used for integration and collision detection.
	bool Engine::UseGpu() const
	{
		return true;
	}
	void Engine::UseGpu(bool use_gpu)
	{
		// Dropping non-gpu support
		(void)use_gpu;
	}

	// Access the broadphase for registering bodies and enumerating overlapping pairs.
	IBroadphase& Engine::Broadphase() const
	{
		return *m_gpu_sort_and_sweep;
	}

	// Evolve the physics objects forward in time and resolve any collisions.
	void Engine::Step(float dt, std::span<RigidBody*> bodies)
	{
		// Before here, callers should have set forces on the rigid bodies (including gravity).
		// The simulation pipeline is: Evolve → Broad Phase → Narrow Phase → Resolve.

		// This is the core collision pipeline, called after all bodies have been evolved.
		m_cache->BeginFrame();

		// 
		if (m_body_ptrs.empty())
			m_body_ptrs.append_range(bodies);

		// Pack ALL bodies into a GPU-friendly format (including static bodies so the
		// GPU resolver can reference them by index). Static bodies are no-ops during
		// integration (zero inv_mass → zero velocity → no position change).
		{
			m_rb_dynamics.resize(0);
			for (auto body : m_body_ptrs)
				m_rb_dynamics.push_back(PackDynamics(*body));
		}

		// GPU split pipeline: integrate + readback AABBs → broadphase → GJK → readback bodies.
		// Bodies stay GPU-resident between the integrate and GJK steps, avoiding a round trip.
		{
			// Split pipeline: integrate + readback AABBs only (bodies stay GPU-resident)
			m_integrate_output.resize(m_rb_dynamics.size());
			m_integrate_aabbs.resize(m_rb_dynamics.size());
			m_gpu_integrator->IntegrateAndReadbackAABBs(m_gpu->m_job, m_rb_dynamics, dt, m_integrate_output, m_integrate_aabbs);
		}

		// Read back and update the RBs so that collision detection and resolution
		// apply to the updated dynamics. The GPU resolve path will readback again after impulse resolution.
		{
			m_gpu_integrator->ReadbackBodies(m_gpu->m_job, m_rb_dynamics);
			for (auto [body, i] : with_index(bodies))
				UnpackDynamics(m_rb_dynamics[i], *body);
		}

		// Broadphase uses GPU-computed AABBs
		DetectAndResolveCollisions(dt);

		m_body_ptrs.resize(0);
	}


	// Broad phase overlap query → narrow phase collision detection → impulse resolution.
	void Engine::DetectAndResolveCollisions(float dt)
	{
		auto& shape_cache = m_cache->m_shape_cache;
		auto& col_pairs = m_cache->m_col_pairs;
		auto& body_pairs = m_cache->m_body_pairs;
		auto& collision_queue = m_cache->m_collision_queue;
		auto& gpu_contacts = m_cache->m_gpu_contacts;

		// Phase 1: Broadphase — collect overlapping pairs and look up cached shapes.
		// Use GPU-computed AABBs when available to avoid recomputing on CPU.
		int pair_idx = 0;
		m_gpu_sort_and_sweep->EnumOverlappingPairs(m_gpu->m_job, std::span<IntegrateAABB const>(m_integrate_aabbs), [&](RigidBody const& objA, RigidBody const& objB)
		{
			auto idx_a = shape_cache.GetOrAdd(objA.Shape());
			auto idx_b = shape_cache.GetOrAdd(objB.Shape());

			// Compute B-to-A transform (collision runs in A's object space)
			auto w2a = InvertOrthonormal(objA.O2W());
			auto b2a = w2a * objB.O2W();

			col_pairs.push_back(GpuCollisionPair{
				.shape_idx_a = idx_a,
				.shape_idx_b = idx_b,
				.pair_index = pair_idx,
				.pad0 = 0,
				.b2a = b2a,
			});				

			body_pairs.push_back(BodyPair{
				.b2a = b2a,
				.objA = &objA,
				.objB = &objB,
			});

			++pair_idx;
		});

		// Nothing close enough to be colliding? Skip the GPU GJK step.
		if (col_pairs.empty())
			return;

		// Periodically flush stale shapes (every 10 frames)
		if (shape_cache.m_frame % CollisionShapeCache::StaleFrameLimit == 0)
			shape_cache.Flush();

		// Phase 2: Dispatch GPU GJK collision detection.
		// Pass the dirty flag so the detector knows whether to re-upload shapes.
		m_gpu_collision_detector->DetectCollisions(m_gpu->m_job, col_pairs, shape_cache.m_shapes, shape_cache.m_verts, gpu_contacts, shape_cache.IsDirty());
		shape_cache.ClearDirty();

		// Phase 3: Convert GPU contacts to physics::Contact structs and resolve.
		for (int ci = 0; ci != static_cast<int>(gpu_contacts.size()); ++ci)
		{
			auto const& gc = gpu_contacts[ci];

			// Validate the pair_index from GPU readback
			if (gc.pair_index < 0 || gc.pair_index >= static_cast<int>(body_pairs.size()))
			{
				OutputDebugStringA(FmtS("GPU contact[%d/%d]: pair_index=%d (out of range 0..%d), depth=%f\n",
					ci, static_cast<int>(gpu_contacts.size()), gc.pair_index, static_cast<int>(body_pairs.size()), gc.depth));
				continue;
			}

			auto const& bp = body_pairs[gc.pair_index];
			auto c = RbContact{*bp.objA, *bp.objB, gc};

			// Check if the collision point is separating (relative velocity positive along axis)
			auto rel_vel_at_point = c.m_velocity.LinAt(c.m_point);
			if (Dot(rel_vel_at_point, c.m_axis) > 0)
				continue;

			// Look up the combined material properties
			c.m_mat = m_materials(c.m_mat_idA, c.m_mat_idB);

			// Estimate sub-step collision time
			auto point_at_t0 = c.m_point - dt * c.m_velocity.LinAt(c.m_point);
			auto distance = Abs(Dot(c.m_point - point_at_t0, c.m_axis));
			auto sub_step = distance > c.m_depth ? -c.m_depth / distance : 0.0f;

			// Recompute contact data at the estimated collision time
			c.Update(sub_step * dt);

			collision_queue.push_back(c);
		}

		// Sort the collisions by estimated time of impact so earlier collisions are resolved first.
		std::sort(std::begin(collision_queue), std::end(collision_queue), [](auto& lhs, auto& rhs)
		{
			return lhs.m_time < rhs.m_time;
		});

		// Notify of detected collisions, and allow updates/additions
		PostCollisionDetection(*this, { collision_queue });

		// GPU impulse resolution via graph-coloured batches
		if (!collision_queue.empty())
		{
			auto& body_ptrs = m_body_ptrs;

			// Map RigidBody* → index in the dynamics buffer
			auto find_body_index = [&body_ptrs](RigidBody const* body) -> int
			{
				for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
					if (body_ptrs[i] == body) return i;
				return -1;
			};

			// Build GpuResolveContact buffer with body indices and materials
			std::vector<GpuResolveContact> resolve_contacts;
			resolve_contacts.reserve(collision_queue.size());
			for (auto const& c : collision_queue)
			{
				auto idx_a = find_body_index(c.m_objA);
				auto idx_b = find_body_index(c.m_objB);
				assert(idx_a >= 0 && idx_b >= 0);

				resolve_contacts.push_back(GpuResolveContact{
					.axis = c.m_axis,
					.point = c.m_point_at_t,
					.b2a = c.m_b2a,
					.body_idx_a = idx_a,
					.body_idx_b = idx_b,
					.elasticity = c.m_mat.m_elasticity_norm,
					.friction = c.m_mat.m_friction_static,
				});
			}

			// Graph-colour the contacts
			auto [colours, max_colour] = GraphColourContacts(resolve_contacts);

			if (m_gpu_resolve)
			{
				#if PR_DBG
				// Save pre-resolve dynamics for comparison
				auto pre_resolve_dynamics = m_rb_dynamics;
				#endif

				// GPU resolve path
				m_gpu_resolver->Resolve(m_gpu->m_job, resolve_contacts, colours, max_colour, m_gpu_integrator->BodiesResource());

				// Read back the post-resolve body state and unpack into RigidBodies
				m_gpu_integrator->ReadbackBodies(m_gpu->m_job, m_rb_dynamics);
				for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
				{
					if (body_ptrs[i]->Mass() >= 0.5f * InfiniteMass) continue;
					UnpackDynamics(m_rb_dynamics[i], *body_ptrs[i]);
				}

				#if PR_DBG
				{
					// Run CPU resolve on the same contacts for comparison.
					// Restore bodies to pre-resolve state first.
					for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
					{
						if (body_ptrs[i]->Mass() >= 0.5f * InfiniteMass) continue;
						UnpackDynamics(pre_resolve_dynamics[i], *body_ptrs[i]);
					}

					// Run CPU resolve
					for (auto& c : collision_queue)
						ResolveCollision(c);

					// Save CPU-resolved momenta
					auto cpu_dynamics = std::vector<RigidBodyDynamics>(body_ptrs.size());
					for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
						cpu_dynamics[i] = PackDynamics(*body_ptrs[i]);

					// Compare and log differences
					auto f = fopen("dump\\resolve_compare.log", "w");
					if (f)
					{
						fprintf(f, "=== Resolve comparison: %d contacts, %d colours ===\n",
							static_cast<int>(collision_queue.size()), max_colour);

						for (int i = 0; i != static_cast<int>(resolve_contacts.size()); ++i)
						{
							auto const& rc = resolve_contacts[i];
							fprintf(f, "Contact[%d] colour=%d bodies=(%d,%d) e=%.3f mu=%.3f\n",
								i, colours[i], rc.body_idx_a, rc.body_idx_b, rc.elasticity, rc.friction);
							fprintf(f, "  axis=(%.6f, %.6f, %.6f) pt=(%.6f, %.6f, %.6f)\n",
								rc.axis.x, rc.axis.y, rc.axis.z, rc.point.x, rc.point.y, rc.point.z);
						}

						fprintf(f, "\n--- Per-body momentum comparison ---\n");
						for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
						{
							auto const& gpu = m_rb_dynamics[i];
							auto const& cpu = cpu_dynamics[i];
							auto const& pre = pre_resolve_dynamics[i];

							auto ang_diff = v4{gpu.momentum_ang.x - cpu.momentum_ang.x, gpu.momentum_ang.y - cpu.momentum_ang.y, gpu.momentum_ang.z - cpu.momentum_ang.z, 0};
							auto lin_diff = v4{gpu.momentum_lin.x - cpu.momentum_lin.x, gpu.momentum_lin.y - cpu.momentum_lin.y, gpu.momentum_lin.z - cpu.momentum_lin.z, 0};
							auto ang_err = Length(ang_diff);
							auto lin_err = Length(lin_diff);

							if (ang_err > 1e-4f || lin_err > 1e-4f)
							{
								fprintf(f, "Body[%d] mass=%.2f MISMATCH ang_err=%.6f lin_err=%.6f\n", i, 1.0f / body_ptrs[i]->InertiaInvOS().InvMass(), ang_err, lin_err);
								fprintf(f, "  PRE  mom_ang=(%.6f, %.6f, %.6f) mom_lin=(%.6f, %.6f, %.6f)\n",
									pre.momentum_ang.x, pre.momentum_ang.y, pre.momentum_ang.z,
									pre.momentum_lin.x, pre.momentum_lin.y, pre.momentum_lin.z);
								fprintf(f, "  GPU  mom_ang=(%.6f, %.6f, %.6f) mom_lin=(%.6f, %.6f, %.6f)\n",
									gpu.momentum_ang.x, gpu.momentum_ang.y, gpu.momentum_ang.z,
									gpu.momentum_lin.x, gpu.momentum_lin.y, gpu.momentum_lin.z);
								fprintf(f, "  CPU  mom_ang=(%.6f, %.6f, %.6f) mom_lin=(%.6f, %.6f, %.6f)\n",
									cpu.momentum_ang.x, cpu.momentum_ang.y, cpu.momentum_ang.z,
									cpu.momentum_lin.x, cpu.momentum_lin.y, cpu.momentum_lin.z);
							}
						}
						fclose(f);
					}

					// Restore to GPU-resolved state (since that's the active path)
					for (int i = 0; i != static_cast<int>(body_ptrs.size()); ++i)
					{
						if (body_ptrs[i]->Mass() >= 0.5f * InfiniteMass) continue;
						UnpackDynamics(m_rb_dynamics[i], *body_ptrs[i]);
					}
				}
				#endif
			}
			else
			{
				// CPU fallback path
				for (auto& c : collision_queue)
					ResolveCollision(c);
			}
		}
	}

	// Narrow phase collision detection.
	// Tests whether 'objA' and 'objB' are geometrically in contact using GJK/SAT.
	// All collision data (point, axis, depth) is computed in objA's object space to
	// minimise floating-point error. Returns true if the objects are in contact and
	// the contact is approaching (not separating).
	bool Engine::NarrowPhaseCollision(float dt, RbContact& c)
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

	void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}
	void Deleter<Gpu>::operator()(Gpu* p) const
	{
		delete p;
	}
}







// Debug: stashed pre-integration state for A/B comparison between Evolve() and EvolveCPU().
#if PR_DBG&&0
std::vector<RigidBodyDynamics> m_compare_dynamics;
std::vector<int> m_compare_indices;
float m_compare_dt = 0;

// A/B comparison: replay the last integration step through EvolveCPU and compare.
// Uses the pre-integration state (stashed below) to run EvolveCPU independently.
void CompareIntegrationPaths([[maybe_unused]] float dt, [[maybe_unused]] RigidBodyRange auto& bodies)
{
	#if PR_DBG
	// Stash pre-integration state for comparison on the NEXT step.
	// On the first call, m_compare_dynamics is empty so we skip comparison.
	if (!m_compare_dynamics.empty())
	{
		// Run EvolveCPU on the stashed pre-integration dynamics
		for (auto& dyn : m_compare_dynamics)
			Evolve(dyn, m_compare_dt);

		// Compare EvolveCPU results with what Evolve() produced (now stored in bodies).
		// m_compare_indices maps dynamics[i] to bodies[idx].
		for (int i = 0; i != static_cast<int>(m_compare_dynamics.size()); ++i)
		{
			auto body_it = std::next(std::begin(bodies), m_compare_indices[i]);
			auto const& ref = *body_it; // Evolve() result
			auto const& gpu = m_compare_dynamics[i]; // EvolveCPU result

			// Compare transforms
			auto pos_err = Length(ref.O2W().pos - gpu.o2w.pos);
			auto rot_err = Length(ref.O2W().x - gpu.o2w.x)
					        + Length(ref.O2W().y - gpu.o2w.y)
					        + Length(ref.O2W().z - gpu.o2w.z);
			auto mom_ang_err = Length(ref.MomentumWS().ang - gpu.momentum_ang);
			auto mom_lin_err = Length(ref.MomentumWS().lin - gpu.momentum_lin);

			if (pos_err > 1e-4f || rot_err > 1e-4f || mom_ang_err > 1e-4f || mom_lin_err > 1e-4f)
			{
				auto f = fopen("dump\\evolve_compare.log", "a");
				if (f)
				{
					auto const& com = ref.InertiaInvOS().CoM();
					fprintf(f, "[MISMATCH] body=%d com=(%.4f,%.4f,%.4f) pos_err=%.6f rot_err=%.6f mom_ang_err=%.6f mom_lin_err=%.6f\n",
						m_compare_indices[i], com.x, com.y, com.z, pos_err, rot_err, mom_ang_err, mom_lin_err);
					fprintf(f, "  Evolve   pos=(%.6f,%.6f,%.6f) mom_ang=(%.6f,%.6f,%.6f) mom_lin=(%.6f,%.6f,%.6f)\n",
						ref.O2W().pos.x, ref.O2W().pos.y, ref.O2W().pos.z,
						ref.MomentumWS().ang.x, ref.MomentumWS().ang.y, ref.MomentumWS().ang.z,
						ref.MomentumWS().lin.x, ref.MomentumWS().lin.y, ref.MomentumWS().lin.z);
					fprintf(f, "  EvolveCPU pos=(%.6f,%.6f,%.6f) mom_ang=(%.6f,%.6f,%.6f) mom_lin=(%.6f,%.6f,%.6f)\n",
						gpu.o2w.pos.x, gpu.o2w.pos.y, gpu.o2w.pos.z,
						gpu.momentum_ang.x, gpu.momentum_ang.y, gpu.momentum_ang.z,
						gpu.momentum_lin.x, gpu.momentum_lin.y, gpu.momentum_lin.z);
					fclose(f);
				}
			}
		}
	}

	// Stash current pre-integration state for next step's comparison.
	// We pack dynamics NOW (before the next Evolve) so we have the same starting state.
	m_compare_dynamics.clear();
	m_compare_indices.clear();
	m_compare_dt = dt;
	{
		int idx = 0;
		for (auto& body : bodies)
		{
			if (body.Mass() < InfiniteMass * 0.5f)
			{
				m_compare_dynamics.push_back(PackDynamics(body));
				m_compare_indices.push_back(idx);
			}
			++idx;
		}
	}
	#endif
}
#endif