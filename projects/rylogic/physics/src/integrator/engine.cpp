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
#include "src/compute/physics_types.h"
#include "src/compute/integrate_gpu.h"
#include "src/compute/sweep_gpu.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/resolve_gpu.h"
#include "src/collision/shape_cache.h"
#include "src/materials/material_map.h"
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
		ShapeCache m_shape_cache;
		
		// Staging buffer for packing body dynamics
		std::vector<GpuRigidBody> m_rb_dynamics;
	};
	void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}

	Engine::Engine(ID3D12Device4* existing_device)
		: m_gpu(new Gpu(existing_device))
		, m_gpu_integrator(new GpuIntegrator(*m_gpu))
		, m_gpu_sort_and_sweep(new GpuSortAndSweep(*m_gpu))
		, m_gpu_collision_detector(new GpuCollisionDetector(*m_gpu))
		, m_gpu_resolver(new GpuResolver(*m_gpu))
		, m_materials(new MaterialMap)
		, m_cache(new EngineBufferCache)
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

	// Get/Set whether the GPU is used for narrow-phase collision detection (GJK).
	bool Engine::UseGpuDetect() const
	{
		return m_gpu_detect;
	}
	void Engine::UseGpuDetect(bool use)
	{
		m_gpu_detect = use;
	}

	// Get/Set whether the GPU is used for collision resolution (impulse application).
	bool Engine::UseGpuResolve() const
	{
		return m_gpu_resolve;
	}
	void Engine::UseGpuResolve(bool use)
	{
		m_gpu_resolve = use;
	}

	// Evolve the physics objects forward in time and resolve any collisions.
	void Engine::Step(float dt, std::span<RigidBody*> rigid_bodies)
	{
		// Notes:
		//  - There is a limitation on the number of collision pairs that can be generated per frame.
		//    If this limit becomes a problem, the options are increase the max number of collision pairs
		//    or run Engine::Step() multiple times on "islands" of physics objects
		constexpr auto max_col_pairs = 65536;
		int body_count = static_cast<int>(rigid_bodies.size());
		if (body_count == 0)
			return;

		#if PR_DBG_PHYSICS
		static int dbg_frame = 0;
		static FILE* dbg_log = nullptr;
		static bool dbg_logging = true;
		if (!dbg_log) dbg_log = fopen("dump\\physics_pipeline.log", "w");
		if (dbg_frame >= 0 && dbg_frame < 100) dbg_logging = true;
		dbg_logging = dbg_log ? dbg_logging : false;
		#endif

		#if PR_PIX_ENABLED
		static bool capture = false;
		rdr12::pix::CaptureScope pix_capture("E:/Dump/PIXCaptures/Physics.wpix", capture);
		capture = false;
		#endif

		if (m_body_ptrs.empty())
			m_body_ptrs.append_range(rigid_bodies);

		// Populate the shape cache
		m_cache->m_shape_cache.BeginFrame();

		// Pack all bodies into a GPU-friendly format
		{
			m_cache->m_rb_dynamics.resize(0);
			for (auto body : m_body_ptrs)
			{
				auto shape_id = m_cache->m_shape_cache.GetOrAdd(body->Shape());
				m_cache->m_rb_dynamics.push_back(PackDynamics(*body, shape_id));
			}
		}

		// Integrate -> Updates dynamics, generates AABBs, debug data
		{
			m_gpu_integrator->Integrate(m_gpu->m_job, m_cache->m_rb_dynamics, dt);

			#if PR_DBG_PHYSICS
			{
				std::vector<GpuRigidBody> bodies(body_count);
				std::vector<BBox> aabbs(body_count);
				std::vector<GpuIntegrateDiag> diag(body_count);
				m_gpu_integrator->Readback(m_gpu->m_job, bodies, aabbs, diag);

				if (dbg_logging)
				{
					fprintf(dbg_log, "=== Frame %d: %d bodies, dt=%.4f ===\n", dbg_frame, body_count, dt);
					for (int i = 0; i != body_count; ++i)
					{
						auto& b = bodies[i];
						auto pos = b.o2w.w;
						auto vel = b.os_com_and_invmass.w * b.momentum_lin;
						fprintf(dbg_log, "  body[%d]: pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f) inv_mass=%.6f shape=%d\n",
							i, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, b.os_com_and_invmass.w, b.shape_id);
						fprintf(dbg_log, "           aabb=(%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)\n",
							aabbs[i].Lower().x, aabbs[i].Lower().y, aabbs[i].Lower().z,
							aabbs[i].Upper().x, aabbs[i].Upper().y, aabbs[i].Upper().z);
					}
				}
			}
			#endif
		}

		// Broadphase -> uses AABBs from integrate -> generates collision pairs
		{
			auto counters = m_gpu_integrator->Counters();
			auto aabb = m_gpu_integrator->AABBAxisX(); // Todo: Should be choosing based on largest axis variance
			auto aabb_idx = m_gpu_integrator->AABBBodyIndices();
			auto bodies = m_gpu_integrator->Bodies();
			m_gpu_sort_and_sweep->Sort(m_gpu->m_job, body_count, aabb, aabb_idx);
			m_gpu_sort_and_sweep->Sweep(m_gpu->m_job, body_count, max_col_pairs, counters, aabb_idx, bodies);

			#if PR_DBG_PHYSICS
			{
				std::vector<GpuCollisionPair> out_pairs(max_col_pairs);
				auto pairs = m_gpu_sort_and_sweep->Readback(m_gpu->m_job, counters, out_pairs);
				assert(pairs.size() < max_col_pairs && "Hit capacity on pairs");

				if (dbg_logging)
				{
					fprintf(dbg_log, "  Broadphase: %d pairs\n", static_cast<int>(pairs.size()));
					for (int i = 0; i != static_cast<int>(pairs.size()) && i < 10; ++i)
					{
						auto& p = pairs[i];
						fprintf(dbg_log, "    pair[%d]: body(%d,%d) shape(%d,%d) b2a_pos=(%.3f,%.3f,%.3f)\n",
							i, p.body_idx_a, p.body_idx_b, p.shape_idx_a, p.shape_idx_b,
							p.b2a.w.x, p.b2a.w.y, p.b2a.w.z);
					}
				}
			}
			#endif
		}

		// Narrow phase -> uses collision pairs -> generates contacts
		{
			auto counters = m_gpu_integrator->Counters();
			auto dispatch = m_gpu_sort_and_sweep->CDDispatchArgs();
			auto col_pairs = m_gpu_sort_and_sweep->CollisionPairs();
			m_gpu_collision_detector->DetectCollisions(m_gpu->m_job, max_col_pairs, dispatch, col_pairs, counters, m_cache->m_shape_cache);

			#if PR_DBG_PHYSICS
			{
				std::vector<GpuResolveContact> out_contacts(max_col_pairs);
				std::vector<GpuPairDiag> out_diag(max_col_pairs);
				auto [contacts, diag] = m_gpu_collision_detector->Readback(m_gpu->m_job, counters, out_contacts, out_diag);
				assert(contacts.size() < max_col_pairs && "Hit capacity on contacts");

				if (dbg_logging)
				{
					fprintf(dbg_log, "  NarrowPhase: %d contacts, %d diag\n", static_cast<int>(contacts.size()), static_cast<int>(diag.size()));
					for (int i = 0; i != static_cast<int>(diag.size()) && i < 20; ++i)
					{
						auto& d = diag[i];
						fprintf(dbg_log, "    diag[%d]: body(%d,%d) shape(%d,%d) gjk=%d epa=%d hit=%d\n",
							i, d.body_idx_a, d.body_idx_b, d.shape_type_a, d.shape_type_b,
							d.gjk_iters, d.epa_iters, d.hit);
					}
					for (int i = 0; i != static_cast<int>(contacts.size()) && i < 20; ++i)
					{
						auto& c = contacts[i];
						fprintf(dbg_log, "    contact[%d]: body(%d,%d) axis=(%.4f,%.4f,%.4f) pt=(%.4f,%.4f,%.4f) depth=%.6f\n",
							i, c.body_idx_a, c.body_idx_b,
							c.axis.x, c.axis.y, c.axis.z,
							c.contact_point.x, c.contact_point.y, c.contact_point.z, c.depth);
					}
					fprintf(dbg_log, "\n");
					fflush(dbg_log);
					++dbg_frame;
				}
			}
			#endif
		}
		
		// Resolve -> uses contacts -> applies impulses to bodies
		{
			auto counters = m_gpu_integrator->Counters();
			auto dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
			auto contacts = m_gpu_collision_detector->Contacts();
			auto bodies = m_gpu_integrator->Bodies();
			m_gpu_resolver->Resolve(m_gpu->m_job, dt, max_col_pairs, dispatch, counters, contacts, bodies, m_materials->span());
		}

		// Readback dynamics from GPU and unpack into bodies
		{
			// This will wait on the gpu work to complete
			m_gpu_integrator->Readback(m_gpu->m_job, m_cache->m_rb_dynamics, {}, {});
			for (auto [body, i] : with_index(rigid_bodies))
				UnpackDynamics(m_cache->m_rb_dynamics[i], *body);
		}

		m_body_ptrs.resize(0);
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
}







	#if 0
	// Broad phase overlap query → narrow phase collision detection → impulse resolution.
	void Engine::DetectAndResolveCollisions(float dt)
	{

		#if PR_DBG
		static FILE* f_step = nullptr;
		if (!f_step) f_step = fopen("dump\\step_timing.log", "w");
		static int frame = 0;
		++frame;
		#endif

		// Nothing close enough to be colliding? Skip the GPU GJK step.
		if (col_pairs.empty())
			return;

		// Phase 2: Narrow phase collision detection.
		if (m_gpu_detect)
		{
			// GPU collision detection path
			m_gpu_collision_detector->DetectCollisions(m_gpu->m_job, col_pairs, shape_cache.m_shapes, shape_cache.m_verts, gpu_contacts, shape_cache.IsDirty());
			shape_cache.ClearDirty();

			#if PR_DBG
			if (f_step && col_pairs.size() > 10)
				fprintf(f_step, "[frame %d] GPU detect: %d contacts from %d pairs\n", frame, static_cast<int>(gpu_contacts.size()), static_cast<int>(col_pairs.size()));
			#endif
		}
		else
		{
			// CPU narrow phase path — use SAT-based Collide() for each broadphase pair
			for (auto const& bp : body_pairs)
			{
				auto c = RbContact{*bp.objA, *bp.objB};
				if (NarrowPhaseCollision(dt, c))
				{
					// Compare with CPU GJK for debugging
					#if PR_DBG
					{
						collision::Contact gjk_c;
						auto w2a = InvertOrthonormal(bp.objA->O2W());
						auto b2a = w2a * bp.objB->O2W();
						auto& sA = bp.objA->Shape();
						auto& sB = bp.objB->Shape();
						if (collision::GjkCollide(sA, m4x4::Identity(), sB, b2a, gjk_c))
						{
							static FILE* f = nullptr;
							if (!f) f = fopen("dump\\sat_vs_gjk.log", "w");
							if (f)
							{
								fprintf(f, "SAT: axis=(%.4f,%.4f,%.4f) depth=%.6f pt=(%.4f,%.4f,%.4f)\n",
									c.m_axis.x, c.m_axis.y, c.m_axis.z, c.m_depth, c.m_point.x, c.m_point.y, c.m_point.z);
								fprintf(f, "GJK: axis=(%.4f,%.4f,%.4f) depth=%.6f pt=(%.4f,%.4f,%.4f)\n",
									gjk_c.m_axis.x, gjk_c.m_axis.y, gjk_c.m_axis.z, gjk_c.m_depth, gjk_c.m_point.x, gjk_c.m_point.y, gjk_c.m_point.z);
								fprintf(f, "  bodyA pos=(%.4f,%.4f,%.4f) bodyB pos=(%.4f,%.4f,%.4f)\n\n",
									bp.objA->O2W().pos.x, bp.objA->O2W().pos.y, bp.objA->O2W().pos.z,
									bp.objB->O2W().pos.x, bp.objB->O2W().pos.y, bp.objB->O2W().pos.z);
								fflush(f);
							}
						}
					}
					#endif
					collision_queue.push_back(c);
				}
			}
		}

		// Phase 3: Convert GPU contacts to physics::Contact structs and resolve.
		if (m_gpu_detect)
		{
			for (int ci = 0; ci != static_cast<int>(gpu_contacts.size()); ++ci)
			{
				auto const& gc = gpu_contacts[ci];

				// Log GPU contact data for debugging
				{
					static FILE* f = nullptr;
					if (!f) f = fopen("dump\\gpu_contacts.log", "w");
					if (f)
					{
						auto const& bp = (gc.pair_index >= 0 && gc.pair_index < static_cast<int>(body_pairs.size())) ? body_pairs[gc.pair_index] : body_pairs[0];
						fprintf(f, "Contact[%d] pair=%d axis=(%.4f,%.4f,%.4f) pt=(%.4f,%.4f,%.4f) depth=%.6f\n",
							ci, gc.pair_index, gc.axis.x, gc.axis.y, gc.axis.z, gc.pt.x, gc.pt.y, gc.pt.z, gc.depth);
						fprintf(f, "  bodyA pos=(%.4f,%.4f,%.4f) bodyB pos=(%.4f,%.4f,%.4f)\n",
							bp.objA->O2W().pos.x, bp.objA->O2W().pos.y, bp.objA->O2W().pos.z,
							bp.objB->O2W().pos.x, bp.objB->O2W().pos.y, bp.objB->O2W().pos.z);
						fflush(f);
					}
				}

				// Validate the pair_index from GPU readback
				if (gc.pair_index < 0 || gc.pair_index >= static_cast<int>(body_pairs.size()))
				{
					OutputDebugStringA(FmtS("GPU contact[%d/%d]: pair_index=%d (out of range 0..%d), depth=%f\n",
						ci, static_cast<int>(gpu_contacts.size()), gc.pair_index, static_cast<int>(body_pairs.size()), gc.depth));
					continue;
				}

				auto const& bp = body_pairs[gc.pair_index];

				// Compare GPU GJK depth with CPU GJK depth
				{
					collision::Contact cpu_c;
					auto b2a_cmp = InvertOrthonormal(bp.objA->O2W()) * bp.objB->O2W();
					bool cpu_hit = collision::GjkCollide(bp.objA->Shape(), m4x4::Identity(), bp.objB->Shape(), b2a_cmp, cpu_c);
					static FILE* fcmp = nullptr;
					if (!fcmp) fcmp = fopen("dump\\gpu_vs_cpu_gjk.log", "w");
					if (fcmp)
					{
						fprintf(fcmp, "GPU: axis=(%.4f,%.4f,%.4f) depth=%.6f pt=(%.4f,%.4f,%.4f)\n",
							gc.axis.x, gc.axis.y, gc.axis.z, gc.depth, gc.pt.x, gc.pt.y, gc.pt.z);
						if (cpu_hit)
							fprintf(fcmp, "CPU: axis=(%.4f,%.4f,%.4f) depth=%.6f pt=(%.4f,%.4f,%.4f)\n",
								cpu_c.m_axis.x, cpu_c.m_axis.y, cpu_c.m_axis.z, cpu_c.m_depth, cpu_c.m_point.x, cpu_c.m_point.y, cpu_c.m_point.z);
						else
							fprintf(fcmp, "CPU: NO COLLISION\n");
						fprintf(fcmp, "  bodyB pos=(%.4f,%.4f,%.4f)\n\n", bp.objB->O2W().pos.x, bp.objB->O2W().pos.y, bp.objB->O2W().pos.z);
						fflush(fcmp);
					}
				}

				auto c = RbContact{ *bp.objA, *bp.objB, gc };

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
		}

		// Sort the collisionsby estimated time of impact so earlier collisions are resolved first.
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
					auto cpu_dynamics = std::vector<GpuRigidBody>(body_ptrs.size());
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
	#endif

// Debug: stashed pre-integration state for A/B comparison between Evolve() and EvolveCPU().
#if PR_DBG&&0
std::vector<GpuRigidBody> m_compare_dynamics;
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