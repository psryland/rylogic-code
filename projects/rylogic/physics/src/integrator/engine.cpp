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
#include "src/articulation/articulation_gpu_data.h"
#include "src/articulation/articulation_internal.h"
#include "src/compute/articulation_link_proxies_gpu.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/physics_types.h"
#include "src/compute/integrate_gpu.h"
#include "src/compute/sleep_gpu.h"
#include "src/compute/sweep_gpu.h"
#include "src/compute/collide_gpu.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_solver_gpu.h"
#include "src/compute/frame_output_gpu.h"
#include "src/constraint/constraint_gpu.h"
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
				m_profile.*Field += ElapsedMs(m_beg, Clock::now());
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
		GpuJob::RunHandle SubmitGpuJob(GpuJob& job, Engine::StepProfile& profile)
		{
			++profile.m_submission_count;
			if constexpr (PR_PHYSICS_PROFILE != 0)
			{
				auto run_profile = GpuJob::RunProfile{};
				auto run = job.Submit(&run_profile);
				profile.m_gpu_prepare_ms = run_profile.m_prepare_ms;
				profile.m_gpu_execute_ms = run_profile.m_execute_ms;
				return run;
			}
			else
			{
				return job.Submit();
			}
		}
		void CompleteGpuJob(GpuJob& job, GpuJob::RunHandle& run, Engine::StepProfile& profile)
		{
			++profile.m_wait_count;
			if constexpr (PR_PHYSICS_PROFILE != 0)
			{
				auto run_profile = GpuJob::RunProfile{};
				job.Complete(run, &run_profile);
				profile.m_gpu_wait_ms = run_profile.m_wait_ms;
				profile.m_gpu_reset_ms = run_profile.m_reset_ms;
				profile.m_gpu_run_ms =
					profile.m_gpu_prepare_ms +
					profile.m_gpu_execute_ms +
					profile.m_gpu_wait_ms +
					profile.m_gpu_reset_ms;
			}
			else
			{
				job.Complete(run);
			}
		}
		void RunGpuJob(GpuJob& job, Engine::StepProfile& profile)
		{
			auto run = SubmitGpuJob(job, profile);
			CompleteGpuJob(job, run, profile);
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

		// Return a stable diagnostic label for every known sticky midpoint failure state.
		char const* ArticulationIntegrationStatusName(int status)
		{
			switch (status)
			{
				case GpuArticulationIntegrationStatus_Success:
				{
					return "success";
				}
				case GpuArticulationIntegrationStatus_Singular:
				{
					return "singular";
				}
				case GpuArticulationIntegrationStatus_NonFinite:
				{
					return "non-finite";
				}
				case GpuArticulationIntegrationStatus_NonConverged:
				{
					return "non-converged";
				}
				default:
				{
					return "unknown";
				}
			}
		}

		// Return whether a compact non-negative range is contained by its gathered scalar stream.
		bool ArticulationOutputRangeValid(int offset, int count, size_t size)
		{
			return offset >= 0 && count >= 0 && static_cast<uint64_t>(offset) + static_cast<uint64_t>(count) <= size;
		}
	}

	// Readback resources and flags that must survive between GPU submission and CPU unpack.
	struct GpuBuffers
	{
		GpuFrameOutputReadback rb_output;
		bool emit_collisions = true;
		bool read_collision_events = false;
	};

	// Keep the opaque GPU readback state allocated once with the pending-step state.
	Engine::PendingStep::PendingStep()
		: m_bodies()
		, m_articulations()
		, m_articulation_ranges()
		, m_articulation_range_lookup()
		, m_buffers(new GpuBuffers())
		, m_run()
		, m_substep_seconds()
		, m_elapsed_seconds()
		, m_active()
		, m_submitted()
	{
	}

	// Start tracking a begin/complete step pair using stable copies of every caller-owned object pointer.
	void Engine::PendingStep::Begin(std::span<RigidBody*> bodies, std::span<Articulation*> articulations, float substep_seconds, float elapsed_seconds, bool sleeping_enabled)
	{
		if (bodies.data() != m_bodies.data() || bodies.size() != m_bodies.size())
			m_bodies.assign(bodies.begin(), bodies.end());

		// Sleeping is owned by the complete tree, so inactive trees are omitted before any topology, proxy, or GPU storage is packed.
		m_articulations.clear();
		m_articulations.reserve(articulations.size());
		for (auto* articulation : articulations)
		{
			if (articulation == nullptr)
				throw std::invalid_argument("Engine articulation inputs cannot contain null pointers");
			if ((!sleeping_enabled || articulation->NeverSleep()) && articulation->Sleeping())
				articulation->Wake();
			if (!articulation->Sleeping())
				m_articulations.push_back(articulation);
		}

		m_articulation_ranges.clear();
		m_articulation_range_lookup.clear();
		*m_buffers = {};
		m_run = {};
		m_substep_seconds = substep_seconds;
		m_elapsed_seconds = elapsed_seconds;
		m_active = true;
		m_submitted = false;
	}

	// Clear all per-step state once the GPU result has been consumed.
	void Engine::PendingStep::Clear()
	{
		m_bodies.clear();
		m_articulations.clear();
		m_articulation_ranges.clear();
		m_articulation_range_lookup.clear();
		*m_buffers = {};
		m_run = {};
		m_substep_seconds = 0.0f;
		m_elapsed_seconds = 0.0f;
		m_active = false;
		m_submitted = false;
	}

	Engine::Engine(EngineConfig const& config, IShaderCache* shader_cache, ID3D12Device4* existing_device)
		: m_config(config)
		, m_gpu(new Gpu(existing_device))
		, m_gpu_integrator(new GpuIntegrator(*m_gpu, m_config))
		, m_gpu_sleep_manager(new GpuSleepManager(*m_gpu, m_config))
		, m_gpu_sort_and_sweep(new GpuSortAndSweep(*m_gpu, m_config, shader_cache))
		, m_gpu_collision_detector(new GpuCollisionDetector(*m_gpu, m_config))
		, m_gpu_resolver(new GpuResolver(*m_gpu, m_config, shader_cache))
		, m_gpu_constraint_solver()
		, m_gpu_coupled_constraint_solver()
		, m_gpu_articulation_force_aba()
		, m_gpu_articulation_link_proxies()
		, m_gpu_articulation_midpoint()
		, m_gpu_frame_output(new GpuFrameOutput(*m_gpu))
		, m_gpu_selective_resolver(new GpuResolver(*m_gpu, m_config, shader_cache))
		, m_gpu_selective_refresher(new GpuSelectiveRefresher(*m_gpu, m_config))
		, m_materials(new MaterialMap)
		, m_cache(new EngineBufferCache())
		, m_pending_step()
		, m_constraints_active()
		, m_coupled_constraints_active()
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
		if (m_pending_step.m_active)
			throw std::runtime_error("Engine::Material cannot be modified while a step is pending");

		(*m_materials).Set(mat);
		m_gpu_resolver->MaterialsDirty();
		m_gpu_selective_resolver->MaterialsDirty();
	}

	// Update runtime-tunable engine configuration.
	EngineConfig const& Engine::Config() const
	{
		return m_config;
	}
	void Engine::Config(EngineConfig const& config)
	{
		if (m_pending_step.m_active)
			throw std::runtime_error("Engine::Config cannot be modified while a step is pending");

		m_config = config;
	}
	ID3D12Device4* Engine::Device() const
	{
		return *m_gpu;
	}

	// Drop all internally-cached references to caller-supplied data.
	void Engine::ResetCaches()
	{
		if (m_pending_step.m_active)
			throw std::runtime_error("Engine::ResetCaches cannot run while a step is pending");

		m_cache->Reset();
		m_gpu_coupled_constraint_solver.reset();
		m_gpu_constraint_solver.reset();
		m_constraints_active = false;
		m_coupled_constraints_active = false;
		m_last_collision_stats = {};
	}

	// Evolve the physics objects forward in time and resolve any collisions.
	void Engine::Step(float dt, std::span<RigidBody*> rigid_bodies, double time_s)
	{
		Step(StepInput{
			.m_bodies = rigid_bodies,
			.m_elapsed_seconds = dt,
			.m_time_s = time_s,
		});
	}

	// Evolve rigid bodies while resolving one persistent constraint collection in the same GPU submission.
	void Engine::Step(float dt, std::span<RigidBody*> rigid_bodies, ConstraintSet const& constraints, double time_s)
	{
		Step(StepInput{
			.m_bodies = rigid_bodies,
			.m_constraints = &constraints,
			.m_elapsed_seconds = dt,
			.m_time_s = time_s,
		});
	}

	// Evolve a complete frame using one GPU submission regardless of the requested internal substep count.
	void Engine::Step(StepInput const& input)
	{
		BeginStep(input);
		CompleteStep();
	}

	// Begin evolving the physics objects by submitting GPU work without waiting for it to finish.
	void Engine::BeginStep(float dt, std::span<RigidBody*> rigid_bodies, double time_s)
	{
		BeginStep(StepInput{
			.m_bodies = rigid_bodies,
			.m_elapsed_seconds = dt,
			.m_time_s = time_s,
		});
	}

	// Begin a constrained step while retaining the same single GPU submission and gathered readback contract.
	void Engine::BeginStep(float dt, std::span<RigidBody*> rigid_bodies, ConstraintSet const& constraints, double time_s)
	{
		BeginStep(StepInput{
			.m_bodies = rigid_bodies,
			.m_constraints = &constraints,
			.m_elapsed_seconds = dt,
			.m_time_s = time_s,
		});
	}

	// Submit a complete frame without waiting; intermediate substeps remain entirely GPU-resident.
	void Engine::BeginStep(StepInput const& input)
	{
		BeginStepInternal(input);
	}

	// Submit one frame with optional persistent constraints and internal GPU substeps.
	void Engine::BeginStepInternal(StepInput const& input)
	{
		// Notes:
		//  - There is a limitation on the number of collision pairs that can be generated per frame.
		//    If this limit becomes a problem, the options are increase the max number of collision pairs
		//    or run Engine::Step() multiple times on "islands" of physics objects
		if (m_pending_step.m_active)
			throw std::runtime_error("Engine::BeginStep called while a previous step is pending");
		if (input.m_substep_count < 1 || input.m_substep_count > m_config.max_internal_substeps)
			throw std::runtime_error(std::format("Engine::BeginStep substep count {} is outside the configured range [1, {}]", input.m_substep_count, m_config.max_internal_substeps));
		if (m_config.max_collision_pairs < 1)
			throw std::runtime_error("EngineConfig::max_collision_pairs must be positive");
		if (m_config.max_collision_events < 0)
			throw std::runtime_error("EngineConfig::max_collision_events must be non-negative");
		if (!std::isfinite(input.m_elapsed_seconds) || input.m_elapsed_seconds < 0.0f)
			throw std::runtime_error("Engine::BeginStep elapsed time must be finite and non-negative");
		if (m_config.sleeping_enabled &&
			(!std::isfinite(m_config.sleep_velocity_threshold_lin) || m_config.sleep_velocity_threshold_lin < 0.0f ||
			 !std::isfinite(m_config.sleep_velocity_threshold_ang) || m_config.sleep_velocity_threshold_ang < 0.0f ||
			 !std::isfinite(m_config.sleep_delay_s) || m_config.sleep_delay_s < 0.0f))
			throw std::runtime_error("Engine articulation sleep thresholds and delay must be finite and non-negative");
		auto const dt = input.m_elapsed_seconds / input.m_substep_count;
		if (input.m_constraints != nullptr)
			WakeCoupledConstraintArticulations(*input.m_constraints, input.m_articulations);

		m_pending_step.Begin(input.m_bodies, input.m_articulations, dt, input.m_elapsed_seconds, m_config.sleeping_enabled);
		auto bodies = std::span{ m_pending_step.m_bodies };
		auto articulations = std::span{ m_pending_step.m_articulations };

		if (bodies.empty() && articulations.empty())
		{
			// Empty input also releases retained optional articulation buffers after their last completed use.
			if (m_gpu_articulation_midpoint != nullptr)
				m_gpu_articulation_midpoint->Upload(m_gpu->m_job, GpuArticulationUpload{});
			if (m_gpu_coupled_constraint_solver != nullptr)
				m_gpu_coupled_constraint_solver->Deactivate();

			m_constraints_active = false;
			m_coupled_constraints_active = false;
			m_last_step_profile = {};
			m_last_step_profile.m_substep_count = input.m_substep_count;
			m_last_collision_stats = {};
			return;
		}

		#if PR_PIX_ENABLED
		static bool capture = false;
		::pr::compute::pix::CaptureScope pix_capture("E:/Dump/PIXCaptures/Physics.wpix", capture);
		capture = false;
		#endif

		m_last_step_profile = {};
		m_last_step_profile.m_substep_count = input.m_substep_count;
		m_last_collision_stats = {};

		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_new_frame_ms>(m_last_step_profile);
			m_cache->NewFrame(bodies);
		}

		// Pack all bodies into a GPU-friendly format
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_pack_ms>(m_last_step_profile);
			Pack(bodies);
		}

		// Flatten each independent tree once; all internal substeps retain these immutable topology and force ranges on the GPU.
		auto articulation_upload = GpuArticulationUpload{};
		auto articulation_collision_exclusions = std::vector<GpuCollisionExclusion>{};
		auto articulation_contacts_active = false;
		if (!articulations.empty())
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_articulation_pack_ms>(m_last_step_profile);
			articulation_upload = PackGpuArticulations(articulations);
			m_pending_step.m_articulation_ranges.reserve(articulation_upload.m_articulations.size());
			m_pending_step.m_articulation_range_lookup.reserve(articulation_upload.m_articulations.size());
			for (auto const& articulation : articulation_upload.m_articulations)
			{
				auto const identity = static_cast<uint64_t>(articulation.identity_low) | (static_cast<uint64_t>(articulation.identity_high) << 32);
				auto const range_index = isize(m_pending_step.m_articulation_ranges);
				m_pending_step.m_articulation_ranges.push_back(PendingStep::ArticulationOutputRange{
					.m_identity = identity,
					.m_position_offset = articulation.position_offset,
					.m_position_count = articulation.position_count,
					.m_velocity_offset = articulation.velocity_offset,
					.m_velocity_count = articulation.velocity_count,
					.m_proxy_body_offset = m_cache->RigidBodyCount() + articulation.link_offset,
					.m_link_count = articulation.link_count,
				});
				m_pending_step.m_articulation_range_lookup.emplace(identity, range_index);
			}

			// Append one hidden body per link so every GPU force producer can target rigid bodies and articulation links through the same accumulator ABI.
			auto proxy_shape_ids = std::vector<int>{};
			proxy_shape_ids.reserve(articulation_upload.m_links.size());
			for (auto const* articulation : articulations)
			{
				for (int link_index = 0; link_index != articulation->LinkCount(); ++link_index)
				{
					auto const& link = articulation->LinkDescription(articulation->LinkAt(link_index));
					proxy_shape_ids.push_back(link.m_shape != nullptr ? m_cache->m_shape_cache.GetOrAdd(*link.m_shape) : -1);
					articulation_contacts_active |= link.m_shape != nullptr;
				}
			}
			auto proxies = PackGpuArticulationProxies(articulation_upload, articulations, proxy_shape_ids, m_cache->RigidBodyCount());
			m_cache->m_rb_dynamics.insert(m_cache->m_rb_dynamics.end(), proxies.begin(), proxies.end());

			// Adjacent-link suppression needs one pair per tree edge, while broader same-tree policy remains compactly encoded in each proxy body.
			articulation_collision_exclusions.reserve(articulation_upload.m_links.size());
			for (int articulation_index = 0; articulation_index != isize(articulations); ++articulation_index)
			{
				auto const* articulation = articulations[articulation_index];
				auto const& packed_articulation = articulation_upload.m_articulations[articulation_index];
				for (int local_link_index = 0; local_link_index != articulation->LinkCount(); ++local_link_index)
				{
					auto const packed_link_index = packed_articulation.link_offset + local_link_index;
					auto const parent_link_index = articulation_upload.m_links[packed_link_index].parent_link_index;
					auto const& desc = articulation->LinkDescription(articulation->LinkAt(local_link_index));
					if (parent_link_index < 0 || desc.m_collide_parent || desc.m_shape == nullptr)
						continue;

					auto const parent_local_link_index = parent_link_index - packed_articulation.link_offset;
					auto const& parent_desc = articulation->LinkDescription(articulation->LinkAt(parent_local_link_index));
					if (parent_desc.m_shape == nullptr)
						continue;

					auto const body_index = m_cache->RigidBodyCount() + packed_link_index;
					auto const parent_body_index = m_cache->RigidBodyCount() + parent_link_index;
					articulation_collision_exclusions.push_back(GpuCollisionExclusion{
						.body_idx_a_plus_one = static_cast<uint32_t>(std::min(body_index, parent_body_index) + 1),
						.body_idx_b_plus_one = static_cast<uint32_t>(std::max(body_index, parent_body_index) + 1),
					});
				}
			}
		}

		// Resolve stable identities before submission and retain only compact transfer data for the GPU upload.
		auto constraint_upload = GpuConstraintUpload{};
		if (input.m_constraints != nullptr)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_constraint_pack_ms>(m_last_step_profile);
			constraint_upload = PackGpuConstraints(*input.m_constraints, BodyRemap(bodies, articulations), articulation_collision_exclusions);
		}
		else
		{
			constraint_upload.m_collision_exclusions = BuildGpuCollisionExclusions({}, articulation_collision_exclusions);
		}
		m_constraints_active = constraint_upload.m_rigid_active_count != 0;
		m_coupled_constraints_active = constraint_upload.m_coupled_active_count != 0;
		if (!m_constraints_active && !m_coupled_constraints_active && m_gpu_constraint_solver != nullptr)
			m_gpu_constraint_solver->Deactivate();

		// Persistent constraints may wake sleeping bodies, while active articulations always require their independent pure-tree dispatch.
		if (m_config.sleeping_enabled && m_cache->AwakeDynamicCount() == 0 && !m_constraints_active && !m_coupled_constraints_active && articulations.empty())
		{
			if (m_gpu_articulation_midpoint != nullptr)
				m_gpu_articulation_midpoint->Upload(m_gpu->m_job, GpuArticulationUpload{});
			if (m_gpu_articulation_link_proxies != nullptr)
				m_gpu_articulation_link_proxies->Upload(m_gpu->m_job);
			return;
		}

		// Upload -> transfers staged body dynamics and resets GPU counters
		if (m_cache->BodyCount() != 0)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_upload_ms>(m_last_step_profile);
			Upload();
		}

		// Shared stable-slot streams back both the independent and articulation-coupled constraint lanes.
		if (m_constraints_active || m_coupled_constraints_active)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_constraint_upload_ms>(m_last_step_profile);
			if (m_gpu_constraint_solver == nullptr)
				m_gpu_constraint_solver.reset(new GpuConstraintSolver(*m_gpu, m_config));
			m_gpu_constraint_solver->Upload(m_gpu->m_job, constraint_upload);
		}

		// Instantiate and populate articulation resources only when a frame actually contains reduced-coordinate trees.
		auto articulation_output = GpuArticulationMidpointOutput{};
		if (!articulations.empty())
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_articulation_upload_ms>(m_last_step_profile);
			if (m_gpu_articulation_force_aba == nullptr)
				m_gpu_articulation_force_aba.reset(new GpuArticulationForceAba(*m_gpu));
			if (m_gpu_articulation_midpoint == nullptr)
				m_gpu_articulation_midpoint.reset(new GpuArticulationMidpoint(*m_gpu_articulation_force_aba));
			if (m_gpu_articulation_link_proxies == nullptr)
				m_gpu_articulation_link_proxies.reset(new GpuArticulationLinkProxies(*m_gpu_articulation_force_aba, m_config));
			if (!m_gpu_articulation_midpoint->Upload(m_gpu->m_job, articulation_upload))
				throw std::runtime_error("GPU articulation midpoint rejected a non-empty packed forest");
			m_gpu_articulation_link_proxies->Upload(m_gpu->m_job);

			articulation_output = m_gpu_articulation_midpoint->Output();
		}
		else if (m_gpu_articulation_midpoint != nullptr)
		{
			m_gpu_articulation_midpoint->Upload(m_gpu->m_job, articulation_upload);
			if (m_gpu_articulation_link_proxies != nullptr)
				m_gpu_articulation_link_proxies->Upload(m_gpu->m_job);
		}

		// Coupled topology depends on both preceding uploads and remains absent for rigid-only or articulation-only frames.
		if (m_coupled_constraints_active)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_constraint_upload_ms>(m_last_step_profile);
			if (m_gpu_articulation_force_aba == nullptr || m_gpu_articulation_link_proxies == nullptr || m_gpu_constraint_solver == nullptr)
				throw std::logic_error("Coupled constraint upload requires matching shared constraint and articulation resources");
			if (m_gpu_coupled_constraint_solver == nullptr)
				m_gpu_coupled_constraint_solver.reset(new GpuCoupledConstraintSolver(*m_gpu, *m_gpu_constraint_solver, *m_gpu_articulation_force_aba, *m_gpu_articulation_link_proxies, m_config));
			if (!m_gpu_coupled_constraint_solver->Upload(m_gpu->m_job, constraint_upload, articulation_upload))
				throw std::logic_error("GPU coupled constraint solver rejected active packed topology");
		}
		else if (m_gpu_coupled_constraint_solver != nullptr)
		{
			m_gpu_coupled_constraint_solver->Deactivate();
		}

		// Allocate optional event storage only when the caller has requested collision records.
		auto const collect_collision_events = !bodies.empty() && static_cast<bool>(Collisions);
		auto const event_capacity = collect_collision_events ? m_config.max_collision_events : 0;
		m_gpu_frame_output->BeginFrame(m_gpu->m_job, m_cache->RigidBodyCount(), event_capacity, input.m_substep_count, articulation_output);

		// Record every substep into the same command list so state, warm starts, and counters stay GPU-resident.
		for (int substep_index = 0; substep_index != input.m_substep_count; ++substep_index)
		{
			if (m_cache->BodyCount() != 0)
			{
				// Upload resets the first substep counters; later substeps reset only transient collision state.
				if (substep_index != 0)
					m_gpu_integrator->ResetCounters(m_gpu->m_job);

				// Restore CPU frame-constant inputs before GPU modules add state-dependent forces.
				m_gpu_integrator->SeedWorkingForces(m_gpu->m_job, m_cache->BodyCount());
				{
					auto profile_scope = ProfileScope<&Engine::StepProfile::m_external_forces_ms>(m_last_step_profile);
					auto const substep_time_s = input.m_time_s + static_cast<double>(dt) * substep_index;
					ApplyExternalForces(dt, substep_time_s, substep_index, input.m_substep_count);
				}

				// Gather link forces before either lane consumes its current-substep accumulators.
				auto articulation_external_forces = !articulations.empty()
					? m_gpu_articulation_link_proxies->GatherForces(m_gpu->m_job, m_gpu_integrator->Bodies().get())
					: nullptr;

				// Advance both independent prediction lanes before any shared collision or constraint work so every broadphase consumer sees current-substep poses.
				if (!bodies.empty())
				{
					auto profile_scope = ProfileScope<&Engine::StepProfile::m_integrate_ms>(m_last_step_profile);
					Integrate(dt);
				}

				if (!articulations.empty())
				{
					auto profile_scope = ProfileScope<&Engine::StepProfile::m_articulation_integrate_ms>(m_last_step_profile);
					m_gpu_articulation_midpoint->Integrate(m_gpu->m_job, dt, articulation_external_forces);
					m_gpu_articulation_link_proxies->Refresh(m_gpu->m_job, *m_gpu_integrator, m_cache->BroadphaseSortAxis());
				}

				if (!bodies.empty() || articulation_contacts_active || m_coupled_constraints_active)
				{
					// Broadphase admits shaped proxies, while rigid-only consumers retain the caller-owned body prefix.
					if (!bodies.empty())
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleepwake_ms>(m_last_step_profile);
						SleepWake();
					}
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_broadphase_ms>(m_last_step_profile);
						BroadPhase(m_config.sleeping_enabled, constraint_upload.m_collision_exclusions.m_slots);
					}
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_collide_ms>(m_last_step_profile);
						Collide();
					}
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_resolve_ms>(m_last_step_profile);
						Resolve(dt);
					}

					// Publish the main coupled solve before selective passes recompile rows against the corrected articulation configuration.
					if (m_coupled_constraints_active)
						m_gpu_articulation_link_proxies->Refresh(m_gpu->m_job, *m_gpu_integrator, m_cache->BroadphaseSortAxis());
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_selective_ms>(m_last_step_profile);
						if (!bodies.empty())
							SelectiveRefresh(dt);
					}
					{
						auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleepupdate_ms>(m_last_step_profile);
						if (!bodies.empty())
							SleepUpdate(dt);
					}

					// Preserve raw capacity counters and resolved collision records before transient buffers are reused.
					if (!bodies.empty() || articulation_contacts_active)
						CaptureSubstepOutput(substep_index, input.m_substep_count, collect_collision_events);
				}
			}
		}

		// ReadBody -> read back body dynamics and contact data
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_readback_ms>(m_last_step_profile);
			Readback(*m_pending_step.m_buffers, articulation_output);
		}

		// Submit all GPU work for this step without waiting so callers can overlap other CPU work.
		m_pending_step.m_run = SubmitGpuJob(m_gpu->m_job, m_last_step_profile);
		m_pending_step.m_submitted = true;
	}

	// Complete a previously-begun step and unpack the GPU results into the caller-owned bodies.
	void Engine::CompleteStep()
	{
		if (!m_pending_step.m_active)
			throw std::runtime_error("Engine::CompleteStep called without a pending step");

		if (!m_pending_step.m_submitted)
		{
			m_pending_step.Clear();
			return;
		}

		CompleteGpuJob(m_gpu->m_job, m_pending_step.m_run, m_last_step_profile);

		// Unpack the results back into the caller-owned bodies
		try
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_unpack_ms>(m_last_step_profile);
			Unpack(
				*m_pending_step.m_buffers,
				m_pending_step.m_bodies,
				m_pending_step.m_articulations,
				m_pending_step.m_articulation_ranges,
				m_pending_step.m_substep_seconds,
				m_pending_step.m_elapsed_seconds
			);
		}
		catch (...)
		{
			m_pending_step.Clear();
			throw;
		}

		m_pending_step.Clear();
	}

	// Wait for a pending step during terminal cleanup without updating bodies or reusing command state.
	void Engine::AbandonStep()
	{
		if (!m_pending_step.m_active)
			return;

		if (m_pending_step.m_submitted)
			m_gpu->m_job.Abandon(m_pending_step.m_run);

		m_pending_step.Clear();
	}

	// Explicitly initialise missing sleep islands for newly-created sleeping bodies.
	void Engine::UpdateSleepIslands(std::span<RigidBody*> rigid_bodies)
	{
		if (m_pending_step.m_active)
			throw std::runtime_error("Engine::UpdateSleepIslands cannot run while a step is pending");

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
			m_cache->NewFrame(rigid_bodies);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_pack_ms>(m_last_step_profile);
			Pack(rigid_bodies);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_upload_ms>(m_last_step_profile);
			Upload();
		}
		m_gpu_frame_output->BeginFrame(m_gpu->m_job, m_cache->RigidBodyCount(), 0, 1);
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
		CaptureSubstepOutput(0, 1, false);
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_readback_ms>(m_last_step_profile);
			Readback(buffers, GpuArticulationMidpointOutput{});
		}
		{
			RunGpuJob(m_gpu->m_job, m_last_step_profile);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_unpack_ms>(m_last_step_profile);
			Unpack(buffers, rigid_bodies, {}, {}, 0.0f, 0.0f);
		}
	}

	// Pack the body data into GPU buffers for the current frame.
	void Engine::Pack(std::span<RigidBody*> rigid_bodies)
	{
		for (auto body : rigid_bodies)
		{
			// Populate the shape cache
			auto shape_id = m_cache->m_shape_cache.GetOrAdd(body->Shape());

			// Clear prior-frame collision state only in detached staging so a rejected mixed frame cannot mutate caller-owned bodies.
			auto dyn = PackDynamics(*body, shape_id);
			dyn.state_flags = static_cast<uint32_t>(SetBits(static_cast<ERigidBodyStateFlags>(dyn.state_flags), ERigidBodyStateFlags::Collided, false));
			m_cache->CountAwakeDynamic(*body);
			m_cache->PackSleepIsland(*body, dyn);
			m_cache->AddBroadphaseSortSample(dyn);
			m_cache->m_rb_dynamics.push_back(dyn);
		}
		m_cache->FinaliseBroadphaseSortAxis();
		m_cache->FinaliseRigidBodyPack();
	}

	// Upload staged body data into GPU buffers for the current frame.
	void Engine::Upload()
	{
		m_gpu_integrator->Upload(m_gpu->m_job, m_cache->m_rb_dynamics);
		m_gpu_sleep_manager->Upload(m_gpu->m_job, m_cache->m_sleep_islands);

		// The broadphase reads shape data to expand compound bodies into their convex leaves, so shapes
		// must be resident before the sweep rather than only before the narrowphase.
		m_gpu_collision_detector->UploadShapes(m_gpu->m_job, m_cache->m_shape_cache);
	}

	// Apply user-supplied GPU forces before one internal substep.
	void Engine::ApplyExternalForces(float dt, double time_s, int substep_index, int substep_count)
	{
		if (!ExternalForces)
			return;

		auto bodies = m_gpu_integrator->Bodies();
		auto args = ExternalForceArgs{
			.m_job = m_gpu->m_job,
			.m_bodies = bodies.get(),
			.m_body_count = m_cache->BodyCount(),
			.m_rigid_body_count = m_cache->RigidBodyCount(),
			.m_dt = dt,
			.m_time_s = time_s,
			.m_substep_index = substep_index,
			.m_substep_count = substep_count,
		};
		ExternalForces(*this, args);
	}

	// Resolve a stable articulation link to its hidden body index while a frame's force targets are live.
	int Engine::ArticulationLinkStepIndex(ArticulationId articulation_id, LinkHandle link) const
	{
		if (!m_pending_step.m_active)
			throw std::logic_error("Articulation link step indices are only available during a pending engine step");

		auto const iter = m_pending_step.m_articulation_range_lookup.find(articulation_id.m_value);
		if (iter == m_pending_step.m_articulation_range_lookup.end())
			return -1;

		auto const range_index = iter->second;
		auto const& range = m_pending_step.m_articulation_ranges[range_index];
		auto const* articulation = m_pending_step.m_articulations[range_index];
		if (link.m_index >= static_cast<uint32_t>(range.m_link_count) || articulation->LinkAt(static_cast<int>(link.m_index)) != link)
			throw std::invalid_argument("Articulation link handle does not belong to the resolved tree");

		return range.m_proxy_body_offset + static_cast<int>(link.m_index);
	}

	// Apply forces, evolve body dynamics forward in time, and generate AABBs for broadphase.
	void Engine::Integrate(float dt)
	{
		auto body_count = m_cache->RigidBodyCount();
		m_gpu_integrator->Integrate(m_gpu->m_job, body_count, dt, m_cache->BroadphaseSortAxis());

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

	// Broadphase collision detection with optional connected-body pair suppression.
	void Engine::BroadPhase(bool sleeping_enabled, std::span<GpuCollisionExclusion const> collision_exclusions)
	{
		auto body_count = m_cache->BodyCount();

		// GPU broadphase is only useful when GPU detect will consume the pairs
		auto counters = m_gpu_integrator->Counters();
		auto aabb_sort = m_gpu_integrator->AABBSortAxis();
		auto aabb_idx = m_gpu_integrator->AABBBodyIndices();
		auto aabb_box = m_gpu_integrator->AABBBoxes();
		auto bodies = m_gpu_integrator->Bodies();
		auto sleep_islands = m_gpu_sleep_manager->SleepIslands();
		auto sleep_island_count = m_cache->SleepIslandCount();
		auto shapes = m_gpu_collision_detector->Shapes();
		if (body_count != 0)
			m_gpu_sort_and_sweep->Sort(m_gpu->m_job, body_count, aabb_sort, aabb_idx);
		m_gpu_sort_and_sweep->Sweep(m_gpu->m_job, body_count, m_config.max_collision_pairs, counters, aabb_idx, aabb_box, bodies, shapes, sleep_island_count, sleep_islands, sleeping_enabled, collision_exclusions);

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
		auto body_count = m_cache->BodyCount();
		auto rigid_body_count = m_cache->RigidBodyCount();

		auto counters = m_gpu_integrator->Counters();
		auto dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		auto contacts = m_gpu_collision_detector->Contacts();
		auto bodies = m_gpu_integrator->Bodies();
		auto* constraint_solver = m_constraints_active ? m_gpu_constraint_solver.get() : nullptr;
		auto* coupled_constraint_solver = m_coupled_constraints_active ? m_gpu_coupled_constraint_solver.get() : nullptr;
		m_gpu_resolver->Resolve(m_gpu->m_job, dt, body_count, rigid_body_count, m_config.max_collision_pairs, dispatch, counters, contacts, bodies, m_materials->span(), 1.0f, -1, -1, 1.0f, false, constraint_solver, coupled_constraint_solver);

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

		// The selective pass is intended for local residual stack cleanup. Large body counts or dense contact graphs usually mean the compact
		// set is broad enough that a full substep is the right tool, so avoid paying the extra GPU passes by default. Contact stats are from
		// the previous frame because the current counters stay GPU-resident until readback.
		if (m_config.selective_refresh_body_limit > 0 && body_count > m_config.selective_refresh_body_limit)
			return;
		if (m_config.selective_refresh_contact_limit > 0 && m_last_collision_stats.m_contact_count > m_config.selective_refresh_contact_limit)
			return;

		auto const full_max_pairs = m_config.max_collision_pairs;
		auto const max_pairs = Clamp(m_config.selective_refresh_max_pairs, 1, full_max_pairs);
		auto const max_contacts = max_pairs;
		auto full_counters = m_gpu_integrator->Counters();
		auto full_pairs = m_gpu_sort_and_sweep->CollisionPairs();
		auto full_pair_dispatch = m_gpu_sort_and_sweep->CDDispatchArgs();
		auto bodies = m_gpu_integrator->Bodies();
		auto source_counters = full_counters;
		auto source_contacts = m_gpu_collision_detector->Contacts();
		auto source_dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		auto source_max_contacts = full_max_pairs;
		auto solver_iterations = m_config.selective_refresh_solver_iterations;
		auto* constraint_solver = m_constraints_active ? m_gpu_constraint_solver.get() : nullptr;
		auto* coupled_constraint_solver = m_coupled_constraints_active ? m_gpu_coupled_constraint_solver.get() : nullptr;
		if (m_config.selective_refresh_adaptive_body_limit > 0 && body_count <= m_config.selective_refresh_adaptive_body_limit)
			solver_iterations = std::max(solver_iterations, m_config.selective_refresh_adaptive_solver_iterations);

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
				full_pair_dispatch,
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
				body_count,
				work_set.m_max_contacts,
				work_set.m_resolve_dispatch,
				work_set.m_counters,
				work_set.m_contacts,
				bodies,
				m_materials->span(),
				m_config.selective_refresh_bias_scale,
				solver_iterations,
				m_config.selective_refresh_position_iterations,
				m_config.selective_refresh_restitution_scale,
				m_config.selective_refresh_resolve_support_only,
				constraint_solver,
				coupled_constraint_solver,
				true);

			// Each continuation may change generalized coordinates, so the next pass must consume current link frames.
			if (coupled_constraint_solver != nullptr)
				m_gpu_articulation_link_proxies->Refresh(m_gpu->m_job, *m_gpu_integrator, m_cache->BroadphaseSortAxis());

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
		auto contact_dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		m_gpu_sleep_manager->SleepUpdate(m_gpu->m_job, dt, body_count, island_count, m_config.max_collision_pairs, counters, contacts, contact_dispatch, bodies);
	}

	// Append aggregate counters and optional collision records for one completed GPU substep.
	void Engine::CaptureSubstepOutput(int substep_index, int substep_count, bool collect_events)
	{
		auto counters = m_gpu_integrator->Counters();
		auto contacts = m_gpu_collision_detector->Contacts();
		auto contact_order = m_gpu_resolver->ContactOrder();
		auto contact_dispatch = m_gpu_collision_detector->ResolveDispatchArgs();
		m_gpu_frame_output->CaptureSubstep(
			m_gpu->m_job,
			m_config.max_collision_pairs,
			m_config.max_collision_pairs,
			substep_index,
			substep_count,
			collect_events,
			counters.get(),
			contacts.get(),
			contact_order,
			contact_dispatch.get());
	}

	// Gather selected output and record exactly one core GPU-to-CPU copy.
	void Engine::Readback(GpuBuffers& buffers, GpuArticulationMidpointOutput const& articulations)
	{
		auto body_count = m_cache->RigidBodyCount();
		auto bodies = m_gpu_integrator->Bodies();
		buffers.read_collision_events = buffers.emit_collisions && static_cast<bool>(Collisions);
		buffers.rb_output = m_gpu_frame_output->GatherAndReadback(m_gpu->m_job, body_count, bodies.get(), articulations);
		++m_last_step_profile.m_readback_copy_count;
	}

	// Validate the complete gathered frame before publishing rigid or articulation state.
	void Engine::Unpack(GpuBuffers const& buffers, std::span<RigidBody*> rigid_bodies, std::span<Articulation*> articulations, std::span<PendingStep::ArticulationOutputRange const> articulation_ranges, float articulation_substep_seconds, float articulation_elapsed_seconds)
	{
		auto body_count = m_cache->RigidBodyCount();
		auto max_contacts = m_config.max_collision_pairs;

		auto const& output_header = GpuFrameOutput::Header(buffers.rb_output);
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_readback_access_ms>(m_last_step_profile);
			m_last_collision_stats = Engine::CollisionStats{
				.m_pair_count = output_header.max_pair_count,
				.m_contact_count = output_header.max_contact_count,
				.m_max_pairs = m_config.max_collision_pairs,
				.m_max_contacts = max_contacts,
				.m_event_count = output_header.event_count,
				.m_event_capacity = output_header.event_capacity,
				.m_pair_limit_substep = output_header.pair_limit_substep,
				.m_contact_limit_substep = output_header.contact_limit_substep,
				.m_event_overflow_substep = output_header.event_overflow_substep,
			};
			CheckCollisionCapacity(m_last_collision_stats);
		}

		// Validate every tree, identity, status, range, and scalar before any collision callback or caller-owned state mutation.
		auto articulation_outputs = std::vector<detail::ArticulationIntegrationOutput>{};
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_articulation_unpack_ms>(m_last_step_profile);
			auto const output_articulations = GpuFrameOutput::Articulations(buffers.rb_output);
			auto const output_positions = GpuFrameOutput::Positions(buffers.rb_output);
			auto const output_velocities = GpuFrameOutput::Velocities(buffers.rb_output);
			auto const output_accelerations = GpuFrameOutput::Accelerations(buffers.rb_output);
			if (articulations.size() != articulation_ranges.size() || output_articulations.size() != articulations.size())
				throw std::runtime_error("GPU frame output articulation count does not match the pending forest");

			articulation_outputs.reserve(articulations.size());
			auto position_cursor = 0;
			auto velocity_cursor = 0;
			for (int articulation_index = 0; articulation_index != isize(articulations); ++articulation_index)
			{
				auto* articulation = articulations[articulation_index];
				auto const& range = articulation_ranges[articulation_index];
				auto const& gathered = output_articulations[articulation_index];
				auto const gathered_identity = static_cast<uint64_t>(gathered.identity_low) | (static_cast<uint64_t>(gathered.identity_high) << 32);
				if (articulation == nullptr || articulation->Id().m_value != range.m_identity || gathered_identity != range.m_identity)
					throw std::runtime_error(std::format("GPU frame output articulation identity mismatch at packed index {}", articulation_index));
				if (range.m_position_offset != position_cursor || range.m_velocity_offset != velocity_cursor ||
					!ArticulationOutputRangeValid(range.m_position_offset, range.m_position_count, output_positions.size()) ||
					!ArticulationOutputRangeValid(range.m_velocity_offset, range.m_velocity_count, output_velocities.size()) ||
					!ArticulationOutputRangeValid(range.m_velocity_offset, range.m_velocity_count, output_accelerations.size()))
					throw std::runtime_error(std::format("GPU frame output articulation range mismatch at packed index {}", articulation_index));
				if (gathered.status != GpuArticulationIntegrationStatus_Success)
					throw std::runtime_error(std::format(
						"GPU articulation {} failed implicit-midpoint integration with {} status after {} iterations (residual {})",
						range.m_identity,
						ArticulationIntegrationStatusName(gathered.status),
						gathered.iteration_count,
						gathered.residual));
				if (gathered.iteration_count < 0 || !std::isfinite(gathered.residual) || gathered.residual < 0.0f)
					throw std::runtime_error(std::format("GPU articulation {} returned invalid convergence diagnostics", range.m_identity));

				auto output = detail::ArticulationIntegrationOutput{
					.m_root_to_world = UnpackGpuTransform(gathered.root_to_world),
					.m_positions = output_positions.subspan(range.m_position_offset, range.m_position_count),
					.m_velocities = output_velocities.subspan(range.m_velocity_offset, range.m_velocity_count),
					.m_accelerations = output_accelerations.subspan(range.m_velocity_offset, range.m_velocity_count),
					.m_substep_seconds = articulation_substep_seconds,
				};
				detail::ValidateArticulationIntegrationOutput(*articulation, output);
				articulation_outputs.push_back(output);
				position_cursor += range.m_position_count;
				velocity_cursor += range.m_velocity_count;
			}
			if (position_cursor != isize(output_positions) || velocity_cursor != isize(output_velocities) || velocity_cursor != isize(output_accelerations))
				throw std::runtime_error("GPU frame output generalized streams are not partitioned by the pending articulation forest");
		}

		auto const output_bodies = GpuFrameOutput::Bodies(buffers.rb_output);
		if (body_count != 0)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_body_readback_copy_ms>(m_last_step_profile);
			std::memcpy(m_cache->m_rb_dynamics.data(), output_bodies.data(), body_count * sizeof(GpuRigidBody));
		}

		// Before updating the bodies with new dynamics, raise the collision events
		auto const event_count = std::min(output_header.event_count, output_header.event_capacity);
		if (event_count != 0 && buffers.read_collision_events)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_collision_events_ms>(m_last_step_profile);
			m_cache->m_contacts_cpu.resize(0);
			m_cache->m_contacts_cpu.reserve(event_count);
			auto const events = GpuFrameOutput::Events(buffers.rb_output);
			for (int event_index = 0; event_index != event_count; ++event_index)
			{
				auto const& collision_event = events[event_index];
				auto const& contact = collision_event.contact;
				if (contact.body_idx_a < 0 || contact.body_idx_a >= body_count || contact.body_idx_b < 0 || contact.body_idx_b >= body_count)
					throw std::runtime_error("GPU frame output returned an invalid collision endpoint");

				auto& cpu_contact = m_cache->m_contacts_cpu.emplace_back(*rigid_bodies[contact.body_idx_a], *rigid_bodies[contact.body_idx_b], contact);
				cpu_contact.m_substep_index = collision_event.substep_index;
			}

			Collisions(*this, m_cache->m_contacts_cpu);
		}

		// Unpack the GPU results into the RigidBody objects
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_sleep_island_unpack_ms>(m_last_step_profile);
			for (int i = 0; i != body_count; ++i)
				m_cache->UnpackSleepIsland(m_cache->m_rb_dynamics[i]);
		}
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_body_unpack_ms>(m_last_step_profile);
			for (auto [body, i] : with_index(rigid_bodies))
				UnpackDynamics(m_cache->m_rb_dynamics[i], *body);
		}

		// Publish the already-validated forest only after callbacks and rigid output validation have succeeded.
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_articulation_unpack_ms>(m_last_step_profile);
			for (int articulation_index = 0; articulation_index != isize(articulations); ++articulation_index)
			{
				detail::CommitArticulationIntegrationOutput(*articulations[articulation_index], articulation_outputs[articulation_index]);
				if (m_config.sleeping_enabled)
				{
					articulations[articulation_index]->UpdateSleeping(
						articulation_elapsed_seconds,
						m_config.sleep_velocity_threshold_lin,
						m_config.sleep_velocity_threshold_ang,
						m_config.sleep_delay_s);
				}
			}
		}

		if constexpr (PR_PHYSICS_DIAGNOSTICS)
		{
			auto profile_scope = ProfileScope<&Engine::StepProfile::m_unpack_diagnostics_ms>(m_last_step_profile);

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
	void Deleter<GpuBuffers>::operator()(GpuBuffers* buffers) const
	{
		delete buffers;
	}
	void Deleter<EngineBufferCache>::operator()(EngineBufferCache* cache) const
	{
		delete cache;
	}
}
