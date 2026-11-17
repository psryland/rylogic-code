#include "src/forward.h"
#include "src/ui/sandbox_profiler.h"

namespace physics_sandbox
{
	namespace
	{
		double Avg(double total, int count)
		{
			return count != 0 ? total / count : 0.0;
		}
	}

	SandboxProfiler::SandboxProfiler(bool enabled)
		: m_enabled(enabled)
		, m_log()
		, m_elapsed(0.0)
		, m_discarded_simulation_s(0.0)
		, m_report_beg(Clock::now())
		, m_last_render()
		, m_max_frame_ms(0.0)
		, m_steps(0)
		, m_renders(0)
		, m_step_ms(0.0)
		, m_gravity_ms(0.0)
		, m_physics_ms(0.0)
		, m_kill_zone_ms(0.0)
		, m_engine()
		, m_add_scene_ms(0.0)
		, m_render()
	{
	}

	bool SandboxProfiler::Enabled() const
	{
		return m_enabled;
	}

	void SandboxProfiler::Open(std::filesystem::path filepath)
	{
		// Open the explicit file sink only for enabled profiling.
		if (!Enabled())
			return;

		// Preserve the existing columns and append the sampled steady clock and interval scheduling loss.
		m_log.open(filepath, std::ios::out | std::ios::trunc);
		if (m_log)
		{
			// Load records and runtime records have separate named schemas.
			m_log << "load_profile,file,total_ms,wait_gpu_ms,json_ms,scene_load_ms,prepare_ms,bbox_ms,shapes_ms,bodies_ms,buoyancy_ms,ldraw_build_ms,ldraw_serialise_ms,ldraw_parse_ms,ldraw_assign_ms,logging_ms,camera_ms,bodies,shapes,ldraw_objects,ldraw_bytes,has_renderer\n";
			m_log << "time_s,bodies,steps,renders,fps,step_ms,gravity_ms,physics_ms,kill_zone_ms,eng_new_frame_ms,eng_pack_ms,eng_upload_ms,eng_external_forces_ms,eng_integrate_ms,eng_sleepwake_ms,eng_broadphase_ms,eng_collide_ms,eng_resolve_ms,eng_selective_ms,eng_sleepupdate_ms,eng_readback_ms,eng_gpu_run_ms,eng_unpack_ms,eng_gpu_prepare_ms,eng_gpu_execute_ms,eng_gpu_wait_ms,eng_gpu_reset_ms,eng_readback_access_ms,eng_body_readback_copy_ms,eng_contact_readback_copy_ms,eng_collision_events_ms,eng_sleep_island_unpack_ms,eng_body_unpack_ms,render_ms,sync_gfx_ms,add_scene_ms,do_render_ms,clear_drawlists_ms,new_frame_ms,scene_render_ms,present_ms,details_ms,title_ms,status_ms,max_frame_ms,eng_constraint_pack_ms,eng_constraint_upload_ms,steady_time_s,discarded_simulation_s\n";
			m_log.flush();
		}
	}

	void SandboxProfiler::Reset()
	{
		// Reset interval totals after publication, but retain the last render sample across that publication gap.
		m_report_beg = Clock::now();
		m_elapsed = 0.0;
		m_discarded_simulation_s = 0.0;
		m_max_frame_ms = 0.0;
		m_steps = 0;
		m_renders = 0;
		m_step_ms = 0.0;
		m_gravity_ms = 0.0;
		m_physics_ms = 0.0;
		m_kill_zone_ms = 0.0;
		m_engine = {};
		m_add_scene_ms = 0.0;
		m_render = {};
	}

	bool SandboxProfiler::RecordPopulationStats(rdr12::PopulationStats const& stats)
	{
		// Use the explicit file stream because autoplay redirects the C runtime streams back to CONOUT$.
		m_log << std::format("population_final,capacity={},admitted_capacity={},views={},blocks_rejected={},blocks_overflow={},tested={},emitted={},lod0={},indirect_draws={},live_count_violations={}\n",
			stats.m_instance_capacity, stats.m_admitted_instances, stats.m_views, stats.m_classified.m_blocks_rejected,
			stats.m_classified.m_blocks_overflow, stats.m_classified.m_instances_tested, stats.m_classified.m_entries_emitted,
			stats.m_classified.m_entries_per_lod[0], stats.m_classified.m_indirect_draws, stats.m_classified.m_live_count_violations);
		m_log.flush();
		return m_log.good();
	}

	void SandboxProfiler::RecordStep(Scene::StepProfile const& profile, double elapsed_ms)
	{
		// Preserve the engine's existing phase boundaries and per-submission units.
		if (!Enabled())
			return;

		// Body packing/upload and constraint packing/upload are distinct CPU phases.
		++m_steps;
		m_step_ms += elapsed_ms;
		m_gravity_ms += profile.m_gravity_ms;
		m_physics_ms += profile.m_physics_ms;
		m_kill_zone_ms += profile.m_kill_zone_ms;
		m_engine.m_new_frame_ms += profile.m_engine.m_new_frame_ms;
		m_engine.m_pack_ms += profile.m_engine.m_pack_ms;
		m_engine.m_constraint_pack_ms += profile.m_engine.m_constraint_pack_ms;
		m_engine.m_upload_ms += profile.m_engine.m_upload_ms;
		m_engine.m_constraint_upload_ms += profile.m_engine.m_constraint_upload_ms;
		m_engine.m_external_forces_ms += profile.m_engine.m_external_forces_ms;
		m_engine.m_integrate_ms += profile.m_engine.m_integrate_ms;
		m_engine.m_sleepwake_ms += profile.m_engine.m_sleepwake_ms;
		m_engine.m_broadphase_ms += profile.m_engine.m_broadphase_ms;
		m_engine.m_collide_ms += profile.m_engine.m_collide_ms;
		m_engine.m_resolve_ms += profile.m_engine.m_resolve_ms;
		m_engine.m_selective_ms += profile.m_engine.m_selective_ms;
		m_engine.m_sleepupdate_ms += profile.m_engine.m_sleepupdate_ms;
		m_engine.m_readback_ms += profile.m_engine.m_readback_ms;
		m_engine.m_gpu_run_ms += profile.m_engine.m_gpu_run_ms;
		m_engine.m_gpu_prepare_ms += profile.m_engine.m_gpu_prepare_ms;
		m_engine.m_gpu_execute_ms += profile.m_engine.m_gpu_execute_ms;
		m_engine.m_gpu_wait_ms += profile.m_engine.m_gpu_wait_ms;
		m_engine.m_gpu_reset_ms += profile.m_engine.m_gpu_reset_ms;
		m_engine.m_unpack_ms += profile.m_engine.m_unpack_ms;
		m_engine.m_readback_access_ms += profile.m_engine.m_readback_access_ms;
		m_engine.m_body_readback_copy_ms += profile.m_engine.m_body_readback_copy_ms;
		m_engine.m_contact_readback_copy_ms += profile.m_engine.m_contact_readback_copy_ms;
		m_engine.m_collision_events_ms += profile.m_engine.m_collision_events_ms;
		m_engine.m_sleep_island_unpack_ms += profile.m_engine.m_sleep_island_unpack_ms;
		m_engine.m_body_unpack_ms += profile.m_engine.m_body_unpack_ms;
	}

	void SandboxProfiler::RecordAddScene(double elapsed_ms)
	{
		if (!Enabled())
			return;

		m_add_scene_ms += elapsed_ms;
	}

	// Accumulate simulation time removed by scheduling, not submitted or completed physics time.
	void SandboxProfiler::RecordDiscardedSimulationTime(double seconds)
	{
		// Disabled profiling must not retain counters or affect scheduling.
		if (!Enabled())
			return;

		// The scheduler supplies nonnegative losses from its existing admission and backlog bounds.
		assert(seconds >= 0.0 && std::isfinite(seconds));
		m_discarded_simulation_s += seconds;
	}

	void SandboxProfiler::RecordLoadScene(std::filesystem::path const& filepath, double total_ms, double wait_gpu_ms, double json_ms, double camera_ms, Scene const& scene)
	{
		if (!Enabled())
			return;

		auto const& profile = scene.m_last_load_profile;
		if (m_log)
		{
			m_log << std::format("load_profile,{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{},{},{},{},{}\n",
				filepath.string(),
				total_ms,
				wait_gpu_ms,
				json_ms,
				profile.m_total_ms,
				profile.m_prepare_ms,
				profile.m_bbox_ms,
				profile.m_shapes_ms,
				profile.m_bodies_ms,
				profile.m_buoyancy_ms,
				profile.m_ldraw_build_ms,
				profile.m_ldraw_serialise_ms,
				profile.m_ldraw_parse_ms,
				profile.m_ldraw_assign_ms,
				profile.m_logging_ms,
				camera_ms,
				profile.m_body_count,
				profile.m_shape_count,
				profile.m_ldraw_object_count,
				profile.m_ldraw_byte_count,
				profile.m_has_renderer ? 1 : 0);
			// Construction counts belong to the explicitly enabled report, not to ordinary scene construction.
			if (!scene.m_rigid_ragdoll_joints.empty())
				m_log << std::format("rigid_ragdoll_profile,{},{},{}\n", scene.m_body.size() - (scene.m_ground_body_index != -1 ? 1 : 0), scene.m_rigid_ragdoll_joints.size(), scene.m_constraints.Count());

			m_log.flush();
		}

		OutputDebugStringA(std::format(
			"PhysicsSandbox load profile: total={:.1f}ms json={:.1f}ms scene={:.1f}ms ldraw_parse={:.1f}ms bodies={}\n",
			total_ms,
			json_ms,
			profile.m_total_ms,
			profile.m_ldraw_parse_ms,
			profile.m_body_count).c_str());
	}

	void SandboxProfiler::RecordRender(Scene const& scene, RenderSample const& sample)
	{
		if (!Enabled())
			return;

		// Track the longest interval between rendered frames so each report exposes frame-time spikes hidden by the average FPS.
		auto const now = Clock::now();
		if (m_last_render != Clock::time_point{})
			m_max_frame_ms = std::max(m_max_frame_ms, std::chrono::duration<double, std::milli>(now - m_last_render).count());

		m_last_render = now;

		++m_renders;
		m_render.m_render_ms += sample.m_render_ms;
		m_render.m_sync_gfx_ms += sample.m_sync_gfx_ms;
		m_render.m_do_render_ms += sample.m_do_render_ms;
		m_render.m_clear_drawlists_ms += sample.m_clear_drawlists_ms;
		m_render.m_new_frame_ms += sample.m_new_frame_ms;
		m_render.m_scene_render_ms += sample.m_scene_render_ms;
		m_render.m_present_ms += sample.m_present_ms;
		m_render.m_details_ms += sample.m_details_ms;
		m_render.m_title_ms += sample.m_title_ms;
		m_render.m_status_ms += sample.m_status_ms;
		m_elapsed = std::chrono::duration<double>(Clock::now() - m_report_beg).count();
		if (m_elapsed >= 1.0)
			Report(scene);
	}

	void SandboxProfiler::Report(Scene const& scene)
	{
		// Step averages count submissions, while render averages count presentations.
		auto const fps = m_elapsed > 0 ? m_renders / m_elapsed : 0.0;
		auto const body_count = static_cast<int>(scene.m_body.size());
		if (m_log)
		{
			// All engine durations are CPU wall milliseconds per submitted frame, including blocked GPU waits.
			// Steady-epoch differences include report publication, unlike summed m_elapsed intervals rebased by Reset.
			// Completed scene time excludes pending physics and fractional scheduler phase; those can differ at report boundaries.
			m_log << std::format("{:.3f},{},{},{},{:.1f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},			{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.9f},{:.9f}\n",
				scene.m_clock,
				body_count,
				m_steps,
				m_renders,
				fps,
				Avg(m_step_ms, m_steps),
				Avg(m_gravity_ms, m_steps),
				Avg(m_physics_ms, m_steps),
				Avg(m_kill_zone_ms, m_steps),
				Avg(m_engine.m_new_frame_ms, m_steps),
				Avg(m_engine.m_pack_ms, m_steps),
				Avg(m_engine.m_upload_ms, m_steps),
				Avg(m_engine.m_external_forces_ms, m_steps),
				Avg(m_engine.m_integrate_ms, m_steps),
				Avg(m_engine.m_sleepwake_ms, m_steps),
				Avg(m_engine.m_broadphase_ms, m_steps),
				Avg(m_engine.m_collide_ms, m_steps),
				Avg(m_engine.m_resolve_ms, m_steps),
				Avg(m_engine.m_selective_ms, m_steps),
				Avg(m_engine.m_sleepupdate_ms, m_steps),
				Avg(m_engine.m_readback_ms, m_steps),
				Avg(m_engine.m_gpu_run_ms, m_steps),
				Avg(m_engine.m_unpack_ms, m_steps),
				Avg(m_engine.m_gpu_prepare_ms, m_steps),
				Avg(m_engine.m_gpu_execute_ms, m_steps),
				Avg(m_engine.m_gpu_wait_ms, m_steps),
				Avg(m_engine.m_gpu_reset_ms, m_steps),
				Avg(m_engine.m_readback_access_ms, m_steps),
				Avg(m_engine.m_body_readback_copy_ms, m_steps),
				Avg(m_engine.m_contact_readback_copy_ms, m_steps),
				Avg(m_engine.m_collision_events_ms, m_steps),
				Avg(m_engine.m_sleep_island_unpack_ms, m_steps),
				Avg(m_engine.m_body_unpack_ms, m_steps),
				Avg(m_render.m_render_ms, m_renders),
				Avg(m_render.m_sync_gfx_ms, m_renders),
				Avg(m_add_scene_ms, m_renders),
				Avg(m_render.m_do_render_ms, m_renders),
				Avg(m_render.m_clear_drawlists_ms, m_renders),
				Avg(m_render.m_new_frame_ms, m_renders),
				Avg(m_render.m_scene_render_ms, m_renders),
				Avg(m_render.m_present_ms, m_renders),
				Avg(m_render.m_details_ms, m_renders),
				Avg(m_render.m_title_ms, m_renders),
				Avg(m_render.m_status_ms, m_renders),
				m_max_frame_ms,
				Avg(m_engine.m_constraint_pack_ms, m_steps),
				Avg(m_engine.m_constraint_upload_ms, m_steps),
				std::chrono::duration<double>(m_last_render.time_since_epoch()).count(),
				m_discarded_simulation_s);
			m_log.flush();
		}

		// Keep the existing debugger summary independent of the detailed CSV schema.
		OutputDebugStringA(std::format(
			"PhysicsSandbox profile: bodies={} fps={:.1f} step={:.2f}ms physics={:.2f}ms render={:.2f}ms sync={:.2f}ms add_scene={:.2f}ms do_render={:.2f}ms new_frame={:.2f}ms scene_render={:.2f}ms present={:.2f}ms details={:.2f}ms\n",
			body_count,
			fps,
			Avg(m_step_ms, m_steps),
			Avg(m_physics_ms, m_steps),
			Avg(m_render.m_render_ms, m_renders),
			Avg(m_render.m_sync_gfx_ms, m_renders),
			Avg(m_add_scene_ms, m_renders),
			Avg(m_render.m_do_render_ms, m_renders),
			Avg(m_render.m_new_frame_ms, m_renders),
			Avg(m_render.m_scene_render_ms, m_renders),
			Avg(m_render.m_present_ms, m_renders),
			Avg(m_render.m_details_ms, m_renders)).c_str());

		// Begin a fresh reporting interval after publishing its completed totals.
		Reset();
	}
}
