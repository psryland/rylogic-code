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
		, m_report_beg(Clock::now())
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
		if (!Enabled())
			return;

		m_log.open(filepath, std::ios::out | std::ios::trunc);
		if (m_log)
		{
			m_log << "time_s,bodies,steps,renders,fps,step_ms,gravity_ms,physics_ms,kill_zone_ms,eng_new_frame_ms,eng_pack_ms,eng_integrate_ms,eng_broadphase_ms,eng_collide_ms,eng_resolve_ms,eng_readback_ms,eng_gpu_run_ms,eng_unpack_ms,render_ms,sync_gfx_ms,add_scene_ms,do_render_ms,clear_drawlists_ms,new_frame_ms,scene_render_ms,present_ms,details_ms,title_ms,status_ms\n";
			m_log.flush();
		}
	}

	void SandboxProfiler::Reset()
	{
		m_report_beg = Clock::now();
		m_elapsed = 0.0;
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

	void SandboxProfiler::RecordStep(Scene::StepProfile const& profile, double elapsed_ms)
	{
		if (!Enabled())
			return;

		++m_steps;
		m_step_ms += elapsed_ms;
		m_gravity_ms += profile.m_gravity_ms;
		m_physics_ms += profile.m_physics_ms;
		m_kill_zone_ms += profile.m_kill_zone_ms;
		m_engine.m_new_frame_ms += profile.m_engine.m_new_frame_ms;
		m_engine.m_pack_ms += profile.m_engine.m_pack_ms;
		m_engine.m_integrate_ms += profile.m_engine.m_integrate_ms;
		m_engine.m_broadphase_ms += profile.m_engine.m_broadphase_ms;
		m_engine.m_collide_ms += profile.m_engine.m_collide_ms;
		m_engine.m_resolve_ms += profile.m_engine.m_resolve_ms;
		m_engine.m_readback_ms += profile.m_engine.m_readback_ms;
		m_engine.m_gpu_run_ms += profile.m_engine.m_gpu_run_ms;
		m_engine.m_unpack_ms += profile.m_engine.m_unpack_ms;
	}

	void SandboxProfiler::RecordAddScene(double elapsed_ms)
	{
		if (!Enabled())
			return;

		m_add_scene_ms += elapsed_ms;
	}

	void SandboxProfiler::RecordRender(Scene const& scene, RenderSample const& sample)
	{
		if (!Enabled())
			return;

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
		auto const fps = m_elapsed > 0 ? m_renders / m_elapsed : 0.0;
		auto const body_count = static_cast<int>(scene.m_body.size());

		if (m_log)
		{
			m_log << std::format("{:.3f},{},{},{},{:.1f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n",
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
				Avg(m_engine.m_integrate_ms, m_steps),
				Avg(m_engine.m_broadphase_ms, m_steps),
				Avg(m_engine.m_collide_ms, m_steps),
				Avg(m_engine.m_resolve_ms, m_steps),
				Avg(m_engine.m_readback_ms, m_steps),
				Avg(m_engine.m_gpu_run_ms, m_steps),
				Avg(m_engine.m_unpack_ms, m_steps),
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
				Avg(m_render.m_status_ms, m_renders));
			m_log.flush();
		}

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

		Reset();
	}
}
