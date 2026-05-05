#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	// Lightweight frame profiler enabled with '-profile'.
	struct SandboxProfiler
	{
		using Clock = std::chrono::steady_clock;

		struct RenderSample
		{
			double m_render_ms = 0;
			double m_sync_gfx_ms = 0;
			double m_do_render_ms = 0;
			double m_clear_drawlists_ms = 0;
			double m_new_frame_ms = 0;
			double m_scene_render_ms = 0;
			double m_present_ms = 0;
			double m_details_ms = 0;
			double m_title_ms = 0;
			double m_status_ms = 0;
		};

		explicit SandboxProfiler(bool enabled = false);

		bool Enabled() const;
		void Open(std::filesystem::path filepath);
		void Reset();
		void RecordStep(Scene::StepProfile const& profile, double elapsed_ms);
		void RecordAddScene(double elapsed_ms);
		void RecordLoadScene(std::filesystem::path const& filepath, double total_ms, double wait_gpu_ms, double json_ms, double camera_ms, Scene::LoadProfile const& profile);
		void RecordRender(Scene const& scene, RenderSample const& sample);

	private:

		bool m_enabled;
		std::ofstream m_log;
		double m_elapsed;
		Clock::time_point m_report_beg;
		int m_steps;
		int m_renders;
		double m_step_ms;
		double m_gravity_ms;
		double m_physics_ms;
		double m_kill_zone_ms;
		physics::Engine::StepProfile m_engine;
		double m_add_scene_ms;
		RenderSample m_render;

		void Report(Scene const& scene);
	};
}
