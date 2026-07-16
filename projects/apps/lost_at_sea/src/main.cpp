//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/main.h"
#include "src/core/cameras/ship_camera.h"
#include "src/core/cameras/free_camera.h"
#include "src/world/terrain/shaders/terrain_shader.h"

namespace las
{
	Main::Main(MainUI& ui)
		:base(pr::app::DefaultSetup(), ui)
		, m_input()
		, m_camera()
		, m_water()
		, m_physics(m_rdr)
		, m_sky(m_rdr)
		, m_day_cycle()
		, m_ocean(m_rdr)
		, m_distant_ocean(m_rdr)
		, m_terrain(m_rdr)
		, m_height_field(42)
		, m_ship(m_rdr, m_physics, v4::Origin())
		, m_sim_state()
		, m_sim_time(0.0)
		, m_render_frame(0)
		, m_camera_mode(0)
		, m_water_generator_position_mutex()
		, m_water_generator_position(v2::Zero())
		, m_step_graph(2)   // Step graph: small thread pool (input-heavy, will grow with physics/AI)
		, m_render_graph(4) // Render graph: larger pool for parallel CB prep
		, m_imgui({
			.m_device = m_rdr.D3DDevice(),
			.m_cmd_queue = m_rdr.GfxQueue(),
			.m_hwnd = HWND(ui),
			.m_rtv_format = ::pr::compute::ToSRGB(m_window.m_rt_props.Format),
			.m_num_frames_in_flight = m_window.BBCount(),
			.m_font_scale = 1.5f,
		})
		, m_diag()
	{
		// Camera setup
		m_cam.FocusDist(10.0f);
		m_cam.Near(0.5f, false);
		m_cam.Far(7000.0f, false);
		m_cam.Align(v4::ZAxis());

		// Default camera: ship camera (third-person follow)
		m_camera = CameraPtr(new camera::ShipCamera(m_cam, m_input, m_ship));
		m_input.Mode(input::EMode::ShipControl);

		// Watch for scene renders
		m_scene.OnUpdateScene += std::bind(&Main::UpdateScene, this, _1, _2);

		// Register diagnostic panels
		auto& tuning = m_terrain.m_shader->m_tuning;
		m_diag.AddPanel("Terrain Tuning", [&tuning](ImGuiUI& ui)
		{
			ui.Text("-- Noise --");
			ui.SliderFloat("Amplitude", &tuning.m_amplitude, 100.0f, 3000.0f);
			ui.SliderFloat("Base Freq", &tuning.m_base_freq, 0.0001f, 0.01f);
			ui.SliderFloat("Persistence", &tuning.m_persistence, 0.1f, 0.9f);
			ui.SliderFloat("Sea Level Bias", &tuning.m_sea_level_bias, -0.8f, 0.2f);

			ui.Separator();
			ui.Text("-- Weathering --");
			ui.SliderFloat("Warp Freq", &tuning.m_warp_freq, 0.0001f, 0.002f);
			ui.SliderFloat("Warp Strength", &tuning.m_warp_strength, 0.0f, 1000.0f);
			ui.SliderFloat("Ridge Threshold", &tuning.m_ridge_threshold, 10.0f, 200.0f);

			ui.Separator();
			ui.Text("-- Archipelago --");
			ui.SliderFloat("Macro Freq", &tuning.m_macro_freq, 0.00001f, 0.001f);
			ui.SliderFloat("Scale Min", &tuning.m_macro_scale_min, 0.0f, 1.0f);
			ui.SliderFloat("Scale Max", &tuning.m_macro_scale_max, 0.0f, 1.0f);

			ui.Separator();
			ui.Text("-- Beach --");
			ui.SliderFloat("Beach Height", &tuning.m_beach_height, 5.0f, 200.0f);

			ui.Separator();
			ui.Text("-- Underwater --");
			ui.SliderFloat("Smooth Depth", &tuning.m_underwater_smooth_depth, 10.0f, 200.0f);
		});

		// Water controls are edited on the render thread and committed through the system's validated, thread-safe value API.
		m_diag.AddPanel("Water Events", [this](ImGuiUI& ui)
		{
			char text[128];
			*std::format_to(text, "Active events: {} / {}", m_water.ActiveEventCount(), water::System::MaxStoneDropCount) = 0;
			ui.Text(text);

			auto settings = m_water.GeneratorSettingsSnapshot();
			auto changed = false;
			changed |= ui.Checkbox("Generate stone drops", &settings.m_enabled);
			changed |= ui.SliderFloat("Spawn interval (s)", &settings.m_spawn_interval_s, 0.1f, 5.0f);

			auto near_mesh_wireframe = m_ocean.Wireframe();
			if (ui.Checkbox("Near mesh wireframe", &near_mesh_wireframe))
				m_ocean.Wireframe(near_mesh_wireframe);

			ui.Separator();
			ui.Text("-- Packet --");
			changed |= ui.SliderFloat("Amplitude min", &settings.m_amplitude_min, 0.1f, settings.m_amplitude_max);
			changed |= ui.SliderFloat("Amplitude max", &settings.m_amplitude_max, settings.m_amplitude_min, 4.0f);
			changed |= ui.SliderFloat("Wavelength min", &settings.m_wavelength_min, 1.0f, settings.m_wavelength_max);
			changed |= ui.SliderFloat("Wavelength max", &settings.m_wavelength_max, settings.m_wavelength_min, 20.0f);
			changed |= ui.SliderFloat("Half-width min", &settings.m_packet_half_width_min, 0.5f, settings.m_packet_half_width_max);
			changed |= ui.SliderFloat("Half-width max", &settings.m_packet_half_width_max, settings.m_packet_half_width_min, 12.0f);
			changed |= ui.SliderFloat("Speed min", &settings.m_propagation_speed_min, 1.0f, settings.m_propagation_speed_max);
			changed |= ui.SliderFloat("Speed max", &settings.m_propagation_speed_max, settings.m_propagation_speed_min, 25.0f);

			ui.Separator();
			ui.Text("-- Lifetime --");
			changed |= ui.SliderFloat("Lifetime min (s)", &settings.m_lifetime_min_s, 1.0f, settings.m_lifetime_max_s);
			changed |= ui.SliderFloat("Lifetime max (s)", &settings.m_lifetime_max_s, settings.m_lifetime_min_s, 20.0f);
			changed |= ui.SliderFloat("Attack (s)", &settings.m_attack_time_s, 0.01f, settings.m_lifetime_min_s - 0.01f);
			changed |= ui.SliderFloat("Attenuation scale", &settings.m_attenuation_scale, 1.0f, 100.0f);

			ui.Separator();
			ui.Text("-- Spawn annulus --");
			changed |= ui.SliderFloat("Radius min", &settings.m_spawn_radius_min, 0.0f, settings.m_spawn_radius_max);
			changed |= ui.SliderFloat("Radius max", &settings.m_spawn_radius_max, settings.m_spawn_radius_min, 100.0f);

			if (changed)
				m_water.SetGeneratorSettings(settings);
		});

		// The initial render snapshot contains the canonical base ocean even before the first simulation step.
		auto sim = m_sim_state.Lock();
		sim->m_water = m_water.CurrentSnapshot();
	}
	Main::~Main()
	{
		m_scene.ClearDrawlists();

		// Ensure the GPU has finished all in-flight frames before destroying
		// pipeline state objects (shaders) owned by ocean/terrain models.
		m_window.m_gsync.Wait();
	}

	// Cycle to the next camera mode
	void Main::CycleCamera()
	{
		static constexpr int CameraModeCount = 2;
		m_camera_mode = (m_camera_mode + 1) % CameraModeCount;

		switch (m_camera_mode)
		{
			case 0: // Ship camera (default)
				m_camera = CameraPtr(new camera::ShipCamera(m_cam, m_input, m_ship));
				m_input.Mode(input::EMode::ShipControl);
				break;
			case 1: // Free camera
				m_camera = CameraPtr(new camera::FreeCamera(m_cam, m_input));
				m_input.Mode(input::EMode::FreeCamera);
				break;
		}
	}

	// Simulation step — submits physics GPU work, overlaps remaining step tasks, then publishes results.
	void Main::SimStep(double elapsed_seconds)
	{
		m_sim_time += elapsed_seconds;
		auto dt = static_cast<float>(elapsed_seconds);

		// Water ages and generation are advanced before physics submission so rendering and buoyancy receive the same materialised event ages.
		auto generator_position = v2::Zero();
		{
			auto lock = std::lock_guard{m_water_generator_position_mutex};
			generator_position = m_water_generator_position;
		}
		m_water.Update(m_sim_time, generator_position);
		auto water_snapshot = m_water.CurrentSnapshot();

		// BeginStep records and submits the GPU physics work on the simulation owner thread. Generic task-graph work can run while the GPU is
		// processing, but tasks that need post-physics snapshots must wait for StepTaskId::Physics.
		m_physics.BeginStep(dt, m_sim_time, water_snapshot);

		// Finalise task: commit state snapshot for the render graph
		m_step_graph.Add(StepTaskId::Finalise, [&, water_snapshot](auto&) -> pr::task_graph::Task
		{
			// Update time of day
			m_day_cycle.Update(dt);

			auto lock = m_sim_state.Lock();
			lock->m_sim_time = m_sim_time;
			lock->m_sun_direction = m_day_cycle.SunDirection();
			lock->m_sun_colour = m_day_cycle.SunColour();
			lock->m_sun_intensity = m_day_cycle.SunIntensity();
			lock->m_water = water_snapshot;
			co_return;
		});

		// Start the task graph for this step.
		m_step_graph.Start();

		// CompleteStep mutates the authoritative physics bodies and publishes snapshots, so keep it on the simulation owner thread.
		// Signal the graph afterwards, even on failure, so any task waiting on physics can drain before the exception leaves this frame.
		auto physics_error = m_physics.CompleteStep();
		m_step_graph.Signal(StepTaskId::Physics);

		// Wait for the step graph to complete
		auto graph_error = m_step_graph.Wait();
		m_step_graph.Reset();

		// Rethrow any exceptions from physics or the step graph after the graph has completed and signalled
		if (physics_error)
			std::rethrow_exception(physics_error);
		if (graph_error)
			std::rethrow_exception(graph_error);

		RenderNeeded();
	}

	// Update the scene with things to render (called from render graph's Submit task)
	void Main::UpdateScene(Scene& scene, UpdateSceneArgs const&)
	{
		// AddInstance is not thread-safe, so all scene population happens here serially
		m_sky.AddToScene(scene);
		m_ocean.AddToScene(scene);
		m_distant_ocean.AddToScene(scene);
		m_terrain.AddToScene(scene);
		m_ship.AddToScene(scene);
	}

	// Render step — builds and runs the render task graph
	void Main::DoRender(bool force)
	{
		if (!m_rdr_pending && !force)
			return;

		m_rdr_pending = false;
		++m_render_frame;

		// Read the latest simulation state snapshot
		auto const& sim = m_sim_state.Read();
		auto cam_pos = m_cam.CameraToWorld().pos; // Current camera position (updated by input handler in render loop)
		auto sun_dir = sim.m_sun_direction;
		auto sun_col = sim.m_sun_colour;
		auto sun_int = sim.m_sun_intensity;

		// Publish the plain camera XY input used by the simulation-time event generator without exposing camera ownership to the water system.
		{
			auto lock = std::lock_guard{m_water_generator_position_mutex};
			m_water_generator_position = v2{cam_pos.x, cam_pos.y};
		}

		// Update the scene's global light to match the day/night cycle
		m_scene.m_global_light.m_direction = -sun_dir;
		m_scene.m_global_light.m_cam_relative = false;
		m_scene.m_global_light.m_diffuse = Colour(sun_col.x * 0.5f, sun_col.y * 0.5f, sun_col.z * 0.5f, 1.0f);
		m_scene.m_global_light.m_ambient = Colour(0.15f * sun_int, 0.15f * sun_int, 0.2f * sun_int, 1.0f);

		// PrepareFrame task: set up the frame (must be serial, touches GPU resources)
		m_render_graph.Add(RenderTaskId::PrepareFrame, [&](auto&) -> pr::task_graph::Task {
			m_scene.ClearDrawlists();
			co_return;
		});

		// Per-system tasks: prepare shader constant buffers (thread-safe, parallel)
		m_render_graph.Add(RenderTaskId::Skybox, [&, sun_dir, sun_col, sun_int](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::PrepareFrame);
			m_sky.PrepareRender(sun_dir, sun_col, sun_int);
			co_return;
		});
		m_render_graph.Add(RenderTaskId::Ocean, [&, cam_pos, sun_dir, sun_col](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::PrepareFrame);
			m_ocean.PrepareRender(sim.m_water, cam_pos, m_scene.m_global_envmap != nullptr, sun_dir, sun_col);
			co_return;
		});
		m_render_graph.Add(RenderTaskId::DistantOcean, [&, cam_pos, sun_dir, sun_col](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::PrepareFrame);
			m_distant_ocean.PrepareRender(cam_pos, m_scene.m_global_envmap != nullptr, sun_dir, sun_col);
			co_return;
		});
		m_render_graph.Add(RenderTaskId::Terrain, [&, cam_pos, sun_dir, sun_col](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::PrepareFrame);
			m_terrain.PrepareRender(cam_pos, sun_dir, sun_col);
			co_return;
		});
		m_render_graph.Add(RenderTaskId::Ship, [&, cam_pos](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::PrepareFrame);
			m_ship.PrepareRender(cam_pos);
			co_return;
		});

		// Submit task: populate scene and present (serial, after all CB prep is done)
		m_render_graph.Add(RenderTaskId::Submit, [&](auto ctx) -> pr::task_graph::Task {
			co_await ctx.Wait(RenderTaskId::Skybox);
			co_await ctx.Wait(RenderTaskId::Ocean);
			co_await ctx.Wait(RenderTaskId::DistantOcean);
			co_await ctx.Wait(RenderTaskId::Terrain);
			co_await ctx.Wait(RenderTaskId::Ship);
			co_return;
		});

		m_render_graph.Run();
		m_render_graph.Reset();

		// Scene population and presentation happen on the main thread after the graph completes
		auto& frame = m_window.NewFrame();
		m_scene.Render(frame);
		RenderUI(frame);
		m_window.Present(frame);
	}

	// Render imgui overlay into the post-resolve back buffer
	void Main::RenderUI(Frame& frame)
	{
		if (!m_imgui)
			return;

		// Override display size to match the actual render target (fixes DPI mismatch)
		auto const& vp = m_scene.m_viewport;
		m_imgui.SetDisplaySize(vp.Width, vp.Height);

		// Start a new imgui frame
		m_imgui.NewFrame();

		// Build the debug overlay
		m_imgui.SetNextWindowPos(10, 10, 1 /*ImGuiCond_Once*/);
		m_imgui.SetNextWindowBgAlpha(0.5f);
		if (m_imgui.BeginWindow("Debug Info", nullptr, (1 << 0) | (1 << 1) | (1 << 6)))
		{
			// ImGuiWindowFlags_NoTitleBar=1<<0, NoResize=1<<1, AlwaysAutoResize=1<<6
			char buf[128];
			*std::format_to(buf, "Sim Time: {:.2f} s", m_sim_time) = 0;
			m_imgui.Text(buf);
			*std::format_to(buf, "Time: {:02.0f}:{:02.0f}", std::floor(m_day_cycle.m_time_of_day), std::fmod(m_day_cycle.m_time_of_day, 1.0f) * 60.0f) = 0;
			m_imgui.Text(buf);
			*std::format_to(buf, "Frame: {}", m_render_frame) = 0;
			m_imgui.Text(buf);
			auto cam_pos = m_cam.CameraToWorld().pos;
			*std::format_to(buf, "Pos: ({:.1f}, {:.1f}, {:.1f}", cam_pos.x, cam_pos.y, cam_pos.z) = 0;
			m_imgui.Text(buf);

			m_imgui.Separator();

			*std::format_to(buf, "Terrain Patches: {}", m_terrain.PatchCount()) = 0;
			m_imgui.Text(buf);
			*std::format_to(buf, "Input Queue: {}", m_input.EventCount()) = 0;
			m_imgui.Text(buf);
			*std::format_to(buf, "Camera: {} [{:.1f} m/s]", m_camera->Name(), m_camera->Speed()) = 0;
			m_imgui.Text(buf);

			m_imgui.Separator();

			// Ship position
			auto ship_pos = m_ship.O2W().pos;
			*std::format_to(buf, "Ship: ({:.1f}, {:.1f}, {:.1f})", ship_pos.x, ship_pos.y, ship_pos.z) = 0;
			m_imgui.Text(buf);

			m_imgui.Separator();

			// Render loop FPS (loop index 1 = variable-step render loop)
			float fps_history[WinGuiMsgLoop::FpsHistory::Length];
			m_ui.m_msg_loop.LoopFps(1).Fps<float>(fps_history);
			*std::format_to(buf, "Render: {:.0f} fps", fps_history[0]) = 0;
			m_imgui.PlotLines(buf, &fps_history[0], _countof(fps_history), 0, nullptr, 0.0f, 120.0f, 200, 40);

			// Sim loop FPS (loop index 0 = fixed-step sim loop)
			m_ui.m_msg_loop.LoopFps(0).Fps<float>(fps_history);
			*std::format_to(buf, "Sim: {:.0f} fps", fps_history[0]) = 0;
			m_imgui.PlotLines(buf, &fps_history[0], _countof(fps_history), 0, nullptr, 0.0f, 120.0f, 200, 40);
		}
		m_imgui.EndWindow();

		// Draw diagnostic panels (if visible)
		m_diag.Draw(m_imgui);

		// Draw after View3D's alpha resolve so the scene composite cannot overwrite the UI.
		pr::compute::BarrierBatch barriers(frame.m_present);
		barriers.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_RENDER_TARGET);
		barriers.Commit();

		frame.m_present.OMSetRenderTargets({ &frame.bb_post().m_rtv, 1 }, FALSE, nullptr);

		// Set viewport and scissor
		frame.m_present.RSSetViewports({ &vp, 1 });
		frame.m_present.RSSetScissorRects(vp.m_clip);

		// Render imgui draw data
		m_imgui.Render(frame.m_present.get());

		pr::compute::BarrierBatch present_barriers(frame.m_present);
		present_barriers.Transition(frame.bb_post().m_render_target.get(), D3D12_RESOURCE_STATE_PRESENT);
		present_barriers.Commit();
	}

	// --------------------------------------------------------------------------------------------

	MainUI::MainUI(wchar_t const*, int)
		:base(Params().title(AppTitle()))
	{
		// Drain all pending messages each frame so input is responsive
		// (default is 1000, which is fine for real-time games)

		// Fixed step simulation at 60Hz, render as fast as possible (capped by vsync/present)
		m_msg_loop.AddLoop(60.0, false, [this](double dt)
		{
			if (!m_main) return;
			
			m_main->SimStep(dt);
		});
		m_msg_loop.AddLoop(120.0, true, [this](double dt)
		{
			if (!m_main) return;

			// Process input in the render loop so the camera works even when the simulation is paused
			m_main->m_input.Step(static_cast<float>(dt));
			m_main->m_camera->Update(static_cast<float>(dt));
			m_main->DoRender(true);
		});
	}

	bool MainUI::ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
	{
		// F1 cycles camera mode, F3 toggles diagnostics (before ImGui gets them)
		if (m_main && message == WM_KEYDOWN)
		{
			if (wparam == VK_F1)
			{
				m_main->CycleCamera();
				result = 0;
				return true;
			}
			if (wparam == VK_F3)
			{
				m_main->m_diag.Toggle();
				result = 0;
				return true;
			}
		}

		// Let imgui see all messages (for hover, panel interaction, etc.)
		// but don't suppress game input — the game's input handler also needs them.
		if (m_main)
			m_main->m_imgui.WndProc(parent_hwnd, message, wparam, lparam);

		// On WM_CLOSE, tear down the renderer before the HWND is destroyed.
		// DXGI's swap chain can post internal messages that starve WM_QUIT
		// (which has the lowest priority). Destroying 'm_main' releases the
		// swap chain so that PostQuitMessage's WM_QUIT actually gets dequeued.
		if (message == WM_CLOSE)
			m_main.reset();

		return base::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
	}
}

namespace pr::app
{
	std::unique_ptr<IAppMainUI> CreateUI(wchar_t const* lpstrCmdLine, int nCmdShow)
	{
		return std::unique_ptr<IAppMainUI>(new las::MainUI(lpstrCmdLine, nCmdShow));
	}
}