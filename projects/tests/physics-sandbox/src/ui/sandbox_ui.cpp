#include "src/forward.h"
#include "src/ui/menu_id.h"
#include "src/ui/sandbox_ui.h"

namespace physics_sandbox
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		// Fixed-tick batching preserves the 60 Hz solver interval while limiting each UI update to one bounded GPU submission.
		constexpr double PhysicsStepSeconds = 1.0 / 60.0;
		constexpr double MaxFrameSeconds = 0.25;
		constexpr int MaxTicksPerSubmission = 2;

		// Build behavior-oriented submenus from both programmatic and file-backed demonstration catalogues.
		HMENU CreateDemoMenu()
		{
			auto menu = Menu(Menu::EKind::Popup);
			for (auto const& category : DemoCategoryCatalogue())
			{
				auto category_menu = Menu(Menu::EKind::Popup);

				// Keep command identifiers tied to stable catalogue indexes while grouping entries by demonstrated behavior.
				auto const demos = DemoCatalogue();
				for (auto index = 0; index != isize(demos); ++index)
				{
					if (demos[index].m_category != category.m_category)
						continue;

					auto const label = pr::Widen(demos[index].m_display_name);
					category_menu.Insert(MenuItem(label.c_str(), MenuID::DemoBase + index));
				}
				auto const scene_demos = SceneDemoCatalogue();
				for (auto index = 0; index != isize(scene_demos); ++index)
				{
					if (scene_demos[index].m_category != category.m_category)
						continue;

					auto const label = pr::Widen(scene_demos[index].m_display_name);
					category_menu.Insert(MenuItem(label.c_str(), MenuID::SceneDemoBase + index));
				}

				// Categories own the conceptual grouping regardless of whether their demos are code- or data-defined.
				auto const label = pr::Widen(category.m_display_name);
				menu.Insert(MenuItem(label.c_str(), category_menu));
			}
			return menu;
		}

		// Return the number of fixed ticks that can share one submission within the requested and engine-owned bounds.
		constexpr int ScheduledTickCount(double accumulated_seconds, int requested_tick_capacity, int substeps_per_tick, int max_internal_substeps, bool require_tick_boundary)
		{
			auto const available_tick_count = static_cast<int>(accumulated_seconds / PhysicsStepSeconds);
			if (available_tick_count == 0)
				return 0;

			auto const engine_tick_capacity = max_internal_substeps / std::max(substeps_per_tick, 1);
			if (engine_tick_capacity < 1)
				return 1;

			auto const submission_tick_capacity = require_tick_boundary ? 1 : std::min(requested_tick_capacity, engine_tick_capacity);
			return std::clamp(available_tick_count, 0, submission_tick_capacity);
		}

		// Bound whole-tick backlog while retaining the fractional phase needed for stable fixed-step pacing.
		constexpr double BoundedAccumulatedTime(double accumulated_seconds, int requested_tick_capacity)
		{
			auto const accumulated_tick_count = static_cast<int>(accumulated_seconds / PhysicsStepSeconds);
			return accumulated_tick_count > requested_tick_capacity
				? accumulated_seconds - (accumulated_tick_count - requested_tick_capacity) * PhysicsStepSeconds
				: accumulated_seconds;
		}

		static_assert(ScheduledTickCount(2.0 * PhysicsStepSeconds, 2, 1, 64, false) == 2);
		static_assert(ScheduledTickCount(2.0 * PhysicsStepSeconds, 1, 1, 64, false) == 1);
		static_assert(ScheduledTickCount(2.0 * PhysicsStepSeconds, 2, 64, 64, false) == 1);
		static_assert(ScheduledTickCount(2.0 * PhysicsStepSeconds, 2, 1, 64, true) == 1);
		static_assert(ScheduledTickCount(PhysicsStepSeconds, 2, 65, 64, false) == 1);
		static_assert(BoundedAccumulatedTime(0.5 * PhysicsStepSeconds, 1) == 0.5 * PhysicsStepSeconds);
		static_assert(BoundedAccumulatedTime(2.5 * PhysicsStepSeconds, 1) > PhysicsStepSeconds);
		static_assert(BoundedAccumulatedTime(2.5 * PhysicsStepSeconds, 1) < 2.0 * PhysicsStepSeconds);
		static_assert(BoundedAccumulatedTime(3.5 * PhysicsStepSeconds, 2) > 2.0 * PhysicsStepSeconds);
		static_assert(BoundedAccumulatedTime(3.5 * PhysicsStepSeconds, 2) < 3.0 * PhysicsStepSeconds);

		double ElapsedMs(Clock::time_point beg, Clock::time_point end)
		{
			return std::chrono::duration<double, std::milli>(end - beg).count();
		}
	}

	SandboxUI::SandboxUI(bool profile_enabled)
		: Form(Params<>()
			.name("physics-sandbox")
			.title(L"Rylogic Physics Sandbox")
			.wh(1600, 1000)
			.xy(10, 10)
			.start_pos(EStartPosition::Manual)
			.padding(0)
			.menu({ {L"&File", Menu(Menu::EKind::Popup, {
				MenuItem(L"&Open Scene...\tCtrl+O", MenuID::OpenFile),
				MenuItem(MenuItem::Separator),
				MenuItem(L"Recent Files", ::CreatePopupMenu()),
				MenuItem(MenuItem::Separator),
				MenuItem(L"E&xit", IDCLOSE),
			})},
			{L"&View", Menu(Menu::EKind::Popup, {
				MenuItem(L"&Normal", MenuID::VisualModeNormal, MenuItem::EState::Checked),
				MenuItem(L"&Contact Priority", MenuID::VisualModeContactPriority),
			})},
			{L"&Demos", CreateDemoMenu()} })
			.main_wnd(true)
			.wndclass(RegisterWndClass<SandboxUI>()))
		, m_status(StatusBar::Params<>().parent(this_).dock(EDock::Bottom))
		, m_media(MediaPanel::Params<>().parent(this_))
		, m_details(DetailsPanel::Params<>().parent(this_))
		, m_view3d(View3DPanelStatic::Params()
			.parent(this_)
			.bkgd_colour(Colour(0xFF808080))
			.dock(EDock::Fill))
		, m_scene(&m_view3d.m_rdr)
		, m_steps_remaining(0)
		, m_pause_on_collision(false)
		, m_physics_accumulator(0)
		, m_scenario(EScenario::Sandbox)
		, m_demo()
		, m_scene_filepath()
		, m_recent()
		, m_last_status()
		, m_frame_count(0)
		, m_fps_elapsed(0)
		, m_fps(0)
		, m_title_elapsed(0)
		, m_details_elapsed(0)
		, m_status_elapsed(0)
		, m_profile(profile_enabled)
		, m_closing(false)
	{
		m_profile.Open(AppDataPath() / "profile.csv");

		// Load the recent files list from disk and populate the submenu
		m_recent.Load();
		RebuildRecentFilesMenu();
		UpdateVisualModeMenu();

		// Wire media panel events to simulation control
		m_media.OnPlay += [&](auto&, auto&)
		{
			m_physics_accumulator = 0;
			m_steps_remaining = -1; // Run continuously
		};
		m_media.OnPause += [&](auto&, auto&)
		{
			PauseSimulation();
		};
		m_media.OnStep += [&](auto&, auto&)
		{
			SingleStepSimulation();
		};
		m_media.OnReset += [&](auto&, auto&)
		{
			ResetScene();
		};
		m_media.OnAllowSleepingChanged += [&](auto&, auto&)
		{
			m_scene.AllowSleeping(m_media.AllowSleeping());
		};

		// Keyboard shortcuts for power-user control
		m_view3d.Key += [&](Control&, KeyEventArgs const& args)
		{
			if (!args.m_down)
				return;

			// 0-5: select scenario (0 = sandbox, 1-5 = test scenarios)
			if (args.m_vk_key >= '0' && args.m_vk_key <= '5')
			{
				m_scenario = static_cast<EScenario>(args.m_vk_key - '0');
				m_demo.reset();
				m_scene_filepath.clear();
				ResetScene();
				m_steps_remaining = -1; // Auto-start
			}

			// R=reset, S=single step, G=go (play), P=pause
			if (args.m_vk_key == 'R')
				ResetScene();
			if (args.m_vk_key == 'S')
				SingleStepSimulation();
			if (args.m_vk_key == 'G')
				m_steps_remaining = -1;
			if (args.m_vk_key == 'P')
				PauseSimulation();

			// C=toggle pause-on-collision
			if (args.m_vk_key == 'C')
				m_pause_on_collision = !m_pause_on_collision;

			// T=run all test scenarios
			if (args.m_vk_key == 'T')
			{
				PauseSimulation();
				m_scene.RunAllTests();
			}

			// Speed control: [=slower, ]=faster, \=reset to 1.0x
			// Each press adjusts by 0.1x (10 slider ticks)
			if (args.m_vk_key == VK_OEM_4) // '[' key
				m_media.TimeScale(m_media.TimeScale() - 0.1f);
			if (args.m_vk_key == VK_OEM_6) // ']' key
				m_media.TimeScale(m_media.TimeScale() + 0.1f);
			if (args.m_vk_key == VK_OEM_5) // '\' key
				m_media.TimeScale(1.0f);

			// Ctrl+O=open scene file
			if (args.m_vk_key == 'O' && (GetKeyState(VK_CONTROL) & 0x8000))
				OpenSceneFile();

			// D=toggle details panel visibility
			if (args.m_vk_key == 'D')
				m_details.TogglePin();

			// B=toggle buoyancy sample-cloud debug overlay
			if (args.m_vk_key == 'B')
				m_scene.m_show_buoyancy_debug = !m_scene.m_show_buoyancy_debug;
		};

		// Hook the scene population event — called each frame during DoRender() to add
		// objects to the scene's drawlist before rendering.
		m_view3d.OnAddToScene += [&](auto&, rdr12::Scene& scene)
		{
			auto const beg = Clock::now();
			auto const w2c = scene.m_cam.WorldToCamera();
			auto const frustum = scene.m_cam.ViewFrustum();
			auto const clip_planes = scene.m_cam.ClipPlanes(false);
			for (int i = 0; i != std::ssize(m_scene.m_body); ++i)
				m_scene.m_body[i].AddToScene(scene, w2c, frustum, clip_planes);
			m_scene.AddArticulationsToScene(scene, w2c, frustum, clip_planes);

			if (m_scene.m_ground_gfx)
				m_scene.m_ground_gfx->AddToScene(scene);
			if (m_scene.m_water_gfx)
				m_scene.m_water_gfx->AddToScene(scene, static_cast<float>(m_scene.m_clock));
			if (m_scene.m_origin_gfx)
				m_scene.m_origin_gfx->AddToScene(scene);
			if (m_scene.m_contacts_gfx)
				m_scene.m_contacts_gfx->AddToScene(scene);

			// Rebuild and draw the buoyancy sample-cloud overlay when enabled. Rebuilt each frame because
			// it samples the bodies' current transforms; cheap relative to the physics step and only active
			// while the user is debugging buoyancy.
			if (m_scene.m_show_buoyancy_debug)
			{
				m_scene.BuildBuoyancyDebugGfx();
				if (m_scene.m_buoyancy_debug_gfx)
					m_scene.m_buoyancy_debug_gfx->AddToScene(scene);
			}

			// Install the procedural sky cube as the scene's global environment map and draw the matching
			// skybox model centred on the camera so reflective surfaces have something to reflect against
			// AND so the user sees a consistent sky background. Gated on m_sky_gfx so non-water scenes
			// keep the existing grey background.
			if (m_scene.m_sky_gfx)
			{
				scene.m_global_envmap = m_scene.m_env_map;
				m_scene.m_sky_gfx->AddToScene(scene, m4x4::Translation(scene.m_cam.CameraToWorld().pos));
			}
			else
			{
				scene.m_global_envmap = nullptr;
			}

			if (m_profile.Enabled())
				m_profile.RecordAddScene(ElapsedMs(beg, Clock::now()));
		};

		// Start with the sandbox scenario
		ResetScene();
	}
	SandboxUI::~SandboxUI()
	{
		m_view3d.WaitForGpu();
	}

	// Override message processing to ensure clean shutdown
	bool SandboxUI::ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
	{
		if (message == WM_CLOSE)
		{
			PauseSimulation();

			// Set closing flag so the step/render lambdas stop accessing the renderer
			m_closing = true;
		}

		// Handle menu commands
		if (message == WM_COMMAND)
		{
			auto id = LOWORD(wparam);
			if (id == MenuID::OpenFile)
			{
				OpenSceneFile();
				result = 0;
				return true;
			}

			if (id == MenuID::VisualModeNormal || id == MenuID::VisualModeContactPriority)
			{
				auto mode = EVisualMode::Normal;
				if (id == MenuID::VisualModeContactPriority)
					mode = EVisualMode::ContactPriority;

				m_scene.VisualMode(mode);
				UpdateVisualModeMenu();
				Render(0);
				result = 0;
				return true;
			}

			auto const demos = DemoCatalogue();
			if (id >= MenuID::DemoBase && id < MenuID::DemoBase + isize(demos))
			{
				auto const demo_index = id - MenuID::DemoBase;
				LoadDemo(demos[demo_index].m_demo);

				result = 0;
				return true;
			}

			auto const scene_demos = SceneDemoCatalogue();
			if (id >= MenuID::SceneDemoBase && id < MenuID::SceneDemoBase + isize(scene_demos))
			{
				auto const demo_index = id - MenuID::SceneDemoBase;
				LoadSceneFile(SceneDemoPath(scene_demos[demo_index]));

				result = 0;
				return true;
			}

			// Recent Files submenu items (IDs in range RecentFileBase..RecentFileBase+MaxRecentFiles-1)
			if (id >= MenuID::RecentFileBase && id < MenuID::RecentFileBase + MaxRecentFiles)
			{
				auto index = id - MenuID::RecentFileBase;
				if (index < static_cast<int>(m_recent.m_paths.size()))
					LoadSceneFile(m_recent.m_paths[index]);

				result = 0;
				return true;
			}
		}

		return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
	}

	// Reset the scene and sync graphics.
	// If a scene file was loaded, reload it from disk. Otherwise, reset the built-in scenario.
	void SandboxUI::ResetScene()
	{
		PauseSimulation();

		// Make sure the GPU has finished with the models before releasing them.
		m_view3d.WaitForGpu();

		// Recreate whichever source currently owns the scene rather than retaining stale dynamics objects.
		if (m_demo)
		{
			LoadDemo(*m_demo);
		}
		else if (!m_scene_filepath.empty())
		{
			LoadSceneFile(m_scene_filepath);
		}
		else
		{
			m_scene.Reset();
			m_scene.SetupScenario(m_scenario);

			// Frame the camera to see the whole scene: look from +Y toward origin, Z-up
			m_view3d.m_cam.LookAt(v4(0, -35, 10, 1), v4::Origin(), v4{0, 0, 1, 0});
		}

		Render(0);
	}

	// Show the Open File dialog and load a JSON scene file
	void SandboxUI::OpenSceneFile()
	{
		COMDLG_FILTERSPEC filters[] = {
			{L"Scene Files (*.json)", L"*.json"},
			{L"All Files (*.*)", L"*.*"},
		};
		auto opts = FileUIOptions()
			.def_extn(L"json")
			.filters(filters, std::size(filters), 0);

		auto files = OpenFileUI(m_hwnd, opts);
		if (files.empty())
			return;

		LoadSceneFile(std::filesystem::path(files[0]));
	}

	// Rebuild the Recent Files submenu from the current m_recent list.
	// Finds the "Recent Files" popup in the File menu and replaces its items
	// with numbered entries for each recently opened scene file.
	void SandboxUI::RebuildRecentFilesMenu()
	{
		// Navigate to the File menu (first submenu of the menu bar)
		auto menu_bar = ::GetMenu(m_hwnd);
		if (!menu_bar)
			return;

		auto file_menu = ::GetSubMenu(menu_bar, 0);
		if (!file_menu)
			return;

		// The "Recent Files" item is at position 1 (after "Open Scene...")
		auto recent_menu = ::GetSubMenu(file_menu, MenuItemIndex::RecentFiles);
		if (!recent_menu)
			return;

		// Clear existing items
		while (::GetMenuItemCount(recent_menu) > 0)
			::RemoveMenu(recent_menu, 0, MF_BYPOSITION);

		// Populate with current recent files list
		if (m_recent.m_paths.empty())
		{
			::AppendMenuW(recent_menu, MF_STRING | MF_GRAYED, 0, L"(empty)");
		}
		else
		{
			for (int i = 0; i != static_cast<int>(m_recent.m_paths.size()); ++i)
			{
				// Format as "&1 filename.json" for keyboard accelerator
				auto label = std::format(L"&{} {}", i + 1, m_recent.m_paths[i].native());
				::AppendMenuW(recent_menu, MF_STRING, MenuID::RecentFileBase + i, label.c_str());
			}
		}

		::DrawMenuBar(m_hwnd);
	}

	void SandboxUI::UpdateVisualModeMenu()
	{
		auto menu_bar = ::GetMenu(m_hwnd);
		if (!menu_bar)
			return;

		auto view_menu = ::GetSubMenu(menu_bar, 1);
		if (!view_menu)
			return;

		auto checked_id = MenuID::VisualModeNormal;
		switch (m_scene.VisualMode())
		{
			case EVisualMode::Normal:
			{
				checked_id = MenuID::VisualModeNormal;
				break;
			}
			case EVisualMode::ContactPriority:
			{
				checked_id = MenuID::VisualModeContactPriority;
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown visual mode");
			}
		}

		::CheckMenuRadioItem(view_menu, MenuID::VisualModeNormal, MenuID::VisualModeContactPriority, checked_id, MF_BYCOMMAND);
		::DrawMenuBar(m_hwnd);
	}

	// Load a scene from a JSON file path.
	// Takes filepath by value to avoid dangling references when m_recent.Add()
	// reorders the MRU list (the caller may pass m_recent.m_paths[i]).
	void SandboxUI::LoadSceneFile(std::filesystem::path filepath)
	{
		try
		{
			auto const load_beg = Clock::now();
			auto mark = load_beg;

			// Pause the simulation
			PauseSimulation();

			// Remember the filepath so Reset can reload it
			m_scene_filepath = filepath;
			m_demo.reset();

			// Add to the recent files list (MRU order) and rebuild the submenu
			m_recent.Add(filepath);
			RebuildRecentFilesMenu();

			// Wait for the GPU to finish before loading. LoadFromJson triggers ShapeChange events which destroy old LdrObjects. 
			m_view3d.WaitForGpu();
			auto const wait_gpu_end = Clock::now();
			auto const wait_gpu_ms = ElapsedMs(mark, wait_gpu_end);
			mark = wait_gpu_end;

			// Load the scene from JSON (creates new body graphics automatically)
			auto scene_desc = scene_loader::LoadFromFile(filepath);
			auto const json_end = Clock::now();
			auto const json_ms = ElapsedMs(mark, json_end);
			mark = json_end;

			m_scene.LoadScene(scene_desc);
			auto const scene_load_end = Clock::now();

			// Frame the camera to see all loaded bodies
			auto bbox = ComputeSceneBBox();
			if (scene_desc.camera)
			{
				m_view3d.m_cam.LookAt(scene_desc.camera->position, scene_desc.camera->lookat, ZAxis<v4>());
			}
			else
			{
				m_view3d.m_cam.View(bbox,
					v4{ 0, -1, 0, 0 },  // Forward direction
					v4{ 0, 0, 1, 0 });  // Up direction (Z-up)
			}
			auto const camera_end = Clock::now();
			auto const camera_ms = ElapsedMs(scene_load_end, camera_end);
			m_profile.RecordLoadScene(filepath, ElapsedMs(load_beg, camera_end), wait_gpu_ms, json_ms, camera_ms, m_scene.m_last_load_profile);

			Render(0);
		}
		catch (std::exception const& ex)
		{
			auto msg = std::format("Failed to load scene:\n{}", ex.what());
			OutputDebugStringA(msg.c_str());

			auto log_path = AppDataPath() / "load_error.log";
			if (std::ofstream log(log_path); log)
				log << msg;
			
			::MessageBoxA(m_hwnd, msg.c_str(), "Load Error", MB_OK | MB_ICONERROR);
		}
	}

	// Load one programmatic constraint or articulation demonstration.
	void SandboxUI::LoadDemo(EDemo demo)
	{
		try
		{
			PauseSimulation();
			m_view3d.WaitForGpu();
			m_scene_filepath.clear();
			m_demo = demo;
			m_scene.LoadDemo(demo);

			auto const bounds = ComputeSceneBBox();
			m_view3d.m_cam.View(bounds, v4{0.0f, -1.0f, -0.2f, 0.0f}, v4::ZAxis());
			Render(0);
		}
		catch (std::exception const& ex)
		{
			auto const message = std::format("Failed to load demo '{}':\n{}", GetDemoInfo(demo).m_display_name, ex.what());
			OutputDebugStringA(message.c_str());
			::MessageBoxA(m_hwnd, message.c_str(), "Demo Load Error", MB_OK | MB_ICONERROR);
		}
	}

	// Complete submitted physics work before inspecting or replacing scene state.
	bool SandboxUI::CompletePendingStep()
	{
		if (!m_scene.StepPending())
			return false;

		auto const collision = m_scene.CompleteStep();
		if (m_profile.Enabled())
			m_profile.RecordStep(m_scene.m_last_step_profile, m_scene.m_last_step_profile.m_total_ms);

		return collision;
	}

	// Pause after publishing the most recently submitted physics result.
	void SandboxUI::PauseSimulation()
	{
		CompletePendingStep();
		m_physics_accumulator = 0;
		m_steps_remaining = 0;
	}

	// Queue one fixed step that the scheduler will complete before returning to rendering.
	void SandboxUI::SingleStepSimulation()
	{
		CompletePendingStep();
		m_physics_accumulator = 0;
		m_steps_remaining = 1;
	}

	// Pipeline continuous simulation so rendering overlaps the next submitted physics step.
	void SandboxUI::Step(double elapsed_seconds)
	{
		// Don't step after close begins
		if (m_closing)
			return;

		auto collision = false;
		auto complete_pending_step = [&]
		{
			if (!m_scene.StepPending())
				return false;
			auto const step_collision = CompletePendingStep();
			collision = step_collision || collision;
			if (step_collision && m_pause_on_collision && m_scene.m_diag.count == 1)
			{
				m_steps_remaining = 0;
				m_physics_accumulator = 0;
				return true;
			}
			return false;
		};

		// Publish the previous result before deciding whether another step should be submitted.
		if (complete_pending_step())
			return;

		if (m_steps_remaining == 0)
			return;

		// A manual step is completed immediately so pausing cannot strand a submitted result.
		if (m_steps_remaining > 0)
		{
			--m_steps_remaining;
			m_scene.BeginStep(PhysicsStepSeconds);
			complete_pending_step();
		}
		else
		{
			// Cap each submission at the requested playback-rate ceiling so 1x overload slows rather than adding catch-up work.
			auto const time_scale = std::max(0.0, static_cast<double>(m_media.TimeScale()));
			auto const requested_tick_capacity = std::clamp(static_cast<int>(std::ceil(time_scale)), 1, MaxTicksPerSubmission);
			auto const scaled_elapsed_seconds = std::min(elapsed_seconds, MaxFrameSeconds) * time_scale;
			m_physics_accumulator += scaled_elapsed_seconds;

			// Discard only excess whole ticks so render-rate jitter cannot erase the fractional phase of the fixed-step clock.
			m_physics_accumulator = BoundedAccumulatedTime(m_physics_accumulator, requested_tick_capacity);

			// Multiple fixed ticks remain internal GPU substeps, preserving solver dt and one submission/wait/readback per UI update.
			auto const tick_count = ScheduledTickCount(
				m_physics_accumulator,
				requested_tick_capacity,
				m_scene.m_physics_substeps,
				m_scene.m_physics.Config().max_internal_substeps,
				m_pause_on_collision);
			if (tick_count != 0)
			{
				m_scene.BeginStep(tick_count * PhysicsStepSeconds, tick_count);
				m_physics_accumulator -= tick_count * PhysicsStepSeconds;
			}
		}

		// Pause on first collision if requested
		if (collision && m_pause_on_collision && m_scene.m_diag.count == 1)
		{
			m_steps_remaining = 0;
			m_physics_accumulator = 0;
		}
	}

	// Render a frame: sync graphics, rebuild overlays, update details panel.
	// Expensive UI operations are rate-limited to avoid dominating frame time.
	void SandboxUI::Render(double elapsed_seconds)
	{
		auto const render_beg = Clock::now();

		// Don't render after close begins
		if (m_closing)
			return;

		++m_frame_count;
		auto profile_sample = SandboxProfiler::RenderSample{};

		// Sync each body's View3D graphics to its physics transform.
		// This is cheap — just copying an O2W matrix per body.
		auto const sync_beg = Clock::now();
		for (int i = 0; i != std::ssize(m_scene.m_body); ++i)
			m_scene.m_body[i].UpdateGfx();
		m_scene.UpdateArticulationGfx();
		auto const sync_end = Clock::now();

		// Render the 3D viewport. Objects are added to the scene via the
		// OnAddToScene event during DoRender(), so no explicit Add/Remove needed.
		auto const render3d_beg = Clock::now();
		auto const clear_beg = Clock::now();
		m_view3d.m_scene.ClearDrawlists();
		auto const clear_end = Clock::now();

		m_view3d.OnAddToScene(m_view3d, m_view3d.m_scene);

		auto const new_frame_beg = Clock::now();
		auto& frame = m_view3d.m_wnd.NewFrame();
		auto const new_frame_end = Clock::now();

		auto const scene_render_beg = Clock::now();
		m_view3d.m_scene.Render(frame);
		auto const scene_render_end = Clock::now();

		auto const present_beg = Clock::now();
		m_view3d.m_wnd.Present(frame, rdr12::EGpuFlush::Async);
		auto const present_end = Clock::now();
		auto const render3d_end = Clock::now();

		// Accumulate time for FPS measurement and rate-limited UI updates.
		// These use wall-clock time (not scaled time) so the UI stays responsive.
		m_fps_elapsed += elapsed_seconds;
		m_title_elapsed += elapsed_seconds;
		m_details_elapsed += elapsed_seconds;
		m_status_elapsed += elapsed_seconds;

		// Update the FPS each second
		if (m_fps_elapsed >= 1.0)
		{
			m_fps = m_frame_count / m_fps_elapsed;
			m_frame_count = 0;
			m_fps_elapsed = 0;
		}

		// Update the details panel only while paused. Formatting thousands of bodies is expensive
		// and it is usually unreadable while the simulation is running anyway.
		if (m_steps_remaining == 0 && m_details_elapsed >= 0.2)
		{
			m_details_elapsed = 0;
			auto const details_beg = Clock::now();
			m_details.Update(m_scene);
			if (m_profile.Enabled())
				profile_sample.m_details_ms += ElapsedMs(details_beg, Clock::now());
		}

		// Update title bar at reduced rate (~4 Hz). SetWindowTextA triggers
		// non-client repaint which is expensive at high frequency.
		if (m_title_elapsed >= 0.25)
		{
			m_title_elapsed = 0;
			auto const title_beg = Clock::now();

			// Keep the slider's speed label text in sync with the trackbar position
			m_media.UpdateSpeedLabel();

			auto const active_name = m_scene.m_current_demo
				? GetDemoInfo(*m_scene.m_current_demo).m_display_name
				: std::string_view(ScenarioName(m_scene.m_current_scenario));
			SetWindowTextA(*this, std::format("Physics Sandbox [{}] t={:.3f} frame={} col={}  FPS: {:.0f}",
				active_name,
				m_scene.m_clock,
				m_scene.m_step_count,
				m_scene.m_diag.count,
				m_fps).c_str());

			if (m_profile.Enabled())
				profile_sample.m_title_ms += ElapsedMs(title_beg, Clock::now());
		}

		// Update status bar (only when text changes to avoid flicker)
		if (m_status_elapsed >= 0.2)
		{
			m_status_elapsed = 0;
			auto const status_beg = Clock::now();
			auto const active_name = m_scene.m_current_demo
				? GetDemoInfo(*m_scene.m_current_demo).m_display_name
				: std::string_view(ScenarioName(m_scene.m_current_scenario));
			auto new_status = std::format(L"t={:.3f}  {}  Collisions: {}  {}  FPS: {:.0f}",
				m_scene.m_clock,
				pr::Widen(active_name),
				m_scene.m_diag.count,
				m_steps_remaining == 0 ? L"[Paused]" : L"[Running]",
				m_fps);

			if (new_status != m_last_status)
			{
				m_last_status = std::move(new_status);
				m_status.Text(0, m_last_status.c_str());
			}

			if (m_profile.Enabled())
				profile_sample.m_status_ms += ElapsedMs(status_beg, Clock::now());
		}

		if (m_profile.Enabled())
		{
			profile_sample.m_sync_gfx_ms = ElapsedMs(sync_beg, sync_end);
			profile_sample.m_do_render_ms = ElapsedMs(render3d_beg, render3d_end);
			profile_sample.m_clear_drawlists_ms = ElapsedMs(clear_beg, clear_end);
			profile_sample.m_new_frame_ms = ElapsedMs(new_frame_beg, new_frame_end);
			profile_sample.m_scene_render_ms = ElapsedMs(scene_render_beg, scene_render_end);
			profile_sample.m_present_ms = ElapsedMs(present_beg, present_end);
			profile_sample.m_render_ms = ElapsedMs(render_beg, Clock::now());
			m_profile.RecordRender(m_scene, profile_sample);
		}
	}

	// Compute a bounding box that encompasses all bodies in the scene.
	// Used to frame the camera when loading a new scene.
	BBox SandboxUI::ComputeSceneBBox() const
	{
		if (m_scene.m_body.empty() && m_scene.m_articulation_visuals.empty())
			return BBox{ v4{0, 0, 0, 1}, v4{5, 5, 5, 0} };

		// Include complete transformed shape bounds so long rods and offset link geometry cannot be clipped.
		// Defer the broad ground plane because its horizontal collision extent is not useful camera content.
		auto bbox = BBox::Reset();
		auto ground_top = std::optional<v4>{};
		for (auto body_index = 0; body_index != isize(m_scene.m_body); ++body_index)
		{
			auto const bounds = m_scene.m_body[body_index].BBoxWS();
			if (body_index == m_scene.m_ground_body_index)
			{
				ground_top = (bounds.m_centre + v4{0.0f, 0.0f, bounds.m_radius.z, 0.0f}).w1();
				continue;
			}

			Grow(bbox, bounds);
		}
		for (auto const& visual : m_scene.m_articulation_visuals)
			Grow(bbox, visual.Bounds(m_scene.m_articulation));

		// Keep ground height visible without allowing a deliberately oversized floor to zoom out the scene.
		if (!bbox.valid())
			bbox = BBox{ground_top.value_or(v4::Origin()), v4{5.0f, 5.0f, 5.0f, 0.0f}};
		else if (ground_top)
			Grow(bbox, *ground_top);

		// Reserve perspective and window-layout margin around the exact geometry bounds.
		bbox *= 1.5f;
		return bbox;
	}
}
