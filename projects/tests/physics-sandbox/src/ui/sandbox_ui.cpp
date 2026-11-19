#include "src/forward.h"
#include "src/scene/demo.h"
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

		// Separate mutually exclusive base rendering from independent, additive sample overlays.
		HMENU CreateViewMenu()
		{
			return Menu(Menu::EKind::Popup, {
				MenuItem(L"&Normal", MenuID::VisualModeNormal, MenuItem::EState::Checked),
				MenuItem(L"&Contact Priority", MenuID::VisualModeContactPriority),
				MenuItem(MenuItem::Separator),
				MenuItem(L"&Overlays and diagnostics", Menu(Menu::EKind::Popup, {
					MenuItem(L"&Surface samples + normals", MenuID::SurfaceSamples),
					MenuItem(L"Sleeping-body &transparency", MenuID::SleepingTransparency, MenuItem::EState::Checked),
				})),
				MenuItem(L"&Atmosphere", Menu(Menu::EKind::Popup, {
					MenuItem(L"&Grid", MenuID::AtmosphereGrid),
					MenuItem(L"&Particles", MenuID::AtmosphereParticles),
				})),
				MenuItem(MenuItem::Separator),
				MenuItem(L"&Details panel\tD", MenuID::DetailsPanel),
			});
		}

		// Refresh from the panel when opening the menu so keyboard and pin-button toggles need no separate menu state.
		void UpdateDetailsMenu(HMENU view_menu, bool visible)
		{
			::CheckMenuItem(view_menu, MenuID::DetailsPanel, MF_BYCOMMAND | (visible ? MF_CHECKED : MF_UNCHECKED));
		}

		// Apply independent check marks without including overlay commands in the base-mode radio range.
		void UpdateOverlayMenu(HMENU view_menu, SampleOverlays const& overlays, bool sleeping_transparency)
		{
			// Overlay submenus mirror the current scene state whenever the View menu opens or a command changes.
			auto const menu = ::GetSubMenu(view_menu, 3);
			::CheckMenuItem(menu, MenuID::SurfaceSamples, MF_BYCOMMAND | (overlays.m_surface_enabled ? MF_CHECKED : MF_UNCHECKED));
			::CheckMenuItem(menu, MenuID::SleepingTransparency, MF_BYCOMMAND | (sleeping_transparency ? MF_CHECKED : MF_UNCHECKED));
		}

		// Apply atmosphere diagnostic checks and disabled state from the loaded scene.
		void UpdateDiagnosticMenu(HMENU view_menu, Scene const& scene)
		{
			// Scene-owned atmosphere graphics are optional, so grey the commands when the current scene has none.
			UpdateOverlayMenu(view_menu, scene.m_sample_overlays, scene.SleepingTransparency());
			auto const atmosphere_menu = ::GetSubMenu(view_menu, 4);
			auto const has_atmosphere = scene.m_atmosphere_gfx != nullptr;
			::EnableMenuItem(atmosphere_menu, MenuID::AtmosphereGrid, MF_BYCOMMAND | (has_atmosphere ? MF_ENABLED : MF_GRAYED));
			::EnableMenuItem(atmosphere_menu, MenuID::AtmosphereParticles, MF_BYCOMMAND | (has_atmosphere ? MF_ENABLED : MF_GRAYED));
			::CheckMenuItem(atmosphere_menu, MenuID::AtmosphereGrid, MF_BYCOMMAND | (has_atmosphere && scene.m_atmosphere_gfx->ShowGrid() ? MF_CHECKED : MF_UNCHECKED));
			::CheckMenuItem(atmosphere_menu, MenuID::AtmosphereParticles, MF_BYCOMMAND | (has_atmosphere && scene.m_atmosphere_gfx->ShowParticles() ? MF_CHECKED : MF_UNCHECKED));
		}

		// Build behavior-oriented submenus from the runtime-discovered JSON demonstration catalogue.
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

				// Categories own the conceptual grouping while scene files own each entry's placement and label.
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

		// Add scaled wall time under the existing limits and record the simulation seconds those limits remove.
		double AccumulateSimulationTime(double accumulated_seconds, double elapsed_seconds, double time_scale, int requested_tick_capacity, SandboxProfiler& profile)
		{
			// Apply the elapsed clamp before scaling, preserving the existing admission order and backlog policy.
			auto const admitted_elapsed_seconds = std::min(elapsed_seconds, MaxFrameSeconds);
			auto const scaled_elapsed_seconds = admitted_elapsed_seconds * time_scale;
			accumulated_seconds += scaled_elapsed_seconds;
			auto const bounded_seconds = BoundedAccumulatedTime(accumulated_seconds, requested_tick_capacity);

			// Clamp loss is scaled simulation time; backlog loss is only the whole ticks actually removed.
			profile.RecordDiscardedSimulationTime((elapsed_seconds - admitted_elapsed_seconds) * time_scale + (accumulated_seconds - bounded_seconds));
			return bounded_seconds;
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

	// Create the sandbox with optional profiling and a literal window-caption prefix.
	SandboxUI::SandboxUI(bool profile_enabled, std::string title_prefix)
		: Form(Params<>()
			.name("physics-sandbox")
			.title(pr::Widen(std::format("{}Rylogic Physics Sandbox", title_prefix)).c_str())
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
			{L"&View", CreateViewMenu()},
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
		, m_scene_filepath()
		, m_active_name(ScenarioName(EScenario::Sandbox))
		, m_recent()
		, m_last_status()
		, m_title_prefix(std::move(title_prefix))
		, m_frame_count(0)
		, m_fps_elapsed(0)
		, m_fps(0)
		, m_title_elapsed(0)
		, m_status_elapsed(0)
		, m_profile(profile_enabled)
		, m_closing(false)
	{
		// Open the requested profiler output before simulation events are connected.
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
			CompletePendingStep();
			m_scene.AllowSleeping(m_media.AllowSleeping());
			m_details.InvalidateValues();
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
				m_scene_filepath.clear();
				m_active_name = ScenarioName(m_scenario);
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
				// Retire both presentation owners before tests replace the simulation objects.
				PauseSimulation();
				m_view3d.WaitForGpu();
				m_view3d.m_scene.ClearDrawlists();
				if (m_scene.m_body_population)
					m_view3d.m_scene.RemovePopulation(*m_scene.m_body_population.get());

				m_details.InvalidateValues();
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
			// Submit batched bodies once; the remaining objects and debug overlays keep their existing routes.
			auto const beg = Clock::now();
			auto const w2c = scene.m_cam.WorldToCamera();
			auto const frustum = scene.m_cam.ViewFrustum();
			auto const clip_planes = scene.m_cam.ClipPlanes(false);
			if (m_scene.m_body_population)
				scene.AddPopulation(m_scene.m_body_population);

			for (auto i = isize(m_scene.m_body_instances); i != isize(m_scene.m_body); ++i)
				m_scene.m_body[i].AddToScene(scene, w2c, frustum, clip_planes);

			m_scene.AddArticulationsToScene(scene, w2c, frustum, clip_planes);
			m_scene.AddSampleOverlays(scene);

			if (m_scene.m_ground_gfx)
				m_scene.m_ground_gfx->AddToScene(scene);
			if (m_scene.m_terrain_gfx)
				m_scene.m_terrain_gfx->AddToScene(scene);
			if (m_scene.m_water_gfx)
				m_scene.m_water_gfx->AddToScene(scene, static_cast<float>(m_scene.m_clock));
			if (m_scene.m_atmosphere_gfx)
				m_scene.m_atmosphere_gfx->AddToScene(scene);
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
		// Retire GPU references before the sandbox releases its retained presentation.
		m_view3d.WaitForGpu();
		m_view3d.m_scene.ClearDrawlists();
		if (m_scene.m_body_population)
		{
			// Read existing completed GPU counters after the measured window, so a no-draw result cannot masquerade as throughput.
			if (m_profile.Enabled())
			{
				// Retain completed classifier and draw evidence without adding work to measured frames.
				auto const stats = m_scene.m_body_population->Stats();
				if (!m_profile.RecordPopulationStats(stats))
				{
					// Surface a failed diagnostic write without throwing from renderer shutdown.
					auto const* message = "ERROR: Failed to persist completed Population counters to profile.csv\n";
					OutputDebugStringA(message);
					std::fputs(message, stderr);
					std::fflush(stderr);
				}
			}
			m_view3d.m_scene.RemovePopulation(*m_scene.m_body_population.get());
		}
	}

	// Stop all scene interaction before explicit closure or after a simulation failure.
	void SandboxUI::BeginClose()
	{
		// Unlike ordinary pause, shutdown must not impose an infinite completion wait before its bounded close.
		m_closing = true;
		m_physics_accumulator = 0;
		m_steps_remaining = 0;
		m_media.Enabled(false);
		m_details.Enabled(false);
		m_view3d.Enabled(false);
	}

	// Override message processing to ensure clean shutdown
	bool SandboxUI::ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result)
	{
		if (message == WM_CLOSE)
		{
			// Stop rendering and simulation while closure is attempted, including after a retryable failure.
			BeginClose();
			auto const close_result = m_scene.TryClose(5000);
			if (close_result != S_OK)
			{
				// This interactive host chooses a five-second budget and retains the paused window for the user's next close attempt.
				auto const error = close_result == S_FALSE ? std::string("GPU cleanup timed out.") : pr::HrMsg(close_result);
				auto const message_text = error + "\nThe scene is retained safely. Close the window again to retry.";
				::MessageBoxA(hwnd, message_text.c_str(), "Cleanup pending", MB_OK | MB_ICONERROR);
				result = 0;
				return true;
			}
		}

		// A partly closed scene accepts only another window-close attempt.
		if (m_closing && message == WM_COMMAND)
		{
			// Do not let menu commands restart work on a closing physics engine.
			result = 0;
			return true;
		}

		// Reflect visibility changes made through any control before displaying the View menu.
		if (message == WM_INITMENUPOPUP && reinterpret_cast<HMENU>(wparam) == ::GetSubMenu(::GetMenu(hwnd), 1))
		{
			UpdateDetailsMenu(reinterpret_cast<HMENU>(wparam), m_details.m_pinned);
			UpdateDiagnosticMenu(reinterpret_cast<HMENU>(wparam), m_scene);
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

			if (id == MenuID::DetailsPanel)
			{
				m_details.TogglePin();
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

			// The sample overlay can accompany either base mode.
			if (id == MenuID::SurfaceSamples)
			{
				auto& overlays = m_scene.m_sample_overlays;
				overlays.Surface(!overlays.m_surface_enabled);

				UpdateVisualModeMenu();
				Render(0);
				result = 0;
				return true;
			}

			// Sleeping transparency changes appearance only, not the simulation's sleeping policy.
			if (id == MenuID::SleepingTransparency)
			{
				m_scene.SleepingTransparency(!m_scene.SleepingTransparency());
				UpdateVisualModeMenu();
				Render(0);
				result = 0;
				return true;
			}

			// Atmosphere overlays are independent diagnostics for the loaded climate scene.
			if (id == MenuID::AtmosphereGrid || id == MenuID::AtmosphereParticles)
			{
				if (m_scene.m_atmosphere_gfx != nullptr)
				{
					if (id == MenuID::AtmosphereGrid)
						m_scene.m_atmosphere_gfx->ShowGrid(!m_scene.m_atmosphere_gfx->ShowGrid());
					else
						m_scene.m_atmosphere_gfx->ShowParticles(!m_scene.m_atmosphere_gfx->ShowParticles());
				}
				UpdateVisualModeMenu();
				Render(0);
				result = 0;
				return true;
			}

			auto const demos = DemoCatalogue();
			if (id >= MenuID::DemoBase && id < MenuID::DemoBase + isize(demos))
			{
				auto const demo_index = id - MenuID::DemoBase;
				LoadSceneFile(demos[demo_index].m_filepath);

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
		m_details.InvalidateValues();

		// Make sure the GPU has finished with the models before releasing them.
		m_view3d.WaitForGpu();
		m_view3d.m_scene.ClearDrawlists();
		if (m_scene.m_body_population)
			m_view3d.m_scene.RemovePopulation(*m_scene.m_body_population.get());

		// Recreate whichever source currently owns the scene rather than retaining stale dynamics objects.
		if (!m_scene_filepath.empty())
		{
			LoadSceneFile(m_scene_filepath);
		}
		else
		{
			m_scene.Reset();
			m_scene.SetupScenario(m_scenario);
			m_active_name = ScenarioName(m_scenario);

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
		UpdateDiagnosticMenu(view_menu, m_scene);
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

			// Add to the recent files list (MRU order) and rebuild the submenu
			m_recent.Add(filepath);
			RebuildRecentFilesMenu();

			// Wait for the GPU to finish before loading. LoadFromJson triggers ShapeChange events which destroy old LdrObjects. 
			m_view3d.WaitForGpu();
			m_view3d.m_scene.ClearDrawlists();
			if (m_scene.m_body_population)
				m_view3d.m_scene.RemovePopulation(*m_scene.m_body_population.get());

			auto const wait_gpu_end = Clock::now();
			auto const wait_gpu_ms = ElapsedMs(mark, wait_gpu_end);
			mark = wait_gpu_end;

			// Load the scene from JSON (creates new body graphics automatically)
			m_details.InvalidateValues();
			auto scene_desc = scene_loader::LoadFromFile(filepath);
			auto const json_end = Clock::now();
			auto const json_ms = ElapsedMs(mark, json_end);
			mark = json_end;

			m_scene.LoadScene(scene_desc);
			m_scene_filepath = filepath;
			m_active_name = scene_desc.metadata ? scene_desc.metadata->m_name : filepath.stem().string();
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
			m_profile.RecordLoadScene(filepath, ElapsedMs(load_beg, camera_end), wait_gpu_ms, json_ms, camera_ms, m_scene);

			Render(0);
		}
		catch (std::exception const& ex)
		{
			// A GPU failure is terminal, unlike a rejected scene file; preserve its owners until the user closes the window.
			if (m_scene.m_physics.Closing() || (m_scene.m_atmosphere_gfx && m_scene.m_atmosphere_gfx->m_gpu.m_job.m_closing))
				BeginClose();

			auto msg = std::format("Failed to load scene:\n{}", ex.what());
			OutputDebugStringA(msg.c_str());

			auto log_path = AppDataPath() / "load_error.log";
			if (std::ofstream log(log_path); log)
				log << msg;
			
			::MessageBoxA(m_hwnd, msg.c_str(), "Load Error", MB_OK | MB_ICONERROR);
		}
	}

	// Complete submitted physics work before inspecting or replacing scene state.
	bool SandboxUI::CompletePendingStep()
	{
		if (!m_scene.StepPending())
			return false;

		auto const collision = m_scene.CompleteStep();
		m_details.InvalidateValues();
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

			// Discard only excess whole ticks so render-rate jitter cannot erase the fractional phase of the fixed-step clock.
			m_physics_accumulator = AccumulateSimulationTime(m_physics_accumulator, elapsed_seconds, time_scale, requested_tick_capacity, m_profile);

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

		// Sync the active body presentation to accepted physics transforms and visual colours.
		auto const sync_beg = Clock::now();
		m_scene.UpdateBodyGfx();
		m_scene.UpdateArticulationGfx();
		auto const sync_end = Clock::now();

		// Render the 3D viewport. Objects are added to the scene via the
		// OnAddToScene event during DoRender(), so no explicit Add/Remove needed.
		auto const render3d_beg = Clock::now();
		auto const clear_beg = Clock::now();
		m_view3d.m_scene.ClearDrawlists();
		if (m_scene.m_sample_overlays.m_refresh_pending)
		{
			m_view3d.WaitForGpu();
			m_scene.m_sample_overlays.Reset();
		}
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
		m_status_elapsed += elapsed_seconds;

		// Update the FPS each second
		if (m_fps_elapsed >= 1.0)
		{
			m_fps = m_frame_count / m_fps_elapsed;
			m_frame_count = 0;
			m_fps_elapsed = 0;
		}

		// Refresh visible details from completed scene changes, never repeatedly format an unchanged paused scene.
		if (m_steps_remaining == 0 && m_details.NeedsUpdate())
		{
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

			// Keep the caller's instance label while refreshing the live simulation statistics.
			SetWindowTextA(*this, std::format("{}Physics Sandbox [{}] t={:.3f} frame={} col={}  FPS: {:.0f}",
				m_title_prefix,
				m_active_name,
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
			auto new_status = std::format(L"t={:.3f}  {}  Collisions: {}  {}  FPS: {:.0f}",
				m_scene.m_clock,
				pr::Widen(m_active_name),
				m_scene.m_diag.count,
				m_steps_remaining == 0 ? L"[Paused]" : L"[Running]",
				m_fps);
			if (m_scene.m_sample_overlays.Enabled() && m_scene.m_sample_overlays.m_failed_targets != 0)
				new_status += std::format(L"  Sample overlays: {} target(s) with errors: {}", m_scene.m_sample_overlays.m_failed_targets, pr::Widen(m_scene.m_sample_overlays.m_first_error));

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
		if (m_scene.m_body.empty() && m_scene.m_articulation_visuals.empty() && !m_scene.m_terrain_mesh.has_value())
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

		// Terrain-only scenes still need a meaningful camera frame even without rigid bodies.
		if (m_scene.m_terrain_mesh.has_value())
		{
			auto const& terrain_bbox = m_scene.m_terrain_mesh->m_world_bounds;
			if (terrain_bbox.valid())
			{
				Grow(bbox, v4(static_cast<float>(terrain_bbox.m_centre.x), static_cast<float>(terrain_bbox.m_centre.y), static_cast<float>(terrain_bbox.m_centre.z), 1.0f) + v4(static_cast<float>(terrain_bbox.m_radius.x), static_cast<float>(terrain_bbox.m_radius.y), static_cast<float>(terrain_bbox.m_radius.z), 0.0f));
				Grow(bbox, v4(static_cast<float>(terrain_bbox.m_centre.x), static_cast<float>(terrain_bbox.m_centre.y), static_cast<float>(terrain_bbox.m_centre.z), 1.0f) - v4(static_cast<float>(terrain_bbox.m_radius.x), static_cast<float>(terrain_bbox.m_radius.y), static_cast<float>(terrain_bbox.m_radius.z), 0.0f));
			}
		}

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

#if PR_UNITTESTS
namespace physics_sandbox::tests
{
	// Verify the scheduler's actual loss accounting and the profiler's native CSV publication.
	PRUnitTestClass(SandboxClockTests, Serial)
	{
		// Read the actual runtime header and rows, excluding the separate load-record schema.
		static std::vector<std::vector<std::string>> ReadRows(std::filesystem::path const& path)
		{
			// A missing output is a test failure, not an empty successful profile.
			auto input = std::ifstream(path);
			if (!input)
				throw std::runtime_error("Profiler test output is missing");

			// Keep every runtime field, including the original whitespace accepted by numeric CSV readers.
			auto rows = std::vector<std::vector<std::string>>{};
			for (std::string line; std::getline(input, line);)
			{
				// Load records are not part of the runtime schema.
				if (!line.starts_with("time_s,") && (line.empty() || !std::isdigit(static_cast<unsigned char>(line.front()))))
					continue;

				// Preserve the real serializer's field boundaries.
				auto fields = std::vector<std::string>{};
				auto stream = std::istringstream(line);
				for (std::string field; std::getline(stream, field, ',');)
					fields.push_back(field);

				rows.push_back(std::move(fields));
			}
			return rows;
		}

		// Account for both removal decisions without losing fractional phase or changing admitted time.
		PRUnitTestMethod(SchedulerDiscardAccounting, Quick)
		{
			// Explicit expected seconds distinguish scaled clamp loss from whole-tick removal.
			struct Case
			{
				double m_accumulated, m_elapsed, m_scale;
				int m_capacity;
				double m_retained, m_discarded;
			};
			auto const dt = PhysicsStepSeconds;
			auto const cases = std::array{
				Case{0.0, dt, 1.0, 1, dt, 0.0},
				Case{0.25 * dt, 0.25 * dt, 1.0, 1, 0.5 * dt, 0.0},
				Case{3.5 * dt, 0.0, 1.0, 1, 1.5 * dt, 2.0 * dt},
				Case{0.0, 1.75 * dt, 2.0, 2, 2.5 * dt, dt},
				Case{0.0, 0.5, 0.03125, 1, 0.0078125, 0.0078125},
				Case{0.0, 0.5, 1.0, 1, dt, 0.25 + 14.0 * dt},
				Case{0.5 * dt, 5.0, 0.0, 1, 0.5 * dt, 0.0},
			};
			auto profile = SandboxProfiler(true);
			for (auto const& c : cases)
			{
				// The production helper must conserve admitted plus discarded time and match the prior scheduling expression exactly.
				profile.Reset();
				auto const retained = AccumulateSimulationTime(c.m_accumulated, c.m_elapsed, c.m_scale, c.m_capacity, profile);
				PR_EXPECT(std::abs(retained - c.m_retained) < 1e-12);
				PR_EXPECT(std::abs(profile.m_discarded_simulation_s - c.m_discarded) < 1e-12);
				PR_EXPECT(std::abs(c.m_accumulated + c.m_elapsed * c.m_scale - retained - profile.m_discarded_simulation_s) < 1e-12);
				PR_EXPECT(retained == BoundedAccumulatedTime(c.m_accumulated + std::min(c.m_elapsed, MaxFrameSeconds) * c.m_scale, c.m_capacity));
			}

			// Multiple scheduling decisions accumulate within one interval; reset clears loss, not the sampled steady clock.
			profile.Reset();
			AccumulateSimulationTime(3.5 * dt, 0.0, 1.0, 1, profile);
			AccumulateSimulationTime(0.0, 0.5, 0.03125, 1, profile);
			PR_EXPECT(std::abs(profile.m_discarded_simulation_s - (2.0 * dt + 0.0078125)) < 1e-12);
			auto const stamp = Clock::now();
			profile.m_last_render = stamp;
			profile.Reset();
			PR_EXPECT(profile.m_discarded_simulation_s == 0.0 && profile.m_last_render == stamp);

			// Profiling does not alter retention when disabled.
			auto disabled = SandboxProfiler(false);
			PR_EXPECT(std::abs(AccumulateSimulationTime(3.5 * dt, 0.0, 1.0, 1, disabled) - 1.5 * dt) < 1e-12);
			PR_EXPECT(disabled.m_discarded_simulation_s == 0.0);
		}

		// Preserve all 47 original fields and expose sample-to-sample time across the report publication gap.
		PRUnitTestMethod(CsvClockAndIntervalReset, Extended)
		{
			// Serialize known interval values through the actual Report method.
			auto scene = Scene(nullptr);
			scene.m_clock = 12.0;
			auto profile = SandboxProfiler(true);
			auto const path = temp_dir() / "clock-schema.csv";
			profile.Open(path);
			PR_EXPECT(profile.m_log.good());
			auto step = Scene::StepProfile{};
			step.m_engine.m_constraint_pack_ms = 2.75;
			step.m_engine.m_constraint_upload_ms = 1.25;
			profile.RecordStep(step, 16.0);
			profile.RecordStep(step, 18.0);
			profile.m_renders = 4;
			profile.m_elapsed = 2.0;
			AccumulateSimulationTime(3.5 * PhysicsStepSeconds, 0.0, 1.0, 1, profile);
			auto const first_stamp = Clock::time_point(std::chrono::seconds(12345678) + std::chrono::nanoseconds(123456789));
			profile.m_last_render = first_stamp;
			auto const before_report = Clock::now();
			profile.Report(scene);
			PR_EXPECT(profile.m_last_render == first_stamp);
			PR_EXPECT(profile.m_report_beg >= before_report && profile.m_elapsed == 0.0);
			PR_EXPECT(profile.m_discarded_simulation_s == 0.0 && profile.m_steps == 0 && profile.m_renders == 0);

			// Model a 250ms publication gap followed by a one-second interval without sleeping or changing runtime clocks.
			profile.m_report_beg = first_stamp + std::chrono::milliseconds(250);
			profile.m_last_render = first_stamp + std::chrono::milliseconds(1250);
			profile.m_elapsed = 1.0;
			scene.m_clock = 13.0;
			profile.Report(scene);
			auto const rows = ReadRows(path);
			PR_EXPECT(rows.size() == 3 && rows.at(0).size() == 49 && rows.at(1).size() == 49 && rows.at(2).size() == 49);

			// The original schema is an independent ordered oracle, not inferred from the new serializer.
			auto const original_header = std::string{
				"time_s,bodies,steps,renders,fps,step_ms,gravity_ms,physics_ms,kill_zone_ms,eng_new_frame_ms,eng_pack_ms,eng_upload_ms,"
				"eng_external_forces_ms,eng_integrate_ms,eng_sleepwake_ms,eng_broadphase_ms,eng_collide_ms,eng_resolve_ms,eng_selective_ms,"
				"eng_sleepupdate_ms,eng_readback_ms,eng_gpu_run_ms,eng_unpack_ms,eng_gpu_prepare_ms,eng_gpu_execute_ms,eng_gpu_wait_ms,"
				"eng_gpu_reset_ms,eng_readback_access_ms,eng_body_readback_copy_ms,eng_contact_readback_copy_ms,eng_collision_events_ms,"
				"eng_sleep_island_unpack_ms,eng_body_unpack_ms,render_ms,sync_gfx_ms,add_scene_ms,do_render_ms,clear_drawlists_ms,new_frame_ms,"
				"scene_render_ms,present_ms,details_ms,title_ms,status_ms,max_frame_ms,eng_constraint_pack_ms,eng_constraint_upload_ms"};
			auto header = std::string{};
			for (auto index = 0; index != 47; ++index)
			{
				// Verify positions, names and the two untouched trailing CPU phase fields.
				header += (index != 0 ? "," : "") + rows.at(0).at(index);
			}
			PR_EXPECT(header == original_header);
			PR_EXPECT(rows.at(0).at(47) == "steady_time_s" && rows.at(0).at(48) == "discarded_simulation_s");
			auto expected = std::array<double, 47>{};
			expected[0] = 12.0;
			expected[2] = 2.0;
			expected[3] = 4.0;
			expected[4] = 2.0;
			expected[5] = 17.0;
			expected[45] = 2.75;
			expected[46] = 1.25;
			for (auto index = 0; index != 47; ++index)
				PR_EXPECT(std::stod(rows.at(1).at(index)) == expected[index]);

			// Steady-epoch timestamps retain sub-tick precision and include the gap excluded by rebased m_elapsed.
			auto const first_seconds = std::chrono::duration<double>(first_stamp.time_since_epoch()).count();
			PR_EXPECT(std::abs(std::stod(rows.at(1).at(47)) - first_seconds) < 1e-8);
			PR_EXPECT(std::abs(std::stod(rows.at(2).at(47)) - std::stod(rows.at(1).at(47)) - 1.25) < 1e-8);
			PR_EXPECT(std::abs(std::stod(rows.at(1).at(48)) - 2.0 * PhysicsStepSeconds) < 1e-9);
			PR_EXPECT(std::stod(rows.at(2).at(48)) == 0.0 && std::stod(rows.at(2).at(2)) == 0.0);
			PR_EXPECT(std::stod(rows.at(2).at(0)) == 13.0);
		}

		// The public render path publishes its real existing steady-clock sample and accumulated scheduling losses.
		PRUnitTestMethod(RenderSamplesExistingSteadyClock, Extended)
		{
			// Force one report by moving only the fixture's report deadline into the past.
			auto scene = Scene(nullptr);
			auto profile = SandboxProfiler(true);
			auto const path = temp_dir() / "clock-render.csv";
			profile.Open(path);
			AccumulateSimulationTime(0.0, 0.5, 1.0, 1, profile);
			profile.m_report_beg = Clock::now() - std::chrono::seconds(2);
			auto const before = Clock::now();
			profile.RecordRender(scene, {});
			auto const after = Clock::now();

			// Check real serialization rather than a second formatter or synthetic timestamp-only path.
			auto const rows = ReadRows(path);
			PR_EXPECT(rows.size() == 2 && rows.at(1).size() == 49);
			PR_EXPECT(profile.m_last_render >= before && profile.m_last_render <= after);
			PR_EXPECT(std::abs(std::stod(rows.at(1).at(47)) - std::chrono::duration<double>(profile.m_last_render.time_since_epoch()).count()) < 1e-8);
			PR_EXPECT(std::abs(std::stod(rows.at(1).at(48)) - (0.25 + 14.0 * PhysicsStepSeconds)) < 1e-9);
			PR_EXPECT(std::stod(rows.at(1).at(0)) == 0.0 && std::stod(rows.at(1).at(2)) == 0.0);
			PR_EXPECT(profile.m_discarded_simulation_s == 0.0);
		}
	};

	// The visibility command is independent of the renderer's visualisation modes.
	PRUnitTestClass(DetailsPanelMenuTests)
	{
		// Opening the menu reflects the panel's current visibility without changing other menu choices.
		PRUnitTestMethod(VisibilityAndShortcut, Quick)
		{
			auto const menu = CreateViewMenu();
			auto label = std::array<wchar_t, 64>{};
			PR_EXPECT(::GetMenuStringW(menu, MenuID::DetailsPanel, label.data(), static_cast<int>(label.size()), MF_BYCOMMAND) != 0);
			PR_EXPECT(std::wstring_view(label.data()) == L"&Details panel\tD");
			PR_EXPECT((::GetMenuState(menu, MenuID::DetailsPanel, MF_BYCOMMAND) & MF_CHECKED) == 0);
			for (auto visible : {true, false, true})
			{
				UpdateDetailsMenu(menu, visible);
				PR_EXPECT(((::GetMenuState(menu, MenuID::DetailsPanel, MF_BYCOMMAND) & MF_CHECKED) != 0) == visible);
				PR_EXPECT((::GetMenuState(menu, MenuID::VisualModeNormal, MF_BYCOMMAND) & MF_CHECKED) != 0);
			}
			::DestroyMenu(menu);
		}
	};

	// The native menu exposes the sample overlay outside the base-mode radio group.
	PRUnitTestClass(SampleOverlayMenuTests)
	{
		// Check actual menu state without opening or driving an application window.
		PRUnitTestMethod(OverlayAccompaniesEitherBaseMode, Quick)
		{
			auto const menu = CreateViewMenu();
			auto const submenu = ::GetSubMenu(menu, 3);
			auto overlays = SampleOverlays{};
			PR_EXPECT(submenu != nullptr && ::GetMenuItemCount(submenu) == 2);
			PR_EXPECT((::GetMenuState(submenu, MenuID::SleepingTransparency, MF_BYCOMMAND) & MF_CHECKED) != 0);
			for (auto mode : {MenuID::VisualModeNormal, MenuID::VisualModeContactPriority})
			{
				::CheckMenuRadioItem(menu, MenuID::VisualModeNormal, MenuID::VisualModeContactPriority, mode, MF_BYCOMMAND);
				overlays.Surface(true);
				UpdateOverlayMenu(menu, overlays, true);
				PR_EXPECT((::GetMenuState(menu, mode, MF_BYCOMMAND) & MF_CHECKED) != 0);
				PR_EXPECT((::GetMenuState(submenu, MenuID::SurfaceSamples, MF_BYCOMMAND) & MF_CHECKED) != 0);
				overlays.Surface(false);
				UpdateOverlayMenu(menu, overlays, false);
				PR_EXPECT((::GetMenuState(submenu, MenuID::SurfaceSamples, MF_BYCOMMAND) & MF_CHECKED) == 0);
				PR_EXPECT((::GetMenuState(menu, mode, MF_BYCOMMAND) & MF_CHECKED) != 0);
				PR_EXPECT((::GetMenuState(submenu, MenuID::SleepingTransparency, MF_BYCOMMAND) & MF_CHECKED) == 0);
			}
			::DestroyMenu(menu);
		}
	};
}
#endif
