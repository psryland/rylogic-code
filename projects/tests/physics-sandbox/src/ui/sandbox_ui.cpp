#include "src/forward.h"
#include "src/ui/menu_id.h"
#include "src/ui/sandbox_ui.h"

namespace physics_sandbox
{
	SandboxUI::SandboxUI()
		: Form(Params<>()
			.name("physics-sandbox")
			.title(L"Rylogic Physics Sandbox")
			.wh(1600, 1000)
			.start_pos(EStartPosition::CentreParent)
			.padding(0)
			.menu({ {L"&File", Menu(Menu::EKind::Popup, {
				MenuItem(L"&Open Scene...\tCtrl+O", MenuID::OpenFile),
				MenuItem(MenuItem::Separator),
				MenuItem(L"Recent Files", ::CreatePopupMenu()),
				MenuItem(MenuItem::Separator),
				MenuItem(L"E&xit", IDCLOSE),
			})} })
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
		, m_scenario(EScenario::Sandbox)
		, m_scene_filepath()
		, m_recent()
		, m_last_status()
		, m_frame_count(0)
		, m_fps_elapsed(0)
		, m_fps(0)
		, m_title_elapsed(0)
		, m_details_elapsed(0)
		, m_status_elapsed(0)
		, m_closing(false)
	{
		// Load the recent files list from disk and populate the submenu
		m_recent.Load();
		RebuildRecentFilesMenu();

		// Wire media panel events to simulation control
		m_media.OnPlay += [&](auto&, auto&)
		{
			m_steps_remaining = -1; // Run continuously
		};
		m_media.OnPause += [&](auto&, auto&)
		{
			m_steps_remaining = 0; // Pause
		};
		m_media.OnReset += [&](auto&, auto&)
		{
			ResetScene();
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
				ResetScene();
				m_steps_remaining = -1; // Auto-start
			}

			// R=reset, S=single step, G=go (play), P=pause
			if (args.m_vk_key == 'R')
				ResetScene();
			if (args.m_vk_key == 'S')
				m_steps_remaining = 1;
			if (args.m_vk_key == 'G')
				m_steps_remaining = -1;
			if (args.m_vk_key == 'P')
				m_steps_remaining = 0;

			// C=toggle pause-on-collision
			if (args.m_vk_key == 'C')
				m_pause_on_collision = !m_pause_on_collision;

			// U=toggle GPU/CPU integration
			if (args.m_vk_key == 'U')
				m_scene.m_physics.UseGpu(!m_scene.m_physics.UseGpu());

			// T=run all test scenarios
			if (args.m_vk_key == 'T')
				m_scene.RunAllTests();

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
		};

		// Hook the scene population event — called each frame during DoRender() to add
		// objects to the scene's drawlist before rendering.
		m_view3d.OnAddToScene += [&](auto&, rdr12::Scene& scene)
		{
			for (int i = 0; i != std::ssize(m_scene.m_body); ++i)
				m_scene.m_body[i].AddToScene(scene);

			if (m_scene.m_ground_gfx)
				m_scene.m_ground_gfx->AddToScene(scene);
			if (m_scene.m_origin_gfx)
				m_scene.m_origin_gfx->AddToScene(scene);
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
		// Pause the simulation
		m_steps_remaining = 0;

		// Make sure the GPU has finished with the models before releasing them.
		m_view3d.WaitForGpu();
		m_scene.Reset();

		// Reset to the last loaded scene file
		if (!m_scene_filepath.empty())
		{
			LoadSceneFile(m_scene_filepath);
		}
		else
		{
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

	// Load a scene from a JSON file path.
	// Takes filepath by value to avoid dangling references when m_recent.Add()
	// reorders the MRU list (the caller may pass m_recent.m_paths[i]).
	void SandboxUI::LoadSceneFile(std::filesystem::path filepath)
	{
		try
		{
			// Pause the simulation
			m_steps_remaining = 0;

			// Remember the filepath so Reset can reload it
			m_scene_filepath = filepath;

			// Add to the recent files list (MRU order) and rebuild the submenu
			m_recent.Add(filepath);
			RebuildRecentFilesMenu();

			// Wait for the GPU to finish before loading. LoadFromJson triggers ShapeChange events which destroy old LdrObjects. 
			m_view3d.WaitForGpu();

			// Load the scene from JSON (creates new body graphics automatically)
			auto scene_desc = scene_loader::LoadFromFile(filepath);
			m_scene.LoadScene(scene_desc);

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

	// Advance the simulation by one timestep
	void SandboxUI::Step(double elapsed_seconds)
	{
		// Don't step after close begins
		if (m_closing)
			return;

		// Check if we should be stepping
		if (m_steps_remaining == 0)
			return;
		if (m_steps_remaining > 0)
			--m_steps_remaining;

		// Apply the time scale from the slow-mo slider.
		// This scales the physics dt so that 0.5x = half speed, 2.0x = double speed, etc.
		// Wall-clock elapsed_seconds drives the render loop; sim_dt drives the physics.
		auto sim_dt = elapsed_seconds * m_media.TimeScale();

		// Step the physics scene with the scaled timestep
		auto collision = m_scene.Step(sim_dt);

		// Pause on first collision if requested
		if (collision && m_pause_on_collision && m_scene.m_diag.count == 1)
			m_steps_remaining = 0;
	}

	// Render a frame: sync graphics, rebuild overlays, update details panel.
	// Expensive UI operations are rate-limited to avoid dominating frame time.
	void SandboxUI::Render(double elapsed_seconds)
	{
		// Don't render after close begins
		if (m_closing)
			return;

		++m_frame_count;

		// Sync each body's View3D graphics to its physics transform.
		// This is cheap — just copying an O2W matrix per body.
		for (int i = 0; i != std::ssize(m_scene.m_body); ++i)
			m_scene.m_body[i].UpdateGfx();

		// Render the 3D viewport. Objects are added to the scene via the
		// OnAddToScene event during DoRender(), so no explicit Add/Remove needed.
		m_view3d.DoRender();

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

		// Update the details panel text at reduced rate (~5 Hz).
		if (m_details_elapsed >= 0.2)
		{
			m_details_elapsed = 0;
			m_details.Update(m_scene);
		}

		// Update title bar at reduced rate (~4 Hz). SetWindowTextA triggers
		// non-client repaint which is expensive at high frequency.
		if (m_title_elapsed >= 0.25)
		{
			m_title_elapsed = 0;

			// Keep the slider's speed label text in sync with the trackbar position
			m_media.UpdateSpeedLabel();

			SetWindowTextA(*this, std::format("Physics Sandbox [{}: {}] t={:.3f} col={}  {}  FPS: {:.0f}",
				static_cast<int>(m_scene.m_current_scenario),
				ScenarioName(m_scene.m_current_scenario),
				m_scene.m_clock,
				m_scene.m_diag.count,
				m_scene.m_physics.UseGpu() ? "GPU" : "CPU",
				m_fps).c_str());
		}

		// Update status bar (only when text changes to avoid flicker)
		if (m_status_elapsed >= 0.2)
		{
			m_status_elapsed = 0;
			auto new_status = std::format(L"t={:.3f}  {}  Collisions: {}  {}  FPS: {:.0f}",
				m_scene.m_clock,
				pr::Widen(ScenarioName(m_scene.m_current_scenario)),
				m_scene.m_diag.count,
				m_steps_remaining == 0 ? L"[Paused]" : L"[Running]",
				m_fps);

			if (new_status != m_last_status)
			{
				m_last_status = std::move(new_status);
				m_status.Text(0, m_last_status.c_str());
			}
		}
	}

	// Compute a bounding box that encompasses all bodies in the scene.
	// Used to frame the camera when loading a new scene.
	BBox SandboxUI::ComputeSceneBBox() const
	{
		if (m_scene.m_body.empty())
			return BBox{ v4{0, 0, 0, 1}, v4{5, 5, 5, 0} };

		auto bbox = BBox::Reset();
		for (auto const& body : m_scene.m_body)
			Grow(bbox, body.O2W().pos);

		// If there's a ground visual, include the ground height in the bbox
		// so the camera can see where objects will land.
		if (m_scene.m_ground_gfx)
			Grow(bbox, v4::Origin());

		bbox *= 1.2f;
		return bbox;
	}
}
