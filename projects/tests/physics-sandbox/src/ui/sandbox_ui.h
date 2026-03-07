#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"
#include "src/ui/media_panel.h"
#include "src/ui/details_panel.h"

namespace physics_sandbox
{
	// Menu command IDs for the sandbox application
	namespace MenuID
	{
		static constexpr int OpenFile = 1001;
	}

	// Main window for the physics sandbox application.
	// Assembles the 3D viewport, media controls, details panel, and menu bar
	// into a resizable layout, and wires up the simulation loop.
	struct SandboxUI : Form
	{
		// UI components, ordered by docking precedence (bottom-up):
		// StatusBar docks to bottom, MediaPanel above it, DetailsPanel to right,
		// View3DPanel fills the remaining space.
		StatusBar   m_status;
		MediaPanel  m_media;
		DetailsPanel m_details;
		View3DPanel m_view3d;

		// Physics simulation
		Scene       m_scene;
		bool        m_pause_on_collision;

		// Path of the last loaded scene file, so Reset reloads the same scene
		std::filesystem::path m_scene_filepath;

		// Diagnostic visualisation
		view3d::Object m_trail_gfx;
		view3d::Object m_diag_gfx;

		// Cached status text to avoid flickering from constant WM_SETTEXT
		std::wstring m_last_status;

		// FPS measurement
		int m_frame_count;
		double m_fps_elapsed;
		double m_fps;

		// Rate-limiting accumulators for expensive operations.
		// These prevent costly Win32 API calls and LDraw object rebuilds
		// from running at the full sim/render rate.
		double m_title_elapsed;      // Title bar update interval (every 0.25s)
		double m_diag_gfx_elapsed;   // Diagnostic gfx rebuild interval (every 0.1s)
		double m_details_elapsed;    // Details panel update interval (every 0.2s)

		// Set when WM_CLOSE is received to prevent step/render after destruction begins
		bool m_closing;

		SandboxUI()
			: Form(Params<>()
				.name("physics-sandbox")
				.title(L"Rylogic Physics Sandbox")
				.wh(1280, 800)
				.start_pos(EStartPosition::CentreParent)
				.padding(0)
				.menu({ {L"&File", Menu(Menu::EKind::Popup, {
					MenuItem(L"&Open Scene...\tCtrl+O", MenuID::OpenFile),
					MenuItem(MenuItem::Separator),
					MenuItem(L"E&xit", IDCLOSE),
				})} })
				.main_wnd(true)
				.wndclass(RegisterWndClass<SandboxUI>()))
			, m_status(StatusBar::Params<>().parent(this_).dock(EDock::Bottom))
			, m_media(MediaPanel::Params<>().parent(this_))
			, m_details(DetailsPanel::Params<>().parent(this_))
			, m_view3d(View3DPanel::Params()
				.parent(this_)
				.error_cb(ReportErrorCB, this_)
				.dock(EDock::Fill)
				.show_focus_point())
			, m_scene()
			, m_pause_on_collision(false)
			, m_trail_gfx()
			, m_diag_gfx()
			, m_frame_count(0)
			, m_fps_elapsed(0)
			, m_fps(0)
			, m_title_elapsed(0)
			, m_diag_gfx_elapsed(0)
			, m_details_elapsed(0)
			, m_closing(false)
		{
			// Wire media panel events to simulation control
			m_media.OnPlay += [&](auto&, auto&)
			{
				m_scene.m_steps_remaining = -1; // Run continuously
			};
			m_media.OnPause += [&](auto&, auto&)
			{
				m_scene.m_steps_remaining = 0; // Pause
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
					m_scene.m_scenario = static_cast<EScenario>(args.m_vk_key - '0');
					ResetScene();
					m_scene.m_steps_remaining = -1; // Auto-start
				}

				// R=reset, S=single step, G=go (play), P=pause
				if (args.m_vk_key == 'R')
					ResetScene();
				if (args.m_vk_key == 'S')
					m_scene.m_steps_remaining = 1;
				if (args.m_vk_key == 'G')
					m_scene.m_steps_remaining = -1;
				if (args.m_vk_key == 'P')
					m_scene.m_steps_remaining = 0;

				// C=toggle pause-on-collision
				if (args.m_vk_key == 'C')
					m_pause_on_collision = !m_pause_on_collision;

				// T=run all test scenarios
				if (args.m_vk_key == 'T')
					m_scene.RunAllTests();

				// Ctrl+O=open scene file
				if (args.m_vk_key == 'O' && (GetKeyState(VK_CONTROL) & 0x8000))
					OpenSceneFile();
			};

			// Camera setup: Z-up alignment and perspective projection
			View3D_CameraAlignAxisSet(m_view3d.m_win, view3d::Vec4{ 0, 0, 1, 0 });
			View3D_CameraOrthographicSet(m_view3d.m_win, FALSE);

			// Start with the sandbox scenario
			ResetScene();
		}

		~SandboxUI()
		{
			// Remove all graphics from the window and delete them BEFORE member
			// destructors run. Body::~Body deletes the View3D object, but if they're
			// still associated with the window, View3D_Shutdown (in ~m_view3d) asserts.
			for (int i = 0; i != m_scene.m_body_count; ++i)
			{
				if (m_scene.m_body[i].m_gfx)
					View3D_WindowRemoveObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
			}
			RemoveGroundGfx();
			CleanupDiagGfx();
		}

		// Override message processing to ensure clean shutdown
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			if (message == WM_CLOSE)
			{
				// Set closing flag so the step/render lambdas stop accessing View3D
				m_closing = true;

				// Clean up View3D objects before the window is destroyed.
				// After DestroyWindow, the View3DPanel's HWND is invalid and swap chain
				// present may hang or crash.
				for (int i = 0; i != m_scene.m_body_count; ++i)
				{
					if (m_scene.m_body[i].m_gfx)
						View3D_WindowRemoveObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
				}
				RemoveGroundGfx();
				CleanupDiagGfx();
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
			}

			return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
		}

		// Reset the scene and sync graphics.
		// If a scene file was loaded, reload it from disk. Otherwise, reset the built-in scenario.
		void ResetScene()
		{
			// If a scene file was loaded, reload it from scratch
			if (!m_scene_filepath.empty())
			{
				LoadSceneFile(m_scene_filepath);
				return;
			}

			// Remove old body graphics from the viewport
			for (int i = 0; i != m_scene.m_body_count; ++i)
			{
				if (m_scene.m_body[i].m_gfx)
					View3D_WindowRemoveObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
			}

			RemoveGroundGfx();
			CleanupDiagGfx();
			m_scene.Reset();

			// Add body graphics to the viewport
			for (int i = 0; i != m_scene.m_body_count; ++i)
			{
				if (m_scene.m_body[i].m_gfx)
					View3D_WindowAddObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
			}

			// Add ground visual if the scene has one
			AddGroundGfx();

			// Set up a sensible default camera position looking at the origin
			View3D_ResetView(m_view3d.m_win,
				view3d::Vec4{ 0, -1, 0, 0 },  // Forward direction (looking from +Y toward -Y)
				view3d::Vec4{ 0, 0, 1, 0 },   // Up direction (Z-up)
				20,                           // Distance
				TRUE, TRUE);

			Render();
		}

		// Show the Open File dialog and load a JSON scene file
		void OpenSceneFile()
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

		// Load a scene from a JSON file path.
		// Can be called before or after Show() — if the window isn't visible yet,
		// the graphics will be added when ResetScene-equivalent logic runs.
		void LoadSceneFile(std::filesystem::path const& filepath)
		{
			try
			{
				// Remove old body graphics from the viewport before loading
				for (int i = 0; i != m_scene.m_body_count; ++i)
				{
					if (m_scene.m_body[i].m_gfx)
						View3D_WindowRemoveObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
				}
				RemoveGroundGfx();
				CleanupDiagGfx();

				// Remember the filepath so Reset can reload it
				m_scene_filepath = filepath;

				// Load the scene from JSON
				m_scene.LoadFromJson(filepath);

				// Add new body graphics to the viewport
				for (int i = 0; i != m_scene.m_body_count; ++i)
				{
					if (m_scene.m_body[i].m_gfx)
						View3D_WindowAddObject(m_view3d.m_win, m_scene.m_body[i].m_gfx);
				}

				// Add ground visual if the scene has one
				AddGroundGfx();

				// Reset camera to frame all bodies in the scene.
				// Compute a bounding box that encompasses all body positions,
				// then use View3D_ResetViewBBox for proper framing.
				auto bbox = ComputeSceneBBox();
				View3D_ResetViewBBox(m_view3d.m_win, bbox,
					view3d::Vec4{ 0, -1, 0, 0 },
					view3d::Vec4{ 0, 0, 1, 0 },
					0, TRUE, TRUE);

				Render();
			}
			catch (std::exception const& ex)
			{
				::MessageBoxA(m_hwnd, pr::FmtS("Failed to load scene:\n%s", ex.what()), "Load Error", MB_OK | MB_ICONERROR);
			}
		}

		// Advance the simulation by one timestep
		void Step(double elapsed_seconds)
		{
			// Don't step after close begins
			if (m_closing)
				return;

			// Accumulate time for FPS measurement and rate-limited UI updates
			m_fps_elapsed += elapsed_seconds;
			m_title_elapsed += elapsed_seconds;
			m_diag_gfx_elapsed += elapsed_seconds;
			m_details_elapsed += elapsed_seconds;
			if (m_fps_elapsed >= 1.0)
			{
				m_fps = m_frame_count / m_fps_elapsed;
				m_frame_count = 0;
				m_fps_elapsed = 0;
			}

			// Check if we should be stepping
			if (m_scene.m_steps_remaining == 0)
				return;
			if (m_scene.m_steps_remaining > 0)
				--m_scene.m_steps_remaining;

			// Step the physics scene
			auto collision = m_scene.Step(elapsed_seconds);

			// Pause on first collision if requested
			if (collision && m_pause_on_collision && m_scene.m_diag.count == 1)
				m_scene.m_steps_remaining = 0;
		}

		// Render a frame: sync graphics, rebuild overlays, update details panel.
		// Expensive UI operations are rate-limited to avoid dominating frame time.
		void Render()
		{
			// Don't render after close begins
			if (m_closing)
				return;

			++m_frame_count;

			// Sync each body's View3D graphics to its physics transform.
			// This is cheap — just copying an O2W matrix per body.
			for (int i = 0; i != m_scene.m_body_count; ++i)
				m_scene.m_body[i].UpdateGfx();

			// Render the 3D viewport directly. Calling View3D_WindowRender bypasses
			// the WM_PAINT message queue, which is low-priority and can cause frame
			// starvation when the sim loop is busy. This matches how Lost at Sea renders.
			View3D_WindowRender(m_view3d.m_win);

			// Rebuild trail and diagnostic visualisation at reduced rate (~2 Hz).
			// Creating View3D objects from LDraw text is expensive (text generation,
			// parsing, GPU resource allocation). Keep the rate low to avoid frame drops.
			if (m_diag_gfx_elapsed >= 0.5)
			{
				m_diag_gfx_elapsed = 0;
				RebuildDiagGfx();
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
				SetWindowTextA(*this, pr::FmtS("Physics Sandbox [%d: %s] t=%.3f col=%d  FPS: %.0f",
					static_cast<int>(m_scene.m_scenario),
					ScenarioName(m_scene.m_scenario),
					m_scene.m_clock,
					m_scene.m_diag.count,
					m_fps));
			}

			// Update status bar (only when text changes to avoid flicker)
			auto new_status = std::wstring(pr::FmtS(L"t=%.3f  %hs  Collisions: %d  %s  FPS: %.0f",
				m_scene.m_clock,
				ScenarioName(m_scene.m_scenario),
				m_scene.m_diag.count,
				m_scene.m_steps_remaining == 0 ? L"[Paused]" : L"[Running]",
				m_fps));
			if (new_status != m_last_status)
			{
				m_last_status = std::move(new_status);
				m_status.Text(0, m_last_status.c_str());
			}
		}

		// Rebuild the LDraw objects for trails, velocity arrows, and collision markers
		void RebuildDiagGfx()
		{
			using namespace pr::rdr12::ldraw;

			CleanupDiagGfx();

			// Build trail line strips for each body
			bool has_trails = false;
			for (int i = 0; i != m_scene.m_body_count; ++i)
			{
				if (m_scene.m_trail[i].size() >= 2)
					has_trails = true;
			}

			if (has_trails)
			{
				Builder trail_builder;

				// Per-body trails with distinct colours
				Colour32 trail_colours[] = { 0xFFFF4040, 0xFF4040FF, 0xFF40FF40, 0xFFFFFF40 };
				for (int i = 0; i != m_scene.m_body_count; ++i)
				{
					if (m_scene.m_trail[i].size() < 2)
						continue;

					auto colour = trail_colours[i % std::size(trail_colours)];
					auto& line = trail_builder.Line(std::string(pr::FmtS("trail_%d", i)), colour);
					line.strip(m_scene.m_trail[i][0]);
					for (size_t j = 1; j < m_scene.m_trail[i].size(); ++j)
						line.line_to(m_scene.m_trail[i][j]);
				}

				// System centre of mass trail (yellow, if 2+ bodies have trails)
				if (m_scene.m_body_count >= 2 && m_scene.m_trail[0].size() >= 2 && m_scene.m_trail[1].size() >= 2)
				{
					auto total_mass = m_scene.m_body[0].Mass() + m_scene.m_body[1].Mass();
					auto& line = trail_builder.Line("trail_CoM", 0xFFFFFF00);
					auto com0 = (m_scene.m_trail[0][0] * m_scene.m_body[0].Mass() + m_scene.m_trail[1][0] * m_scene.m_body[1].Mass()) / total_mass;
					com0.w = 1;
					line.strip(com0);
					auto n = std::min(m_scene.m_trail[0].size(), m_scene.m_trail[1].size());
					for (size_t i = 1; i < n; ++i)
					{
						auto com = (m_scene.m_trail[0][i] * m_scene.m_body[0].Mass() + m_scene.m_trail[1][i] * m_scene.m_body[1].Mass()) / total_mass;
						com.w = 1;
						line.line_to(com);
					}
				}

				auto trail_text = trail_builder.ToText(false);
				m_trail_gfx = View3D_ObjectCreateLdrA(trail_text.c_str(), false, nullptr, nullptr);
				if (m_trail_gfx)
					View3D_WindowAddObject(m_view3d.m_win, m_trail_gfx);
			}

			// Build diagnostic overlays: velocity arrows, contact marker, collision normal
			{
				Builder diag_builder;
				Colour32 vel_colours[] = { 0xFFFF8080, 0xFF8080FF, 0xFF80FF80, 0xFFFFFF80 };

				for (int i = 0; i != m_scene.m_body_count; ++i)
				{
					auto vel = m_scene.m_body[i].VelocityWS();
					auto pos = m_scene.m_body[i].O2W().pos;

					// Linear velocity arrow
					auto speed = Length(vel.lin);
					if (speed > 0.01f)
					{
						auto colour = vel_colours[i % std::size(vel_colours)];
						auto& arrow = diag_builder.Line(std::string(pr::FmtS("vel_%d", i)), colour);
						arrow.line(pos, pos + vel.lin);
						arrow.arrow(EArrowType::Fwd, 8.0f);
					}

					// Angular velocity arrow (green)
					auto aspeed = Length(vel.ang);
					if (aspeed > 0.01f)
					{
						auto& arrow = diag_builder.Line(std::string(pr::FmtS("avel_%d", i)), 0xFF00FF00);
						arrow.line(pos, pos + vel.ang);
						arrow.arrow(EArrowType::Fwd, 6.0f);
					}
				}

				// Collision markers (shown after collision has occurred)
				if (m_scene.m_diag.count > 0)
				{
					diag_builder.Sphere("contact_pt", 0xFFFFFFFF).radius(0.08f).pos(m_scene.m_diag.contact_point_ws);

					auto& normal = diag_builder.Line("contact_normal", 0xFF00FFFF);
					normal.line(m_scene.m_diag.contact_point_ws, m_scene.m_diag.contact_point_ws + m_scene.m_diag.contact_normal_ws * 2.0f);
					normal.arrow(EArrowType::Fwd, 8.0f);
				}

				auto diag_text = diag_builder.ToText(false);
				m_diag_gfx = View3D_ObjectCreateLdrA(diag_text.c_str(), false, nullptr, nullptr);
				if (m_diag_gfx)
					View3D_WindowAddObject(m_view3d.m_win, m_diag_gfx);
			}
		}

		void CleanupDiagGfx()
		{
			if (m_trail_gfx)
			{
				View3D_WindowRemoveObject(m_view3d.m_win, m_trail_gfx);
				View3D_ObjectDelete(m_trail_gfx);
				m_trail_gfx = nullptr;
			}
			if (m_diag_gfx)
			{
				View3D_WindowRemoveObject(m_view3d.m_win, m_diag_gfx);
				View3D_ObjectDelete(m_diag_gfx);
				m_diag_gfx = nullptr;
			}
		}

		// Add the ground visual to the viewport (if the scene has one)
		void AddGroundGfx()
		{
			if (m_scene.m_ground_gfx)
				View3D_WindowAddObject(m_view3d.m_win, m_scene.m_ground_gfx);
		}

		// Remove the ground visual from the viewport (but don't delete it — Scene owns it)
		void RemoveGroundGfx()
		{
			if (m_scene.m_ground_gfx)
				View3D_WindowRemoveObject(m_view3d.m_win, m_scene.m_ground_gfx);
		}

		// Compute a bounding box that encompasses all bodies in the scene.
		// Used to frame the camera when loading a new scene.
		view3d::BBox ComputeSceneBBox() const
		{
			if (m_scene.m_body_count == 0)
				return view3d::BBox{ {0, 0, 0, 1}, {5, 5, 5, 0} };

			// Find the min/max extents of all dynamic body positions.
			// Also include the ground plane height (Z=0 typically) so the camera
			// can see the ground surface, not just the bodies floating in space.
			auto lo = pr::v4{ +1e10f, +1e10f, +1e10f, 1 };
			auto hi = pr::v4{ -1e10f, -1e10f, -1e10f, 1 };
			for (int i = 0; i != m_scene.m_body_count; ++i)
			{
				// Skip static (ground) bodies — they're huge and would dominate the bbox
				if (m_scene.m_body[i].Mass() > pr::physics::InfiniteMass * 0.5f)
					continue;

				auto pos = m_scene.m_body[i].O2W().pos;
				lo.x = pr::Min(lo.x, pos.x - 2.0f);
				lo.y = pr::Min(lo.y, pos.y - 2.0f);
				lo.z = pr::Min(lo.z, pos.z - 2.0f);
				hi.x = pr::Max(hi.x, pos.x + 2.0f);
				hi.y = pr::Max(hi.y, pos.y + 2.0f);
				hi.z = pr::Max(hi.z, pos.z + 2.0f);
			}

			// If there's a ground visual, include the ground height in the bbox
			// so the camera can see where objects will land.
			if (m_scene.m_ground_gfx)
			{
				lo.z = pr::Min(lo.z, 0.0f);
			}

			auto centre = (lo + hi) * 0.5f;
			auto radius = (hi - lo) * 0.5f;
			centre.w = 1;
			radius.w = 0;
			return view3d::BBox{
				{centre.x, centre.y, centre.z, 1},
				{radius.x, radius.y, radius.z, 0}
			};
		}

		// Handle errors reported within view3d
		static void __stdcall ReportErrorCB(void* ctx, char const* msg, char const* filepath, int line, int64_t)
		{
			auto this_ = static_cast<SandboxUI*>(ctx);
			auto message = pr::FmtS(L"%s(%d): %s", filepath, line, msg);
			::MessageBoxW(this_->m_hwnd, message, L"Error", MB_OK);
		}
	};
}