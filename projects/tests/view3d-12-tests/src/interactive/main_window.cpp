//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// The interactive demo host: a main window with menus, a 3D view, a controls panel, and a status bar.
#include <cstdlib>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <windows.h>
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/gui/view3d_panel.h"
#include "pr/win32/windows_com.h"
#include "pr/win32/win32.h"
#include "pr/storage/json.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/audio/audio-dll.h"
#include "interactive.h"
#include "demo.h"
#include "controls_panel.h"

using namespace pr;
using namespace pr::gui;

namespace view3d_test
{
	// Settings for interactive mode, read from 'view3d-12-tests.config.json' beside the executable.
	struct InteractiveConfig
	{
		std::filesystem::path m_rylogic_assets; // Root of a rylogic-assets checkout

		// Load the config file. Throws if the file or a required key is missing.
		static InteractiveConfig Load(std::filesystem::path const& filepath)
		{
			// The config file is copied beside the executable by the build
			if (!std::filesystem::exists(filepath))
				throw std::runtime_error(std::format("Config file not found: {}", filepath.string()));

			auto doc = json::Read(filepath, json::Options{ .AllowComments = true, .AllowTrailingCommas = true });
			auto const& root = doc.to_object();
			auto const* rylogic_assets = root.find("RylogicAssets");
			if (rylogic_assets == nullptr)
				throw std::runtime_error(std::format("'RylogicAssets' is missing from {}", filepath.string()));

			return InteractiveConfig{
				.m_rylogic_assets = rylogic_assets->to<std::filesystem::path>(),
			};
		}
	};

	// Per-user settings that persist between runs
	struct UserSettings
	{
		std::string m_last_demo; // The name of the demo shown when the window last closed

		// The settings file location. Creates the directory if needed.
		static std::filesystem::path FilePath()
		{
			// Store settings under %APPDATA% so they survive rebuilds and are independent of the build output folder
			char appdata[MAX_PATH] = {};
			size_t len = 0;
			if (getenv_s(&len, appdata, "APPDATA") != 0 || len == 0)
				throw std::runtime_error("The APPDATA environment variable is not set");

			auto dir = std::filesystem::path(appdata) / "RylogicView3d12Tests";
			std::filesystem::create_directories(dir);
			return dir / "settings.json";
		}

		// Load the settings. Missing or unreadable settings give the defaults, since they only affect convenience.
		static UserSettings Load()
		{
			// Read 'LastDemo' if the file exists and contains it
			auto settings = UserSettings{};
			try
			{
				auto filepath = FilePath();
				if (!std::filesystem::exists(filepath))
					return settings;

				auto doc = json::Read(filepath, json::Options{ .AllowComments = true, .AllowTrailingCommas = true });
				if (auto const* last_demo = doc.to_object().find("LastDemo"); last_demo != nullptr)
					settings.m_last_demo = last_demo->to<std::string>();
			}
			catch (std::exception const& ex)
			{
				std::cerr << "Ignoring unreadable settings: " << ex.what() << std::endl;
			}
			return settings;
		}

		// Save the settings
		void Save() const
		{
			// Demo names are plain identifiers, so they need no JSON escaping
			std::ofstream file(FilePath());
			file << std::format("{{\n\t\"LastDemo\": \"{}\"\n}}\n", m_last_demo);
		}
	};

	// View3d error handler. Errors are programming or asset errors, so they stop the current operation.
	static void __stdcall ReportError(void*, char const* msg, char const* filepath, int line, int64_t)
	{
		// Report to the console as well, since the exception may be caught and summarised
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
		throw std::runtime_error(std::string(msg));
	}

	// Audio error handler. Failures are also returned as status codes, so only report them here.
	static void __stdcall ReportAudioError(void*, char const* msg, char const* filepath, int line)
	{
		// Make asynchronous audio failures visible in the console
		std::cout << filepath << "(" << line << "): " << msg << std::endl;
	}

	// The scene settings that demos may change. The host restores them between demos so each demo starts from the same state.
	struct SceneDefaults
	{
		std::vector<view3d::Light> m_lights;
		view3d::Colour m_ambient;
		view3d::ShadowSettings m_shadows;
		view3d::FarClipFadeProps m_far_clip_fade;
		view3d::UnderwaterProps m_underwater;
		float m_dither;
		view3d::Vec2 m_clip_planes;

		// Record the current settings of 'window'
		static SceneDefaults Capture(view3d::Window window)
		{
			// Read every setting that a demo is allowed to change
			auto defaults = SceneDefaults{};
			for (int i = 0, iend = View3D_LightCount(window); i != iend; ++i)
				defaults.m_lights.push_back(View3D_LightGet(window, i));

			defaults.m_ambient = View3D_AmbientGet(window);
			defaults.m_shadows = View3D_ShadowSettingsGet(window);
			defaults.m_far_clip_fade = View3D_FarClipFadePropertiesGet(window);
			defaults.m_underwater = View3D_PostEffectUnderwaterGet(window);
			defaults.m_dither = View3D_DitherAmountGet(window);
			defaults.m_clip_planes = View3D_CameraClipPlanesGet(window, view3d::EClipPlanes::Both);
			return defaults;
		}

		// Apply the recorded settings to 'window'
		void Restore(view3d::Window window) const
		{
			// Match the light count first, then overwrite every light
			while (View3D_LightCount(window) > static_cast<int>(m_lights.size()))
				View3D_LightRemove(window, View3D_LightCount(window) - 1);
			while (View3D_LightCount(window) < static_cast<int>(m_lights.size()))
				View3D_LightAdd(window, m_lights[View3D_LightCount(window)]);
			for (int i = 0, iend = static_cast<int>(m_lights.size()); i != iend; ++i)
				View3D_LightSet(window, i, m_lights[i]);

			// Restore the remaining scene settings
			View3D_AmbientSet(window, m_ambient);
			View3D_ShadowSettingsSet(window, m_shadows);
			View3D_FarClipFadePropertiesSet(window, m_far_clip_fade);
			View3D_PostEffectUnderwaterSet(window, m_underwater);
			View3D_DitherAmountSet(window, m_dither);
			View3D_CameraClipPlanesSet(window, m_clip_planes.x, m_clip_planes.y, view3d::EClipPlanes::Both);
		}
	};

	// The 3D view. Input goes to the active demo before the default view3d navigation and key bindings.
	struct ViewHost :View3DPanel
	{
		IDemo* m_demo; // The active demo, not owned. Null when no demo is active.

		explicit ViewHost(View3DPanel::Params const& p)
			: View3DPanel(p)
			, m_demo()
		{
		}

		// Give raw window messages to the demo first, so that demos with their own UI see unmodified input
		LRESULT WndProc(UINT message, WPARAM wparam, LPARAM lparam) override
		{
			// Demos that consume a message prevent the default handling
			LRESULT result = 0;
			if (m_demo != nullptr && m_demo->ProcessWindowMessage(m_hwnd, message, wparam, lparam, result))
				return result;

			return View3DPanel::WndProc(message, wparam, lparam);
		}

		// Give key events to the demo before the default key bindings
		void OnKey(KeyEventArgs& args) override
		{
			// The base class skips its bindings for keys that the demo handled
			if (m_demo != nullptr)
				m_demo->OnKey(args);

			View3DPanel::OnKey(args);
		}

		// Give mouse button events to the demo before the default navigation
		void OnMouseButton(MouseEventArgs& args) override
		{
			// Demos that handle a click prevent it from starting a camera navigation
			if (m_demo != nullptr)
				m_demo->OnMouseButton(args);
			if (args.m_handled)
				return;

			View3DPanel::OnMouseButton(args);
		}

		// Keep the viewport matched to the window size
		void OnWindowPosChange(WindowPosEventArgs const& args) override
		{
			// The base class resizes the back buffer. The viewport is set to cover all of it.
			View3DPanel::OnWindowPosChange(args);
			if (!args.m_before && args.IsResize() && !args.Iconic())
			{
				auto w = args.m_wp->cx;
				auto h = args.m_wp->cy;
				View3D_WindowViewportSet(m_win, view3d::Viewport{
					.m_x = 0,
					.m_y = 0,
					.m_width = 1.f * w,
					.m_height = 1.f * h,
					.m_min_depth = 0,
					.m_max_depth = 1,
					.m_screen_w = w,
					.m_screen_h = h,
				});
			}
		}
	};

	// Menu command identifiers
	enum EMenuId :int
	{
		ID_ReloadScripts = 1000,
		ID_ResetCamera,
		ID_ShowFocusPoint,
		ID_ShowOrigin,
		ID_MultiSampling1,
		ID_MultiSampling2,
		ID_MultiSampling4,
		ID_MultiSampling8,
		ID_FillSolid,
		ID_FillWireframe,
		ID_FillSolidWire,
		ID_FillPoints,
		ID_Background0,
		ID_DemoBase = 2000,
	};

	// Multi-sampling menu options
	struct MultiSamplingOption
	{
		int m_id;
		int m_samples;
		wchar_t const* m_label;
	};
	static MultiSamplingOption const s_multi_sampling[] =
	{
		{ ID_MultiSampling1, 1, L"&Off" },
		{ ID_MultiSampling2, 2, L"&2x" },
		{ ID_MultiSampling4, 4, L"&4x" },
		{ ID_MultiSampling8, 8, L"&8x" },
	};

	// Fill mode menu options
	struct FillModeOption
	{
		int m_id;
		view3d::EFillMode m_mode;
		wchar_t const* m_label;
	};
	static FillModeOption const s_fill_modes[] =
	{
		{ ID_FillSolid, view3d::EFillMode::Solid, L"&Solid" },
		{ ID_FillWireframe, view3d::EFillMode::Wireframe, L"&Wireframe" },
		{ ID_FillSolidWire, view3d::EFillMode::SolidWire, L"Solid + W&ire" },
		{ ID_FillPoints, view3d::EFillMode::Points, L"&Points" },
	};

	// Background colour menu options. The first is the default.
	struct BackgroundOption
	{
		unsigned int m_argb;
		wchar_t const* m_label;
	};
	static BackgroundOption const s_backgrounds[] =
	{
		{ 0xFF908080, L"&Grey" },
		{ 0xFF000000, L"&Black" },
		{ 0xFFFFFFFF, L"&White" },
		{ 0xFF203050, L"&Navy" },
	};

	// Build the 'View' menu
	static HMENU CreateViewMenu()
	{
		// Radio-style sub menus for the multi-valued settings. Check marks are refreshed when the menu opens.
		auto msaa = Menu(Menu::EKind::Popup);
		for (auto const& opt : s_multi_sampling)
			msaa.Insert(MenuItem(opt.m_label, opt.m_id));

		auto fill = Menu(Menu::EKind::Popup);
		for (auto const& opt : s_fill_modes)
			fill.Insert(MenuItem(opt.m_label, opt.m_id));

		auto bkgd = Menu(Menu::EKind::Popup);
		for (int i = 0, iend = static_cast<int>(std::size(s_backgrounds)); i != iend; ++i)
			bkgd.Insert(MenuItem(s_backgrounds[i].m_label, ID_Background0 + i));

		return Menu(Menu::EKind::Popup, {
			MenuItem(L"&Multi-sampling", msaa),
			MenuItem(L"&Fill mode", fill),
			MenuItem(L"&Background", bkgd),
			MenuItem(MenuItem::Separator),
			MenuItem(L"Show &focus point", ID_ShowFocusPoint),
			MenuItem(L"Show &origin", ID_ShowOrigin),
			MenuItem(MenuItem::Separator),
			MenuItem(L"&Reset camera", ID_ResetCamera),
		});
	}

	// Build the 'Demos' menu, with one sub menu per category
	static HMENU CreateDemoMenu()
	{
		// The catalogue keeps demos of the same category adjacent, so a new sub menu starts whenever the category changes
		auto menu = Menu(Menu::EKind::Popup);
		auto demos = DemoCatalogue();
		for (int i = 0, iend = static_cast<int>(demos.size()); i != iend;)
		{
			// Collect the demos in this category
			auto category = demos[i].m_category;
			auto sub = Menu(Menu::EKind::Popup);
			for (; i != iend && std::wstring_view(demos[i].m_category) == category; ++i)
				sub.Insert(MenuItem(demos[i].m_display_name, ID_DemoBase + i));

			menu.Insert(MenuItem(category, sub));
		}
		return menu;
	}

	// The 3D view creation parameters
	static View3DPanel::Params ViewParams(Control* parent)
	{
		// The view fills the space that the status bar and controls panel leave
		auto p = View3DPanel::Params();
		p.parent(parent).dock(EDock::Fill).error_cb(ReportError, nullptr).multisamp(8);
		p.m_win_opts.back_colour(s_backgrounds[0].m_argb).alt_enter().name("TestWnd");
		return p;
	}

	// The application window
	struct Main :Form
	{
		// Members are constructed in order. 'm_view' creates all window handles, so it must follow the other controls,
		// and 'm_ui_ready' must come first because messages arrive while the controls are being created.
		bool m_ui_ready;
		InteractiveConfig m_config;
		UserSettings m_settings;
		StatusBar m_status;
		ControlsPanel m_controls;
		ViewHost m_view;
		audio::DllHandle m_audio;
		SceneDefaults m_defaults;
		std::unique_ptr<IDemo> m_demo;
		int m_demo_index;
		std::wstring m_status_text;
		std::chrono::steady_clock::time_point m_fps_start;
		int m_fps_frames;
		double m_fps;

		Main(InteractiveConfig const& config, UserSettings const& settings)
			: Form(Params<>()
				.name("main")
				.title(L"View3d 12 Tests")
				.wh(1600, 1000)
				.xy(10, 10)
				.start_pos(EStartPosition::Manual)
				.padding(0)
				.menu({
					{ L"&File", Menu(Menu::EKind::Popup, {
						MenuItem(L"&Reload scripts", ID_ReloadScripts),
						MenuItem(MenuItem::Separator),
						MenuItem(L"E&xit", IDCLOSE),
					})},
					{ L"&View", CreateViewMenu() },
					{ L"&Demos", CreateDemoMenu() },
				})
				.main_wnd(true)
				.wndclass(RegisterWndClass<Main>()))
			, m_ui_ready(false)
			, m_config(config)
			, m_settings(settings)
			, m_status(StatusBar::Params<>().parent(this_).dock(EDock::Bottom))
			, m_controls(ControlsPanel::Params().parent(this_))
			, m_view(ViewParams(this_))
			, m_audio(Audio_Initialise({ this, ReportAudioError }))
			, m_defaults()
			, m_demo()
			, m_demo_index(-1)
			, m_status_text()
			, m_fps_start(std::chrono::steady_clock::now())
			, m_fps_frames(0)
			, m_fps(0)
		{
			// All controls exist now, so messages can be handled
			m_ui_ready = true;
			if (m_audio == nullptr)
				throw std::runtime_error("Audio initialization failed");

			// The baseline light is a fixed directional light, so that shading does not change as the camera moves
			demo::EditLight(m_view.m_win, 0, [](view3d::Light& light)
			{
				light.m_type = view3d::ELight::Directional;
				light.m_direction = view3d::Vec4{ -0.577f, -0.577f, -0.577f, 0 };
				light.m_cast_shadow = 0.0f;
				light.m_cam_relative = FALSE;
			});

			// Record the baseline that every demo starts from
			m_defaults = SceneDefaults::Capture(m_view.m_win);
			ResetCamera();

			// Allow external tools to stream LDraw script into the view (see StreamingTest.csx)
			View3D_StreamingEnable(TRUE, 1976);
		}
		~Main()
		{
			// Destroy the demo before the audio context and the 3D view that it uses
			m_ui_ready = false;
			CloseDemo();
			if (m_audio != nullptr)
				Audio_Shutdown(m_audio);
		}

		// Advance the active demo and render a frame
		void Step(double dt)
		{
			// Demos animate before rendering so that each frame shows the latest state
			if (m_demo != nullptr)
				m_demo->Step(dt);

			UpdateStatus();
			View3D_WindowRender(m_view.m_win);
		}

		// Show the demo at 'index' in the catalogue, replacing the active demo
		void SelectDemo(int index)
		{
			// Remove the previous demo and return the scene to the baseline state
			CloseDemo();
			m_defaults.Restore(m_view.m_win);
			ResetCamera();

			// Describe the demo above its own controls
			auto const& info = DemoCatalogue()[index];
			m_demo_index = index;
			m_controls.AddHeading(info.m_display_name);
			m_controls.AddLabel(info.m_description);
			Text(std::format(L"View3d 12 Tests - {}", info.m_display_name).c_str());

			// Remember the choice for the next run, even if the demo fails, so the failure is reproducible
			m_settings.m_last_demo = std::string(info.m_name);
			m_settings.Save();

			// A failing demo is reported in the controls panel so that other demos remain usable
			try
			{
				auto ctx = DemoContext{
					.m_view3d = m_view.m_ctx,
					.m_window = m_view.m_win,
					.m_view_hwnd = m_view,
					.m_assets = m_config.m_rylogic_assets,
					.m_audio = m_audio,
					.m_controls = m_controls,
				};
				m_demo = info.m_create(ctx);
				m_view.m_demo = m_demo.get();
			}
			catch (std::exception const& ex)
			{
				// Show the failure in the panel, and on the console for scripted runs
				CloseDemo();
				m_controls.AddHeading(info.m_display_name);
				m_controls.AddLabel(std::format(L"Demo failed: {}", Widen(ex.what())));
				std::cerr << "Demo '" << info.m_name << "' failed: " << ex.what() << std::endl;
			}
		}

		// Destroy the active demo and remove everything it added to the scene
		void CloseDemo()
		{
			// The controls' callbacks capture the demo, so they go first
			m_view.m_demo = nullptr;
			m_controls.Clear();
			View3D_WindowRemoveAllObjects(m_view.m_win);
			View3D_WindowEnvMapSet(m_view.m_win, nullptr);
			m_demo.reset();
		}

		// Move the camera to the default viewing position
		void ResetCamera()
		{
			// Look at the origin from above and to one side, with Z up. Navigation keeps the camera's up axis aligned to Z, so the horizon stays level.
			View3D_CameraPositionSet(m_view.m_win, { 5, -5, 4, 1 }, { 0, 0, 0, 1 }, { 0, 0, 1, 0 });
			View3D_CameraAlignAxisSet(m_view.m_win, { 0, 0, 1, 0 });
		}

		// Show the frame rate, camera position, and camera direction in the status bar
		void UpdateStatus()
		{
			// Average the frame rate over half-second windows so the value is readable and the text does not change every frame
			++m_fps_frames;
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration<double>(now - m_fps_start).count();
			if (elapsed >= 0.5)
			{
				m_fps = m_fps_frames / elapsed;
				m_fps_frames = 0;
				m_fps_start = now;
			}

			// Only update the status bar when the text changes, to avoid redrawing it every frame
			auto c2w = View3D_CameraToWorldGet(m_view.m_win);
			auto text = std::format(L"FPS: {:.1f}  Cam: {:.3f} {:.3f} {:.3f}  Dir: {:.3f} {:.3f} {:.3f}", m_fps, c2w.w.x, c2w.w.y, c2w.w.z, -c2w.z.x, -c2w.z.y, -c2w.z.z);
			if (text == m_status_text)
				return;

			m_status_text = text;
			m_status.Text(0, m_status_text);
		}

		// Update the menu check marks to match the current state
		void UpdateMenuChecks()
		{
			// The check marks are read from the renderer, so there is no separate menu state to keep in sync
			auto menu = ::GetMenu(m_hwnd);
			auto check = [=](int id, bool on)
			{
				::CheckMenuItem(menu, id, MF_BYCOMMAND | (on ? MF_CHECKED : MF_UNCHECKED));
			};

			auto samples = View3D_MultiSamplingGet(m_view.m_win);
			for (auto const& opt : s_multi_sampling)
				check(opt.m_id, opt.m_samples == samples);

			auto fill_mode = View3D_WindowFillModeGet(m_view.m_win);
			for (auto const& opt : s_fill_modes)
				check(opt.m_id, opt.m_mode == fill_mode || (fill_mode == view3d::EFillMode::Default && opt.m_mode == view3d::EFillMode::Solid));

			auto bkgd = View3D_WindowBackgroundColourGet(m_view.m_win);
			for (int i = 0, iend = static_cast<int>(std::size(s_backgrounds)); i != iend; ++i)
				check(ID_Background0 + i, s_backgrounds[i].m_argb == bkgd);

			check(ID_ShowFocusPoint, View3D_StockObjectVisibleGet(m_view.m_win, view3d::EStockObject::FocusPoint) != 0);
			check(ID_ShowOrigin, View3D_StockObjectVisibleGet(m_view.m_win, view3d::EStockObject::OriginPoint) != 0);
			for (int i = 0, iend = static_cast<int>(DemoCatalogue().size()); i != iend; ++i)
				check(ID_DemoBase + i, i == m_demo_index);
		}

		// Handle a menu command. Returns false for commands that this window does not handle.
		bool HandleCommand(int id)
		{
			// Fixed commands
			auto win = m_view.m_win;
			switch (id)
			{
				case ID_ReloadScripts:
				{
					View3D_ReloadScriptSources();
					return true;
				}
				case ID_ResetCamera:
				{
					ResetCamera();
					return true;
				}
				case ID_ShowFocusPoint:
				{
					View3D_StockObjectVisibleSet(win, view3d::EStockObject::FocusPoint, !View3D_StockObjectVisibleGet(win, view3d::EStockObject::FocusPoint));
					return true;
				}
				case ID_ShowOrigin:
				{
					View3D_StockObjectVisibleSet(win, view3d::EStockObject::OriginPoint, !View3D_StockObjectVisibleGet(win, view3d::EStockObject::OriginPoint));
					return true;
				}
				default:
				{
					break;
				}
			}

			// Option commands
			for (auto const& opt : s_multi_sampling)
			{
				if (opt.m_id != id)
					continue;

				View3D_MultiSamplingSet(win, opt.m_samples);
				return true;
			}
			for (auto const& opt : s_fill_modes)
			{
				if (opt.m_id != id)
					continue;

				View3D_WindowFillModeSet(win, opt.m_mode);
				return true;
			}
			if (id >= ID_Background0 && id < ID_Background0 + static_cast<int>(std::size(s_backgrounds)))
			{
				View3D_WindowBackgroundColourSet(win, s_backgrounds[id - ID_Background0].m_argb);
				return true;
			}

			// Demo selection
			if (id >= ID_DemoBase && id < ID_DemoBase + static_cast<int>(DemoCatalogue().size()))
			{
				SelectDemo(id - ID_DemoBase);
				return true;
			}
			return false;
		}

		// Handle menu messages
		bool ProcessWindowMessage(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			// Messages arrive while the child controls are being created, before the members are ready
			if (m_ui_ready && hwnd == m_hwnd)
			{
				// Refresh the check marks just before a menu is shown
				if (message == WM_INITMENUPOPUP)
					UpdateMenuChecks();

				// Menu commands have a zero 'lparam'. Control notifications carry the control's handle instead.
				if (message == WM_COMMAND && lparam == 0 && HandleCommand(LOWORD(wparam)))
				{
					result = 0;
					return true;
				}
			}
			return Form::ProcessWindowMessage(hwnd, message, wparam, lparam, result);
		}
	};
}

// Run the interactive demos until the window closes
int RunInteractive(std::string_view demo_name)
{
	using namespace view3d_test;
	try
	{
		// Choose the initial demo: the command line, then the last used demo, then the first demo
		auto settings = UserSettings::Load();
		auto const* initial = FindDemo(demo_name.empty() ? std::string_view(settings.m_last_demo) : demo_name);
		if (initial == nullptr && !demo_name.empty())
		{
			std::cerr << "Unknown demo: " << demo_name << "\nAvailable demos:\n";
			for (auto const& info : DemoCatalogue())
				std::cerr << "  " << info.m_name << "\n";

			return 2;
		}
		if (initial == nullptr)
			initial = &DemoCatalogue()[0];

		// Load settings and the runtime DLLs that only interactive mode uses
		pr::InitCom com;
		auto config = InteractiveConfig::Load(pr::win32::ExeDir() / "view3d-12-tests.config.json");
		pr::win32::LoadDll<struct Audio>("audio.dll");

		// Register the owning message loop so window destruction can drain continuously posted renderer messages and observe WM_QUIT
		WinGuiMsgLoop loop;
		Main main(config, settings);
		main.cp().msg_loop(&loop);
		main.Show();

		// Start the demo once the window has its final size, since some demos lay out against the view size
		main.SelectDemo(static_cast<int>(initial - DemoCatalogue().data()));

		loop.AddMessageFilter(main);
		loop.AddLoop(100.0, true, [&main](auto dt) { main.Step(dt); });
		return loop.Run();
	}
	catch (std::exception const& ex)
	{
		// Report to the console and the debugger, since either may be watching
		std::cerr << "Died: " << ex.what() << std::endl;
		OutputDebugStringA("Died: ");
		OutputDebugStringA(ex.what());
		OutputDebugStringA("\n");
		return -1;
	}
}
