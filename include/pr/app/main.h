//*****************************************************************************************
// Application Framework
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
// How To Use:
//  namespace ns
//  {
//      // Derive a application logic type from app::Main
//      struct Main :app::Main<UserSettings, MainUI>
//      {
//          static char const* AppName() const { return "MyNewApp"; };
//          Main(MainUI& gui)
//              :base(app::DefaultSetup(), gui)
//          {}
//      };
//
//      // Derive a GUI class from app::MainUI
//      struct MainUI :app::MainUI<MainUI, Main>
//      {
//          static wchar_t const* AppTitle() const { return L"My New App"; };
//          MainUI()
//              :app::MainUI<MainUI, Main>(Params().title(AppTitle()))
//          {}
//      };
//  }
//  namespace pr::app
//  {
//      // Create the GUI window
//      std::unique_ptr<IAppMainGui> CreateGUI(wchar_t const* lpstrCmdLine, int nCmdShow)
//      {
//          return std::unique_ptr<IAppMainGui>(new ns::MainUI(lpstrCmdLine, nCmdShow));
//      }
//  }
//
// Notes:
//  - If you're trying to declare everything in a header file, you'll need to define
//    'PR_APP_MAIN_INCLUDE' to be the header file containing the CreateGUI implementation.

#pragma once
#include "pr/app/forward.h"

namespace pr::app
{
	// The application business logic
	template<typename Derived, typename MainUI, typename UserSettings>
	struct alignas(16) Main
	{
		// Notes:
		//  - Apps should inherit this type providing custom functionality where required.
		//  - The App framework creates the UI first so that the HWND exists before 'Main'
		//    is created. This allows normal construction of the renderer etc.

		log::Logger     m_log;         // App log
		UserSettings    m_settings;    // Application-wide user settings
		rdr12::Renderer m_rdr;         // The renderer
		rdr12::Window   m_window;      // The window that will be rendered into
		rdr12::Scene    m_scene;       // The main view
		Camera&         m_cam;         // The main scene camera
		MainUI&         m_ui;          // The GUI that owns this app logic class
		bool            m_rdr_pending; // Render call batching, true if 'RenderNeeded' has been called

		static log::Logger::OutputCB LoggerOutput()
		{
			return log::ToFile(FmtS("%s.log", Derived::AppName()));
		}

		// Construct using a template set up object.
		template <typename Setup>
		Main(Setup setup, MainUI& ui)
			:m_log(Derived::AppName(), LoggerOutput(), log::EMode::Async)
			,m_settings(setup.UserSettings())
			,m_rdr(setup.RdrSettings())
			,m_window(m_rdr, setup.RdrWindowSettings(ui, m_rdr.Settings()))
			,m_scene(m_window)
			,m_cam(m_scene.m_cam)
			,m_ui(ui)
			,m_rdr_pending(false)
		{
			// Position the camera
			m_cam.FovY(constants<float>::tau_by_8);
			m_cam.LookAt(
				v4(0, 0, 1.0f / (float)tan(m_cam.FovY()/2.0f), 1.0f),
				v4::Origin(),
				v4::YAxis());

			// The first frame is needed
			RenderNeeded();
		}
		Main(Main const&) = delete;
		Main& operator=(Main const&) = delete;
		virtual ~Main() {}

		// Default mouse navigation
		virtual void Nav(v2 pt, gui::EMouseKey btn_state, bool nav_start_stop)
		{
			auto op = camera::MouseBtnToNavOp(int(btn_state));
			m_cam.MouseControl(pt, op, nav_start_stop);
			RenderNeeded();
		}
		virtual void NavZ(v2 pt, float delta, bool along_ray)
		{
			m_cam.MouseControlZ(pt, delta, along_ray);
			RenderNeeded();
		}
		virtual void NavRevert()
		{
			m_cam.Revert();
			RenderNeeded();
		}

		// The size of the window has changed
		virtual void Resize(iv2 const& size)
		{
			m_window.BackBufferSize(size, false);
			m_scene.m_viewport.Set(size);
			m_scene.m_cam.Aspect(1.0 * size.x / size.y);
		}

		// Request a render.
		// Note: this can be called many times per frame with minimal cost
		virtual void RenderNeeded()
		{
			m_rdr_pending = true;
		}

		// Render the scene. This is left to the derived app to call when appropriate.
		// For game-style apps that use a SimMsgLoop, DoRender can be called in a step context
		//  e.g.
		//   m_msg_loop.AddStepContext("render loop", [this](double){ m_main->DoRender(true); }, 60.0f, false);
		// For general apps, DoRender could be called from a Timer, or in Paint
		void DoRender(bool force = false)
		{
			// Only render if asked to
			if (!m_rdr_pending && !force)
				return;

			// Allow new render requests now
			m_rdr_pending = false;

			m_scene.ClearDrawlists();
			auto& frame = m_window.NewFrame();
			m_scene.Render(frame);
			m_window.Present(frame, rdr12::EGpuFlush::Block);
		}

		// Show the last rendered scene
		void Present()
		{
			m_window.Present();
		}
	};
}