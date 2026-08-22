//*****************************************************************************************
// Application Framework
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
// See pr/app/main.h for usage instructions
#pragma once
#include "pr/app/forward.h"
#pragma warning (disable:4355)

namespace pr::app
{
	// A base class for a main app window.
	template<typename DerivedUI, typename Main, typename MessageLoop = gui::WinGuiMsgLoop>
	struct MainUI :gui::Form, IAppMainUI
	{
		// Notes:
		//  - The MainUI derived class handles the HWND and WndProc for the application
		//    It should forward all meaning full work to 'm_main'.
		//  - MessageLoop can also be SimMsgLoop

		using base = gui::Form;
		using MainPtr = std::unique_ptr<Main>;

		MessageLoop m_msg_loop;                  // The message pump
		MainPtr     m_main;                      // The app logic object
		bool        m_resizing;                  // True during a resize of the main window
		bool        m_painting;                  // Blocks reentrancy into OnPaint
		bool        m_nav_enabled;               // True while a mouse button is down during default mouse navigation
		bool        m_fullscreen_toggle_enabled; // Allow Alt+Enter to toggle between full screen and windowed
		LONG        m_click_thres;               // Single click time threshold in ms
		LONG        m_down_at[4];                // Button down time stamp
		int         m_exit_code;                 // Exit code

		// WinGui form construction parameters
		template <typename Derived = void>
		struct Params :gui::Form::Params<not_void_t<Derived, Params<Derived>>>
		{
			bool m_default_mouse_navigation;
			Params()
			{
				this->wndclass(RegisterWndClass<DerivedUI>())
					.main_wnd(true)
					.padding(8)
					.default_mouse_navigation(true);
			}
			Params& default_mouse_navigation(bool on)
			{
				m_default_mouse_navigation = on;
				return this->me();
			}
		};

		// Create the main application window.
		// This class is sub-classed from gui::Form which actually does the 'CreateWindowEx' call in
		// it's constructor. This means the HWND is valid after the base class has been constructed.
		// If your window uses common controls, remember to call InitCtrls() before this constructor
		MainUI()
			:MainUI(Params())
		{}
		template <typename TParams>
		explicit MainUI(TParams const& p)
			:Form(p)
			,m_msg_loop()
			,m_main(new Main(*static_cast<DerivedUI*>(&CreateHandle())))
			,m_resizing(false)
			,m_painting(false)
			,m_nav_enabled(false)
			,m_fullscreen_toggle_enabled(true)
			,m_click_thres(200)
			,m_down_at()
			,m_exit_code()
		{
			// Route main-window destruction through the loop that owns this UI's scheduled callbacks.
			cp().msg_loop(&m_msg_loop);

			// Note: derived classes may need to set up a method for rendering,
			// By default, rendering occurs in OnPaint(), however if a SimMsgLoop
			// is used, the derived class will need to register a step context that
			// calls Render()
			Show();
		}
		virtual ~MainUI()
		{
			// The base Form can outlive member destruction, so do not leave it pointing at the loop member.
			cp().msg_loop(nullptr);
		}

		// Pump messages
		virtual int Run() override
		{
			return m_msg_loop.Run();
		}

		// Message map function
		bool ProcessWindowMessage(HWND parent_hwnd, UINT message, WPARAM wparam, LPARAM lparam, LRESULT& result) override
		{
			switch (message)
			{
			case WM_SYSKEYDOWN:
				{
					auto vk_key  = UINT(wparam);
					auto repeats = UINT(lparam & 0xFFFF);
					auto flags   = UINT((lparam & 0xFFFF0000) >> 16);
					OnSysKeyDown(vk_key, repeats, flags);
					break;
				}
			}
			return base::ProcessWindowMessage(parent_hwnd, message, wparam, lparam, result);
		}

		// Invalidate the control for redraw
		virtual void Invalidate(bool erase = false, gui::Rect* rect = nullptr)
		{
			if (m_main) m_main->RenderNeeded();
			return base::Invalidate(erase, rect);
		}

		// Called when the system menu key command to switch between full-screen and windowed is detected.
		// Derived types need to actually implement the mode switch as well as hide or show status bars, menus etc
		virtual void OnFullScreenToggle(bool enable_fullscreen)
		{
			(void)enable_fullscreen;
		}

		// Rendering the window
		virtual void OnPaint(gui::PaintEventArgs& args) override
		{
			// Render the scene before raising the event, so that handlers
			// have the option of drawing over the top of the 3D scene
			if (m_main && !m_painting)
			{
				pr::Scope<bool&> paint_guard(m_painting);
				m_main->DoRender(true); // We've been asked to paint, so paint, regardless of RenderNeeded()
				args.m_handled = true;

				// Tell windows we've drawn the viewport area
				gui::Rect cr = m_main->m_scene.m_viewport.AsRECT();
				Validate(&cr);
			}

			// Call the base to raise the paint event
			base::OnPaint(args);
		}

		// Default mouse navigation behaviour
		virtual void OnMouseButton(gui::MouseEventArgs& args) override
		{
			base::OnMouseButton(args);
			if (args.m_handled || !cp<Params<>>().m_default_mouse_navigation) return;

			m_nav_enabled = args.m_down;
			m_main->Nav(NormalisePoint(*this, args.m_point), args.m_down ? args.m_button : gui::EMouseKey::None, true);
			Invalidate();
		}
		virtual void OnMouseClick(gui::MouseEventArgs& args) override
		{
			base::OnMouseClick(args);
			if (args.m_handled || !cp<Params<>>().m_default_mouse_navigation) return;

			m_main->NavRevert(); // If a mouse single click is detected, revert any navigation
			Invalidate();
		}
		virtual void OnMouseMove(gui::MouseEventArgs& args) override
		{
			base::OnMouseMove(args);
			if (args.m_handled || !cp<Params<>>().m_default_mouse_navigation) return;

			if (m_nav_enabled)
			{
				auto pt = NormalisePoint(*this, args.m_point);
				m_main->Nav(pt, args.m_key_state, false);
				Invalidate();
			}
		}
		virtual void OnMouseWheel(gui::MouseWheelArgs& args) override
		{
			base::OnMouseWheel(args);
			if (args.m_handled || !cp<Params<>>().m_default_mouse_navigation) return;

			auto pt = NormalisePoint(*this, args.m_point);
			m_main->NavZ(pt, args.m_delta, true);
			Invalidate();
		}

		// Resizing handlers
		virtual void OnWindowPosChange(gui::WindowPosEventArgs const& args) override
		{
			base::OnWindowPosChange(args);
			if (!args.m_before && args.IsResize() && !IsIconic(*this))
			{
				auto rect = ClientRect(false);
				auto dpi = GetDpiForWindow(*this);
				auto w = s_cast<int>(rect.width() * dpi / 96.0);
				auto h = s_cast<int>(rect.height() * dpi / 96.0);
				m_main->Resize({ w, h });
				m_main->RenderNeeded();
			}
		}

		// Handle system menu keys
		virtual void OnSysKeyDown(uint32_t vk_key, UINT repeats, UINT flags)
		{
			(void)repeats,flags;

			// Watch for full screen alt-enter transitions
			if (m_fullscreen_toggle_enabled && vk_key == VK_RETURN)
			{
				// If currently in full screen mode, switch to windowed or visa versa
				//OnFullScreenToggle(!m_main->m_window.FullScreenMode());

				m_main->DoRender(true);
			}
		}
	};
}
