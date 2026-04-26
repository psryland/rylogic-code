//*********************************************
// View 3d - Flight Camera Controller
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Native flight-camera controller for a V3dWindow.
//
// When enabled:
//   - Owns a hidden message-only window that receives Raw Input (keyboard/mouse)
//     via RIDEV_INPUTSINK so input is captured even when WPF presents the scene
//     via D3DImage (no real HWND on the V3dWindow).
//   - Owns a pr::input::Joystick instance (first connected device) for
//     gamepad / HOTAS support, sampled each tick.
//   - Hooks Renderer::AddPollCB and ticks pr::camera::FlightCtrl::Step(dt) at
//     message-pump rate, invalidating the window when the camera has moved.
//
// Cursor visibility / clipping is intentionally NOT handled here - that lives
// in the C# host (which owns the visible WPF window and cursor). Raw Input
// mouse deltas are relative, so the camera math doesn't depend on cursor
// position.
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/win32/dummy_window.h"
#include "pr/input/keyboard.h"
#include "pr/input/mouse.h"
#include "pr/input/joystick.h"
#include "pr/camera/flight.h"

namespace pr::rdr12
{
	struct V3dWindow;

	struct FlightCameraController
	{
		// A message-only window dedicated to receiving WM_INPUT for this controller.
		// We can't use V3dWindow::m_hwnd because in WPF hosting it is null
		// (the scene is presented via D3DImage / off-screen rendering).
		struct InputWindow : pr::DummyWindow
		{
			input::Keyboard* m_kb;
			input::Mouse*    m_mouse;

			InputWindow()
				:DummyWindow()
				,m_kb()
				,m_mouse()
			{}

		protected:

			LRESULT WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) override
			{
				if (message == WM_INPUT)
				{
					if (m_kb != nullptr)    m_kb->OnRawInput(lparam);
					if (m_mouse != nullptr) m_mouse->OnRawInput(lparam);
				}
				return DummyWindow::WndProc(hwnd, message, wparam, lparam);
			}
		};

		V3dWindow*                       m_owner;       // Non-owning back-pointer
		std::unique_ptr<InputWindow>     m_input_wnd;   // Hidden HWND_MESSAGE window for Raw Input
		std::unique_ptr<input::Keyboard> m_kb;          // Raw Input keyboard
		std::unique_ptr<input::Mouse>    m_mouse;       // Raw Input mouse
		std::optional<input::Joystick>   m_js;          // First connected gamepad / stick (if any)
		std::unique_ptr<camera::FlightCtrl> m_ctrl;     // The flight camera integrator
		std::chrono::steady_clock::time_point m_t_last; // Time of the previous tick (for dt)
		bool                             m_enabled;    // True between Enable() and Disable()
		bool                             m_poll_registered; // Is the poll callback currently registered

		explicit FlightCameraController(V3dWindow& owner);
		~FlightCameraController();

		FlightCameraController(FlightCameraController&&) = delete;
		FlightCameraController(FlightCameraController const&) = delete;
		FlightCameraController& operator=(FlightCameraController&&) = delete;
		FlightCameraController& operator=(FlightCameraController const&) = delete;

		// Enable / disable the controller.
		// While enabled, Raw Input is captured globally for this process and
		// the camera is driven on a poll callback. Disable() releases everything.
		void Enable(bool on);
		bool IsEnabled() const { return m_enabled; }

	private:

		// Poll callback wired via Renderer::AddPollCB while enabled.
		// Pumps WM_INPUT messages targeted at our hidden window, samples the
		// joystick, steps the FlightCtrl, and invalidates if the camera moved.
		static void PollCB(void* ctx);
		void Tick();
	};
}
