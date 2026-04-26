//*********************************************
// View 3d - Flight Camera Controller
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "view3d-12/src/dll/v3d_flight_camera.h"
#include "view3d-12/src/dll/v3d_window.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12
{
	FlightCameraController::FlightCameraController(V3dWindow& owner)
		:m_owner(&owner)
		,m_input_wnd()
		,m_kb()
		,m_mouse()
		,m_js()
		,m_ctrl()
		,m_t_last()
		,m_enabled(false)
		,m_poll_registered(false)
	{}

	FlightCameraController::~FlightCameraController()
	{
		// Ensure the poll callback is removed and Raw Input is unregistered while
		// the renderer / our hidden HWND are still valid.
		Enable(false);
	}

	void FlightCameraController::Enable(bool on)
	{
		if (on == m_enabled)
			return;

		if (on)
		{
			// Create the message-only HWND first so Keyboard / Mouse can register
			// Raw Input against it.
			m_input_wnd = std::make_unique<InputWindow>();

			// Register Raw Input devices. RIDEV_INPUTSINK ensures we receive events
			// even when the hidden window is not foreground - which it never is.
			m_kb = std::make_unique<input::Keyboard>(m_input_wnd->Hwnd(), /*background*/ true);
			m_mouse = std::make_unique<input::Mouse>(m_input_wnd->Hwnd(), /*background*/ true);
			m_input_wnd->m_kb = m_kb.get();
			m_input_wnd->m_mouse = m_mouse.get();

			// Pick up the first connected joystick / gamepad. Hot-plug while flight
			// mode is on is intentionally out of scope for v1 - toggle off / on to
			// re-enumerate.
			auto sticks = input::Joystick::Enumerate();
			if (!sticks.empty())
				m_js.emplace(std::move(sticks.front()));

			auto cfg = camera::FlightCtrl::Config{};
			auto bbox = m_owner->SceneBounds(view3d::ESceneBounds::All, 0, nullptr);
			if (bbox.valid())
			{
				auto diametre = static_cast<float>(bbox.Diametre());
				if (std::isfinite(diametre) && diametre > 0.0f)
					cfg.m_max_lvel = 0.25f * diametre;
			}
			if (cfg.m_max_lvel < 1.0f)
				cfg.m_max_lvel = 1.0f;
			cfg.m_mouse_look_requires_rmb = false;

			// Scale the starting speed from the scene size so WASD movement feels
			// reasonable for small models and large scenes. The wheel then adjusts
			// this scale interactively without using wheel input as forward thrust.
			// SceneCamera : Camera, so the upcast is implicit.
			m_ctrl = std::make_unique<camera::FlightCtrl>(
				m_owner->m_scene.m_cam,
				m_kb.get(),
				m_mouse.get(),
				m_js.has_value() ? &*m_js : nullptr,
				cfg);

			// Hook the renderer's poll loop. This fires as fast as the windows
			// message queue will allow - same mechanism the animation system uses.
			m_owner->m_wnd.m_rdr->AddPollCB({ this, &FlightCameraController::PollCB }, std::chrono::seconds::zero());
			m_poll_registered = true;

			m_t_last = std::chrono::steady_clock::now();
			m_enabled = true;
		}
		else
		{
			m_enabled = false;

			if (m_poll_registered)
			{
				m_owner->m_wnd.m_rdr->RemovePollCB({ this, &FlightCameraController::PollCB });
				m_poll_registered = false;
			}

			// Tear down in reverse order. The Keyboard / Mouse destructors call
			// RegisterRawInputDevices(REMOVE) so do that before destroying the HWND.
			m_ctrl.reset();
			m_js.reset();
			if (m_input_wnd)
			{
				m_input_wnd->m_kb = nullptr;
				m_input_wnd->m_mouse = nullptr;
			}
			m_mouse.reset();
			m_kb.reset();
			m_input_wnd.reset();
		}
	}

	void FlightCameraController::PollCB(void* ctx)
	{
		static_cast<FlightCameraController*>(ctx)->Tick();
	}

	void FlightCameraController::Tick()
	{
		if (!m_enabled || m_ctrl == nullptr)
			return;

		// Drain WM_INPUT messages targeted at our hidden window.
		// Filtered to (m_input_wnd, WM_INPUT, WM_INPUT) to avoid stealing
		// unrelated messages from other windows in this thread.
		auto hwnd = m_input_wnd->Hwnd();
		for (MSG msg; ::PeekMessageW(&msg, hwnd, WM_INPUT, WM_INPUT, PM_REMOVE);)
		{
			::TranslateMessage(&msg);
			::DispatchMessageW(&msg);
		}

		// Compute dt since the last tick. Clamp to a sane upper bound so a
		// long stall (e.g. debugger break) doesn't catapult the camera.
		auto now = std::chrono::steady_clock::now();
		auto dt = std::chrono::duration<float>(now - m_t_last).count();
		m_t_last = now;
		if (dt > 0.1f) dt = 0.1f;
		if (dt <= 0.0f) return;

		// Snapshot detector edges before stepping (FlightCtrl uses KeyDown only,
		// but Snapshot also clears mouse-button edge state for next frame).
		// Note: FlightCtrl::ReadMouse already calls m_mouse->Snapshot() internally
		// so that the accumulated dx/dy/dz are consumed exactly once per Step.
		// Step the controller and request a redraw. We always invalidate while flight
		// mode is enabled - the controller's integrator may produce sub-pixel motion
		// even between explicit input events (e.g. inertial decay), and the cost of an
		// extra invalidate is negligible compared to running input checks.
		m_ctrl->Step(dt);

		// Snapshot the keyboard for next frame edge detection.
		m_kb->Snapshot();

		m_owner->Invalidate();
	}
}
