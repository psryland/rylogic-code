//******************************************
// Raw Input Mouse
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Lightweight, header-only mouse input using Win32 Raw Input (WM_INPUT).
//
// Usage:
//   - Construct with the HWND that will receive WM_INPUT messages.
//   - The owning window must forward WM_INPUT to OnRawInput(lparam).
//   - Read accumulated dx/dy/dz and button state at any time.
//   - Call Snapshot() once per frame to clear accumulated deltas and enable
//     btn_pressed / btn_released edge detection.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <Windows.h>

namespace pr::input
{
	// Polled raw-input mouse state.
	class Mouse
	{
	public:

		enum EBtn { Left = 0, Right = 1, Middle = 2, X1 = 3, X2 = 4, NumberOf = 5 };

	private:

		// Accumulated relative motion since last Snapshot().
		long m_dx, m_dy, m_dz;
		uint8_t m_btn[NumberOf];
		uint8_t m_prev[NumberOf];
		HWND m_hwnd;

	public:

		Mouse() = delete;
		Mouse(Mouse const&) = delete;
		Mouse& operator=(Mouse const&) = delete;

		// Construct and register for raw mouse input on 'hwnd'.
		// 'background' = true requests events even when the window is not focused.
		explicit Mouse(HWND hwnd, bool background = true)
			:m_dx(), m_dy(), m_dz()
			,m_btn()
			,m_prev()
			,m_hwnd(hwnd)
		{
			RAWINPUTDEVICE rid = {};
			rid.usUsagePage = 0x01; // Generic Desktop Controls
			rid.usUsage     = 0x02; // Mouse
			rid.dwFlags     = static_cast<DWORD>(background ? RIDEV_INPUTSINK : 0);
			rid.hwndTarget  = hwnd;
			if (!::RegisterRawInputDevices(&rid, 1, sizeof(rid)))
				throw std::runtime_error("RegisterRawInputDevices(mouse) failed");
		}

		~Mouse()
		{
			RAWINPUTDEVICE rid = {};
			rid.usUsagePage = 0x01;
			rid.usUsage     = 0x02;
			rid.dwFlags     = RIDEV_REMOVE;
			rid.hwndTarget  = nullptr;
			::RegisterRawInputDevices(&rid, 1, sizeof(rid));
		}

		// Accumulated relative motion since the last Snapshot().
		long dx() const { return m_dx; }
		long dy() const { return m_dy; }
		long dz() const { return m_dz; } // Wheel detents (multiples of WHEEL_DELTA, 120).

		// Absolute cursor position in screen coordinates (pixels). Queried fresh from
		// the OS each call - Raw Input cannot deliver a reliable absolute position so
		// we never try to accumulate one. Returns {0,0} if the call fails.
		POINT screen_pos() const
		{
			POINT pt = {};
			::GetCursorPos(&pt);
			return pt;
		}

		// Absolute cursor position in client-area coordinates of the window the device
		// is registered against. Pixels, +y down, origin at top-left of the client area.
		POINT client_pos() const
		{
			POINT pt = screen_pos();
			if (m_hwnd != nullptr)
				::ScreenToClient(m_hwnd, &pt);
			return pt;
		}

		// Current button state.
		bool btn(int i) const
		{
			return i >= 0 && i < NumberOf && m_btn[i] != 0;
		}

		// True if the button transitioned from up to down between the last two Snapshot() calls.
		bool btn_pressed(int i) const
		{
			return i >= 0 && i < NumberOf && m_btn[i] != 0 && m_prev[i] == 0;
		}
		bool btn_released(int i) const
		{
			return i >= 0 && i < NumberOf && m_btn[i] == 0 && m_prev[i] != 0;
		}

		// Save current button state as previous, and clear accumulated deltas.
		// Call once per frame before reading dx/dy/dz for that frame's input.
		void Snapshot()
		{
			for (int i = 0; i != NumberOf; ++i) m_prev[i] = m_btn[i];
			m_dx = m_dy = m_dz = 0;
		}

		// Forward a WM_INPUT message to update internal state.
		// Returns true if it was a raw-mouse message.
		bool OnRawInput(LPARAM lparam)
		{
			UINT size = 0;
			::GetRawInputData(reinterpret_cast<HRAWINPUT>(lparam), RID_INPUT, nullptr, &size, sizeof(RAWINPUTHEADER));
			if (size == 0 || size > sizeof(RAWINPUT) * 4)
				return false;

			alignas(8) uint8_t buf[sizeof(RAWINPUT) * 4];
			if (::GetRawInputData(reinterpret_cast<HRAWINPUT>(lparam), RID_INPUT, buf, &size, sizeof(RAWINPUTHEADER)) != size)
				return false;

			auto const& ri = *reinterpret_cast<RAWINPUT const*>(buf);
			if (ri.header.dwType != RIM_TYPEMOUSE)
				return false;

			auto const& mo = ri.data.mouse;

			// Relative motion accumulates. Absolute coordinates (e.g. RDP, tablets) are converted to deltas
			// against the last absolute sample so callers see consistent relative motion.
			if ((mo.usFlags & MOUSE_MOVE_ABSOLUTE) == 0)
			{
				m_dx += mo.lLastX;
				m_dy += mo.lLastY;
			}
			else
			{
				// For absolute mode, treat the provided point as a delta from the last absolute point.
				static thread_local long last_abs_x = 0, last_abs_y = 0;
				static thread_local bool have_last = false;
				if (have_last)
				{
					m_dx += mo.lLastX - last_abs_x;
					m_dy += mo.lLastY - last_abs_y;
				}
				last_abs_x = mo.lLastX;
				last_abs_y = mo.lLastY;
				have_last = true;
			}

			auto bf = mo.usButtonFlags;
			if (bf & RI_MOUSE_LEFT_BUTTON_DOWN)   m_btn[Left]   = 1;
			if (bf & RI_MOUSE_LEFT_BUTTON_UP)     m_btn[Left]   = 0;
			if (bf & RI_MOUSE_RIGHT_BUTTON_DOWN)  m_btn[Right]  = 1;
			if (bf & RI_MOUSE_RIGHT_BUTTON_UP)    m_btn[Right]  = 0;
			if (bf & RI_MOUSE_MIDDLE_BUTTON_DOWN) m_btn[Middle] = 1;
			if (bf & RI_MOUSE_MIDDLE_BUTTON_UP)   m_btn[Middle] = 0;
			if (bf & RI_MOUSE_BUTTON_4_DOWN)      m_btn[X1]     = 1;
			if (bf & RI_MOUSE_BUTTON_4_UP)        m_btn[X1]     = 0;
			if (bf & RI_MOUSE_BUTTON_5_DOWN)      m_btn[X2]     = 1;
			if (bf & RI_MOUSE_BUTTON_5_UP)        m_btn[X2]     = 0;

			if (bf & RI_MOUSE_WHEEL)
				m_dz += static_cast<short>(mo.usButtonData);

			return true;
		}

		HWND Hwnd() const { return m_hwnd; }
	};
}
