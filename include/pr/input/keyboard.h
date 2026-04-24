//******************************************
// Raw Input Keyboard
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Lightweight, header-only keyboard input using Win32 Raw Input (WM_INPUT).
//
// Usage:
//   - Construct with the HWND that will receive WM_INPUT messages.
//   - The constructor calls RegisterRawInputDevices with RIDEV_INPUTSINK so
//     events are delivered even when the window is not focused.
//   - The owning window must forward WM_INPUT to OnRawInput(lparam).
//   - Call KeyDown(vk) at any time to query the current state.
//   - Call Snapshot() once per frame to enable KeyPressed/KeyReleased edge detection.
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <Windows.h>

namespace pr::input
{
	// Polled raw-input keyboard state.
	class Keyboard
	{
		// Each slot is non-zero when the key is currently down. Indexed by VK_*.
		uint8_t m_keys[256];
		uint8_t m_prev[256];
		HWND    m_hwnd;

	public:

		Keyboard() = delete;
		Keyboard(Keyboard const&) = delete;
		Keyboard& operator=(Keyboard const&) = delete;

		// Construct and register for raw keyboard input on 'hwnd'.
		// 'background' = true requests events even when the window is not focused.
		explicit Keyboard(HWND hwnd, bool background = true)
			:m_keys()
			,m_prev()
			,m_hwnd(hwnd)
		{
			RAWINPUTDEVICE rid = {};
			rid.usUsagePage = 0x01; // Generic Desktop Controls
			rid.usUsage     = 0x06; // Keyboard
			rid.dwFlags     = static_cast<DWORD>(background ? RIDEV_INPUTSINK : 0);
			rid.hwndTarget  = hwnd;
			if (!::RegisterRawInputDevices(&rid, 1, sizeof(rid)))
				throw std::runtime_error("RegisterRawInputDevices(keyboard) failed");
		}

		~Keyboard()
		{
			// Unregister by passing RIDEV_REMOVE with hwndTarget = nullptr.
			RAWINPUTDEVICE rid = {};
			rid.usUsagePage = 0x01;
			rid.usUsage     = 0x06;
			rid.dwFlags     = RIDEV_REMOVE;
			rid.hwndTarget  = nullptr;
			::RegisterRawInputDevices(&rid, 1, sizeof(rid));
		}

		// True if the given VK is currently held down.
		bool KeyDown(int vk) const
		{
			return vk >= 0 && vk < 256 && m_keys[vk] != 0;
		}

		// True if the key transitioned from up to down between the last two Snapshot() calls.
		// Call Snapshot() once per frame, then KeyPressed/KeyReleased reflect the change since the previous frame.
		bool KeyPressed(int vk) const
		{
			return vk >= 0 && vk < 256 && m_keys[vk] != 0 && m_prev[vk] == 0;
		}
		bool KeyReleased(int vk) const
		{
			return vk >= 0 && vk < 256 && m_keys[vk] == 0 && m_prev[vk] != 0;
		}

		// Save current state as the previous-frame snapshot for edge detection.
		void Snapshot()
		{
			std::memcpy(m_prev, m_keys, sizeof(m_keys));
		}

		// Forward a WM_INPUT message to update internal state.
		// Call from your WndProc when message == WM_INPUT, passing lparam.
		// Returns true if the message was handled (raw keyboard event), false otherwise.
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
			if (ri.header.dwType != RIM_TYPEKEYBOARD)
				return false;

			auto const& kb = ri.data.keyboard;

			// Translate the raw key event to a VK code, accounting for E0/E1 prefixes.
			int vk = kb.VKey;
			if (vk == 0 || vk >= 256)
				return true;

			// Normalise extended keys (Right-Shift, etc.). Most apps don't care about L/R distinction beyond Shift.
			if (vk == VK_SHIFT)
				vk = ::MapVirtualKeyW(kb.MakeCode, MAPVK_VSC_TO_VK_EX);
			else if (vk == VK_CONTROL)
				vk = (kb.Flags & RI_KEY_E0) ? VK_RCONTROL : VK_LCONTROL;
			else if (vk == VK_MENU)
				vk = (kb.Flags & RI_KEY_E0) ? VK_RMENU : VK_LMENU;

			if (vk <= 0 || vk >= 256)
				return true;

			bool key_up = (kb.Flags & RI_KEY_BREAK) != 0;
			m_keys[vk] = key_up ? 0u : 1u;
			return true;
		}

		// The window the device is registered against.
		HWND Hwnd() const { return m_hwnd; }
	};
}
