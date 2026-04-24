//******************************************
// Joystick / Gamepad input via Windows.Gaming.Input
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Lightweight, header-only joystick / gamepad / flight-stick / wheel input
// using Windows.Gaming.Input through C++/WinRT.
//
// C++/WinRT is pure standard C++ (header-only, no managed runtime, no GC).
// Only the Windows SDK is required; the WinRT activation factories live in
// already-loaded system DLLs.
//
// Usage:
//   auto sticks = pr::input::Joystick::Enumerate();
//   for (auto& js : sticks) { js.Sample(); use js.lx() etc. or js.axis(i) }
//
// Re-call Enumerate() any time to pick up newly attached devices; existing
// Joystick objects keep working until their underlying device is removed,
// after which Sample() returns false and the polled state freezes.
#pragma once
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unknwn.h>
#include <winrt/base.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Gaming.Input.h>
#pragma comment(lib, "windowsapp.lib")

namespace pr::input
{
	enum class EHat : int32_t
	{
		// Match Windows.Gaming.Input.GameControllerSwitchPosition.
		Center    = 0,
		Up        = 1,
		UpRight   = 2,
		Right     = 3,
		DownRight = 4,
		Down      = 5,
		DownLeft  = 6,
		Left      = 7,
		UpLeft    = 8,
	};

	// Standard Xbox-style gamepad button bits, mirroring WGI's GamepadButtons enum.
	// These are only meaningful for is_gamepad() == true devices.
	enum class EGpBtn : uint16_t
	{
		None           = 0,
		Menu           = 1 <<  0,
		View           = 1 <<  1,
		A              = 1 <<  2,
		B              = 1 <<  3,
		X              = 1 <<  4,
		Y              = 1 <<  5,
		DPadUp         = 1 <<  6,
		DPadDown       = 1 <<  7,
		DPadLeft       = 1 <<  8,
		DPadRight      = 1 <<  9,
		LeftShoulder   = 1 << 10,
		RightShoulder  = 1 << 11,
		LeftThumbstick = 1 << 12,
		RightThumbstick= 1 << 13,
		_flags_enum    = 0,
	};
	constexpr EGpBtn operator|(EGpBtn a, EGpBtn b) { return EGpBtn(uint16_t(a) | uint16_t(b)); }
	constexpr EGpBtn operator&(EGpBtn a, EGpBtn b) { return EGpBtn(uint16_t(a) & uint16_t(b)); }

	// Polled joystick / gamepad / raw HID controller.
	// Cheap to copy - holds only WinRT smart pointers and small vectors.
	class Joystick
	{
		// Underlying WGI device. m_raw is always set; m_gp is non-null only for Xbox-style
		// gamepads (also exposed via the higher-level Gamepad API).
		winrt::Windows::Gaming::Input::RawGameController m_raw{nullptr};
		winrt::Windows::Gaming::Input::Gamepad m_gp{nullptr};

		// Polled state. Buttons stored as uint8_t because std::vector<bool> can't be
		// passed to array_view<bool>.
		std::unique_ptr<bool[]> m_buttons;
		std::unique_ptr<winrt::Windows::Gaming::Input::GameControllerSwitchPosition[]> m_switches;
		std::vector<double> m_axes;
		std::unique_ptr<bool[]> m_buttons_prev;
		size_t m_button_count;
		size_t m_switch_count;
		size_t m_axis_count;

		// Most recent Gamepad reading - only populated when m_gp is non-null.
		winrt::Windows::Gaming::Input::GamepadReading m_gp_reading{};
		uint64_t m_timestamp{};
		bool m_valid{};

		// One-time WinRT init + WGI subscription.
		// Subscribing to RawGameControllerAdded forces WGI to start tracking devices
		// before the first Enumerate() call - without this the first Enumerate() may
		// return an empty list on some systems.
		static void EnsureInit()
		{
			static std::once_flag once;
			std::call_once(once, []
			{
				try { winrt::init_apartment(winrt::apartment_type::multi_threaded); }
				catch (...) { /* already initialised in this thread */ }

				using namespace winrt::Windows::Gaming::Input;
				static auto added_token   = RawGameController::RawGameControllerAdded([](auto&&, auto&&){});
				static auto removed_token = RawGameController::RawGameControllerRemoved([](auto&&, auto&&){});
				(void)added_token; (void)removed_token;
			});
		}

	public:

		// Construct from a WinRT RawGameController (used by Enumerate).
		explicit Joystick(winrt::Windows::Gaming::Input::RawGameController const& raw)
			:m_raw(raw)
			,m_button_count(static_cast<size_t>(raw.ButtonCount()))
			,m_switch_count(static_cast<size_t>(raw.SwitchCount()))
			,m_axis_count  (static_cast<size_t>(raw.AxisCount()))
		{
			m_buttons      = std::make_unique<bool[]>(m_button_count);
			m_buttons_prev = std::make_unique<bool[]>(m_button_count);
			m_switches     = std::make_unique<winrt::Windows::Gaming::Input::GameControllerSwitchPosition[]>(m_switch_count);
			m_axes.resize(m_axis_count);

			// Try to also expose this device as an Xbox-style Gamepad.
			m_gp = winrt::Windows::Gaming::Input::Gamepad::FromGameController(raw);
		}

		// Enumerate all currently-attached controllers.
		// Call any time to refresh - existing Joystick objects are unaffected.
		static std::vector<Joystick> Enumerate()
		{
			EnsureInit();
			std::vector<Joystick> out;
			auto controllers = winrt::Windows::Gaming::Input::RawGameController::RawGameControllers();
			out.reserve(controllers.Size());
			for (auto const& raw : controllers)
				out.emplace_back(raw);
			return out;
		}

		// Sample current state. Returns false if the underlying device has gone away
		// or the read failed - in that case the previously-polled state is preserved.
		bool Sample()
		{
			try
			{
				// Save previous button state for edge-detection accessors.
				for (size_t i = 0; i != m_button_count; ++i)
					m_buttons_prev[i] = m_buttons[i];

				m_timestamp = m_raw.GetCurrentReading(
					winrt::array_view<bool>(m_buttons.get(), m_buttons.get() + m_button_count),
					winrt::array_view<winrt::Windows::Gaming::Input::GameControllerSwitchPosition>(m_switches.get(), m_switches.get() + m_switch_count),
					winrt::array_view<double>(m_axes.data(), m_axes.data() + m_axis_count));

				if (m_gp != nullptr)
					m_gp_reading = m_gp.GetCurrentReading();

				m_valid = true;
				return true;
			}
			catch (...)
			{
				m_valid = false;
				return false;
			}
		}

		// True if the most recent Sample() succeeded.
		bool valid() const { return m_valid; }

		// Display name reported by the device (e.g. "Xbox Wireless Controller", "T.16000M").
		std::wstring name() const
		{
			try { return std::wstring(m_raw.DisplayName().c_str()); }
			catch (...) { return L""; }
		}

		// True if this device is reported as a standard Xbox-style gamepad and the lx/ly/... accessors are meaningful.
		bool is_gamepad() const { return m_gp != nullptr; }

		// Generic accessors (work for every device type).
		size_t axis_count()   const { return m_axis_count; }
		size_t button_count() const { return m_button_count; }
		size_t switch_count() const { return m_switch_count; }

		// Axis value in [0..1] (per WGI). For sticks, 0.5 is centred.
		// Returns 0.5 for out-of-range indices so callers can treat unknown axes as centred.
		double axis(size_t i) const { return i < m_axis_count ? m_axes[i] : 0.5; }

		// Button currently held.
		bool btn(size_t i) const { return i < m_button_count && m_buttons[i]; }

		// Edge detection - true on the Sample() that observed the transition.
		bool btn_pressed(size_t i)  const { return i < m_button_count &&  m_buttons[i] && !m_buttons_prev[i]; }
		bool btn_released(size_t i) const { return i < m_button_count && !m_buttons[i] &&  m_buttons_prev[i]; }

		// Hat / D-pad position.
		EHat hat(size_t i) const
		{
			return i < m_switch_count ? static_cast<EHat>(static_cast<int32_t>(m_switches[i])) : EHat::Center;
		}

		// Gamepad-only convenience accessors.
		// Sticks return centred (-1..+1, 0 = centre); triggers return (0..+1).
		// Returns 0 for non-gamepad devices.
		double lx() const { return m_gp != nullptr ? m_gp_reading.LeftThumbstickX  : 0.0; }
		double ly() const { return m_gp != nullptr ? m_gp_reading.LeftThumbstickY  : 0.0; }
		double rx() const { return m_gp != nullptr ? m_gp_reading.RightThumbstickX : 0.0; }
		double ry() const { return m_gp != nullptr ? m_gp_reading.RightThumbstickY : 0.0; }
		double lt() const { return m_gp != nullptr ? m_gp_reading.LeftTrigger      : 0.0; }
		double rt() const { return m_gp != nullptr ? m_gp_reading.RightTrigger     : 0.0; }

		// Gamepad button mask (Xbox-style). Returns EGpBtn::None for non-gamepads.
		EGpBtn buttons() const
		{
			if (m_gp == nullptr)
				return EGpBtn::None;
			return static_cast<EGpBtn>(static_cast<uint16_t>(m_gp_reading.Buttons));
		}
		bool gp_btn(EGpBtn b) const
		{
			return (uint16_t(buttons()) & uint16_t(b)) != 0;
		}

		// Apply a symmetric deadzone to a centred axis value in [-1..+1].
		static double deadzone(double v, double dz)
		{
			if (v >  dz) return (v - dz) / (1.0 - dz);
			if (v < -dz) return (v + dz) / (1.0 - dz);
			return 0.0;
		}
	};
}
