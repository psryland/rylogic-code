//******************************************
// GameInput Joystick / Gamepad
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Lightweight, header-only joystick / gamepad / flight-stick / wheel input
// using the modern GameInput API (Windows SDK 10.0.22621+, runs on Win10 1809+).
//
// Why GameInput (and not WGI / XInput / DirectInput):
//   - Single API for every controller class - Xbox pads, generic HID sticks, HOTAS,
//     racing wheels, arcade sticks - presented uniformly.
//   - Pure C polling API: no COM apartment dance, no message-pump dependency,
//     no foreground-focus quirks. GameInputCreate() is all the setup needed.
//   - Microsoft's recommended replacement for both Windows.Gaming.Input (WGI)
//     and XInput. WGI is being deprecated; XInput only handles Xbox-class.
//   - Cooperates with virtual gamepads / GIP-bus devices that confuse XInput.
//
// Dependencies: only the Windows SDK (GameInput.h + gameinput.lib). The runtime
// (GameInput.dll) ships as a system component on Windows 10 1809+.
//
// Usage:
//   auto sticks = pr::input::Joystick::Enumerate();
//   for (auto& js : sticks) { js.Sample(); use js.lx() etc. or js.axis(i) }
//
// Re-call Enumerate() any time to pick up newly attached devices. Existing
// Joystick objects keep working as long as their device stays connected;
// after disconnect, Sample() returns false and the polled state freezes.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <Windows.h>
#include <GameInput.h>
#pragma comment(lib, "gameinput.lib")

namespace pr::input
{
	enum class EHat : int32_t
	{
		// Match GameInputSwitchPosition.
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

	// Standard Xbox-style gamepad button bits, mirroring GameInputGamepadButtons.
	// Only meaningful for is_gamepad() == true devices.
	enum class EGpBtn : uint16_t
	{
		None            = 0,
		Menu            = 1 <<  0,
		View            = 1 <<  1,
		A               = 1 <<  2,
		B               = 1 <<  3,
		X               = 1 <<  4,
		Y               = 1 <<  5,
		DPadUp          = 1 <<  6,
		DPadDown        = 1 <<  7,
		DPadLeft        = 1 <<  8,
		DPadRight       = 1 <<  9,
		LeftShoulder    = 1 << 10,
		RightShoulder   = 1 << 11,
		LeftThumbstick  = 1 << 12,
		RightThumbstick = 1 << 13,
		_flags_enum     = 0,
	};
	constexpr EGpBtn operator|(EGpBtn a, EGpBtn b) { return EGpBtn(uint16_t(a) | uint16_t(b)); }
	constexpr EGpBtn operator&(EGpBtn a, EGpBtn b) { return EGpBtn(uint16_t(a) & uint16_t(b)); }

	// Polled joystick / gamepad / generic HID controller backed by GameInput.
	class Joystick
	{
		// Reference to the device. The Joystick owns one ref; releases it on destruction.
		IGameInputDevice* m_device{};

		// Polled state. m_buttons stored as uint8_t so it can be passed to GameInput's
		// bool* API without std::vector<bool>'s proxy issues.
		std::unique_ptr<bool[]> m_buttons;
		std::unique_ptr<bool[]> m_buttons_prev;
		std::unique_ptr<GameInputSwitchPosition[]> m_switches;
		std::vector<float> m_axes;
		size_t m_button_count{};
		size_t m_switch_count{};
		size_t m_axis_count{};

		// Most recent gamepad reading. Only populated if the device exposes a Gamepad view.
		GameInputGamepadState m_gp_state{};
		bool m_is_gamepad{};
		uint64_t m_timestamp{};
		bool m_valid{};

		// Cached display name and whether we've fetched it yet (UTF-8 from GameInputString).
		mutable std::string m_name_cache;
		mutable bool m_name_cached{};

		// Process-wide IGameInput pointer. Created lazily on first use; never released
		// (lifetime tied to process). Returned as a raw pointer for cheap reuse.
		static IGameInput* Instance()
		{
			static std::once_flag once;
			static IGameInput* s_inst = nullptr;
			std::call_once(once, []
			{
				IGameInput* p = nullptr;
				if (SUCCEEDED(::GameInputCreate(&p)))
					s_inst = p;
			});
			return s_inst;
		}

	public:

		// Construct from a device pointer (used internally by Enumerate).
		// Takes ownership of the existing reference - call AddRef() first if sharing.
		explicit Joystick(IGameInputDevice* device)
			:m_device(device)
		{
			if (m_device == nullptr)
				return;

			auto const* info = m_device->GetDeviceInfo();
			if (info != nullptr)
			{
				m_axis_count   = info->controllerAxisCount;
				m_button_count = info->controllerButtonCount;
				m_switch_count = info->controllerSwitchCount;
				m_is_gamepad   = (info->supportedInput & GameInputKindGamepad) != 0;
			}

			m_buttons      = std::make_unique<bool[]>(m_button_count > 0 ? m_button_count : 1);
			m_buttons_prev = std::make_unique<bool[]>(m_button_count > 0 ? m_button_count : 1);
			m_switches     = std::make_unique<GameInputSwitchPosition[]>(m_switch_count > 0 ? m_switch_count : 1);
			m_axes.resize(m_axis_count);
		}

		~Joystick()
		{
			if (m_device != nullptr)
				m_device->Release();
		}

		Joystick(Joystick&& other) noexcept
			:m_device(other.m_device)
			,m_buttons(std::move(other.m_buttons))
			,m_buttons_prev(std::move(other.m_buttons_prev))
			,m_switches(std::move(other.m_switches))
			,m_axes(std::move(other.m_axes))
			,m_button_count(other.m_button_count)
			,m_switch_count(other.m_switch_count)
			,m_axis_count(other.m_axis_count)
			,m_gp_state(other.m_gp_state)
			,m_is_gamepad(other.m_is_gamepad)
			,m_timestamp(other.m_timestamp)
			,m_valid(other.m_valid)
			,m_name_cache(std::move(other.m_name_cache))
			,m_name_cached(other.m_name_cached)
		{
			other.m_device = nullptr;
		}
		Joystick(Joystick const&) = delete;
		Joystick& operator=(Joystick const&) = delete;
		Joystick& operator=(Joystick&&) = delete;

		// Enumerate all currently-connected controllers. Synchronous; returns immediately.
		// Re-call any time to pick up hot-plugged devices.
		static std::vector<Joystick> Enumerate()
		{
			std::vector<Joystick> out;

			auto* gi = Instance();
			if (gi == nullptr)
				return out;

			// We collect raw device pointers in a callback driven by blocking enumeration.
			// The callback fires synchronously for each currently-connected controller and
			// returns before RegisterDeviceCallback returns, so 'devices' is fully populated
			// by the time we wrap up.
			struct Ctx
			{
				std::vector<IGameInputDevice*> devices;
				std::mutex mtx;
			};
			Ctx ctx;

			GameInputCallbackToken token = 0;
			auto cb = +[](GameInputCallbackToken, void* context, IGameInputDevice* device, uint64_t, GameInputDeviceStatus current, GameInputDeviceStatus)
			{
				if ((current & GameInputDeviceConnected) == 0)
					return;

				auto& c = *reinterpret_cast<Ctx*>(context);
				device->AddRef();
				std::lock_guard<std::mutex> lock(c.mtx);
				c.devices.push_back(device);
			};

			if (FAILED(gi->RegisterDeviceCallback(
				nullptr,
				GameInputKindController,
				GameInputDeviceConnected,
				GameInputBlockingEnumeration,
				&ctx,
				cb,
				&token)))
			{
				return out;
			}

			// Stop the callback - no further notifications wanted from this enumeration.
			// Use a 0 timeout: callback was synchronous so unregistration is immediate.
			if (token != 0)
				gi->UnregisterCallback(token, 0);

			out.reserve(ctx.devices.size());
			for (auto* d : ctx.devices)
				out.emplace_back(d); // takes ownership of the AddRef'd ref

			return out;
		}

		// Sample current state. Returns false if the underlying device is disconnected
		// or no reading is currently available; previously-polled state is preserved.
		bool Sample()
		{
			// Save previous button state for edge-detection accessors.
			for (size_t i = 0; i != m_button_count; ++i)
				m_buttons_prev[i] = m_buttons[i];

			auto* gi = Instance();
			if (gi == nullptr || m_device == nullptr)
			{
				m_valid = false;
				return false;
			}

			IGameInputReading* reading = nullptr;
			if (FAILED(gi->GetCurrentReading(GameInputKindController, m_device, &reading)) || reading == nullptr)
			{
				m_valid = false;
				return false;
			}

			m_timestamp = reading->GetTimestamp();

			if (m_axis_count > 0)
				reading->GetControllerAxisState(static_cast<uint32_t>(m_axis_count), m_axes.data());

			if (m_button_count > 0)
				reading->GetControllerButtonState(static_cast<uint32_t>(m_button_count), m_buttons.get());

			if (m_switch_count > 0)
				reading->GetControllerSwitchState(static_cast<uint32_t>(m_switch_count), m_switches.get());

			if (m_is_gamepad)
			{
				if (!reading->GetGamepadState(&m_gp_state))
					m_gp_state = {};
			}

			reading->Release();
			m_valid = true;
			return true;
		}

		// True if the most recent Sample() succeeded.
		bool valid() const { return m_valid; }

		// Display name (UTF-8). Falls back to a synthesised "<family> VID:PID" string when
		// GameInput doesn't expose a displayName (common for Xbox controllers).
		std::string name() const
		{
			if (m_name_cached || m_device == nullptr)
				return m_name_cache;

			auto const* info = m_device->GetDeviceInfo();
			if (info != nullptr && info->displayName != nullptr && info->displayName->data != nullptr && info->displayName->sizeInBytes > 0)
			{
				m_name_cache.assign(info->displayName->data, info->displayName->sizeInBytes);
			}
			else if (info != nullptr)
			{
				char const* family = "Controller";
				switch (info->deviceFamily)
				{
					case GameInputFamilyXboxOne:   family = "Xbox One Controller"; break;
					case GameInputFamilyXbox360:   family = "Xbox 360 Controller"; break;
					case GameInputFamilyHid:       family = "HID Controller"; break;
					case GameInputFamilyI8042:     family = "PS/2 Controller"; break;
					case GameInputFamilyAggregate: family = "Aggregate Controller"; break;
					case GameInputFamilyVirtual:   family = "Virtual Controller"; break;
					default: break;
				}

				char buf[64];
				::sprintf_s(buf, "%s [%04X:%04X]", family, info->vendorId, info->productId);
				m_name_cache = buf;
			}

			m_name_cached = true;
			return m_name_cache;
		}

		// True if this device exposes a standard Xbox-style gamepad layout (A/B/X/Y/sticks/triggers).
		// When true, the lx/ly/... gamepad accessors are meaningful.
		bool is_gamepad() const { return m_is_gamepad; }

		// Generic accessors - work for every device type.
		size_t axis_count()   const { return m_axis_count; }
		size_t button_count() const { return m_button_count; }
		size_t switch_count() const { return m_switch_count; }

		// Axis value in [0..1]. For sticks, 0.5 is centred.
		// Returns 0.5 for out-of-range indices so callers can treat unknown axes as centred.
		double axis(size_t i) const { return i < m_axis_count ? double(m_axes[i]) : 0.5; }

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
		// Sticks return centred (-1..+1); triggers return (0..+1).
		// Returns 0 for non-gamepad devices.
		double lx() const { return m_is_gamepad ? double(m_gp_state.leftThumbstickX)  : 0.0; }
		double ly() const { return m_is_gamepad ? double(m_gp_state.leftThumbstickY)  : 0.0; }
		double rx() const { return m_is_gamepad ? double(m_gp_state.rightThumbstickX) : 0.0; }
		double ry() const { return m_is_gamepad ? double(m_gp_state.rightThumbstickY) : 0.0; }
		double lt() const { return m_is_gamepad ? double(m_gp_state.leftTrigger)      : 0.0; }
		double rt() const { return m_is_gamepad ? double(m_gp_state.rightTrigger)     : 0.0; }

		// Gamepad button mask (Xbox-style). Returns EGpBtn::None for non-gamepads.
		EGpBtn buttons() const
		{
			return m_is_gamepad ? static_cast<EGpBtn>(static_cast<uint16_t>(m_gp_state.buttons)) : EGpBtn::None;
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
