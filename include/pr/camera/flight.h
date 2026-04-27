//******************************************
// Inertia "flight" camera controller
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Six-degree-of-freedom inertia camera controller. Supports keyboard + mouse
// and gamepad / flight-stick inputs simultaneously - inputs from each source
// are summed.
//
// Most rotation is in camera-local axes:
//   +X = right  (pitch)
//   +Y = up     (yaw)
//   -Z = forward (roll about forward)
// Mouse look and gamepad right-stick yaw use world-up for a helicopter/free-fly feel.
//
// Unlike WASDCtrller this controller does not lock to a world-up vector;
// banking and inverted flight are permitted.
//
// Usage:
//   pr::camera::FlightCtrl ctrl(camera, &kb, &mouse, &joystick);
//   while (running) { window.Pump(); ctrl.Step(dt); render(); }
#pragma once
#include <cmath>
#include <stdexcept>
#include "pr/math/math.h"
#include "pr/camera/camera.h"
#include "pr/input/keyboard.h"
#include "pr/input/mouse.h"
#include "pr/input/joystick.h"

namespace pr::camera
{
	struct FlightCtrl
	{
		struct KeyBindings
		{
			int m_forward       = 'W';       // Translate forward
			int m_back          = 'S';       // Translate backward
			int m_left          = 'A';       // Translate left
			int m_right         = 'D';       // Translate right
			int m_down          = 'Q';       // Translate down
			int m_up            = 'E';       // Translate up
			int m_roll_modifier = VK_LSHIFT; // Modifier for roll controls
			int m_roll_left     = 'Q';       // Roll left while 'm_roll_modifier' is held
			int m_roll_right    = 'E';       // Roll right while 'm_roll_modifier' is held
			int m_yaw_left      = VK_LEFT;   // Yaw left
			int m_yaw_right     = VK_RIGHT;  // Yaw right
			int m_pitch_up      = VK_UP;     // Pitch up
			int m_pitch_down    = VK_DOWN;   // Pitch down
		};

		enum class EGamepadAxis
		{
			None,
			LeftStickX,
			LeftStickY,
			RightStickX,
			RightStickY,
			LeftTrigger,
			RightTrigger,
		};

		struct GamepadAxisBinding
		{
			EGamepadAxis m_axis = EGamepadAxis::None;
			float m_scale = 0.0f;
		};

		struct GamepadBindings
		{
			GamepadAxisBinding m_translate_x = { EGamepadAxis::LeftStickX, +1.0f };
			GamepadAxisBinding m_translate_y = { EGamepadAxis::RightTrigger, +1.0f };
			GamepadAxisBinding m_translate_y_alt = { EGamepadAxis::LeftTrigger, -1.0f };
			GamepadAxisBinding m_translate_z = { EGamepadAxis::LeftStickY, -1.0f };
			GamepadAxisBinding m_pitch = { EGamepadAxis::RightStickY, +1.0f };
			GamepadAxisBinding m_yaw = { EGamepadAxis::RightStickX, -1.0f };
			GamepadAxisBinding m_roll = { EGamepadAxis::RightStickX, -1.0f };
			input::EGpBtn m_roll_modifier = input::EGpBtn::RightThumbstick;
			input::EGpBtn m_speed_down = input::EGpBtn::LeftShoulder;
			input::EGpBtn m_speed_up = input::EGpBtn::RightShoulder;
		};

		struct JoystickAxisBinding
		{
			size_t m_axis = 0;
			float m_scale = 0.0f;
		};

		struct JoystickBindings
		{
			JoystickAxisBinding m_roll = { 0, -1.0f };
			JoystickAxisBinding m_pitch = { 1, -1.0f };
			JoystickAxisBinding m_yaw = { 2, -1.0f };
			JoystickAxisBinding m_translate_z = { 3, -1.0f };
		};

		struct Config
		{
			float m_max_lvel = 10.0f;                            // Maximum linear velocity (units / s)
			float m_max_avel = float(constants<float>::tau_by_2); // Maximum angular velocity (rad / s)
			float m_roll_rate_scale = 0.5f;                       // Roll rate relative to pitch/yaw
			float m_accel_time = 0.5f;                           // Time to reach max linear velocity (s)
			float m_aaccel_time = 0.25f;                         // Time to reach max angular velocity (s)
			float m_drag_time = 0.5f;                            // Time to decay velocity to zero (s)
			float m_mouse_sensitivity = 0.003f;                  // Radians per mouse-pixel delta
			float m_gamepad_look_scale = 1.0f / 3.0f;            // Right-stick angular sensitivity relative to keyboard max
			float m_initial_speed_scale = 1.0f;                  // Initial movement-speed scale
			float m_min_speed_scale = 0.01f;                     // Lower bound for wheel-adjusted movement-speed scale
			float m_max_speed_scale = 100.0f;                    // Upper bound for wheel-adjusted movement-speed scale
			float m_wheel_speed_step = 1.25f;                    // Multiplicative speed-scale change per wheel detent
			float m_button_speed_steps_per_second = 4.0f;        // Movement-speed scale button rate, in wheel detents per second
			float m_stick_deadzone = 0.15f;                      // Joystick stick deadzone (0..1)
			bool  m_mouse_look_requires_rmb = true;              // If true, mouse only rotates while RMB held
			bool  m_gamepad_invert_y = true;                     // If true, invert gamepad right-stick pitch
		};

		Camera*          m_cam;       // The camera being controlled
		input::Keyboard* m_kb;        // Optional - keyboard input source
		input::Mouse*    m_mouse;     // Optional - mouse input source
		input::Joystick* m_js;        // Optional - joystick / gamepad input source
		KeyBindings      m_key;       // Keyboard bindings
		GamepadBindings  m_gamepad;   // Xbox-style gamepad bindings
		JoystickBindings m_joystick;  // Generic HID joystick bindings
		Config           m_cfg;       // Tuning parameters

		// State
		float m_speed_scale;          // Current movement-speed scale, adjusted by the mouse wheel
		v4 m_lin_vel;                 // Linear velocity in camera-local space
		v4 m_ang_vel;                 // Angular velocity in camera-local axes (pitch/yaw/roll)
		v4 m_mouse_ang_vel;           // Mouse angular velocity (pitch/world-yaw)
		v4 m_gamepad_ang_vel;         // Gamepad right-stick angular velocity (pitch/world-yaw/roll)

		FlightCtrl(
			Camera& cam,
			input::Keyboard* kb,
			input::Mouse* mouse,
			input::Joystick* js,
			Config cfg = {},
			KeyBindings key_bindings = {},
			GamepadBindings gamepad_bindings = {},
			JoystickBindings joystick_bindings = {})
			:m_cam(&cam)
			,m_kb(kb)
			,m_mouse(mouse)
			,m_js(js)
			,m_key(key_bindings)
			,m_gamepad(gamepad_bindings)
			,m_joystick(joystick_bindings)
			,m_cfg(cfg)
			,m_speed_scale(cfg.m_initial_speed_scale)
			,m_lin_vel(v4::Zero())
			,m_ang_vel(v4::Zero())
			,m_mouse_ang_vel(v4::Zero())
			,m_gamepad_ang_vel(v4::Zero())
		{}

		// Advance the controller by 'dt' seconds. Reads input, integrates with
		// exponential drag, and applies the result to the camera.
		void Step(float dt)
		{
			auto const max_lvel = m_cfg.m_max_lvel * m_speed_scale;
			float const lacc = max_lvel / m_cfg.m_accel_time;
			float const aacc = m_cfg.m_max_avel / m_cfg.m_aaccel_time;

			// Accumulate accelerations from each available input source.
			v4 lin_acc = v4::Zero();
			v4 ang_acc = v4::Zero();
			v4 gamepad_ang_acc = v4::Zero();
			ReadKeyboard(lin_acc, ang_acc, lacc, aacc);
			ReadMouse();
			ReadJoystick(lin_acc, ang_acc, gamepad_ang_acc, lacc, aacc, dt);

			// Integrate velocity. Exponential drag decays towards zero only on the
			// axes that have no current input, so a held key sustains motion.
			ApplyAxis(m_lin_vel.x, lin_acc.x, dt, m_cfg.m_drag_time);
			ApplyAxis(m_lin_vel.y, lin_acc.y, dt, m_cfg.m_drag_time);
			ApplyAxis(m_lin_vel.z, lin_acc.z, dt, m_cfg.m_drag_time);
			ApplyAxis(m_ang_vel.x, ang_acc.x, dt, m_cfg.m_drag_time);
			ApplyAxis(m_ang_vel.y, ang_acc.y, dt, m_cfg.m_drag_time);
			ApplyAxis(m_ang_vel.z, ang_acc.z, dt, m_cfg.m_drag_time);
			ApplyAxis(m_mouse_ang_vel.x, 0.0f, dt, m_cfg.m_drag_time);
			ApplyAxis(m_mouse_ang_vel.y, 0.0f, dt, m_cfg.m_drag_time);
			ApplyAxis(m_mouse_ang_vel.z, 0.0f, dt, m_cfg.m_drag_time);
			ApplyAxis(m_gamepad_ang_vel.x, gamepad_ang_acc.x, dt, m_cfg.m_drag_time);
			ApplyAxis(m_gamepad_ang_vel.y, gamepad_ang_acc.y, dt, m_cfg.m_drag_time);
			ApplyAxis(m_gamepad_ang_vel.z, gamepad_ang_acc.z, dt, m_cfg.m_drag_time);

			// Clamp to per-class maximums.
			ClampLength(m_lin_vel, max_lvel);
			ClampAbs(m_ang_vel.z, m_cfg.m_max_avel * m_cfg.m_roll_rate_scale);
			ClampLength(m_ang_vel, m_cfg.m_max_avel);
			ClampLength(m_mouse_ang_vel, m_cfg.m_max_avel);
			auto const gamepad_max_avel = m_cfg.m_max_avel * m_cfg.m_gamepad_look_scale;
			ClampAbs(m_gamepad_ang_vel.z, gamepad_max_avel * m_cfg.m_roll_rate_scale);
			ClampLength(m_gamepad_ang_vel, gamepad_max_avel);

			// Compose the new camera-to-world transform.
			auto c2w = m_cam->CameraToWorld();

			// Translation: m_lin_vel is expressed in camera-local axes.
			auto local_step = (m_lin_vel + 0.5f * lin_acc * dt) * dt;
			c2w.pos += c2w.rot * v4(local_step.x, local_step.y, local_step.z, 0.0f);

			// Rotation: m_ang_vel components are pitch / yaw / roll about local x / y / z.
			auto av_dt = m_ang_vel * dt;
			auto inc = m3x3::RotationRad(av_dt.x, av_dt.y, av_dt.z);
			c2w.rot = c2w.rot * inc;
			ApplyWorldLook(c2w, (m_mouse_ang_vel + m_gamepad_ang_vel) * dt);

			// Re-orthonormalise to prevent numerical drift accumulating in the basis.
			c2w = Orthonorm(c2w);

			if (IsFinite(c2w))
				m_cam->CameraToWorld(c2w);
		}

	private:

		// WASD for planar translation, QE for vertical translation, LShift+QE for roll.
		void ReadKeyboard(v4& lin_acc, v4& ang_acc, float lacc, float aacc)
		{
			if (m_kb == nullptr) return;

			if (m_kb->KeyDown(m_key.m_forward)) lin_acc.z -= lacc;  // forward (camera looks down -Z)
			if (m_kb->KeyDown(m_key.m_back))    lin_acc.z += lacc;
			if (m_kb->KeyDown(m_key.m_left))    lin_acc.x -= lacc;
			if (m_kb->KeyDown(m_key.m_right))   lin_acc.x += lacc;

			if (m_kb->KeyDown(m_key.m_roll_modifier))
			{
				// Roll about the forward axis.
				auto const roll_aacc = aacc * m_cfg.m_roll_rate_scale;
				if (m_kb->KeyDown(m_key.m_roll_left))  ang_acc.z += roll_aacc;
				if (m_kb->KeyDown(m_key.m_roll_right)) ang_acc.z -= roll_aacc;
			}
			else
			{
				if (m_kb->KeyDown(m_key.m_down)) lin_acc.y -= lacc; // descend
				if (m_kb->KeyDown(m_key.m_up))   lin_acc.y += lacc; // ascend
			}

			// Keyboard arrow keys for explicit pitch / yaw.
			if (m_kb->KeyDown(m_key.m_yaw_left))    ang_acc.y += aacc;
			if (m_kb->KeyDown(m_key.m_yaw_right))   ang_acc.y -= aacc;
			if (m_kb->KeyDown(m_key.m_pitch_up))    ang_acc.x += aacc;
			if (m_kb->KeyDown(m_key.m_pitch_down))  ang_acc.x -= aacc;
		}

		// Mouse delta drives world-up look, wheel adjusts movement speed.
		// Always Snapshots the mouse so accumulated deltas don't carry into the next frame.
		void ReadMouse()
		{
			if (m_mouse == nullptr) return;

			bool rmb_held = m_mouse->btn(input::Mouse::Right);
			if (!m_cfg.m_mouse_look_requires_rmb || rmb_held)
			{
				// dx -> yaw (rotate around world up), dy -> pitch (rotate around horizon/right).
				// Mouse delta is integrated as an immediate angular velocity rather than
				// an acceleration so it tracks the cursor without lag.
				float yaw_rate   = -m_mouse->dx() * m_cfg.m_mouse_sensitivity;
				float pitch_rate = -m_mouse->dy() * m_cfg.m_mouse_sensitivity;
				m_mouse_ang_vel.y += yaw_rate;
				m_mouse_ang_vel.x += pitch_rate;
			}

			// Wheel scale: dz is in WHEEL_DELTA units (120 per detent).
			auto const wheel_steps = m_mouse->dz() / 120.0f;
			AdjustSpeedScale(wheel_steps);

			// Clear accumulated deltas so the next Step sees a fresh sample.
			m_mouse->Snapshot();
		}

		// Joystick / gamepad. Both Xbox-style sticks and generic raw HID controllers are supported.
		void ReadJoystick(v4& lin_acc, v4& ang_acc, v4& gamepad_ang_acc, float lacc, float aacc, float dt)
		{
			if (m_js == nullptr) return;

			m_js->Sample();
			if (!m_js->valid()) return;

			float dz = m_cfg.m_stick_deadzone;
			if (m_js->is_gamepad())
			{
				// Left stick and triggers provide camera-relative translation.
				lin_acc.x += ReadGamepadAxis(m_gamepad.m_translate_x, dz) * lacc;
				lin_acc.y += (ReadGamepadAxis(m_gamepad.m_translate_y, dz) + ReadGamepadAxis(m_gamepad.m_translate_y_alt, dz)) * lacc;
				lin_acc.z += ReadGamepadAxis(m_gamepad.m_translate_z, dz) * lacc;

				// Right stick: yaw/pitch normally, roll/pitch while the right stick is clicked.
				auto const gamepad_aacc = aacc * m_cfg.m_gamepad_look_scale;
				if (m_js->gp_btn(m_gamepad.m_roll_modifier))
					gamepad_ang_acc.z += ReadGamepadAxis(m_gamepad.m_roll, dz) * gamepad_aacc * m_cfg.m_roll_rate_scale;
				else
					gamepad_ang_acc.y += ReadGamepadAxis(m_gamepad.m_yaw, dz) * gamepad_aacc;
				gamepad_ang_acc.x += ReadGamepadAxis(m_gamepad.m_pitch, dz) * gamepad_aacc * (m_cfg.m_gamepad_invert_y ? -1.0f : +1.0f);

				// Shoulders: movement speed scale, matching mouse-wheel scaling.
				auto speed_steps = 0.0f;
				if (m_js->gp_btn(m_gamepad.m_speed_down)) speed_steps -= m_cfg.m_button_speed_steps_per_second * dt;
				if (m_js->gp_btn(m_gamepad.m_speed_up))   speed_steps += m_cfg.m_button_speed_steps_per_second * dt;
				AdjustSpeedScale(speed_steps);
			}
			else
			{
				// Generic HID stick (e.g. T.16000M). GameInput raw axes are 0..1 - recentre to -1..+1.
				// Common convention: axis 0 = X (roll), 1 = Y (pitch), 2 = twist (yaw / rudder), 3 = throttle (forward).
				auto const roll_aacc = aacc * m_cfg.m_roll_rate_scale;
				ang_acc.z += ReadJoystickAxis(m_joystick.m_roll, dz) * roll_aacc;  // X -> roll
				ang_acc.x += ReadJoystickAxis(m_joystick.m_pitch, dz) * aacc;  // Y -> pitch (stick forward = nose down)
				ang_acc.y += ReadJoystickAxis(m_joystick.m_yaw, dz) * aacc;  // twist -> yaw
				lin_acc.z += ReadJoystickAxis(m_joystick.m_translate_z, dz) * lacc;  // throttle -> forward
			}
		}

		float ReadGamepadAxis(GamepadAxisBinding const& binding, float dz) const
		{
			switch (binding.m_axis)
			{
				case EGamepadAxis::None:
				{
					return 0.0f;
				}
				case EGamepadAxis::LeftStickX:
				{
					return float(input::Joystick::deadzone(m_js->lx(), dz)) * binding.m_scale;
				}
				case EGamepadAxis::LeftStickY:
				{
					return float(input::Joystick::deadzone(m_js->ly(), dz)) * binding.m_scale;
				}
				case EGamepadAxis::RightStickX:
				{
					return float(input::Joystick::deadzone(m_js->rx(), dz)) * binding.m_scale;
				}
				case EGamepadAxis::RightStickY:
				{
					return float(input::Joystick::deadzone(m_js->ry(), dz)) * binding.m_scale;
				}
				case EGamepadAxis::LeftTrigger:
				{
					return float(m_js->lt()) * binding.m_scale;
				}
				case EGamepadAxis::RightTrigger:
				{
					return float(m_js->rt()) * binding.m_scale;
				}
				default:
				{
					throw std::runtime_error("Unknown gamepad axis binding");
				}
			}
		}
		float ReadJoystickAxis(JoystickAxisBinding const& binding, float dz) const
		{
			if (binding.m_scale == 0.0f)
				return 0.0f;

			return binding.m_axis < m_js->axis_count()
				? float(input::Joystick::deadzone(m_js->axis(binding.m_axis) * 2.0 - 1.0, dz)) * binding.m_scale
				: 0.0f;
		}

		// Integrate one velocity component with exponential drag.
		// If 'acc' is non-zero, accelerate; otherwise decay towards zero.
		static void ApplyAxis(float& vel, float acc, float dt, float drag_time)
		{
			vel += acc * dt;
			if (acc == 0.0f && drag_time > 0.0f)
				vel *= std::exp(-dt / drag_time);
		}
		void AdjustSpeedScale(float steps)
		{
			if (steps == 0.0f)
				return;

			m_speed_scale *= std::pow(m_cfg.m_wheel_speed_step, steps);
			if (m_speed_scale < m_cfg.m_min_speed_scale)
				m_speed_scale = m_cfg.m_min_speed_scale;
			if (m_speed_scale > m_cfg.m_max_speed_scale)
				m_speed_scale = m_cfg.m_max_speed_scale;
		}
		static void ApplyWorldLook(m4x4& c2w, v4 av_dt)
		{
			auto const world_up = v3::YAxis();

			if (av_dt.y != 0.0f)
				c2w.rot = m3x3::Rotation(world_up, av_dt.y) * c2w.rot;

			if (av_dt.x != 0.0f)
			{
				auto forward = -(c2w.rot * v3::ZAxis());
				auto right = Cross(forward, world_up);
				if (LengthSq(right) < constants<float>::tiny)
					right = c2w.rot * v3::XAxis();
				else
					right = Normalise(right);

				c2w.rot = m3x3::Rotation(right, av_dt.x) * c2w.rot;
			}

			if (av_dt.z != 0.0f)
				c2w.rot = c2w.rot * m3x3::Rotation(v3::ZAxis(), av_dt.z);
		}

		static void ClampLength(v4& v, float max_len)
		{
			float lsq = LengthSq(v);
			if (lsq < constants<float>::tiny) { v = v4::Zero(); return; }
			if (lsq > max_len * max_len)
				v *= max_len / std::sqrt(lsq);
		}
		static void ClampAbs(float& value, float max_abs)
		{
			if (value < -max_abs)
			{
				value = -max_abs;
				return;
			}
			if (value > +max_abs)
			{
				value = +max_abs;
				return;
			}
		}
	};
}
