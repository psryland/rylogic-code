//******************************************
// Inertia "flight" camera controller
//  Copyright (c) Rylogic Ltd 2025
//******************************************
// Six-degree-of-freedom inertia camera controller. Supports keyboard + mouse
// and gamepad / flight-stick inputs simultaneously - inputs from each source
// are summed.
//
// All rotation is in camera-local axes:
//   +X = right  (pitch)
//   +Y = up     (yaw)
//   -Z = forward (roll about forward)
//
// Unlike WASDCtrller this controller does not lock to a world-up vector;
// banking and inverted flight are permitted.
//
// Usage:
//   pr::camera::FlightCtrl ctrl(camera, &kb, &mouse, &joystick);
//   while (running) { window.Pump(); ctrl.Step(dt); render(); }
#pragma once
#include <cmath>
#include "pr/math/math.h"
#include "pr/camera/camera.h"
#include "pr/input/keyboard.h"
#include "pr/input/mouse.h"
#include "pr/input/joystick.h"

namespace pr::camera
{
	struct FlightCtrl
	{
		Camera*          m_cam;       // The camera being controlled
		input::Keyboard* m_kb;        // Optional - keyboard input source
		input::Mouse*    m_mouse;     // Optional - mouse input source
		input::Joystick* m_js;        // Optional - joystick / gamepad input source

		// Tuning (defaults chosen for general-purpose flight feel)
		float m_max_lvel;             // Maximum linear velocity (units / s)
		float m_max_avel;             // Maximum angular velocity (rad / s)
		float m_accel_time;           // Time to reach max linear velocity (s)
		float m_aaccel_time;          // Time to reach max angular velocity (s)
		float m_drag_time;            // Time to decay velocity to zero (s)
		float m_mouse_sensitivity;    // Radians per mouse-pixel delta
		float m_zoom_speed;           // Scale of wheel-driven forward acceleration
		float m_stick_deadzone;       // Joystick stick deadzone (0..1)
		bool  m_mouse_look_requires_rmb; // If true, mouse only rotates while RMB held

		// State
		v4 m_lin_vel;                 // Linear velocity in camera-local space
		v4 m_ang_vel;                 // Angular velocity in camera-local axes (pitch/yaw/roll)

		FlightCtrl(Camera& cam, input::Keyboard* kb, input::Mouse* mouse, input::Joystick* js)
			:m_cam(&cam)
			,m_kb(kb)
			,m_mouse(mouse)
			,m_js(js)
			,m_max_lvel(10.0f)
			,m_max_avel(float(constants<float>::tau_by_2))
			,m_accel_time(0.5f)
			,m_aaccel_time(0.25f)
			,m_drag_time(0.5f)
			,m_mouse_sensitivity(0.003f)
			,m_zoom_speed(1.0f)
			,m_stick_deadzone(0.15f)
			,m_mouse_look_requires_rmb(true)
			,m_lin_vel(v4::Zero())
			,m_ang_vel(v4::Zero())
		{}

		// Advance the controller by 'dt' seconds. Reads input, integrates with
		// exponential drag, and applies the result to the camera.
		void Step(float dt)
		{
			float const lacc = m_max_lvel / m_accel_time;
			float const aacc = m_max_avel / m_aaccel_time;

			// Accumulate accelerations from each available input source.
			v4 lin_acc = v4::Zero();
			v4 ang_acc = v4::Zero();
			ReadKeyboard(lin_acc, ang_acc, lacc, aacc);
			ReadMouse(lin_acc, ang_acc, lacc, aacc);
			ReadJoystick(lin_acc, ang_acc, lacc, aacc);

			// Integrate velocity. Exponential drag decays towards zero only on the
			// axes that have no current input, so a held key sustains motion.
			ApplyAxis(m_lin_vel.x, lin_acc.x, dt, m_drag_time);
			ApplyAxis(m_lin_vel.y, lin_acc.y, dt, m_drag_time);
			ApplyAxis(m_lin_vel.z, lin_acc.z, dt, m_drag_time);
			ApplyAxis(m_ang_vel.x, ang_acc.x, dt, m_drag_time);
			ApplyAxis(m_ang_vel.y, ang_acc.y, dt, m_drag_time);
			ApplyAxis(m_ang_vel.z, ang_acc.z, dt, m_drag_time);

			// Clamp to per-class maximums.
			ClampLength(m_lin_vel, m_max_lvel);
			ClampLength(m_ang_vel, m_max_avel);

			// Compose the new camera-to-world transform.
			auto c2w = m_cam->CameraToWorld();

			// Translation: m_lin_vel is expressed in camera-local axes.
			auto local_step = (m_lin_vel + 0.5f * lin_acc * dt) * dt;
			c2w.pos += c2w.rot * v4(local_step.x, local_step.y, local_step.z, 0.0f);

			// Rotation: m_ang_vel components are pitch / yaw / roll about local x / y / z.
			auto av_dt = m_ang_vel * dt;
			auto inc = m3x3::RotationRad(av_dt.x, av_dt.y, av_dt.z);
			c2w.rot = c2w.rot * inc;

			// Re-orthonormalise to prevent numerical drift accumulating in the basis.
			c2w = Orthonorm(c2w);

			if (IsFinite(c2w))
				m_cam->CameraToWorld(c2w);
		}

	private:

		// WASD + QE for translation, ZC for roll.
		void ReadKeyboard(v4& lin_acc, v4& ang_acc, float lacc, float aacc)
		{
			if (m_kb == nullptr) return;

			if (m_kb->KeyDown('W')) lin_acc.z -= lacc;  // forward (camera looks down -Z)
			if (m_kb->KeyDown('S')) lin_acc.z += lacc;
			if (m_kb->KeyDown('A')) lin_acc.x -= lacc;
			if (m_kb->KeyDown('D')) lin_acc.x += lacc;
			if (m_kb->KeyDown('E')) lin_acc.y += lacc;  // ascend
			if (m_kb->KeyDown('Q')) lin_acc.y -= lacc;  // descend

			// Roll about the forward axis.
			if (m_kb->KeyDown('Z')) ang_acc.z += aacc;
			if (m_kb->KeyDown('C')) ang_acc.z -= aacc;

			// Keyboard arrow keys for explicit pitch / yaw.
			if (m_kb->KeyDown(VK_LEFT))  ang_acc.y += aacc;
			if (m_kb->KeyDown(VK_RIGHT)) ang_acc.y -= aacc;
			if (m_kb->KeyDown(VK_UP))    ang_acc.x += aacc;
			if (m_kb->KeyDown(VK_DOWN))  ang_acc.x -= aacc;
		}

		// Mouse delta drives pitch / yaw, wheel drives forward thrust.
		// Always Snapshots the mouse so accumulated deltas don't carry into the next frame.
		void ReadMouse(v4& lin_acc, v4& /*ang_acc*/, float lacc, float /*aacc*/)
		{
			if (m_mouse == nullptr) return;

			bool rmb_held = m_mouse->btn(input::Mouse::Right);
			if (!m_mouse_look_requires_rmb || rmb_held)
			{
				// dx -> yaw (rotate around local Y), dy -> pitch (rotate around local X).
				// Mouse delta is integrated as an immediate angular velocity rather than
				// an acceleration so it tracks the cursor without lag.
				float yaw_rate   = -m_mouse->dx() * m_mouse_sensitivity;
				float pitch_rate = -m_mouse->dy() * m_mouse_sensitivity;
				m_ang_vel.y += yaw_rate;
				m_ang_vel.x += pitch_rate;
			}

			// Wheel thrust: dz is in WHEEL_DELTA units (120 per detent).
			lin_acc.z -= (m_mouse->dz() / 120.0f) * m_zoom_speed * lacc;

			// Clear accumulated deltas so the next Step sees a fresh sample.
			m_mouse->Snapshot();
		}

		// Joystick / gamepad. Both Xbox-style sticks and generic raw HID controllers are supported.
		void ReadJoystick(v4& lin_acc, v4& ang_acc, float lacc, float aacc)
		{
			if (m_js == nullptr) return;

			m_js->Sample();
			if (!m_js->valid()) return;

			float dz = m_stick_deadzone;
			if (m_js->is_gamepad())
			{
				// Left stick: strafe (lx) and forward / back (ly). WGI ly is +up so invert for forward.
				lin_acc.x += float(input::Joystick::deadzone(m_js->lx(), dz)) * lacc;
				lin_acc.z -= float(input::Joystick::deadzone(m_js->ly(), dz)) * lacc;

				// Right stick: yaw (rx) and pitch (ry).
				ang_acc.y -= float(input::Joystick::deadzone(m_js->rx(), dz)) * aacc;
				ang_acc.x += float(input::Joystick::deadzone(m_js->ry(), dz)) * aacc;

				// Triggers: vertical translate (rt = up, lt = down).
				lin_acc.y += float(m_js->rt() - m_js->lt()) * lacc;

				// Shoulders: roll.
				if (m_js->gp_btn(input::EGpBtn::LeftShoulder))  ang_acc.z += aacc;
				if (m_js->gp_btn(input::EGpBtn::RightShoulder)) ang_acc.z -= aacc;
			}
			else
			{
				// Generic HID stick (e.g. T.16000M). WGI raw axes are 0..1 - recentre to -1..+1.
				// Common convention: axis 0 = X (roll), 1 = Y (pitch), 2 = twist (yaw / rudder), 3 = throttle (forward).
				auto raw_axis = [&](size_t i) -> float
				{
					return i < m_js->axis_count() ? float(input::Joystick::deadzone(m_js->axis(i) * 2.0 - 1.0, dz)) : 0.0f;
				};

				ang_acc.z -= raw_axis(0) * aacc;  // X -> roll
				ang_acc.x -= raw_axis(1) * aacc;  // Y -> pitch (stick forward = nose down)
				ang_acc.y -= raw_axis(2) * aacc;  // twist -> yaw
				lin_acc.z -= raw_axis(3) * lacc;  // throttle -> forward
			}
		}

		// Integrate one velocity component with exponential drag.
		// If 'acc' is non-zero, accelerate; otherwise decay towards zero.
		static void ApplyAxis(float& vel, float acc, float dt, float drag_time)
		{
			vel += acc * dt;
			if (acc == 0.0f && drag_time > 0.0f)
				vel *= std::exp(-dt / drag_time);
		}

		static void ClampLength(v4& v, float max_len)
		{
			float lsq = LengthSq(v);
			if (lsq < constants<float>::tiny) { v = v4::Zero(); return; }
			if (lsq > max_len * max_len)
				v *= max_len / std::sqrt(lsq);
		}
	};
}
