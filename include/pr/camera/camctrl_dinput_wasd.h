//******************************************
// Camera Ctrl WASD
//  Copyright (c) Rylogic Ltd 2009
//******************************************
#pragma once
#include "pr/maths/maths.h"
#include "pr/camera/camera.h"
#include "pr/input/dinput.h"

namespace pr::camera
{
	// A "WASD" camera controller
	struct WASDCtrller
	{
		pr::Camera* m_cam;      // The camera being controlled
		dinput::Keyboard m_kb;       // DInput keyboard device
		dinput::Mouse    m_mouse;    // DInput mouse device
		pr::IRect            m_area;     // The screen resolution in pixels
		pr::v4               m_heading;  // The direction of 'forward'
		pr::v4               m_velocity; // The forward velocity

		// Tuning values
		float m_max_lvel;     // The maximum linear velocity (m/s)
		float m_accel_time;   // The time it takes to get to maximum velocity (s)
		float m_drag_time;    // The time it takes to go from max velocity to zero (s)
		float m_turn_speed;   // The rate of keyboard turning
		float m_pan_speed;    // The rate of camera panning
		float m_zoom_speed;   // The rate of zoom in/out using the wheel

		WASDCtrller(pr::Camera& cam, HINSTANCE app_inst, HWND hwnd, pr::IRect const& area)
			:m_cam(&cam)
			, m_kb(dinput::DeviceSettings(app_inst, hwnd, dinput::EDeviceClass::Keyboard))
			, m_mouse(dinput::DeviceSettings(app_inst, hwnd, dinput::EDeviceClass::Mouse))
			, m_area(area)
			, m_heading(pr::v4ZAxis)
			, m_velocity(pr::v4Zero)
			, m_max_lvel(10.0f)
			, m_accel_time(0.5f)
			, m_drag_time(1.0f)
			, m_turn_speed(1.0f)
			, m_pan_speed(0.5f)
			, m_zoom_speed(0.5f)
		{
			m_cam->Align(pr::v4YAxis);
		}

		void Step(float dt)
		{
			// Sample the dinput devices
			// Not using pr::Failed here as it spams the output window
			if (m_kb.Sample() != DI_OK || m_mouse.Sample() != DI_OK)
				return;

			//OutputDebugString(pr::FmtS("mousex: %d  mousey: %d  mousez:  %d\n" ,m_mouse.dx() ,m_mouse.dy() ,m_mouse.dz()));

			// Right mouse controls heading, left controls view direction
			bool lbtn = m_mouse.btn(dinput::Mouse::Left);
			bool rbtn = m_mouse.btn(dinput::Mouse::Right);
			if (lbtn || rbtn)
			{
				pr::v2 mv = pr::v2(
						5.0f * m_pan_speed * m_mouse.dx() / float(m_area.SizeX()),
					-5.0f * m_pan_speed * m_mouse.dy() / float(m_area.SizeY()));
				m_cam->MouseControl(pr::v2Zero, ENavOp::Rotate, true);
				m_cam->MouseControl(mv, ENavOp::Rotate, false);
				m_cam->MouseControl(mv, ENavOp::None, true);
			}

			// Mouse wheel controls zoom
			m_cam->Translate(0.0f, 0.0f, 0.01f * m_zoom_speed * m_mouse.dz(), true);

			// Camera focus point acceleration
			pr::v4 lin_acc = pr::v4Zero; float rot = 0.0f;
			float lacc = m_max_lvel / m_accel_time; // The acceleration to use (m/s/s)
			float ldec = m_max_lvel / m_drag_time;  // The deceleration to use (m/s/s)
			float turn = float(maths::tau_by_8 * m_turn_speed);
			if (m_kb.KeyDown(DIK_Q)) lin_acc.x -= lacc; // strafe left
			if (m_kb.KeyDown(DIK_W)) lin_acc.z -= lacc; // forward
			if (m_kb.KeyDown(DIK_E)) lin_acc.x += lacc; // strafe right
			if (m_kb.KeyDown(DIK_A)) rot -= turn;       // yaw left
			if (m_kb.KeyDown(DIK_S)) lin_acc.z += lacc; // backward
			if (m_kb.KeyDown(DIK_D)) rot += turn;       // yaw right

			// Decelerate if not accelerating
			if (FEql(lin_acc, pr::v4Zero))
			{
				float vel = Length(m_velocity);
				if (vel < ldec * dt) lin_acc = -m_velocity;
				else                 lin_acc = -m_velocity * (ldec * dt / vel);
			}

			// Integrate acceleration
			m_velocity += lin_acc * dt;

			// Limit velocities
			float lvel_sq = LengthSq(m_velocity);
			if (lvel_sq < maths::tinyf) m_velocity = pr::v4Zero;
			else if (lvel_sq > m_max_lvel)      m_velocity *= m_max_lvel / pr::Sqrt(lvel_sq);

			// Integrate velocity
			pr::v4 pos = m_cam->FocusPoint();
			pos += (m_velocity + (0.5f * dt) * lin_acc) * dt;
			m_cam->FocusPoint(pos);

			// Rotate heading
			if (rot != 0.0f)
				m_heading = m3x4::Rotation(v4YAxis, rot) * m_heading;

			//// Integrate the rate of change of orientation implied by m_avel
			//// Note: rate of change of orientation is not the same as angular velocity
			//// because integrating angular velocity does not give you orientation.
			//// Given an orientation quaternion, q, the rate of change of orietation
			//// is:
			////     dq/dt = 0.5 * avel * q.
			//// Since we have avel in camera space, we can find the rate of change of 
			//// orientation in camera space as:
			////     dw2c/dt = 0.5 * avel_cs * conj(c2w)
			////     where c2w is the camera orientation in world space
			//// => integrating:
			////  c2w += conj(dw2c/dt)
			//// Use a 2nd order integrating algorithm (mid point)
			//pr::Quat dw2c_dt, mid_w2c, avel = pr::cast_q(m_avel), w2c = pr::GetConjugate(pr::Quat::make(c2w));
			//float half_dt = dt * 0.5f;
			//dw2c_dt = 0.5f * avel * w2c;
			//mid_w2c = w2c + dw2c_dt * half_dt;
			//dw2c_dt = 0.5f * avel * mid_w2c;
			//w2c     = w2c + dw2c_dt * dt;
			//
			//// Create the new camera to world transform
			//c2w = pr::m4x4::make(pr::GetConjugate(w2c), pos);
			//m_cam->CameraToWorld(pr::Orthonormalise(c2w), true);

			//OutputDebugString(pr::FmtS("pos: %f %f %f  vel: %f %f %f\n", pos.x, pos.y, pos.z, m_lvel.x, m_lvel.y, m_lvel.z));
			//OutputDebugString(pr::FmtS("mx: %d  my: %d  mz: %d\n", m_mouse.dx() ,m_mouse.dy(), m_mouse.dz()));
		}
	};
}