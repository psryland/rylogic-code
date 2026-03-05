//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/core/cameras/ship_camera.h"
#include "src/world/ship/ship.h"

namespace las::camera
{
	ShipCamera::ShipCamera(Camera& cam, InputHandler& input, Ship const& ship)
		: ICamera(cam, input)
		, m_ship(ship)
		, m_yaw(constants<float>::tau_by_4)  // Start looking from the side
		, m_pitch(0.3f)            // Slightly above horizontal
		, m_arm_length(15.0f)
		, m_arm_min(3.0f)
		, m_arm_max(100.0f)
		, m_current_pos(v4::Zero())
		, m_pos_velocity(v4::Zero())
		, m_stiffness(25.0f)
		, m_damping(10.0f)
		, m_look_offset_z(1.0f)
	{
		// Initialise the camera to the desired position immediately (no spring lag on startup)
		auto ship_pos = m_ship.m_body.O2W().pos;
		auto cos_p = std::cos(m_pitch);
		m_current_pos = v4{
			ship_pos.x + m_arm_length * cos_p * std::cos(m_yaw),
			ship_pos.y + m_arm_length * cos_p * std::sin(m_yaw),
			ship_pos.z + m_arm_length * std::sin(m_pitch),
			1};
	}

	void ShipCamera::OnAction(input::Action action)
	{
		switch (action.m_action)
		{
			case input::EAction::ShipCamera_Yaw:
			{
				m_yaw += action.m_axis;
				break;
			}
			case input::EAction::ShipCamera_Pitch:
			{
				// Clamp pitch to avoid flipping (just above horizontal to nearly overhead)
				m_pitch = std::clamp(m_pitch + action.m_axis, 0.05f, 1.4f);
				break;
			}
			case input::EAction::ShipCamera_ZoomIn:
			{
				m_arm_length = std::max(m_arm_length / 1.15f, m_arm_min);
				break;
			}
			case input::EAction::ShipCamera_ZoomOut:
			{
				m_arm_length = std::min(m_arm_length * 1.15f, m_arm_max);
				break;
			}
			default:
				break;
		}
	}

	void ShipCamera::Update(float dt)
	{
		auto ship_pos = m_ship.m_body.O2W().pos;
		auto look_target = ship_pos + v4{0, 0, m_look_offset_z, 0};

		// Compute desired camera position from orbit angles and arm length
		auto cos_p = std::cos(m_pitch);
		auto desired_pos = v4{
			ship_pos.x + m_arm_length * cos_p * std::cos(m_yaw),
			ship_pos.y + m_arm_length * cos_p * std::sin(m_yaw),
			ship_pos.z + m_arm_length * std::sin(m_pitch),
			1};

		// Critically-damped spring: F = -stiffness * (pos - desired) - damping * velocity
		auto displacement = m_current_pos - desired_pos;
		auto spring_force = -m_stiffness * displacement - m_damping * m_pos_velocity;
		m_pos_velocity += spring_force * dt;
		m_current_pos += m_pos_velocity * dt;
		m_current_pos.w = 1;

		// Build camera-to-world matrix directly (avoid LookAt which triggers navigation state)
		auto forward = Normalise(look_target - m_current_pos);
		auto cam_z = -forward; // Camera looks along -Z
		auto cam_x = Normalise(Cross(v4{0, 0, 1, 0}, cam_z));
		auto cam_y = Cross(cam_z, cam_x);
		m_cam.CameraToWorld(m4x4{cam_x, cam_y, cam_z, m_current_pos});
	}
}
