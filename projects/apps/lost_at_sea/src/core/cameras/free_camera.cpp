//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/core/cameras/free_camera.h"

namespace las::camera
{
	FreeCamera::FreeCamera(Camera& cam, InputHandler& input)
		: ICamera(cam, input)
		, m_speed(400.0f)
		, m_speed_min(0.5f)
		, m_speed_max(2000.0f)
		, m_velocity(v4::Zero())
		, m_move_wish(v4::Zero())
		, m_accel(12.0f)
		, m_damping(8.0f)
	{
	}

	void FreeCamera::OnAction(input::Action action)
	{
		auto c2w = m_cam.CameraToWorld();

		switch (action.m_action)
		{
			// Movement actions: accumulate a wish direction (applied in Update)
			case input::EAction::FreeCamera_MoveForward:
			{
				m_move_wish -= c2w.z; // Camera -Z = forward
				break;
			}
			case input::EAction::FreeCamera_MoveBack:
			{
				m_move_wish += c2w.z; // Camera +Z = backward
				break;
			}
			case input::EAction::FreeCamera_MoveLeft:
			{
				// Strafe left: camera -X projected to world XY plane
				m_move_wish += Normalise(v4{-c2w.x.x, -c2w.x.y, 0, 0});
				break;
			}
			case input::EAction::FreeCamera_MoveRight:
			{
				// Strafe right: camera +X projected to world XY plane
				m_move_wish += Normalise(v4{c2w.x.x, c2w.x.y, 0, 0});
				break;
			}
			case input::EAction::FreeCamera_MoveDown:
			{
				m_move_wish -= v4::ZAxis();
				break;
			}
			case input::EAction::FreeCamera_MoveUp:
			{
				m_move_wish += v4::ZAxis();
				break;
			}

			// Speed changes: immediate
			case input::EAction::FreeCamera_SpeedUp:
			{
				m_speed = std::min(m_speed * 1.2f, m_speed_max);
				break;
			}
			case input::EAction::FreeCamera_SlowDown:
			{
				m_speed = std::max(m_speed / 1.2f, m_speed_min);
				break;
			}

			// Rotation: immediate (inertia on rotation feels bad)
			case input::EAction::FreeCamera_Yaw:
			{
				auto rot = m3x4::Rotation(v4{0, 0, 1, 0}, action.m_axis);
				auto pos = c2w.pos;
				c2w = m4x4(rot * c2w.rot, pos);
				m_cam.CameraToWorld(c2w);
				break;
			}
			case input::EAction::FreeCamera_Pitch:
			{
				auto right = c2w.x;
				auto rot = m3x4::Rotation(right, action.m_axis);
				auto pos = c2w.pos;
				c2w = m4x4(rot * c2w.rot, pos);
				m_cam.CameraToWorld(c2w);
				break;
			}
			default:
				break;
		}
	}

	void FreeCamera::Update(float dt)
	{
		// Compute target velocity from the accumulated wish direction
		auto wish_len = Length(m_move_wish);
		auto target_vel = wish_len > 0.001f
			? (m_move_wish / wish_len) * m_speed
			: v4::Zero();

		// Exponential interpolation: accelerate when keys held, damp when released
		auto rate = wish_len > 0.001f ? m_accel : m_damping;
		auto blend = 1.0f - std::exp(-rate * dt);
		m_velocity = m_velocity + (target_vel - m_velocity) * blend;

		// Apply velocity to camera position
		if (LengthSq(m_velocity) > 0.0001f)
		{
			auto c2w = m_cam.CameraToWorld();
			c2w.pos += m_velocity * dt;
			m_cam.CameraToWorld(c2w);
		}

		// Reset wish for next frame
		m_move_wish = v4::Zero();
	}
}
