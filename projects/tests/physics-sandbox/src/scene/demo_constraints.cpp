#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	namespace
	{
		constexpr auto Orange = Colour32(0xFFFFA040U);
		constexpr auto Blue = Colour32(0xFF40A0FFU);
		constexpr auto Green = Colour32(0xFF50D080U);
		constexpr auto Red = Colour32(0xFFFF6060U);
		constexpr auto Purple = Colour32(0xFFB070FFU);
		constexpr auto Yellow = Colour32(0xFFFFD050U);

	}

	// Build one compact gallery covering every public rigid D6 convenience descriptor and breakage.
	void BuildRigidJointGallery(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(14.0f);

		// A ball joint permits unrestricted rotation while keeping the top of the orange pendulum at a fixed point.
		auto const ball_anchor = v4{-8.0f, 0.0f, 7.0f, 1.0f};
		auto const ball_body = demo.AddRod(ball_anchor, ball_anchor + v4{1.0f, 0.0f, -4.0f, 0.0f}, 0.65f, 3.0f, Orange);
		demo.AddBall(demo.WorldFrame(m4x4::Translation(ball_anchor)), demo.BodyFrame(ball_body, m4x4::Translation(ball_anchor)));

		// A hinge exposes only rotation around world Y and begins with angular motion about that permitted axis.
		auto const hinge_anchor = v4{-4.0f, 0.0f, 7.0f, 1.0f};
		auto const hinge_body = demo.AddRod(hinge_anchor, hinge_anchor + v4{0.8f, 0.0f, -4.0f, 0.0f}, 0.65f, 3.0f, Blue);
		auto const hinge_frame = ConstraintAxisFrame(v4::YAxis(), hinge_anchor);
		demo.AddHinge(
			demo.WorldFrame(hinge_frame),
			demo.BodyFrame(hinge_body, hinge_frame),
			physics::ConstraintAxisDesc{.m_mode = physics::EConstraintAxisMode::Free});
		demo.SceneState().m_body[hinge_body].VelocityWS(1.5f * v4::YAxis(), v4::Zero());

		// A limited slider travels vertically because frame-local X is aligned with world Z.
		auto const slider_position = v4{0.0f, 0.0f, 5.0f, 1.0f};
		auto const slider_body = demo.AddBox(v4{1.3f, 1.3f, 1.3f, 0.0f}, m4x4::Translation(slider_position), 2.0f, Green);
		auto const slider_frame = ConstraintAxisFrame(v4::ZAxis(), slider_position);
		demo.AddSlider(
			demo.WorldFrame(slider_frame),
			demo.BodyFrame(slider_body, slider_frame),
			physics::ConstraintAxisDesc{
				.m_mode = physics::EConstraintAxisMode::Limited,
				.m_limits = Range<float>{-2.5f, +2.5f},
			});
		demo.SceneState().m_body[slider_body].VelocityWS(v4::Zero(), 2.0f * v4::ZAxis());

		// A weld preserves the complete initial relative transform.
		auto const weld_transform = m4x4::Transform(v4::XAxis(), 0.35f, v4{4.0f, 0.0f, 5.0f, 1.0f});
		auto const weld_body = demo.AddBox(v4{1.5f, 1.0f, 2.0f, 0.0f}, weld_transform, 2.5f, Purple);
		demo.AddWeld(demo.WorldFrame(weld_transform), demo.BodyFrame(weld_body, weld_transform));

		// A driven hinge demonstrates bounded spring-damper actuation without special motor integration.
		auto const motor_anchor = v4{7.5f, 0.0f, 5.5f, 1.0f};
		auto const motor_body = demo.AddRod(motor_anchor, motor_anchor + v4{0.0f, 0.0f, -3.5f, 0.0f}, 0.7f, 2.5f, Yellow);
		auto const motor_frame = ConstraintAxisFrame(v4::YAxis(), motor_anchor);
		demo.AddHinge(
			demo.WorldFrame(motor_frame),
			demo.BodyFrame(motor_body, motor_frame),
			physics::ConstraintAxisDesc{
				.m_mode = physics::EConstraintAxisMode::Driven,
				.m_target_position = 1.1f,
				.m_stiffness = 80.0f,
				.m_damping = 16.0f,
				.m_max_force = 250.0f,
			});

		// The red body starts with enough momentum to overload its deliberately weak weld and fall free.
		auto const break_transform = m4x4::Translation(0.0f, 4.0f, 6.0f);
		auto const break_body = demo.AddBox(v4{1.4f, 1.4f, 1.4f, 0.0f}, break_transform, 4.0f, Red);
		demo.AddWeld(demo.WorldFrame(break_transform), demo.BodyFrame(break_body, break_transform), 20.0f);
		demo.SceneState().m_body[break_body].VelocityWS(v4::Zero(), v4{0.0f, 0.0f, -8.0f, 0.0f});
	}

	// Build a long pendulous chain whose coloured constraints exercise linear graph scaling.
	void BuildRigidChain(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);

		auto joint_position = v4{0.0f, 0.0f, 12.0f, 1.0f};
		auto previous_body = -1;
		constexpr auto LinkCount = 48;
		for (auto link_index = 0; link_index != LinkCount; ++link_index)
		{
			// A shallow alternating lateral offset seeds visible motion without initial constraint error.
			auto const lateral = link_index % 2 == 0 ? 0.18f : -0.18f;
			auto const next_joint = joint_position + v4{lateral, 0.04f * std::sin(0.4f * link_index), -0.95f, 0.0f};
			auto const colour = link_index % 2 == 0 ? Orange : Blue;
			auto const body = demo.AddRod(joint_position, next_joint, 0.32f, 0.7f, colour);
			auto const frame = m4x4::Translation(joint_position);
			if (previous_body == -1)
				demo.AddBall(demo.WorldFrame(frame), demo.BodyFrame(body, frame));
			else
				demo.AddBall(demo.BodyFrame(previous_body, frame), demo.BodyFrame(body, frame));

			previous_body = body;
			joint_position = next_joint;
		}
	}

	// Build a planar four-bar mechanism whose final hinge closes a cycle outside tree topology.
	void BuildFourBarLinkage(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);

		// The four pivots satisfy all bar lengths exactly so the initial closed loop contains no assembly error.
		auto const pivot_a = v4{-2.5f, 0.0f, 5.5f, 1.0f};
		auto const pivot_b = v4{+2.5f, 0.0f, 5.5f, 1.0f};
		auto const pivot_c = pivot_a + v4{1.25f, 0.0f, 2.15f, 0.0f};
		auto const pivot_d = pivot_b + v4{-1.55f, 0.0f, 1.95f, 0.0f};
		auto const crank = demo.AddRod(pivot_a, pivot_c, 0.55f, 2.0f, Orange);
		auto const coupler = demo.AddRod(pivot_c, pivot_d, 0.55f, 2.8f, Blue);
		auto const rocker = demo.AddRod(pivot_d, pivot_b, 0.55f, 2.2f, Green);
		auto const free_axis = physics::ConstraintAxisDesc{.m_mode = physics::EConstraintAxisMode::Free};

		// All hinge frames align local X with world Y, constraining the mechanism to the XZ plane.
		auto const frame_a = ConstraintAxisFrame(v4::YAxis(), pivot_a);
		auto const frame_b = ConstraintAxisFrame(v4::YAxis(), pivot_b);
		auto const frame_c = ConstraintAxisFrame(v4::YAxis(), pivot_c);
		auto const frame_d = ConstraintAxisFrame(v4::YAxis(), pivot_d);
		demo.AddHinge(demo.WorldFrame(frame_a), demo.BodyFrame(crank, frame_a), free_axis);
		demo.AddHinge(demo.BodyFrame(crank, frame_c), demo.BodyFrame(coupler, frame_c), free_axis);
		demo.AddHinge(demo.BodyFrame(coupler, frame_d), demo.BodyFrame(rocker, frame_d), free_axis);
		demo.AddHinge(demo.BodyFrame(rocker, frame_b), demo.WorldFrame(frame_b), free_axis);
		demo.SceneState().m_body[crank].VelocityWS(1.25f * v4::YAxis(), v4::Zero());
	}
}
