#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	namespace
	{
		// Build a compact serial tree for stress scenes without adding persistent constraint rows.
		std::pair<physics::Articulation, std::vector<physics::LinkHandle>> BuildStressTree(DemoBuilder& demo, int link_count, m4x4 const& root_to_world, bool floating)
		{
			constexpr auto LinkLength = 0.55f;
			auto builder = physics::ArticulationBuilder{};
			auto links = std::vector<physics::LinkHandle>{};
			links.reserve(link_count + 1);
			auto const link_shape = v4{0.22f, 0.22f, LinkLength, 0.0f};
			auto const root_desc = demo.BoxLink(v4{0.4f, 0.4f, 0.4f, 0.0f}, 1.0f, false, false);
			auto const root = floating
				? builder.AddFloatingRoot(root_desc, root_to_world)
				: builder.AddFixedRoot(root_desc, root_to_world);
			links.push_back(root);

			for (auto link_index = 0; link_index != link_count; ++link_index)
			{
				auto joint = physics::ArticulationJointDesc::Revolute(
					link_index % 2 == 0 ? v4::YAxis() : v4::XAxis(),
					m4x4::Translation(0.0f, 0.0f, link_index == 0 ? 0.0f : -0.5f * LinkLength),
					m4x4::Translation(0.0f, 0.0f, +0.5f * LinkLength));
				joint.m_initial_position[0] = 0.04f * static_cast<float>((link_index % 5) - 2);
				joint.m_initial_velocity[0] = floating ? 0.08f * static_cast<float>((link_index % 3) - 1) : 0.0f;
				links.push_back(builder.AddLink(links.back(), joint, demo.BoxLink(link_shape, 0.35f, false, false)));
			}
			return {builder.Build(), std::move(links)};
		}
	}

	// Assemble weak rows, large mass ratios, a closed loop, and breakage in one finite stress scene.
	void BuildConstraintPathologies(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(14.0f);

		// A light chain suspended from an extremely heavy body exposes effective-mass conditioning.
		auto const heavy = demo.AddBox(v4{1.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(-4.0f, 0.0f, 8.0f), 10000.0f, Colour32(0xFF8060FFU));
		auto previous = heavy;
		for (auto link_index = 0; link_index != 14; ++link_index)
		{
			auto const z = 7.2f - 0.8f * static_cast<float>(link_index);
			auto const body = demo.AddBox(v4{0.28f, 0.28f, 0.75f, 0.0f}, m4x4::Translation(-4.0f, 0.0f, z), 0.02f, Colour32(0xFF50C080U));
			auto const joint_frame = m4x4::Translation(-4.0f, 0.0f, z + 0.4f);
			demo.AddBall(demo.BodyFrame(previous, joint_frame), demo.BodyFrame(body, joint_frame));
			previous = body;
		}

		// A mildly inconsistent four-edge loop checks deterministic convergence without requiring exact row compatibility.
		auto const loop_positions = std::array{
			v4{0.0f, -1.5f, 5.0f, 1.0f},
			v4{2.5f, -1.5f, 5.0f, 1.0f},
			v4{2.5f, +1.5f, 5.02f, 1.0f},
			v4{0.0f, +1.5f, 5.0f, 1.0f},
		};
		auto loop_bodies = std::array<int, 4>{};
		for (auto index = 0; index != isize(loop_bodies); ++index)
			loop_bodies[index] = demo.AddSphere(0.38f, m4x4::Translation(loop_positions[index]), 1.0f, Colour32(0xFFFFA040U));
		for (auto index = 0; index != isize(loop_bodies); ++index)
		{
			auto const next = (index + 1) % isize(loop_bodies);
			auto const midpoint = 0.5f * (loop_positions[index] + loop_positions[next]);
			demo.AddBall(
				demo.BodyFrame(loop_bodies[index], m4x4::Translation(midpoint)),
				demo.BodyFrame(loop_bodies[next], m4x4::Translation(midpoint)));
		}

		// A force-clamped motor and a low break threshold expose bounded outputs and event generation.
		auto const motor_body = demo.AddBox(v4{0.4f, 0.4f, 3.0f, 0.0f}, m4x4::Translation(5.0f, -1.5f, 5.5f), 1.0f, Colour32(0xFF40A0FFU));
		auto motor = physics::ConstraintAxisDesc{};
		motor.m_mode = physics::EConstraintAxisMode::Driven;
		motor.m_target_velocity = 1000.0f;
		motor.m_damping = 10.0f;
		motor.m_max_force = 8.0f;
		demo.AddAngularControl(demo.WorldFrame(m4x4::Translation(5.0f, -1.5f, 7.0f)), demo.BodyFrame(motor_body, m4x4::Translation(5.0f, -1.5f, 7.0f)), motor);

		auto const break_body = demo.AddSphere(0.45f, m4x4::Translation(5.0f, 2.0f, 6.0f), 1.0f, Colour32(0xFFFF4040U));
		demo.AddWeld(
			demo.WorldFrame(m4x4::Translation(5.0f, 2.0f, 8.0f)),
			demo.BodyFrame(break_body, m4x4::Translation(5.0f, 2.0f, 8.0f)),
			1.5f);
		demo.SceneState().m_body[break_body].VelocityWS(v4::Zero(), v4{0.0f, 0.0f, -12.0f, 0.0f});
	}

	// Combine a cyclic two-dimensional constraint graph with independent ABA trees to measure end-to-end GPU throughput.
	void BuildConstraintStress(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(22.0f);

		// Independent trees isolate reduced-coordinate linear work from the graph-colouring workload.
		for (auto tree_index = 0; tree_index != 3; ++tree_index)
		{
			auto const x = -7.0f + 7.0f * static_cast<float>(tree_index);
			auto tree = BuildStressTree(demo, 24, m4x4::Translation(x, -5.0f, 17.0f), false);
			demo.AddArticulation(std::move(tree.first));
		}

		// Every interior node participates in a four-edge cycle, preventing the persistent graph from degenerating into a tree.
		constexpr auto GridWidth = 18;
		constexpr auto GridHeight = 18;
		constexpr auto Spacing = 0.62f;
		auto bodies = std::array<int, GridWidth * GridHeight>{};
		auto const grid_index = [](int x, int z)
		{
			return z * GridWidth + x;
		};
		auto const position = [](int x, int z)
		{
			return v4{
				Spacing * (static_cast<float>(x) - 0.5f * static_cast<float>(GridWidth - 1)),
				4.0f,
				15.0f - Spacing * static_cast<float>(z),
				1.0f};
		};

		for (auto z = 0; z != GridHeight; ++z)
		{
			for (auto x = 0; x != GridWidth; ++x)
			{
				auto const colour = (x + z) % 2 == 0 ? Colour32(0xFFFFA040U) : Colour32(0xFF40A0FFU);
				bodies[grid_index(x, z)] = demo.AddSphere(0.14f, m4x4::Translation(position(x, z)), 0.2f, colour);
			}
		}
		for (auto z = 0; z != GridHeight; ++z)
		{
			for (auto x = 0; x != GridWidth; ++x)
			{
				auto const body = bodies[grid_index(x, z)];
				if (x + 1 != GridWidth)
				{
					auto const midpoint = 0.5f * (position(x, z) + position(x + 1, z));
					demo.AddBall(demo.BodyFrame(body, m4x4::Translation(midpoint)), demo.BodyFrame(bodies[grid_index(x + 1, z)], m4x4::Translation(midpoint)));
				}
				if (z + 1 != GridHeight)
				{
					auto const midpoint = 0.5f * (position(x, z) + position(x, z + 1));
					demo.AddBall(demo.BodyFrame(body, m4x4::Translation(midpoint)), demo.BodyFrame(bodies[grid_index(x, z + 1)], m4x4::Translation(midpoint)));
				}
			}
		}

		// Pin the upper corners while leaving the redundant interior cycles free to sag and collide with the floor.
		for (auto x : {0, GridWidth - 1})
		{
			auto const anchor = position(x, 0);
			demo.AddBall(demo.WorldFrame(m4x4::Translation(anchor)), demo.BodyFrame(bodies[grid_index(x, 0)], m4x4::Translation(anchor)));
		}
	}
}
