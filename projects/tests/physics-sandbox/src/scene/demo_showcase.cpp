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

		// Build one branched floating articulation with enough clearance for its limbs to collide after falling.
		physics::Articulation BuildRagdoll(DemoBuilder& demo, m4x4 const& torso_to_world, v8motion root_velocity)
		{
			auto builder = physics::ArticulationBuilder{};
			auto const torso = builder.AddFloatingRoot(demo.BoxLink(v4{0.9f, 0.55f, 1.5f, 0.0f}, 5.0f, false, true), torso_to_world, root_velocity);

			// The head and four two-segment limbs make a shallow branched tree rather than a serial-chain surrogate.
			auto const neck = physics::ArticulationJointDesc::Revolute(v4::YAxis(), m4x4::Translation(0.0f, 0.0f, +0.9f), m4x4::Translation(0.0f, 0.0f, -0.32f));
			builder.AddLink(torso, neck, demo.SphereLink(0.38f, 1.2f, false, true));
			for (auto side : {-1.0f, +1.0f})
			{
				auto shoulder = physics::ArticulationJointDesc::Revolute(
					v4::YAxis(),
					m4x4::Transform(v4::YAxis(), side * constants<float>::tau_by_4, v4{side * 0.55f, 0.0f, +0.45f, 1.0f}),
					m4x4::Translation(0.0f, 0.0f, +0.55f));
				shoulder.m_initial_position[0] = 0.18f * side;
				auto const upper_arm = builder.AddLink(torso, shoulder, demo.BoxLink(v4{0.32f, 0.32f, 1.1f, 0.0f}, 0.8f, false, true));
				auto elbow = physics::ArticulationJointDesc::Revolute(v4::YAxis(), m4x4::Translation(0.0f, 0.0f, -0.55f), m4x4::Translation(0.0f, 0.0f, +0.48f));
				elbow.m_initial_position[0] = -0.3f;
				builder.AddLink(upper_arm, elbow, demo.BoxLink(v4{0.28f, 0.28f, 0.96f, 0.0f}, 0.6f, false, true));

				auto hip = physics::ArticulationJointDesc::Revolute(
					v4::YAxis(),
					m4x4::Translation(side * 0.25f, 0.0f, -0.82f),
					m4x4::Translation(0.0f, 0.0f, +0.68f));
				hip.m_initial_position[0] = 0.12f * side;
				auto const thigh = builder.AddLink(torso, hip, demo.BoxLink(v4{0.38f, 0.38f, 1.36f, 0.0f}, 1.3f, false, true));
				auto knee = physics::ArticulationJointDesc::Revolute(v4::YAxis(), m4x4::Translation(0.0f, 0.0f, -0.68f), m4x4::Translation(0.0f, 0.0f, +0.62f));
				knee.m_initial_position[0] = 0.2f;
				builder.AddLink(thigh, knee, demo.BoxLink(v4{0.34f, 0.34f, 1.24f, 0.0f}, 1.0f, false, true));
			}
			return builder.Build();
		}

		// Build one fixed-base horizontal prismatic actuator and return its articulation plus moving link handle.
		std::pair<physics::Articulation, physics::LinkHandle> BuildLifter(DemoBuilder& demo, float root_x, float axis_sign)
		{
			auto builder = physics::ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(demo.SphereLink(0.45f, 2.0f, false, false), m4x4::Translation(root_x, 0.0f, 6.0f));
			auto joint = physics::ArticulationJointDesc::Prismatic(axis_sign * v4::XAxis());
			joint.m_initial_position[0] = 2.0f;
			auto const link = builder.AddLink(root, joint, demo.SphereLink(0.55f, 1.5f, false, false));
			return {builder.Build(), link};
		}

		// Add a bounded reduced-coordinate drive along one existing prismatic joint.
		void AddPrismaticDrive(DemoBuilder& demo, int articulation_index, physics::LinkHandle link, float target_position, float maximum_force)
		{
			auto& articulation = demo.SceneState().m_articulation[articulation_index];
			articulation.UpdateKinematics();
			auto const parent = articulation.Parent(link);
			auto const& joint = articulation.JointDescription(link);
			auto const joint_to_world = articulation.LinkToWorld(parent) * joint.m_joint_to_parent;
			auto const axis_world = (joint_to_world.rot * joint.m_axes[0].m_axis).w0();
			auto const control_frame = ConstraintAxisFrame(axis_world, joint_to_world.pos);
			demo.AddLinearControl(
				demo.LinkFrame(articulation_index, parent, control_frame),
				demo.LinkFrame(articulation_index, link, control_frame),
				physics::ConstraintAxisDesc{
					.m_mode = physics::EConstraintAxisMode::Driven,
					.m_target_position = target_position,
					.m_stiffness = 120.0f,
					.m_damping = 24.0f,
					.m_max_force = maximum_force,
				});
		}
	}

	// Build a passive physical pendulum whose small-angle period has a closed-form reference.
	void BuildPendulum(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(8.0f);

		// A uniform rod about one endpoint has T = 2*pi*sqrt(2L/(3g)) in the small-angle limit.
		constexpr auto Length = 5.0f;
		constexpr auto Angle = 0.35f;
		auto const anchor = v4{0.0f, 0.0f, 7.0f, 1.0f};
		auto const direction = v4{std::sin(Angle), 0.0f, -std::cos(Angle), 0.0f};
		auto const end = anchor + Length * direction;
		auto const body = demo.AddRod(anchor, end, 0.45f, 3.0f, Orange);
		auto const frame = ConstraintAxisFrame(v4::YAxis(), anchor);
		demo.AddHinge(
			demo.WorldFrame(frame),
			demo.BodyFrame(body, frame),
			physics::ConstraintAxisDesc{.m_mode = physics::EConstraintAxisMode::Free});
		demo.SceneState().m_body[body].NeverSleep(true);
	}

	// Drop several branched floating trees together to demonstrate many small ABA jobs and transient contacts.
	void BuildRagdolls(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(12.0f);

		// Different orientations and root velocities prevent the contacts from remaining artificially symmetric.
		for (auto index = 0; index != 6; ++index)
		{
			auto const x = -4.0f + 1.6f * static_cast<float>(index % 3);
			auto const y = -1.2f + 2.4f * static_cast<float>(index / 3);
			auto const z = 6.0f + 1.7f * static_cast<float>(index);
			auto const pose = m4x4::Transform(v4::YAxis(), 0.12f * static_cast<float>(index - 2), v4{x, y, z, 1.0f});
			auto const velocity = v8motion{
				v4{0.1f * static_cast<float>(index & 1), -0.08f * static_cast<float>(index % 3), 0.05f, 0.0f},
				v4{0.15f * static_cast<float>((index % 3) - 1), 0.0f, 0.0f, 0.0f}};
			demo.AddArticulation(BuildRagdoll(demo, pose, velocity));
		}
	}

	// Build a motorised fixed arm with independently driven prismatic fingers around a rigid payload.
	void BuildRobotGripper(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(10.0f);

		// The serial arm ends in a fixed palm with two symmetric prismatic branches.
		auto builder = physics::ArticulationBuilder{};
		auto const root = builder.AddFixedRoot(demo.SphereLink(0.45f, 2.0f, false, false), m4x4::Translation(0.0f, 0.0f, 8.0f));
		auto shoulder_joint = physics::ArticulationJointDesc::Revolute(v4::YAxis(), m4x4::Identity(), m4x4::Translation(0.0f, 0.0f, +1.2f));
		shoulder_joint.m_initial_position[0] = 0.15f;
		auto const shoulder = builder.AddLink(root, shoulder_joint, demo.BoxLink(v4{0.45f, 0.45f, 2.4f, 0.0f}, 2.0f, false, false));
		auto elbow_joint = physics::ArticulationJointDesc::Revolute(v4::XAxis(), m4x4::Translation(0.0f, 0.0f, -1.2f), m4x4::Translation(0.0f, 0.0f, +0.95f));
		elbow_joint.m_initial_position[0] = -0.2f;
		auto const elbow = builder.AddLink(shoulder, elbow_joint, demo.BoxLink(v4{0.4f, 0.4f, 1.9f, 0.0f}, 1.5f, false, false));
		auto const palm = builder.AddLink(elbow, physics::ArticulationJointDesc::Fixed(m4x4::Translation(0.0f, 0.0f, -1.65f)), demo.BoxLink(v4{1.8f, 0.65f, 0.45f, 0.0f}, 1.0f, false, false));

		auto finger_a_joint = physics::ArticulationJointDesc::Prismatic(-v4::XAxis(), m4x4::Translation(-0.68f, 0.0f, -0.5f), m4x4::Translation(0.0f, 0.0f, +0.65f));
		finger_a_joint.m_initial_position[0] = 0.35f;
		auto const finger_a = builder.AddLink(palm, finger_a_joint, demo.BoxLink(v4{0.28f, 0.45f, 1.3f, 0.0f}, 0.45f, false, false));
		auto finger_b_joint = physics::ArticulationJointDesc::Prismatic(+v4::XAxis(), m4x4::Translation(+0.68f, 0.0f, -0.5f), m4x4::Translation(0.0f, 0.0f, +0.65f));
		finger_b_joint.m_initial_position[0] = 0.35f;
		auto const finger_b = builder.AddLink(palm, finger_b_joint, demo.BoxLink(v4{0.28f, 0.45f, 1.3f, 0.0f}, 0.45f, false, false));

		auto const articulation_index = demo.AddArticulation(builder.Build());
		auto& articulation = demo.SceneState().m_articulation[articulation_index];
		articulation.UpdateKinematics();

		// Bounded rows drive only the articulation's free coordinates and leave its internal locked directions to ABA.
		for (auto const [link, target] : std::array{
			std::pair{shoulder, +0.35f},
			std::pair{elbow, -0.45f},
		})
		{
			auto const parent = articulation.Parent(link);
			auto const& joint = articulation.JointDescription(link);
			auto const joint_to_world = articulation.LinkToWorld(parent) * joint.m_joint_to_parent;
			auto const axis_world = (joint_to_world.rot * joint.m_axes[0].m_axis).w0();
			auto const frame = ConstraintAxisFrame(axis_world, joint_to_world.pos);
			demo.AddAngularControl(
				demo.LinkFrame(articulation_index, parent, frame),
				demo.LinkFrame(articulation_index, link, frame),
				physics::ConstraintAxisDesc{
					.m_mode = physics::EConstraintAxisMode::Driven,
					.m_target_position = target,
					.m_stiffness = 90.0f,
					.m_damping = 18.0f,
					.m_max_force = 180.0f,
				});
		}
		AddPrismaticDrive(demo, articulation_index, finger_a, 0.0f, 90.0f);
		AddPrismaticDrive(demo, articulation_index, finger_b, 0.0f, 90.0f);

		// The payload begins between the fingers so closing motion immediately produces contact feedback through the complete tree.
		auto const payload_position = 0.5f * (articulation.LinkToWorld(finger_a).pos + articulation.LinkToWorld(finger_b).pos);
		demo.AddSphere(0.48f, m4x4::Translation(payload_position), 0.8f, Red);
	}

	// Build a rigid chassis supported by four compliant, travel-limited wheel constraints.
	void BuildVehicleSuspension(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(16.0f);

		auto const chassis = demo.AddBox(v4{4.8f, 3.0f, 0.8f, 0.0f}, m4x4::Translation(0.0f, 0.0f, 3.8f), 14.0f, Blue);
		demo.SceneState().m_body[chassis].VelocityWS(v4::Zero(), v4{2.5f, 0.0f, 0.0f, 0.0f});

		// One D6 descriptor provides the spring and lateral wheel guidance without unnecessarily locking spherical wheel rotation.
		// A second scalar row owns the independent hard travel limit.
		for (auto x : {-1.8f, +1.8f})
		{
			for (auto y : {-1.15f, +1.15f})
			{
				auto const wheel_position = v4{x, y, 2.45f, 1.0f};
				auto const wheel = demo.AddSphere(0.72f, m4x4::Translation(wheel_position), 1.2f, Orange);
				demo.SceneState().m_body[wheel].VelocityWS(v4::Zero(), v4{2.5f, 0.0f, 0.0f, 0.0f});
				auto const frame = ConstraintAxisFrame(v4::ZAxis(), wheel_position);
				auto const frame_a = demo.BodyFrame(chassis, frame);
				auto const frame_b = demo.BodyFrame(wheel, frame);

				auto suspension = physics::D6ConstraintDesc{
					.m_frame_a = frame_a,
					.m_frame_b = frame_b,
				};
				suspension.m_linear[0] = physics::ConstraintAxisDesc{
					.m_mode = physics::EConstraintAxisMode::Driven,
					.m_target_position = 0.0f,
					.m_stiffness = 260.0f,
					.m_damping = 32.0f,
					.m_max_force = 900.0f,
				};
				suspension.m_linear[1].m_mode = physics::EConstraintAxisMode::Locked;
				suspension.m_linear[2].m_mode = physics::EConstraintAxisMode::Locked;
				demo.SceneState().m_constraints.Add(suspension);

				demo.AddLinearControl(
					frame_a,
					frame_b,
					physics::ConstraintAxisDesc{
						.m_mode = physics::EConstraintAxisMode::Limited,
						.m_limits = Range<float>{-0.65f, +0.65f},
					});
			}
		}
	}

	// Build a pinned two-rail bridge whose longitudinal and cross rows form repeated rigid-body cycles.
	void BuildSuspensionBridge(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(18.0f);

		constexpr auto Columns = 24;
		constexpr auto Spacing = 0.9f;
		auto planks = std::array<int, 2 * Columns>{};
		auto const plank_index = [](int row, int column)
		{
			return row * Columns + column;
		};
		auto const position = [](int row, int column)
		{
			return v4{
				Spacing * (static_cast<float>(column) - 0.5f * static_cast<float>(Columns - 1)),
				row == 0 ? -0.85f : +0.85f,
				5.0f,
				1.0f};
		};

		// Two longitudinal chains joined at every column create a cyclic ladder rather than two independent ropes.
		for (auto row = 0; row != 2; ++row)
		{
			for (auto column = 0; column != Columns; ++column)
			{
				auto const colour = column % 2 == 0 ? Orange : Blue;
				planks[plank_index(row, column)] = demo.AddBox(v4{0.82f, 1.45f, 0.24f, 0.0f}, m4x4::Translation(position(row, column)), 0.8f, colour);
			}
		}
		for (auto row = 0; row != 2; ++row)
		{
			for (auto column = 0; column + 1 != Columns; ++column)
			{
				auto const joint_position = 0.5f * (position(row, column) + position(row, column + 1));
				auto const frame = m4x4::Translation(joint_position);
				demo.AddBall(
					demo.BodyFrame(planks[plank_index(row, column)], frame),
					demo.BodyFrame(planks[plank_index(row, column + 1)], frame));
			}
		}
		for (auto column = 0; column != Columns; ++column)
		{
			auto const joint_position = 0.5f * (position(0, column) + position(1, column));
			auto const frame = m4x4::Translation(joint_position);
			demo.AddBall(
				demo.BodyFrame(planks[plank_index(0, column)], frame),
				demo.BodyFrame(planks[plank_index(1, column)], frame));
		}

		// Pin all four corners so gravity loads the complete cyclic graph.
		for (auto row = 0; row != 2; ++row)
		{
			for (auto column : {0, Columns - 1})
			{
				auto const anchor = position(row, column);
				auto const frame = m4x4::Translation(anchor);
				demo.AddBall(demo.WorldFrame(frame), demo.BodyFrame(planks[plank_index(row, column)], frame));
			}
		}
	}

	// Drive one reduced-coordinate pusher into an ordinary rigid stack to expose bidirectional contact coupling.
	void BuildArticulationPush(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(12.0f);

		auto builder = physics::ArticulationBuilder{};
		auto const root = builder.AddFixedRoot(demo.SphereLink(0.45f, 2.0f, false, false), m4x4::Translation(-5.0f, 0.0f, 1.1f));
		auto joint = physics::ArticulationJointDesc::Prismatic(v4::XAxis());
		joint.m_initial_position[0] = 0.8f;
		joint.m_initial_velocity[0] = 2.5f;
		auto const pusher = builder.AddLink(root, joint, demo.BoxLink(v4{1.1f, 2.2f, 1.1f, 0.0f}, 4.0f, false, false));
		auto const articulation_index = demo.AddArticulation(builder.Build());
		AddPrismaticDrive(demo, articulation_index, pusher, 5.5f, 1400.0f);

		// A staggered stack gives the articulation contact lane multiple simultaneous rigid targets.
		for (auto level = 0; level != 4; ++level)
		{
			for (auto column = 0; column != 3; ++column)
			{
				auto const position = v4{0.8f + 1.05f * static_cast<float>(column), 0.0f, 0.55f + 1.05f * static_cast<float>(level), 1.0f};
				demo.AddBox(v4{0.95f, 0.95f, 0.95f, 0.0f}, m4x4::Translation(position), 0.8f, level % 2 == 0 ? Green : Purple);
			}
		}
	}

	// Couple two independent reduced-coordinate actuators to opposite ends of one shared rigid payload.
	void BuildTwoRobotLoad(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(14.0f);

		auto [left_value, left_link] = BuildLifter(demo, -6.0f, +1.0f);
		auto [right_value, right_link] = BuildLifter(demo, +6.0f, -1.0f);
		auto const left = demo.AddArticulation(std::move(left_value));
		auto const right = demo.AddArticulation(std::move(right_value));
		auto& left_articulation = demo.SceneState().m_articulation[left];
		auto& right_articulation = demo.SceneState().m_articulation[right];
		left_articulation.UpdateKinematics();
		right_articulation.UpdateKinematics();

		// Both link-to-rigid edges belong to one mixed island, so payload response feeds back into each separate tree.
		auto const left_endpoint = left_articulation.LinkToWorld(left_link).pos;
		auto const right_endpoint = right_articulation.LinkToWorld(right_link).pos;
		auto const load = demo.AddRod(left_endpoint, right_endpoint, 0.65f, 8.0f, Red);
		auto const left_frame = m4x4::Translation(left_endpoint);
		auto const right_frame = m4x4::Translation(right_endpoint);
		demo.AddBall(demo.LinkFrame(left, left_link, left_frame), demo.BodyFrame(load, left_frame));
		demo.AddBall(demo.LinkFrame(right, right_link, right_frame), demo.BodyFrame(load, right_frame));
		AddPrismaticDrive(demo, left, left_link, 2.0f, 500.0f);
		AddPrismaticDrive(demo, right, right_link, 2.0f, 500.0f);
	}
}
