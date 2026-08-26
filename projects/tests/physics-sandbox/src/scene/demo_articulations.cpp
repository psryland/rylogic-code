#include "src/scene/demo_builder.h"

namespace physics_sandbox
{
	namespace
	{
		// A completed articulation accompanied by its stable topological link handles.
		struct BuiltArticulation
		{
			physics::Articulation m_articulation;
			std::vector<physics::LinkHandle> m_links;
		};

		// Build a serial revolute chain using centred box links and explicit endpoint attachment frames.
		BuiltArticulation BuildChain(DemoBuilder& demo, int link_count, m4x4 const& root_to_world, bool floating, v8motion root_velocity = {})
		{
			constexpr auto LinkLength = 0.9f;
			auto builder = physics::ArticulationBuilder{};
			auto links = std::vector<physics::LinkHandle>{};
			links.reserve(link_count + 1);

			auto const root_desc = demo.BoxLink(v4{0.65f, 0.65f, 0.65f, 0.0f}, 2.0f, false, false);
			auto const root = floating
				? builder.AddFloatingRoot(root_desc, root_to_world, root_velocity)
				: builder.AddFixedRoot(root_desc, root_to_world);
			links.push_back(root);

			for (auto link_index = 0; link_index != link_count; ++link_index)
			{
				auto const parent_offset = link_index == 0 ? 0.0f : -0.5f * LinkLength;
				auto joint = physics::ArticulationJointDesc::Revolute(
					link_index % 2 == 0 ? v4::YAxis() : v4::XAxis(),
					m4x4::Translation(0.0f, 0.0f, parent_offset),
					m4x4::Translation(0.0f, 0.0f, +0.5f * LinkLength));
				joint.m_initial_position[0] = 0.08f * std::sin(0.7f * static_cast<float>(link_index + 1));
				joint.m_initial_velocity[0] = floating ? 0.2f * std::cos(0.5f * static_cast<float>(link_index + 1)) : 0.0f;
				links.push_back(builder.AddLink(
					links.back(),
					joint,
					demo.BoxLink(v4{0.35f, 0.35f, LinkLength, 0.0f}, 0.8f, false, false)));
			}

			return BuiltArticulation{
				.m_articulation = builder.Build(),
				.m_links = std::move(links),
			};
		}

		// Build a shaped floating root suitable for direct coupling and transient contact demonstrations.
		std::pair<physics::Articulation, physics::LinkHandle> BuildFloatingSphere(DemoBuilder& demo, v4 position, float radius, float mass, v4 velocity = v4::Zero())
		{
			auto builder = physics::ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				demo.SphereLink(radius, mass, false, true),
				m4x4::Translation(position),
				v8motion{v4::Zero(), velocity});
			return {builder.Build(), root};
		}
	}

	// Build several fixed-root ABA chains to expose near-linear tree scaling without persistent constraints.
	void BuildFixedArticulations(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(2);

		for (auto const [x, link_count] : std::array{
			std::pair{-5.0f, 8},
			std::pair{0.0f, 16},
			std::pair{5.0f, 28},
		})
		{
			auto chain = BuildChain(demo, link_count, m4x4::Translation(x, 0.0f, 11.0f), false);
			demo.AddArticulation(std::move(chain.m_articulation));
		}
	}

	// Build a fixed robot arm whose reduced coordinates are controlled through bounded persistent rows.
	void BuildRobotMotors(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(12.0f);

		auto chain = BuildChain(demo, 7, m4x4::Translation(0.0f, 0.0f, 8.0f), false);
		auto const articulation_index = demo.AddArticulation(std::move(chain.m_articulation));
		auto& articulation = demo.SceneState().m_articulation[articulation_index];
		articulation.UpdateKinematics();

		// Each control row acts only along the reduced joint axis; the articulation already owns the five locked directions.
		for (auto link_index = 1; link_index != isize(chain.m_links); ++link_index)
		{
			auto const child = chain.m_links[link_index];
			auto const parent = articulation.Parent(child);
			auto const& joint = articulation.JointDescription(child);
			auto const joint_to_world = articulation.LinkToWorld(parent) * joint.m_joint_to_parent;
			auto const axis_world = (joint_to_world.rot * joint.m_axes[0].m_axis).w0();
			auto const control_frame = ConstraintAxisFrame(axis_world, joint_to_world.pos);
			auto axis = physics::ConstraintAxisDesc{};
			if (link_index == isize(chain.m_links) - 1)
			{
				axis.m_mode = physics::EConstraintAxisMode::Limited;
				axis.m_limits = Range<float>{-0.35f, +0.35f};
			}
			else
			{
				axis.m_mode = physics::EConstraintAxisMode::Driven;
				axis.m_target_position = link_index % 2 == 0 ? +0.45f : -0.35f;
				axis.m_stiffness = 90.0f;
				axis.m_damping = 18.0f;
				axis.m_max_force = 180.0f;
			}
			demo.AddAngularControl(
				demo.LinkFrame(articulation_index, parent, control_frame),
				demo.LinkFrame(articulation_index, child, control_frame),
				axis);
		}
	}

	// Couple ordinary rigid bodies, fixed trees, and floating trees in the same persistent solve.
	void BuildMixedCoupling(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);

		// A fixed-root prismatic tree supports the left endpoint of an ordinary rigid connecting rod.
		auto fixed_builder = physics::ArticulationBuilder{};
		auto const fixed_root = fixed_builder.AddFixedRoot(demo.SphereLink(0.45f, 2.0f, false, false), m4x4::Translation(-5.0f, 0.0f, 6.0f));
		auto prismatic = physics::ArticulationJointDesc::Prismatic(v4::XAxis());
		prismatic.m_initial_position[0] = 2.0f;
		auto const sliding_link = fixed_builder.AddLink(fixed_root, prismatic, demo.SphereLink(0.45f, 1.0f, false, false));
		auto const fixed_tree = demo.AddArticulation(fixed_builder.Build());

		auto [floating_articulation, floating_root] = BuildFloatingSphere(demo, v4{1.0f, 0.0f, 6.0f, 1.0f}, 0.45f, 1.0f);
		auto const floating_tree = demo.AddArticulation(std::move(floating_articulation));
		auto const rod = demo.AddRod(v4{-3.0f, 0.0f, 6.0f, 1.0f}, v4{1.0f, 0.0f, 6.0f, 1.0f}, 0.5f, 3.0f, Colour32(0xFFFFA040U));
		auto const left_frame = m4x4::Translation(-3.0f, 0.0f, 6.0f);
		auto const right_frame = m4x4::Translation(1.0f, 0.0f, 6.0f);
		demo.AddBall(demo.LinkFrame(fixed_tree, sliding_link, left_frame), demo.BodyFrame(rod, left_frame));
		demo.AddBall(demo.BodyFrame(rod, right_frame), demo.LinkFrame(floating_tree, floating_root, right_frame));

		// Extend the same supported island with a direct tree-to-tree edge, proving that no rigid-body intermediary is required.
		auto [tree_b_value, tree_b_root] = BuildFloatingSphere(demo, v4{5.0f, 0.0f, 6.0f, 1.0f}, 0.5f, 1.0f);
		auto const tree_b = demo.AddArticulation(std::move(tree_b_value));
		auto const tree_frame = m4x4::Translation(3.0f, 0.0f, 6.0f);
		demo.AddBall(demo.LinkFrame(floating_tree, floating_root, tree_frame), demo.LinkFrame(tree_b, tree_b_root, tree_frame));
	}

	// Exercise rigid/tree, tree/tree, and non-adjacent same-tree transient contacts together.
	void BuildMixedContacts(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.AddGround(12.0f);

		// A rigid sphere impacts two overlapping floating roots so momentum crosses both ownership models.
		auto const rigid = demo.AddSphere(0.6f, m4x4::Translation(-2.0f, 0.0f, 4.0f), 1.0f, Colour32(0xFFFFA040U));
		demo.SceneState().m_body[rigid].VelocityWS(v4::Zero(), v4{4.0f, 0.0f, 0.0f, 0.0f});
		auto [articulation_a, root_a] = BuildFloatingSphere(demo, v4{-0.8f, 0.0f, 4.0f, 1.0f}, 0.6f, 1.0f);
		auto [articulation_b, root_b] = BuildFloatingSphere(demo, v4{+0.25f, 0.0f, 4.0f, 1.0f}, 0.6f, 1.0f);
		demo.AddArticulation(std::move(articulation_a));
		demo.AddArticulation(std::move(articulation_b));

		// A shaped root and a non-adjacent prismatic descendant begin overlapped to expose same-tree contact correction.
		auto self_builder = physics::ArticulationBuilder{};
		auto const self_root = self_builder.AddFixedRoot(
			demo.SphereLink(0.6f, 2.0f, false, true),
			m4x4::Translation(4.5f, 0.0f, 2.5f));
		auto middle_desc = physics::ArticulationLinkDesc{};
		middle_desc.m_inertia = physics::Inertia::Sphere(0.1f, 0.1f);
		auto const middle = self_builder.AddLink(self_root, physics::ArticulationJointDesc::Fixed(), middle_desc);
		auto self_joint = physics::ArticulationJointDesc::Prismatic(v4::XAxis());
		self_joint.m_initial_position[0] = 1.0f;
		self_builder.AddLink(middle, self_joint, demo.SphereLink(0.6f, 1.0f, false, true));
		demo.AddArticulation(self_builder.Build());
	}

	// Apply the existing GPU buoyancy force module independently to every shaped link in a floating tree.
	void BuildBuoyantArticulation(DemoBuilder& demo)
	{
		demo.Gravity(v4{0.0f, 0.0f, -9.81f, 0.0f});
		demo.Substeps(1);
		demo.Water(0.0f, v2{24.0f, 18.0f});

		auto builder = physics::ArticulationBuilder{};
		auto links = std::vector<physics::LinkHandle>{};
		auto const root = builder.AddFloatingRoot(
			demo.BoxLink(v4{2.2f, 1.3f, 0.8f, 0.0f}, 420.0f, false, false),
			m4x4::Transform(v4::YAxis(), 0.18f, v4{0.0f, 0.0f, -0.2f, 1.0f}));
		links.push_back(root);
		for (auto link_index = 0; link_index != 4; ++link_index)
		{
			auto joint = physics::ArticulationJointDesc::Revolute(
				v4::YAxis(),
				m4x4::Translation(link_index == 0 ? 1.2f : 0.8f, 0.0f, 0.0f),
				m4x4::Translation(-0.8f, 0.0f, 0.0f));
			joint.m_initial_position[0] = 0.12f * (link_index % 2 == 0 ? 1.0f : -1.0f);
			links.push_back(builder.AddLink(
				links.back(),
				joint,
				demo.BoxLink(v4{1.6f, 0.7f, 0.55f, 0.0f}, 95.0f, false, false)));
		}

		auto const articulation_index = demo.AddArticulation(builder.Build());
		for (auto link : links)
			demo.MakeBuoyant(articulation_index, link);
	}

	// Build a force-free asymmetric floating tree with internal motion and no collision path.
	void BuildFloatingConservation(DemoBuilder& demo)
	{
		demo.Gravity(v4::Zero());
		demo.Substeps(4);

		auto builder = physics::ArticulationBuilder{};
		auto const root = builder.AddFloatingRoot(
			demo.BoxLink(v4{1.4f, 1.8f, 2.2f, 0.0f}, 3.2f, false, false),
			m4x4::Transform(v4::YAxis(), 0.31f, v4{0.0f, 0.0f, 2.0f, 1.0f}),
			v8motion{v4{0.9f, -0.55f, 0.7f, 0.0f}, v4{0.2f, 0.35f, -0.12f, 0.0f}});
		auto joint_a = physics::ArticulationJointDesc::Revolute(
			Normalise(v4{1.0f, 2.0f, -1.0f, 0.0f}),
			m4x4::Translation(0.7f, 0.0f, 0.2f),
			m4x4::Translation(-0.45f, 0.0f, 0.0f));
		joint_a.m_initial_position[0] = 0.42f;
		joint_a.m_initial_velocity[0] = 1.15f;
		auto const child = builder.AddLink(root, joint_a, demo.BoxLink(v4{0.9f, 1.2f, 1.5f, 0.0f}, 1.4f, false, false));
		auto joint_b = physics::ArticulationJointDesc::Revolute(
			Normalise(v4{-1.0f, 1.0f, 2.0f, 0.0f}),
			m4x4::Translation(-0.3f, 0.5f, 0.1f),
			m4x4::Translation(0.25f, -0.3f, 0.0f));
		joint_b.m_initial_position[0] = -0.36f;
		joint_b.m_initial_velocity[0] = -0.82f;
		builder.AddLink(child, joint_b, demo.BoxLink(v4{0.65f, 0.85f, 1.0f, 0.0f}, 0.8f, false, false));
		auto const articulation_index = demo.AddArticulation(builder.Build());
		demo.SceneState().m_articulation[articulation_index].NeverSleep(true);
	}

	// Place rigid and reduced-coordinate asymmetric bodies side by side under identical torque-free spin.
	void BuildDzhanibekov(DemoBuilder& demo)
	{
		demo.Gravity(v4::Zero());
		demo.Substeps(2);
		auto const dimensions = v4{2.0f, 4.0f, 8.0f, 0.0f};
		auto const angular_velocity = v4{1.0f, 10.0f, 1.0f, 0.0f};

		auto const rigid = demo.AddBox(dimensions, m4x4::Translation(-5.0f, 0.0f, 0.0f), 1.0f, Colour32(0xFFFFA040U));
		demo.SceneState().m_body[rigid].VelocityOS(angular_velocity, v4::Zero());
		demo.SceneState().m_body[rigid].NeverSleep(true);

		auto builder = physics::ArticulationBuilder{};
		auto const root = builder.AddFloatingRoot(
			demo.BoxLink(dimensions, 1.0f, false, false),
			m4x4::Translation(+5.0f, 0.0f, 0.0f),
			v8motion{angular_velocity, v4::Zero()});
		auto const articulation_index = demo.AddArticulation(builder.Build());
		demo.SceneState().m_articulation[articulation_index].NeverSleep(true);
		(void)root;
	}
}
