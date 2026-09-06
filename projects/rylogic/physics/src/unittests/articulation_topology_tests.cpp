//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return finite unit mass properties suitable for every moving articulation link.
		ArticulationLinkDesc MovingLink(float mass = 1.0f)
		{
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(v4{0.3f, 0.4f, 0.5f, 0}, mass),
			};
		}

		// Require two spatial motion vectors to agree component-wise.
		void ExpectMotionNear(v8motion actual, v8motion expected, float tolerance)
		{
			PR_EXPECT(FEqlAbsolute(actual.ang, expected.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(actual.lin, expected.lin, tolerance));
		}

		// Estimate a child-frame spatial velocity from two nearby world poses.
		v8motion FiniteDifferenceVelocity(m4x4 const& pose0, m4x4 const& pose1, float elapsed_seconds)
		{
			auto const body_rotation_delta = Transpose(pose0.rot) * pose1.rot;
			auto const angular = v3{
				body_rotation_delta.y.z - body_rotation_delta.z.y,
				body_rotation_delta.z.x - body_rotation_delta.x.z,
				body_rotation_delta.x.y - body_rotation_delta.y.x,
			} / (2.0f * elapsed_seconds);
			auto const linear = Transpose(pose0.rot) * ((pose1.pos - pose0.pos) / elapsed_seconds);
			return v8motion{v4{angular, 0.0f}, linear};
		}

		// Return a generic ordered joint containing the first 'dof_count' independent scalar axes.
		ArticulationJointDesc OrderedJoint(int dof_count)
		{
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), +0.2f, v4{+0.3f, -0.2f, +0.1f, 1}),
				m4x4::Transform(v4::XAxis(), -0.15f, v4{-0.1f, +0.2f, -0.3f, 1}));
			joint.m_dof_count = dof_count;
			joint.m_axes = {
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::XAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::YAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::ZAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::ZAxis()},
			};
			for (int index = 0; index != dof_count; ++index)
			{
				joint.m_initial_position[index] = 0.03f * static_cast<float>(index + 1);
				joint.m_initial_velocity[index] = -0.07f + 0.025f * static_cast<float>(index);
			}
			return joint;
		}
	}

	PRUnitTestClass(ArticulationTopologyTests)
	{
		// Preserve topology order, stable identities, and root-specific generalized dimensions.
		PRUnitTestMethod(BuilderCreatesStableTree, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				MovingLink(4.0f),
				m4x4::Translation(2.0f, -1.0f, 3.0f),
				v8motion{v4{0, 0, 1, 0}, v4{0.5f, 0, 0, 0}});
			auto joint_a = ArticulationJointDesc::Revolute(v4::YAxis(), m4x4::Translation(0.0f, 1.0f, 0.0f));
			auto const link_a = builder.AddLink(root, joint_a, MovingLink(2.0f));
			auto const link_b = builder.AddLink(link_a, ArticulationJointDesc::Prismatic(v4::XAxis()), MovingLink());
			auto articulation = builder.Build();

			PR_EXPECT(static_cast<bool>(articulation.Id()));
			PR_EXPECT(articulation.RootType() == EArticulationRootType::Floating);
			PR_EXPECT(articulation.Root() == root);
			PR_EXPECT(articulation.LinkCount() == 3);
			PR_EXPECT(articulation.DofCount() == 8);
			PR_EXPECT(articulation.JointDofCount(root) == 0);
			PR_EXPECT(articulation.JointDofCount(link_a) == 1);
			PR_EXPECT(articulation.Parent(root) == LinkHandle{});
			PR_EXPECT(articulation.Parent(link_a) == root);
			PR_EXPECT(articulation.Parent(link_b) == link_a);
			PR_EXPECT(FEql(articulation.RootToWorld().pos, v4{2, -1, 3, 1}));
			ExpectMotionNear(articulation.RootVelocity(), v8motion{v4{0, 0, 1, 0}, v4{0.5f, 0, 0, 0}}, 1.0e-6f);

			PR_THROWS(builder.AddLink(root, joint_a, MovingLink()), std::exception);
			PR_THROWS(builder.Build(), std::exception);
		}

		// Keep link handles isolated between builders even when their slot indices are equal.
		PRUnitTestMethod(LinkHandlesRejectCrossArticulationUse, Quick)
		{
			auto builder_a = ArticulationBuilder{};
			auto builder_b = ArticulationBuilder{};
			auto const root_a = builder_a.AddFixedRoot({});
			auto const root_b = builder_b.AddFixedRoot({});
			PR_EXPECT(root_a.m_index == root_b.m_index);
			PR_EXPECT(root_a.m_generation != root_b.m_generation);

			PR_THROWS(builder_a.AddLink(root_b, ArticulationJointDesc::Revolute(), MovingLink()), std::exception);
			auto articulation_a = builder_a.Build();
			PR_THROWS(articulation_a.Parent(root_b), std::exception);
		}

		// Compose parent and child joint frames around the ordered scalar displacement.
		PRUnitTestMethod(RevoluteForwardKinematics, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot({}, m4x4::Identity());
			auto joint = ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1.0f, 0.0f, 0.0f));
			joint.m_initial_position[0] = constants<float>::tau_by_4;
			joint.m_initial_velocity[0] = 2.0f;
			auto const child = builder.AddLink(root, joint, MovingLink());
			auto articulation = builder.Build();

			auto const expected_pose = m4x4::Translation(1.0f, 0.0f, 0.0f) * m4x4::Transform(v4::ZAxis(), constants<float>::tau_by_4, v4::Origin());
			PR_EXPECT(FEqlAbsolute(articulation.LinkToWorld(child), expected_pose, 1.0e-5f));
			ExpectMotionNear(articulation.LinkVelocity(child), v8motion{v4{0, 0, 2, 0}, v4{}}, 1.0e-5f);
		}

		// Transform floating-root velocity to an offset fixed child without treating the child as an independent body.
		PRUnitTestMethod(FloatingRootVelocityPropagatesToLinks, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(MovingLink(), m4x4::Identity(), v8motion{v4{0, 0, 1, 0}, v4{}});
			auto const child = builder.AddLink(root, ArticulationJointDesc::Fixed(m4x4::Translation(1.0f, 0.0f, 0.0f)), MovingLink());
			auto articulation = builder.Build();

			ExpectMotionNear(articulation.LinkVelocity(child), v8motion{v4{0, 0, 1, 0}, v4{0, 1, 0, 0}}, 1.0e-5f);
		}

		// Match every supported joint dimension against a pose finite difference of the ordered screw coordinates.
		PRUnitTestMethod(OrderedOneToSixDofKinematics, Quick)
		{
			for (int dof_count = 1; dof_count != 7; ++dof_count)
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFixedRoot({}, m4x4::Transform(v4::ZAxis(), 0.1f, v4{1, 2, 3, 1}));
				auto const child = builder.AddLink(root, OrderedJoint(dof_count), MovingLink());
				auto articulation = builder.Build();
				auto const pose0 = articulation.LinkToWorld(child);
				auto const velocity0 = articulation.LinkVelocity(child);

				// Advance only generalized position by a small interval so the pose derivative independently checks S(q)*qdot.
				auto position1 = std::vector<float>(articulation.JointPosition(child).begin(), articulation.JointPosition(child).end());
				auto const velocity = articulation.JointVelocity(child);
				constexpr auto elapsed_seconds = 1.0e-3f;
				for (int index = 0; index != dof_count; ++index)
					position1[index] += velocity[index] * elapsed_seconds;

				articulation.JointPosition(child, position1);
				auto const pose1 = articulation.LinkToWorld(child);
				auto const finite_difference = FiniteDifferenceVelocity(pose0, pose1, elapsed_seconds);
				ExpectMotionNear(velocity0, finite_difference, 7.0e-4f);
			}
		}

		// Reject malformed roots, transforms, axes, dimensions, and generalized state before they can contaminate dynamics.
		PRUnitTestMethod(InvalidTopologyFailsEarly, Quick)
		{
			auto builder = ArticulationBuilder{};
			PR_THROWS(builder.Build(), std::exception);

			auto const root = builder.AddFixedRoot({});
			PR_THROWS(builder.AddFixedRoot({}), std::exception);
			PR_THROWS(builder.AddFloatingRoot(MovingLink()), std::exception);

			auto invalid_count = ArticulationJointDesc::Revolute();
			invalid_count.m_dof_count = 7;
			PR_THROWS(builder.AddLink(root, invalid_count, MovingLink()), std::exception);

			auto invalid_axis = ArticulationJointDesc::Revolute(v4{2, 0, 0, 0});
			PR_THROWS(builder.AddLink(root, invalid_axis, MovingLink()), std::exception);

			auto invalid_transform = ArticulationJointDesc::Revolute();
			invalid_transform.m_joint_to_parent.x.x = 2.0f;
			PR_THROWS(builder.AddLink(root, invalid_transform, MovingLink()), std::exception);

			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(), MovingLink());
			auto articulation = builder.Build();
			PR_THROWS(articulation.JointPosition(child, std::array{0.0f, 1.0f}), std::exception);
			PR_THROWS(articulation.RootVelocity(v8motion{}), std::exception);
		}
	};
}
#endif
