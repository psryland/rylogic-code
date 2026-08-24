//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/unittests/articulation_oracle.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric finite mass properties with an offset centre of mass to expose spatial coupling errors.
		ArticulationLinkDesc DynamicLink(int seed, float mass = 1.0f)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.13f + 0.02f * scale, 0.21f + 0.015f * scale, 0.31f + 0.01f * scale, 0},
					mass,
					v4{0.017f * scale, -0.011f * scale, 0.009f * scale, 0}),
			};
		}

		// Return a well-conditioned ordered joint containing the requested number of scalar coordinates.
		ArticulationJointDesc DynamicJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.07f * scale, v4{0.2f * scale, -0.1f, 0.08f, 1}),
				m4x4::Transform(v4::XAxis(), -0.04f * scale, v4{-0.06f, 0.09f * scale, -0.12f, 1}));
			joint.m_dof_count = dof_count;
			joint.m_axes = {
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = Normalise(v4{1, 1, 0, 0})},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::ZAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = Normalise(v4{0, 1, 1, 0})},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = Normalise(v4{1, 0, 1, 0})},
			};
			for (int axis_index = 0; axis_index != dof_count; ++axis_index)
			{
				joint.m_initial_position[axis_index] = 0.025f * scale * static_cast<float>(axis_index + 1);
				joint.m_initial_velocity[axis_index] = -0.31f + 0.09f * static_cast<float>(axis_index) + 0.015f * scale;
			}
			return joint;
		}

		// Return generalized velocities in the public root-then-topological-joint order.
		std::vector<double> GeneralizedVelocity(Articulation const& articulation)
		{
			auto result = std::vector<double>(articulation.DofCount(), 0.0);
			auto offset = 0;
			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto const velocity = articulation.RootVelocity();
					result[0] = velocity.ang.x;
					result[1] = velocity.ang.y;
					result[2] = velocity.ang.z;
					result[3] = velocity.lin.x;
					result[4] = velocity.lin.y;
					result[5] = velocity.lin.z;
					offset = 6;
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				for (auto value : articulation.JointVelocity(articulation.LinkAt(link_index)))
					result[offset++] = value;
			}
			return result;
		}

		// Return generalized accelerations in the public root-then-topological-joint order.
		std::vector<double> GeneralizedAcceleration(Articulation const& articulation)
		{
			auto result = std::vector<double>(articulation.DofCount(), 0.0);
			auto offset = 0;
			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto const acceleration = articulation.RootAcceleration();
					result[0] = acceleration.ang.x;
					result[1] = acceleration.ang.y;
					result[2] = acceleration.ang.z;
					result[3] = acceleration.lin.x;
					result[4] = acceleration.lin.y;
					result[5] = acceleration.lin.z;
					offset = 6;
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				for (auto value : articulation.JointAcceleration(articulation.LinkAt(link_index)))
					result[offset++] = value;
			}
			return result;
		}

		// Require two generalized vectors to agree under a mixed absolute and relative tolerance.
		void ExpectGeneralizedNear(std::span<double const> actual, std::span<double const> expected, double tolerance)
		{
			PR_EXPECT(actual.size() == expected.size());
			for (int index = 0; index != isize(actual); ++index)
			{
				auto const scale = std::max({1.0, std::abs(actual[index]), std::abs(expected[index])});
				PR_EXPECT(std::abs(actual[index] - expected[index]) <= tolerance * scale);
			}
		}

		// Return a deterministic link-frame wrench with both torque and force components.
		v8force TestWrench(float scale)
		{
			return v8force{
				v4{+0.17f * scale, -0.23f * scale, +0.11f * scale, 0},
				v4{-0.31f * scale, +0.19f * scale, +0.29f * scale, 0},
			};
		}
	}

	PRUnitTestClass(ArticulationDynamicsTests)
	{
		// Match one-to-six-DOF fixed-base joints against an independent dense inverse-dynamics solution.
		PRUnitTestMethod(FixedBaseOneToSixDofMatchesOracle, Quick)
		{
			for (int dof_count = 1; dof_count != 7; ++dof_count)
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFixedRoot({});
				auto const child = builder.AddLink(root, DynamicJoint(dof_count, dof_count), DynamicLink(dof_count, 1.7f));
				auto articulation = builder.Build();

				auto force = std::vector<float>(dof_count);
				for (int axis_index = 0; axis_index != dof_count; ++axis_index)
					force[axis_index] = -0.41f + 0.16f * static_cast<float>(axis_index);
				articulation.JointForce(child, force);
				articulation.ExternalForce(child, TestWrench(0.7f));

				auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
				articulation.ForwardDynamics();
				ExpectGeneralizedNear(GeneralizedAcceleration(articulation), expected.m_acceleration, 5.0e-4);
			}
		}

		// Match a moving depth-two fixed-base chain so recursive bias and force propagation cannot cancel inside a single link.
		PRUnitTestMethod(FixedBaseChainMatchesOracle, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot({});
			auto const child = builder.AddLink(root, DynamicJoint(2, 1), DynamicLink(1, 1.4f));
			auto const tip = builder.AddLink(child, DynamicJoint(3, 2), DynamicLink(2, 0.8f));
			auto articulation = builder.Build();
			articulation.JointForce(child, std::array{+0.23f, -0.17f});
			articulation.JointForce(tip, std::array{-0.31f, +0.29f, +0.11f});
			articulation.ExternalForce(tip, TestWrench(0.6f));

			auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
			articulation.ForwardDynamics();
			ExpectGeneralizedNear(GeneralizedAcceleration(articulation), expected.m_acceleration, 8.0e-4);
		}

		// Match a floating, branching, asymmetric tree with fixed welds, generalized loading, and external link wrenches.
		PRUnitTestMethod(FloatingBranchingTreeMatchesOracle, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				DynamicLink(0, 3.4f),
				m4x4::Transform(v4::ZAxis(), 0.4f, v4{2, -1, 3, 1}),
				v8motion{v4{0.37f, -0.29f, 0.41f, 0}, v4{-0.17f, 0.23f, 0.31f, 0}});
			auto const branch_a = builder.AddLink(root, DynamicJoint(2, 1), DynamicLink(1, 1.2f));
			auto const branch_b = builder.AddLink(root, DynamicJoint(3, 2), DynamicLink(2, 0.9f));
			auto const welded = builder.AddLink(branch_a, ArticulationJointDesc::Fixed(m4x4::Translation(0.3f, -0.2f, 0.1f)), DynamicLink(3, 0.6f));
			auto const tip = builder.AddLink(branch_b, DynamicJoint(1, 4), DynamicLink(4, 0.4f));
			auto articulation = builder.Build();

			articulation.RootForce(TestWrench(0.8f));
			articulation.JointForce(branch_a, std::array{+0.31f, -0.27f});
			articulation.JointForce(branch_b, std::array{-0.19f, +0.13f, +0.37f});
			articulation.JointForce(tip, std::array{-0.43f});
			articulation.ExternalForce(root, TestWrench(-0.4f));
			articulation.ExternalForce(welded, TestWrench(+0.9f));
			articulation.ExternalForce(tip, TestWrench(-0.6f));

			auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
			articulation.ForwardDynamics();
			ExpectGeneralizedNear(GeneralizedAcceleration(articulation), expected.m_acceleration, 8.0e-4);
		}

		// Match the uncoupled floating-root gyroscopic solve before child-link reductions are introduced.
		PRUnitTestMethod(FloatingRootMatchesOracle, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				DynamicLink(2, 2.3f),
				m4x4::Identity(),
				v8motion{v4{1.7f, -0.8f, 1.1f, 0}, v4{0.4f, -0.6f, 0.2f, 0}});
			auto articulation = builder.Build();
			articulation.RootForce(TestWrench(0.3f));
			articulation.ExternalForce(root, TestWrench(-0.2f));

			auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
			articulation.ForwardDynamics();
			ExpectGeneralizedNear(GeneralizedAcceleration(articulation), expected.m_acceleration, 5.0e-4);
		}

		// Preserve Coriolis and gyroscopic bias rather than degenerating to a velocity-independent mass solve.
		PRUnitTestMethod(ZeroForceRetainsVelocityBias, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				DynamicLink(2, 2.3f),
				m4x4::Identity(),
				v8motion{v4{1.7f, -0.8f, 1.1f, 0}, v4{0.4f, -0.6f, 0.2f, 0}});
			auto const child = builder.AddLink(root, DynamicJoint(3, 3), DynamicLink(4, 1.1f));
			auto const tip = builder.AddLink(child, DynamicJoint(2, 5), DynamicLink(6, 0.7f));
			auto articulation = builder.Build();

			auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
			articulation.ForwardDynamics();
			auto const actual = GeneralizedAcceleration(articulation);
			ExpectGeneralizedNear(actual, expected.m_acceleration, 8.0e-4);
			PR_EXPECT(std::ranges::any_of(actual, [](double value) { return std::abs(value) > 1.0e-3; }));
		}

		// Apply simultaneous link impulses through the same global articulated response as the dense mass-matrix oracle.
		PRUnitTestMethod(BatchedImpulseResponseMatchesOracle, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				DynamicLink(0, 4.0f),
				m4x4::Identity(),
				v8motion{v4{0.2f, -0.1f, 0.3f, 0}, v4{-0.4f, 0.2f, 0.1f, 0}});
			auto const child = builder.AddLink(root, DynamicJoint(2, 1), DynamicLink(2, 1.3f));
			auto const tip = builder.AddLink(child, DynamicJoint(3, 2), DynamicLink(4, 0.8f));
			auto articulation = builder.Build();
			auto const impulses = std::array{
				ArticulationImpulse{.m_link = root, .m_impulse = TestWrench(+0.7f)},
				ArticulationImpulse{.m_link = child, .m_impulse = TestWrench(-0.4f)},
				ArticulationImpulse{.m_link = tip, .m_impulse = TestWrench(+1.1f)},
			};

			auto const velocity_before = GeneralizedVelocity(articulation);
			auto const expected_delta = articulation_oracle::SolveImpulseResponse(articulation, impulses);
			articulation.ApplyImpulses(impulses);
			auto velocity_delta = GeneralizedVelocity(articulation);
			for (int index = 0; index != isize(velocity_delta); ++index)
				velocity_delta[index] -= velocity_before[index];

			ExpectGeneralizedNear(velocity_delta, expected_delta, 8.0e-4);
		}

		// Reject a rank-deficient reduced joint instead of emitting unbounded or non-finite acceleration.
		PRUnitTestMethod(SingularJointFailsExplicitly, Quick)
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];

			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot({});
			builder.AddLink(root, joint, DynamicLink(1));
			auto articulation = builder.Build();
			PR_THROWS(articulation.ForwardDynamics(), std::exception);
		}

		// Validate force and impulse inputs before they can enter persistent articulation state.
		PRUnitTestMethod(InvalidDynamicsInputFailsEarly, Quick)
		{
			auto builder_a = ArticulationBuilder{};
			auto builder_b = ArticulationBuilder{};
			auto const root_a = builder_a.AddFloatingRoot(DynamicLink(0));
			auto const root_b = builder_b.AddFloatingRoot(DynamicLink(1));
			auto articulation_a = builder_a.Build();
			auto articulation_b = builder_b.Build();

			PR_THROWS(articulation_a.ExternalForce(root_a, v8force{v4{NAN, 0, 0, 0}, v4{}}), std::exception);
			PR_THROWS(articulation_a.ApplyImpulse(root_b, TestWrench(1.0f)), std::exception);
			PR_THROWS(articulation_b.RootForce(v8force{v4{}, v4{0, INFINITY, 0, 0}}), std::exception);
		}
	};
}
#endif
