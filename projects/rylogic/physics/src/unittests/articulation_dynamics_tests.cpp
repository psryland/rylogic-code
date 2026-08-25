//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_internal.h"
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

		// Own a detached generalized result and expose the non-owning view accepted by the GPU commit boundary.
		struct CapturedIntegrationOutput
		{
			m4x4 m_root_to_world;
			std::vector<float> m_positions;
			std::vector<float> m_velocities;
			std::vector<float> m_accelerations;

			// Return a view whose spans remain valid for this capture's lifetime.
			detail::ArticulationIntegrationOutput View(float substep_seconds) const
			{
				return detail::ArticulationIntegrationOutput{
					.m_root_to_world = m_root_to_world,
					.m_positions = m_positions,
					.m_velocities = m_velocities,
					.m_accelerations = m_accelerations,
					.m_substep_seconds = substep_seconds,
				};
			}
		};

		// Capture public articulation state in the same root-then-topological order used by GPU buffers.
		CapturedIntegrationOutput CaptureIntegrationOutput(Articulation const& articulation)
		{
			auto output = CapturedIntegrationOutput{
				.m_root_to_world = articulation.RootToWorld(),
			};
			output.m_positions.reserve(articulation.DofCount());
			output.m_velocities.reserve(articulation.DofCount());
			output.m_accelerations.reserve(articulation.DofCount());

			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					auto const velocity = articulation.RootVelocity();
					auto const acceleration = articulation.RootAcceleration();
					output.m_velocities.insert(output.m_velocities.end(), {
						velocity.ang.x, velocity.ang.y, velocity.ang.z,
						velocity.lin.x, velocity.lin.y, velocity.lin.z,
					});
					output.m_accelerations.insert(output.m_accelerations.end(), {
						acceleration.ang.x, acceleration.ang.y, acceleration.ang.z,
						acceleration.lin.x, acceleration.lin.y, acceleration.lin.z,
					});
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}

			// Reduced coordinates follow builder order after the optional floating-root range.
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				auto const position = articulation.JointPosition(link);
				auto const velocity = articulation.JointVelocity(link);
				auto const acceleration = articulation.JointAcceleration(link);
				output.m_positions.insert(output.m_positions.end(), position.begin(), position.end());
				output.m_velocities.insert(output.m_velocities.end(), velocity.begin(), velocity.end());
				output.m_accelerations.insert(output.m_accelerations.end(), acceleration.begin(), acceleration.end());
			}
			return output;
		}

		// Build a moving floating tree whose final midpoint differs from the frame-start midpoint after multiple substeps.
		Articulation BuildIntegrationCommitFixture()
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				DynamicLink(0, 2.8f),
				m4x4::Transform(v4::ZAxis(), 0.31f, v4{0.7f, -1.2f, 0.4f, 1}),
				v8motion{v4{0.43f, -0.27f, 0.38f, 0}, v4{-0.14f, 0.21f, 0.33f, 0}});
			auto const branch = builder.AddLink(root, DynamicJoint(2, 2), DynamicLink(2, 1.1f));
			builder.AddLink(root, DynamicJoint(1, 3), DynamicLink(3, 0.8f));
			builder.AddLink(branch, DynamicJoint(3, 4), DynamicLink(4, 0.6f));
			return builder.Build();
		}

		// Reapply one frame-constant load image because CPU integration consumes forces after each internal substep.
		void ApplyIntegrationCommitLoads(Articulation& articulation)
		{
			articulation.RootForce(TestWrench(+0.45f));
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				articulation.ExternalForce(link, TestWrench(-0.18f + 0.11f * static_cast<float>(link_index)));
				if (link_index == 0)
					continue;

				auto force = std::array<float, 6>{};
				for (int axis_index = 0; axis_index != articulation.JointDofCount(link); ++axis_index)
					force[axis_index] = 0.13f * static_cast<float>((link_index + 1) * (axis_index + 1)) - 0.21f;
				articulation.JointForce(link, std::span{force}.first(articulation.JointDofCount(link)));
			}
		}

		// Require two spatial motions to agree under a component-wise mixed tolerance.
		void ExpectSpatialNear(v8motion actual, v8motion expected, float tolerance)
		{
			auto const actual_values = std::array{actual.ang.x, actual.ang.y, actual.ang.z, actual.lin.x, actual.lin.y, actual.lin.z};
			auto const expected_values = std::array{expected.ang.x, expected.ang.y, expected.ang.z, expected.lin.x, expected.lin.y, expected.lin.z};
			for (int component = 0; component != isize(actual_values); ++component)
			{
				auto const scale = std::max({1.0f, Abs(actual_values[component]), Abs(expected_values[component])});
				PR_EXPECT(Abs(actual_values[component] - expected_values[component]) <= tolerance * scale);
			}
		}

		// Require two transforms to agree component-wise under an absolute tolerance.
		void ExpectTransformNear(m4x4 const& actual, m4x4 const& expected, float tolerance)
		{
			if (tolerance == 0.0f)
			{
				PR_EXPECT(std::memcmp(&actual, &expected, sizeof(actual)) == 0);
				return;
			}

			PR_EXPECT(FEqlAbsolute(actual.x, expected.x, tolerance));
			PR_EXPECT(FEqlAbsolute(actual.y, expected.y, tolerance));
			PR_EXPECT(FEqlAbsolute(actual.z, expected.z, tolerance));
			PR_EXPECT(FEqlAbsolute(actual.w, expected.w, tolerance));
		}

		// Require every production link acceleration to agree with the independent spatial recursion.
		void ExpectLinkAccelerationsNear(Articulation const& articulation, std::span<std::array<double, 6> const> expected, double tolerance)
		{
			PR_EXPECT(articulation.LinkCount() == isize(expected));
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const actual = articulation.LinkAcceleration(articulation.LinkAt(link_index));
				auto const actual_values = std::array<double, 6>{
					actual.ang.x, actual.ang.y, actual.ang.z,
					actual.lin.x, actual.lin.y, actual.lin.z,
				};
				for (int component = 0; component != 6; ++component)
				{
					auto const scale = std::max({1.0, std::abs(actual_values[component]), std::abs(expected[link_index][component])});
					PR_EXPECT(std::abs(actual_values[component] - expected[link_index][component]) <= tolerance * scale);
				}
			}
		}

		// A deterministic generator keeps randomized articulation coverage reproducible.
		struct RandomSource
		{
			uint64_t m_state;

			// Return a deterministic float in [-1,+1).
			float NextSigned()
			{
				m_state ^= m_state << 13;
				m_state ^= m_state >> 7;
				m_state ^= m_state << 17;
				auto const unit = static_cast<double>(m_state >> 11) * (1.0 / 9007199254740992.0);
				return static_cast<float>(2.0 * unit - 1.0);
			}
		};
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
			ExpectLinkAccelerationsNear(articulation, expected.m_link_acceleration, 8.0e-4);
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
			ExpectLinkAccelerationsNear(articulation, expected.m_link_acceleration, 8.0e-4);
		}

		// World-space per-link gravity is reevaluated in current link frames and remains persistent when transient loads are cleared.
		PRUnitTestMethod(LinkGravityMatchesEquivalentExternalWrenches, Quick)
		{
			auto build = []()
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(
					DynamicLink(0, 2.8f),
					m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.43f, v4{0.7f, -0.5f, 1.2f, 1}),
					v8motion{v4{0.19f, -0.23f, 0.17f, 0}, v4{-0.11f, 0.07f, 0.13f, 0}});
				auto const child = builder.AddLink(root, DynamicJoint(2, 3), DynamicLink(2, 1.1f));
				builder.AddLink(child, DynamicJoint(1, 5), DynamicLink(4, 0.6f));
				return builder.Build();
			};
			auto gravity_driven = build();
			auto wrench_driven = build();
			auto const gravity_values = std::array{
				v4{+0.3f, -0.5f, -9.4f, 0},
				v4{-0.7f, +0.2f, -8.8f, 0},
				v4{+0.4f, +0.6f, -9.1f, 0},
			};

			// Express each equivalent load in its current link frame at the link origin.
			for (int link_index = 0; link_index != gravity_driven.LinkCount(); ++link_index)
			{
				auto const gravity_link = gravity_driven.LinkAt(link_index);
				auto const wrench_link = wrench_driven.LinkAt(link_index);
				auto const gravity_ws = gravity_values[link_index];
				gravity_driven.GravityWS(gravity_link, gravity_ws);
				auto const& inertia = wrench_driven.LinkDescription(wrench_link).m_inertia;
				auto const force_link = InvertOrthonormal(wrench_driven.LinkToWorld(wrench_link).rot) * (inertia.Mass() * gravity_ws);
				wrench_driven.ExternalForce(wrench_link, v8force{Cross(inertia.CoM(), force_link), force_link});
			}

			gravity_driven.ForwardDynamics();
			wrench_driven.ForwardDynamics();
			auto expected_link_accelerations = std::vector<std::array<double, 6>>{};
			expected_link_accelerations.reserve(wrench_driven.LinkCount());
			for (int link_index = 0; link_index != wrench_driven.LinkCount(); ++link_index)
			{
				auto const acceleration = wrench_driven.LinkAcceleration(wrench_driven.LinkAt(link_index));
				expected_link_accelerations.push_back({
					acceleration.ang.x,
					acceleration.ang.y,
					acceleration.ang.z,
					acceleration.lin.x,
					acceleration.lin.y,
					acceleration.lin.z,
				});
			}

			ExpectGeneralizedNear(GeneralizedAcceleration(gravity_driven), GeneralizedAcceleration(wrench_driven), 8.0e-4);
			ExpectLinkAccelerationsNear(gravity_driven, expected_link_accelerations, 8.0e-4);

			gravity_driven.ClearForces();
			for (int link_index = 0; link_index != gravity_driven.LinkCount(); ++link_index)
				PR_EXPECT(FEql(gravity_driven.GravityWS(gravity_driven.LinkAt(link_index)), gravity_values[link_index]));
			PR_THROWS(gravity_driven.GravityWS(gravity_driven.Root(), v4{NAN, 0, 0, 0}), std::exception);
		}

		// Sleep state belongs to the complete tree: sleeping clears every link's motion, while any non-zero load or impulse wakes all links together.
		PRUnitTestMethod(CompleteTreeSleepOwnership, Quick)
		{
			auto joint = DynamicJoint(2, 1);
			joint.m_initial_velocity[0] = 0.0f;
			joint.m_initial_velocity[1] = 0.0f;
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(DynamicLink(0, 2.0f));
			auto const child = builder.AddLink(root, joint, DynamicLink(1, 1.0f));
			auto articulation = builder.Build();

			// Sleeping atomically discards root, joint, and link motion together with transient loads.
			articulation.RootVelocity(v8motion{v4{0.2f, -0.1f, 0.3f, 0}, v4{-0.4f, 0.5f, -0.2f, 0}});
			articulation.JointVelocity(child, std::array{0.7f, -0.6f});
			articulation.RootForce(TestWrench(0.4f));
			articulation.JointForce(child, std::array{0.3f, -0.2f});
			articulation.ExternalForce(child, TestWrench(-0.5f));
			articulation.Sleep();
			PR_EXPECT(articulation.Sleeping());
			PR_EXPECT(FEql(articulation.RootVelocity(), v8motion{}));
			PR_EXPECT(std::ranges::all_of(articulation.JointVelocity(child), [](float value) { return value == 0.0f; }));
			PR_EXPECT(FEql(articulation.RootForce(), v8force{}));
			PR_EXPECT(std::ranges::all_of(articulation.JointForce(child), [](float value) { return value == 0.0f; }));
			PR_EXPECT(FEql(articulation.ExternalForce(child), v8force{}));
			PR_EXPECT(FEql(articulation.LinkVelocity(root), v8motion{}));
			PR_EXPECT(FEql(articulation.LinkVelocity(child), v8motion{}));

			// Environmental field changes, direct loads, and projected impulses all wake the same complete tree.
			auto const gravity = v4{0.0f, 0.0f, -9.8f, 0.0f};
			articulation.GravityWS(child, gravity);
			PR_EXPECT(!articulation.Sleeping());
			articulation.Sleep();
			articulation.GravityWS(child, gravity);
			PR_EXPECT(articulation.Sleeping());
			articulation.ExternalForce(child, TestWrench(0.2f));
			PR_EXPECT(!articulation.Sleeping());
			articulation.Sleep();
			articulation.ApplyImpulse(child, TestWrench(-0.3f));
			PR_EXPECT(!articulation.Sleeping());

			// Never-sleep mode is tree-wide and wakes an already sleeping articulation.
			articulation.Sleep();
			articulation.NeverSleep(true);
			PR_EXPECT(articulation.NeverSleep());
			PR_EXPECT(!articulation.Sleeping());
			articulation.NeverSleep(false);
			articulation.Sleeping(true);
			PR_EXPECT(articulation.Sleeping());
			articulation.Sleeping(false);
			PR_EXPECT(!articulation.Sleeping());
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

		// Match randomized fixed and floating trees with varied branching and one-to-six-DOF ordered joints.
		PRUnitTestMethod(RandomizedTreesMatchOracle, Quick)
		{
			auto random = RandomSource{0xD6E8FEB86659FD93ull};
			for (int trial = 0; trial != 12; ++trial)
			{
				auto builder = ArticulationBuilder{};
				auto handles = std::vector<LinkHandle>{};
				auto const root_rotation = v3{
					0.35f * random.NextSigned(),
					0.35f * random.NextSigned(),
					0.35f * random.NextSigned(),
				};
				auto const root_transform = m4x4{
					m3x3::Rotation(root_rotation),
					v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 1},
				};
				if ((trial & 1) == 0)
				{
					handles.push_back(builder.AddFixedRoot({}, root_transform));
				}
				else
				{
					auto const root_velocity = v8motion{
						v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0},
						v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0},
					};
					handles.push_back(builder.AddFloatingRoot(DynamicLink(trial, 1.0f + std::abs(random.NextSigned())), root_transform, root_velocity));
				}

				// Parent selection spans chains and branches while preserving builder topological order.
				auto const child_count = 2 + trial % 5;
				for (int child_index = 0; child_index != child_count; ++child_index)
				{
					auto const dof_count = 1 + (trial + 2 * child_index) % 6;
					auto joint = DynamicJoint(dof_count, trial + child_index);
					for (int axis_index = 0; axis_index != dof_count; ++axis_index)
					{
						joint.m_initial_position[axis_index] = 0.3f * random.NextSigned();
						joint.m_initial_velocity[axis_index] = 0.8f * random.NextSigned();
					}
					auto const parent_index = child_index == 0 ? 0 : (trial + child_index) % isize(handles);
					handles.push_back(builder.AddLink(
						handles[parent_index],
						joint,
						DynamicLink(trial * 7 + child_index, 0.4f + std::abs(random.NextSigned()))));
				}
				auto articulation = builder.Build();

				// Excite every force path so signs and transform directions cannot pass through cancellation.
				if (articulation.RootType() == EArticulationRootType::Floating)
					articulation.RootForce(TestWrench(random.NextSigned()));
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				{
					auto const link = articulation.LinkAt(link_index);
					articulation.ExternalForce(link, TestWrench(random.NextSigned()));
					if (link_index == 0)
						continue;

					auto force = std::array<float, 6>{};
					for (int axis_index = 0; axis_index != articulation.JointDofCount(link); ++axis_index)
						force[axis_index] = random.NextSigned();
					articulation.JointForce(link, std::span{force}.first(articulation.JointDofCount(link)));
				}

				auto const expected = articulation_oracle::SolveForwardDynamics(articulation);
				articulation.ForwardDynamics();
				ExpectGeneralizedNear(GeneralizedAcceleration(articulation), expected.m_acceleration, 2.0e-3);
				ExpectLinkAccelerationsNear(articulation, expected.m_link_acceleration, 2.0e-3);
			}
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

		// Reconstruct the last internal substep's midpoint accelerations while committing only compact generalized GPU output.
		PRUnitTestMethod(GpuIntegrationOutputCommitsTransactionally, Quick)
		{
			auto reference = BuildIntegrationCommitFixture();
			auto target = BuildIntegrationCommitFixture();
			constexpr auto substep_seconds = 0.0025f;
			constexpr auto substep_count = 3;

			// CPU integration supplies the detached final state expected from several GPU-resident internal substeps.
			for (int substep = 0; substep != substep_count; ++substep)
			{
				ApplyIntegrationCommitLoads(reference);
				reference.Integrate(substep_seconds);
			}
			auto const output = CaptureIntegrationOutput(reference);
			auto expected_link_accelerations = std::vector<v8motion>{};
			expected_link_accelerations.reserve(reference.LinkCount());
			for (int link_index = 0; link_index != reference.LinkCount(); ++link_index)
				expected_link_accelerations.push_back(reference.LinkAcceleration(reference.LinkAt(link_index)));

			// Validation is side-effect free; commit consumes the target's frame force image only after all output has passed.
			ApplyIntegrationCommitLoads(target);
			detail::ValidateArticulationIntegrationOutput(target, output.View(substep_seconds));
			detail::CommitArticulationIntegrationOutput(target, output.View(substep_seconds));
			ExpectTransformNear(target.RootToWorld(), reference.RootToWorld(), 1.0e-5f);
			ExpectSpatialNear(target.RootVelocity(), reference.RootVelocity(), 1.0e-6f);
			ExpectSpatialNear(target.RootAcceleration(), reference.RootAcceleration(), 1.0e-6f);
			PR_EXPECT(target.RootForce() == v8force{});
			for (int link_index = 0; link_index != target.LinkCount(); ++link_index)
			{
				auto const target_link = target.LinkAt(link_index);
				auto const reference_link = reference.LinkAt(link_index);
				PR_EXPECT(std::ranges::equal(target.JointPosition(target_link), reference.JointPosition(reference_link)));
				PR_EXPECT(std::ranges::equal(target.JointVelocity(target_link), reference.JointVelocity(reference_link)));
				PR_EXPECT(std::ranges::equal(target.JointAcceleration(target_link), reference.JointAcceleration(reference_link)));
				PR_EXPECT(target.ExternalForce(target_link) == v8force{});
				ExpectSpatialNear(target.LinkAcceleration(target_link), expected_link_accelerations[link_index], 5.0e-5f);
			}
		}

		// Reject malformed detached output before any caller-owned articulation state or force is changed.
		PRUnitTestMethod(GpuIntegrationOutputRejectsInvalidData, Quick)
		{
			auto articulation = BuildIntegrationCommitFixture();
			ApplyIntegrationCommitLoads(articulation);
			auto output = CaptureIntegrationOutput(articulation);
			auto const root_before = articulation.RootToWorld();
			auto const velocity_before = articulation.RootVelocity();

			output.m_accelerations.front() = std::numeric_limits<float>::quiet_NaN();
			PR_THROWS(detail::ValidateArticulationIntegrationOutput(articulation, output.View(0.01f)), std::exception);
			PR_THROWS(detail::CommitArticulationIntegrationOutput(articulation, output.View(0.01f)), std::exception);
			ExpectTransformNear(articulation.RootToWorld(), root_before, 0.0f);
			ExpectSpatialNear(articulation.RootVelocity(), velocity_before, 0.0f);
			PR_EXPECT(articulation.RootForce() != v8force{});

			output = CaptureIntegrationOutput(articulation);
			output.m_positions.pop_back();
			PR_THROWS(detail::ValidateArticulationIntegrationOutput(articulation, output.View(0.01f)), std::exception);
			ExpectTransformNear(articulation.RootToWorld(), root_before, 0.0f);
			ExpectSpatialNear(articulation.RootVelocity(), velocity_before, 0.0f);
		}
	};
}
#endif
