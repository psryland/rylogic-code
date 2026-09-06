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
		// Return a finite asymmetric link inertia with an offset centre of mass.
		ArticulationLinkDesc ConservationLink(v4 half_extents, float mass, v4 centre_of_mass)
		{
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(half_extents, mass, centre_of_mass),
			};
		}

		// Return a revolute joint with explicit parent and child attachment frames.
		ArticulationJointDesc RevoluteJoint(v4 axis, m4x4 const& joint_to_parent, m4x4 const& joint_to_child, float position, float velocity)
		{
			auto joint = ArticulationJointDesc::Revolute(axis, joint_to_parent, joint_to_child);
			joint.m_initial_position[0] = position;
			joint.m_initial_velocity[0] = velocity;
			return joint;
		}

		// Build a free asymmetric three-link tree with internal motion and no external force.
		Articulation BuildConservationArticulation()
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				ConservationLink(v4{0.45f, 0.63f, 0.82f, 0}, 3.1f, v4{0.08f, -0.05f, 0.04f, 0}),
				m4x4::Transform(v4::YAxis(), 0.31f, v4{1.7f, -0.8f, 2.1f, 1}),
				v8motion{v4{0.71f, -0.46f, 0.58f, 0}, v4{-0.23f, 0.37f, 0.19f, 0}});
			auto const child = builder.AddLink(
				root,
				RevoluteJoint(
					Normalise(v4{1, 2, -1, 0}),
					m4x4::Translation(+0.62f, -0.11f, +0.17f),
					m4x4::Translation(-0.31f, +0.09f, -0.14f),
					0.43f,
					1.17f),
				ConservationLink(v4{0.27f, 0.39f, 0.51f, 0}, 1.4f, v4{-0.04f, 0.07f, 0.03f, 0}));
			builder.AddLink(
				child,
				RevoluteJoint(
					Normalise(v4{-1, 1, 2, 0}),
					m4x4::Translation(-0.23f, +0.41f, +0.12f),
					m4x4::Translation(+0.18f, -0.26f, +0.08f),
					-0.36f,
					-0.83f),
				ConservationLink(v4{0.19f, 0.28f, 0.34f, 0}, 0.8f, v4{0.03f, 0.02f, -0.05f, 0}));
			return builder.Build();
		}

		// Long-run invariant errors and least-squares relative energy trend.
		struct ConservationMetrics
		{
			float m_max_linear_momentum_error = 0.0f;
			float m_max_angular_momentum_error = 0.0f;
			float m_max_energy_error = 0.0f;
			double m_energy_slope = 0.0;
		};

		// Measure invariant envelopes and secular energy trend over a force-free trajectory.
		ConservationMetrics MeasureConservation(float timestep, float duration)
		{
			auto articulation = BuildConservationArticulation();
			auto const momentum_start = articulation.MomentumWS();
			auto const energy_start = articulation.KineticEnergy();
			auto const linear_scale = std::max(Length(momentum_start.lin), 1.0f);
			auto const angular_scale = std::max(Length(momentum_start.ang), 1.0f);
			auto const energy_scale = std::max(Abs(energy_start), 1.0f);
			auto metrics = ConservationMetrics{};
			auto sample_count = 0.0;
			auto sum_time = 0.0;
			auto sum_error = 0.0;
			auto sum_time_sq = 0.0;
			auto sum_time_error = 0.0;

			auto const step_count = static_cast<int>(std::round(duration / timestep));
			for (int step = 0; step != step_count; ++step)
			{
				articulation.Integrate(timestep);

				// Track all invariants every step so oscillatory errors cannot hide between sparse samples.
				auto const time = static_cast<double>(step + 1) * timestep;
				auto const momentum = articulation.MomentumWS();
				auto const relative_energy_error = static_cast<double>(articulation.KineticEnergy() - energy_start) / energy_scale;
				metrics.m_max_linear_momentum_error = std::max(metrics.m_max_linear_momentum_error, Length(momentum.lin - momentum_start.lin) / linear_scale);
				metrics.m_max_angular_momentum_error = std::max(metrics.m_max_angular_momentum_error, Length(momentum.ang - momentum_start.ang) / angular_scale);
				metrics.m_max_energy_error = std::max(metrics.m_max_energy_error, static_cast<float>(std::abs(relative_energy_error)));
				sample_count += 1.0;
				sum_time += time;
				sum_error += relative_energy_error;
				sum_time_sq += time * time;
				sum_time_error += time * relative_energy_error;
			}

			// A positive least-squares slope reveals secular gain even when the bounded error oscillates around zero.
			auto const denominator = sample_count * sum_time_sq - sum_time * sum_time;
			metrics.m_energy_slope = denominator != 0.0
				? (sample_count * sum_time_error - sum_time * sum_error) / denominator
				: 0.0;
			return metrics;
		}

		// Advance one fresh articulation to a common endpoint for timestep-convergence comparisons.
		Articulation Simulate(float timestep, float duration)
		{
			auto articulation = BuildConservationArticulation();
			auto const step_count = static_cast<int>(std::round(duration / timestep));
			for (int step = 0; step != step_count; ++step)
				articulation.Integrate(timestep);

			return articulation;
		}

		// Return a scale-independent maximum state difference between equal-topology articulations.
		float StateDifference(Articulation const& lhs, Articulation const& rhs)
		{
			auto difference = 0.0f;
			auto const lhs_root = lhs.RootToWorld();
			auto const rhs_root = rhs.RootToWorld();
			for (int column = 0; column != 4; ++column)
				difference = std::max(difference, Length(lhs_root[column] - rhs_root[column]));
			difference = std::max(difference, Length(lhs.RootVelocity().ang - rhs.RootVelocity().ang));
			difference = std::max(difference, Length(lhs.RootVelocity().lin - rhs.RootVelocity().lin));

			// Stable topological order makes every reduced state range directly comparable.
			for (int link_index = 1; link_index != lhs.LinkCount(); ++link_index)
			{
				auto const lhs_link = lhs.LinkAt(link_index);
				auto const rhs_link = rhs.LinkAt(link_index);
				for (int axis_index = 0; axis_index != lhs.JointDofCount(lhs_link); ++axis_index)
				{
					difference = std::max(difference, Abs(lhs.JointPosition(lhs_link)[axis_index] - rhs.JointPosition(rhs_link)[axis_index]));
					difference = std::max(difference, Abs(lhs.JointVelocity(lhs_link)[axis_index] - rhs.JointVelocity(rhs_link)[axis_index]));
				}
			}
			return difference;
		}
	}

	PRUnitTestClass(ArticulationConservationTests)
	{
		// Integrate applied force once, retain finite state, and clear the per-step force accumulators.
		PRUnitTestMethod(IntegrationConsumesForces, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot({});
			auto const child = builder.AddLink(
				root,
				RevoluteJoint(v4::ZAxis(), m4x4::Identity(), m4x4::Identity(), 0.2f, -0.3f),
				ConservationLink(v4{0.3f, 0.4f, 0.5f, 0}, 1.2f, v4{}));
			auto articulation = builder.Build();
			articulation.JointForce(child, std::array{0.7f});
			auto const position_start = articulation.JointPosition(child)[0];
			auto const velocity_start = articulation.JointVelocity(child)[0];

			articulation.Integrate(1.0f / 240.0f);

			PR_EXPECT(articulation.JointPosition(child)[0] != position_start);
			PR_EXPECT(articulation.JointVelocity(child)[0] != velocity_start);
			PR_EXPECT(articulation.JointForce(child)[0] == 0.0f);
			PR_EXPECT(IsFinite(articulation.KineticEnergy()));
		}

		// Reject invalid timesteps without changing accepted articulation state.
		PRUnitTestMethod(InvalidTimestepPreservesState, Quick)
		{
			auto articulation = BuildConservationArticulation();
			auto const transform_start = articulation.RootToWorld();
			auto const velocity_start = articulation.RootVelocity();
			PR_THROWS(articulation.Integrate(-0.01f), std::exception);
			PR_THROWS(articulation.Integrate(std::numeric_limits<float>::quiet_NaN()), std::exception);
			PR_EXPECT(FEql(articulation.RootToWorld(), transform_start));
			PR_EXPECT(FEql(articulation.RootVelocity(), velocity_start));
		}

		// Restore accepted generalized, pose, and acceleration outputs after a nonlinear midpoint solve fails.
		PRUnitTestMethod(MidpointFailureRollsBackState, Quick)
		{
			auto articulation = BuildConservationArticulation();
			articulation.ForwardDynamics();
			auto const accepted = BuildConservationArticulation();
			auto const root_acceleration = articulation.RootAcceleration();
			auto link_acceleration = std::vector<v8motion>(articulation.LinkCount());
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				link_acceleration[link_index] = articulation.LinkAcceleration(articulation.LinkAt(link_index));

			PR_THROWS(articulation.Integrate(100.0f), std::exception);

			PR_EXPECT(StateDifference(articulation, accepted) == 0.0f);
			PR_EXPECT(FEql(articulation.RootAcceleration(), root_acceleration));
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				PR_EXPECT(FEql(articulation.LinkAcceleration(articulation.LinkAt(link_index)), link_acceleration[link_index]));

			// A failed large step must leave the articulation usable for a smaller retry.
			articulation.Integrate(1.0f / 1000.0f);
			PR_EXPECT(IsFinite(articulation.KineticEnergy()));
		}

		// Preserve force-free world momentum and bounded energy without a positive secular trend.
		PRUnitTestMethod(FreeFloatingConservation, Extended)
		{
			auto const metrics = MeasureConservation(1.0f / 1000.0f, 5.0f);
			PR_EXPECT(metrics.m_max_linear_momentum_error < 1.0e-4f);
			PR_EXPECT(metrics.m_max_angular_momentum_error < 1.0e-4f);
			PR_EXPECT(metrics.m_max_energy_error < 2.0e-5f);
			PR_EXPECT(metrics.m_energy_slope < 1.0e-5);
		}

		// Require trajectory error to decrease when the implicit-midpoint timestep is halved.
		PRUnitTestMethod(TimestepConvergence, Extended)
		{
			auto const coarse = Simulate(1.0f / 30.0f, 1.0f);
			auto const medium = Simulate(1.0f / 60.0f, 1.0f);
			auto const fine = Simulate(1.0f / 120.0f, 1.0f);
			auto const coarse_error = StateDifference(coarse, fine);
			auto const medium_error = StateDifference(medium, fine);
			PR_EXPECT(medium_error < 0.25f * coarse_error);
		}
	};
}
#endif
