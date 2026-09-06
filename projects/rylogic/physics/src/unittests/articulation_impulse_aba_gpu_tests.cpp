//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/interop/articulation_impulse_aba_runner.h"
#include "src/unittests/articulation_oracle.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric finite mass properties so every spatial response coupling remains observable.
		ArticulationLinkDesc ImpulseLink(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.19f + 0.011f * scale, 0.23f + 0.008f * scale, 0.29f + 0.006f * scale, 0},
					0.8f + 0.17f * scale,
					v4{0.009f * scale, -0.007f * scale, 0.005f * scale, 0}),
			};
		}

		// Return one ordered zero-to-six-DOF joint with observable configuration and velocity.
		ArticulationJointDesc ImpulseJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.037f * scale, v4{0.13f * scale, -0.04f, 0.07f, 1}),
				m4x4::Transform(v4::XAxis(), -0.029f * scale, v4{-0.05f, 0.03f * scale, -0.08f, 1}));
			joint.m_dof_count = dof_count;
			joint.m_axes = {
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::YAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::ZAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::XAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::YAxis()},
				ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = v4::ZAxis()},
			};
			for (int axis_index = 0; axis_index != dof_count; ++axis_index)
			{
				joint.m_initial_position[axis_index] = 0.031f * scale * static_cast<float>(axis_index + 1);
				joint.m_initial_velocity[axis_index] = -0.14f + 0.041f * static_cast<float>(axis_index) + 0.008f * scale;
			}
			return joint;
		}

		// Build one deterministic branching tree containing every supported bounded joint dimension.
		Articulation BuildImpulseTree(EArticulationRootType root_type, int seed)
		{
			auto builder = ArticulationBuilder{};
			auto links = std::vector<LinkHandle>{};
			auto const root_to_world = m4x4::Transform(
				Normalise(v4{1, -2, 3, 0}),
				0.13f + 0.02f * seed,
				v4{0.3f * seed, -0.2f, 0.5f, 1});
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					links.push_back(builder.AddFixedRoot(ImpulseLink(seed), root_to_world));
					break;
				}
				case EArticulationRootType::Floating:
				{
					links.push_back(builder.AddFloatingRoot(
						ImpulseLink(seed),
						root_to_world,
						v8motion{v4{0.17f, -0.09f, 0.24f, 0}, v4{-0.31f, 0.18f, 0.07f, 0}}));
					break;
				}
				default:
				{
					throw std::invalid_argument("Impulse ABA test root type is invalid");
				}
			}

			for (int link_index = 1; link_index != 8; ++link_index)
			{
				auto const parent_index = link_index == 1 ? 0 : (link_index - 2) / 2;
				links.push_back(builder.AddLink(
					links[parent_index],
					ImpulseJoint(link_index - 1, seed + link_index),
					ImpulseLink(seed + link_index)));
			}
			return builder.Build();
		}

		// Return one finite link-frame impulse with no symmetric or cancelling components.
		v8force ImpulseWrench(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			return v8force{
				v4{+0.071f * scale, -0.043f * scale, +0.029f * scale, 0},
				v4{-0.083f * scale, +0.037f * scale, +0.061f * scale, 0},
			};
		}

		// Pack one impulse for every link and retain matching production CPU requests.
		std::vector<GpuArticulationSpatialVector> BuildImpulseStream(Articulation& articulation, std::vector<ArticulationImpulse>& requests)
		{
			auto impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			requests.clear();
			requests.reserve(articulation.LinkCount());
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const impulse = ImpulseWrench(link_index);
				requests.push_back(ArticulationImpulse{.m_link = articulation.LinkAt(link_index), .m_impulse = impulse});
				impulses[link_index] = GpuArticulationSpatialVector{.ang = impulse.ang, .lin = impulse.lin};
			}
			return impulses;
		}

		// Require one scalar to agree under a relative tolerance with a stable absolute floor.
		void ExpectImpulseNear(float actual, double expected, float tolerance)
		{
			auto const scale = std::max({1.0, std::abs(static_cast<double>(actual)), std::abs(expected)});
			PR_EXPECT(std::abs(static_cast<double>(actual) - expected) <= static_cast<double>(tolerance) * scale);
		}

		// Require one GPU spatial vector to match one production CPU spatial motion.
		void ExpectImpulseSpatialNear(GpuArticulationSpatialVector const& actual, v8motion const& expected, float tolerance)
		{
			for (int component = 0; component != 3; ++component)
			{
				ExpectImpulseNear(actual.ang[component], expected.ang[component], tolerance);
				ExpectImpulseNear(actual.lin[component], expected.lin[component], tolerance);
			}
		}

		// Reuse one D3D12 device and command job across focused hardware impulse-response tests.
		Gpu& ImpulseTestGpu()
		{
			return SharedTestGpu();
		}
	}

	PRUnitTestClass(ArticulationImpulseAbaGpuTests)
	{
		// Match the dense generalized oracle and production link response for fixed and floating branching trees.
		PRUnitTestMethod(ReplayMatchesCpuAndDenseOracle, Quick)
		{
			for (auto const root_type : {EArticulationRootType::Fixed, EArticulationRootType::Floating})
			{
				auto articulation = BuildImpulseTree(root_type, root_type == EArticulationRootType::Fixed ? 1 : 3);
				auto requests = std::vector<ArticulationImpulse>{};
				auto const impulses = BuildImpulseStream(articulation, requests);
				auto forest = std::array{&articulation};
				auto const initial_upload = PackGpuArticulations(forest);
				auto const expected_delta = articulation_oracle::SolveImpulseResponse(articulation, requests);

				// Production CPU application supplies independent cached link-velocity expectations.
				articulation.ApplyImpulses(requests);
				auto const expected_upload = PackGpuArticulations(forest);
				auto runner = ArticulationImpulseAbaInteropRunner{};
				auto const participants = std::array{0};
				runner.Run(initial_upload, participants, impulses);

				PR_EXPECT(runner.Velocities().size() == expected_upload.m_velocities.size());
				for (int index = 0; index != isize(runner.Velocities()); ++index)
				{
					ExpectImpulseNear(runner.Velocities()[index], expected_upload.m_velocities[index], 2.0e-3f);
					ExpectImpulseNear(runner.Velocities()[index] - initial_upload.m_velocities[index], expected_delta[index], 2.0e-3f);
				}
				for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
					ExpectImpulseSpatialNear(runner.Scratch()[link_index].link_velocity, articulation.LinkVelocity(articulation.LinkAt(link_index)), 2.0e-3f);
			}
		}

		// Applying an equal negative impulse through retained factors restores generalized and cached link velocity.
		PRUnitTestMethod(ReplayReversalRestoresState, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 5);
			auto requests = std::vector<ArticulationImpulse>{};
			auto impulses = BuildImpulseStream(articulation, requests);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto runner = ArticulationImpulseAbaInteropRunner{};
			runner.Run(upload, participants, impulses);
			for (auto& impulse : impulses)
			{
				impulse.ang = -impulse.ang;
				impulse.lin = -impulse.lin;
			}
			runner.Apply(impulses);

			for (int index = 0; index != isize(upload.m_velocities); ++index)
				ExpectImpulseNear(runner.Velocities()[index], upload.m_velocities[index], 4.0e-4f);
			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
				ExpectImpulseSpatialNear(runner.Scratch()[link_index].link_velocity, articulation.LinkVelocity(articulation.LinkAt(link_index)), 4.0e-4f);
		}

		// Identical packed state and impulses produce byte-identical replay output.
		PRUnitTestMethod(ReplayIsDeterministic, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 7);
			auto requests = std::vector<ArticulationImpulse>{};
			auto const impulses = BuildImpulseStream(articulation, requests);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto first = ArticulationImpulseAbaInteropRunner{};
			auto second = ArticulationImpulseAbaInteropRunner{};
			first.Run(upload, participants, impulses);
			second.Run(upload, participants, impulses);

			PR_EXPECT(first.Velocities().size() == second.Velocities().size());
			PR_EXPECT(first.Scratch().size() == second.Scratch().size());
			PR_EXPECT(std::memcmp(first.Velocities().data(), second.Velocities().data(), first.Velocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(first.Scratch().data(), second.Scratch().data(), first.Scratch().size_bytes()) == 0);
		}

		// Detached evaluation exposes the exact link response without changing state, and selective commit matches immediate application.
		PRUnitTestMethod(ReplayEvaluatesBeforeSelectiveCommit, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 8);
			auto requests = std::vector<ArticulationImpulse>{};
			auto const impulses = BuildImpulseStream(articulation, requests);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto const zero_impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			auto runner = ArticulationImpulseAbaInteropRunner{};
			runner.Run(upload, participants, zero_impulses);
			auto const velocities_before = std::vector<float>(runner.Velocities().begin(), runner.Velocities().end());
			auto const accelerations_before = std::vector<float>(runner.Accelerations().begin(), runner.Accelerations().end());
			auto const scratch_before = std::vector<GpuArticulationAbaScratch>(runner.Scratch().begin(), runner.Scratch().end());

			auto const selected = std::array<uint32_t, 1>{1};
			auto const results = runner.Evaluate(impulses, selected);
			PR_EXPECT(results.size() == 1);
			PR_EXPECT(results[0] == 1);
			PR_EXPECT(std::memcmp(runner.Velocities().data(), velocities_before.data(), runner.Velocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Accelerations().data(), accelerations_before.data(), runner.Accelerations().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Scratch().data(), scratch_before.data(), runner.Scratch().size_bytes()) == 0);
			PR_EXPECT(std::ranges::any_of(runner.Work(), [](GpuArticulationSpatialVector const& value)
			{
				return LengthSq(value.ang) + LengthSq(value.lin) > 1.0e-8f;
			}));

			auto expected = ArticulationImpulseAbaInteropRunner{};
			expected.Run(upload, participants, impulses);
			runner.Commit(selected);
			PR_EXPECT(std::memcmp(runner.Velocities().data(), expected.Velocities().data(), runner.Velocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Scratch().data(), expected.Scratch().data(), runner.Scratch().size_bytes()) == 0);
		}

		// A failed detached candidate remains transient so a later bounded retry can reuse the valid retained factors.
		PRUnitTestMethod(ReplayRejectedEvaluationPreservesFactors, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 9);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto const zero_impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			auto runner = ArticulationImpulseAbaInteropRunner{};
			runner.Run(upload, participants, zero_impulses);
			auto const velocities_before = std::vector<float>(runner.Velocities().begin(), runner.Velocities().end());
			auto const accelerations_before = std::vector<float>(runner.Accelerations().begin(), runner.Accelerations().end());
			auto const scratch_before = std::vector<GpuArticulationAbaScratch>(runner.Scratch().begin(), runner.Scratch().end());

			auto impulses = zero_impulses;
			impulses.back().lin.x = std::numeric_limits<float>::quiet_NaN();
			auto const selected = std::array<uint32_t, 1>{1};
			auto const results = runner.Evaluate(impulses, selected);
			PR_EXPECT(results[0] == 0);
			PR_EXPECT(runner.Scratch()[0].solve_valid != 0);
			PR_EXPECT(std::memcmp(runner.Velocities().data(), velocities_before.data(), runner.Velocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Accelerations().data(), accelerations_before.data(), runner.Accelerations().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Scratch().data(), scratch_before.data(), runner.Scratch().size_bytes()) == 0);

			runner.Commit(selected);
			PR_EXPECT(std::memcmp(runner.Velocities().data(), velocities_before.data(), runner.Velocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Accelerations().data(), accelerations_before.data(), runner.Accelerations().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(runner.Scratch().data(), scratch_before.data(), runner.Scratch().size_bytes()) == 0);
		}

		// Reject invalid gathered impulses transactionally without changing any generalized velocity.
		PRUnitTestMethod(ReplayRejectsNonFiniteImpulse, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 4);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			impulses.back().lin.x = std::numeric_limits<float>::quiet_NaN();
			auto const participants = std::array{0};
			auto runner = ArticulationImpulseAbaInteropRunner{};
			runner.Run(upload, participants, impulses);

			PR_EXPECT(runner.Scratch()[0].solve_valid == 0);
			PR_EXPECT(runner.Velocities().size() == upload.m_velocities.size());
			PR_EXPECT(std::memcmp(runner.Velocities().data(), upload.m_velocities.data(), runner.Velocities().size_bytes()) == 0);
		}

		// Reject overflow from a finite extreme impulse before any persistent velocity is committed.
		PRUnitTestMethod(ReplayRejectsOverflowingResponse, Quick)
		{
			auto articulation = BuildImpulseTree(EArticulationRootType::Floating, 4);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			auto const maximum = std::numeric_limits<float>::max();
			impulses.back() = GpuArticulationSpatialVector{
				.ang = v4{maximum, maximum, maximum, 0},
				.lin = v4{maximum, maximum, maximum, 0},
			};
			auto const participants = std::array{0};
			auto runner = ArticulationImpulseAbaInteropRunner{};
			runner.Run(upload, participants, impulses);

			PR_EXPECT(runner.Scratch()[0].solve_valid == 0);
			PR_EXPECT(runner.Velocities().size() == upload.m_velocities.size());
			PR_EXPECT(std::memcmp(runner.Velocities().data(), upload.m_velocities.data(), runner.Velocities().size_bytes()) == 0);
		}

		// Reject a rank-deficient active joint before an impulse response can mutate tree state.
		PRUnitTestMethod(SingularJointFailsExplicitly, Quick)
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(ImpulseLink(0));
			builder.AddLink(root, joint, ImpulseLink(1));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto const impulses = std::vector<GpuArticulationSpatialVector>(articulation.LinkCount());
			auto runner = ArticulationImpulseAbaInteropRunner{};
			PR_THROWS(runner.Run(upload, participants, impulses), std::exception);
		}

		// Match real D3D12 output with replay and release all optional resources when participation becomes empty.
		PRUnitTestMethod(HardwareMatchesReplayAndOptionalCost, Extended)
		{
			auto fixed = BuildImpulseTree(EArticulationRootType::Fixed, 2);
			auto floating = BuildImpulseTree(EArticulationRootType::Floating, 6);
			auto floating_requests = std::vector<ArticulationImpulse>{};
			auto const floating_impulses = BuildImpulseStream(floating, floating_requests);
			auto forest = std::array{&fixed, &floating};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{1};
			auto replay = ArticulationImpulseAbaInteropRunner{};
			replay.Run(upload, participants, floating_impulses);

			auto aba = GpuArticulationForceAba{ImpulseTestGpu()};
			auto mobility = GpuArticulationMobility{aba};
			auto solver = GpuArticulationImpulseAba{ImpulseTestGpu(), aba, mobility};
			auto const hardware = solver.Apply(ImpulseTestGpu().m_job, upload, participants, floating_impulses);
			PR_EXPECT(hardware.AllValid());
			PR_EXPECT(hardware.m_ranges.size() == 1);
			PR_EXPECT(hardware.m_ranges[0].articulation_index == 1);
			PR_EXPECT(hardware.m_velocities.size() == replay.Velocities().size());
			PR_EXPECT(hardware.m_link_velocities.size() == static_cast<size_t>(floating.LinkCount()));
			for (int index = 0; index != isize(hardware.m_velocities); ++index)
				ExpectImpulseNear(hardware.m_velocities[index], replay.Velocities()[index], 2.0e-3f);
			for (int link_index = 0; link_index != isize(hardware.m_link_velocities); ++link_index)
			{
				auto const& expected = replay.Scratch()[fixed.LinkCount() + link_index].link_velocity;
				ExpectImpulseSpatialNear(hardware.m_link_velocities[link_index], v8motion{expected.ang, expected.lin}, 2.0e-3f);
			}
			PR_EXPECT(solver.Stats().m_dispatch_count == 1);
			auto const& floating_header = upload.m_articulations[1];
			PR_EXPECT(
				solver.Stats().m_logical_buffer_bytes ==
				static_cast<size_t>(floating.LinkCount() * 2) * sizeof(GpuArticulationSpatialVector) +
				static_cast<size_t>(floating_header.velocity_count) * sizeof(float));

			// Singular retained factors produce an explicit invalid result and no accepted response.
			auto singular_joint = ArticulationJointDesc::Fixed();
			singular_joint.m_dof_count = 2;
			singular_joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			singular_joint.m_axes[1] = singular_joint.m_axes[0];
			auto singular_builder = ArticulationBuilder{};
			auto const singular_root = singular_builder.AddFixedRoot(ImpulseLink(0));
			singular_builder.AddLink(singular_root, singular_joint, ImpulseLink(1));
			auto singular = singular_builder.Build();
			auto singular_forest = std::array{&singular};
			auto const singular_participants = std::array{0};
			auto const singular_impulses = std::vector<GpuArticulationSpatialVector>(singular.LinkCount());
			auto const singular_result = solver.Apply(
				ImpulseTestGpu().m_job,
				PackGpuArticulations(singular_forest),
				singular_participants,
				singular_impulses);
			PR_EXPECT(!singular_result.AllValid());

			auto const empty = solver.Apply(ImpulseTestGpu().m_job, upload, {}, {});
			PR_EXPECT(empty.m_velocities.empty());
			PR_EXPECT(empty.m_link_velocities.empty());
			PR_EXPECT(solver.Stats().m_link_impulse_capacity == 0);
			PR_EXPECT(solver.Stats().m_work_capacity == 0);
			PR_EXPECT(solver.Stats().m_dispatch_count == 0);
			PR_EXPECT(solver.Stats().m_logical_buffer_bytes == 0);
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}
	};
}
#endif
