//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/interop/articulation_mobility_runner.h"
#include "src/unittests/articulation_oracle.h"
#include "src/unittests/shared_gpu.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric finite mass properties so every spatial mobility coupling remains observable.
		ArticulationLinkDesc MobilityLink(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.18f + 0.013f * scale, 0.24f + 0.009f * scale, 0.31f + 0.007f * scale, 0},
					0.7f + 0.23f * scale,
					v4{0.011f * scale, -0.008f * scale, 0.006f * scale, 0}),
			};
		}

		// Return one ordered zero-to-six-DOF joint with non-trivial attachment transforms.
		ArticulationJointDesc MobilityJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.043f * scale, v4{0.17f * scale, -0.06f, 0.08f, 1}),
				m4x4::Transform(v4::XAxis(), -0.031f * scale, v4{-0.07f, 0.05f * scale, -0.09f, 1}));
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
				joint.m_initial_position[axis_index] = 0.037f * scale * static_cast<float>(axis_index + 1);
				joint.m_initial_velocity[axis_index] = -0.19f + 0.053f * static_cast<float>(axis_index) + 0.009f * scale;
			}
			return joint;
		}

		// Build one deterministic branching tree containing every bounded joint dimension.
		Articulation BuildMobilityTree(EArticulationRootType root_type, int seed)
		{
			auto builder = ArticulationBuilder{};
			auto links = std::vector<LinkHandle>{};
			auto const root_to_world = m4x4::Transform(
				Normalise(v4{1, 2, -1, 0}),
				0.17f + 0.03f * seed,
				v4{0.4f * seed, -0.3f, 0.7f, 1});
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					links.push_back(builder.AddFixedRoot(MobilityLink(seed), root_to_world));
					break;
				}
				case EArticulationRootType::Floating:
				{
					links.push_back(builder.AddFloatingRoot(
						MobilityLink(seed),
						root_to_world,
						v8motion{v4{0.2f, -0.1f, 0.3f, 0}, v4{-0.4f, 0.2f, 0.1f, 0}}));
					break;
				}
				default:
				{
					throw std::invalid_argument("Mobility test root type is invalid");
				}
			}

			for (int link_index = 1; link_index != 8; ++link_index)
			{
				auto const parent_index = link_index == 1 ? 0 : (link_index - 2) / 2;
				links.push_back(builder.AddLink(
					links[parent_index],
					MobilityJoint(link_index - 1, seed + link_index),
					MobilityLink(seed + link_index)));
			}
			return builder.Build();
		}

		// Return one scalar from the symmetric upper-triangular GPU representation.
		float MobilityComponent(GpuArticulationSpatialMobility const& mobility, int row, int column)
		{
			auto const low = std::min(row, column);
			auto const high = std::max(row, column);
			auto const packed_index = low * 6 - low * (low - 1) / 2 + high - low;
			return mobility.packed[packed_index / 4][packed_index % 4];
		}

		// Return one angular-then-linear component from a CPU spatial motion column.
		float MobilityComponent(v8 const& motion, int row)
		{
			return row < 3 ? motion.ang[row] : motion.lin[row - 3];
		}

		// Require one packed GPU mobility to match the production CPU recurrence.
		void ExpectMobilityNear(GpuArticulationSpatialMobility const& actual, detail::SpatialMobility const& expected, float tolerance)
		{
			for (int column = 0; column != 6; ++column)
			{
				auto const expected_column = expected.col(column);
				for (int row = 0; row != 6; ++row)
				{
					auto const actual_value = MobilityComponent(actual, row, column);
					auto const expected_value = MobilityComponent(expected_column, row);
					auto const scale = std::max({1.0f, Abs(actual_value), Abs(expected_value)});
					PR_EXPECT(Abs(actual_value - expected_value) <= tolerance * scale);
				}
			}
		}

		// Reuse one D3D12 device and command job across hardware mobility tests.
		Gpu& MobilityTestGpu()
		{
			return SharedTestGpu();
		}

		// Build a force-loaded slider whose reduced force is six but whose physical acceleration is three.
		std::pair<Articulation, LinkHandle> BuildMobilityAccelerationFixture()
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MobilityLink(0));
			auto const child = builder.AddLink(root, ArticulationJointDesc::Prismatic(v4::XAxis()), ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.2f, 2.0f)});
			auto articulation = builder.Build();
			auto const force = std::array{6.0f};
			articulation.JointForce(child, force);
			return {std::move(articulation), child};
		}
	}

	// Configuration-only factorization leaves accepted integration outputs intact.
	PRUnitTestClass(ArticulationMobilityOutputRegressionTests)
	{
		// Preparing configuration-only mobility must not replace an accepted acceleration with reduced-force scratch.
		PRUnitTestMethod(ReplayPreservesAcceptedGeneralizedAcceleration, Quick)
		{
			auto [articulation, child] = BuildMobilityAccelerationFixture();
			articulation.ForwardDynamics();
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			PR_EXPECT(FEqlAbsolute(upload.m_accelerations[0], 3.0f, 1.0e-6f));
			auto runner = ArticulationMobilityInteropRunner{};
			auto const participants = std::array{0};
			runner.Run(upload, participants);
			PR_EXPECT(runner.Accelerations().size() == upload.m_accelerations.size());
			PR_EXPECT(FEqlAbsolute(runner.Accelerations()[0], 3.0f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(MobilityComponent(runner.Mobilities()[1], 3, 3), 0.5f, 1.0e-6f));
		}

		// The shared GPU acceleration buffer retains the accepted midpoint result through mobility factorization.
		PRUnitTestMethod(HardwarePreservesAcceptedMidpointAcceleration, Extended)
		{
			auto [articulation, child] = BuildMobilityAccelerationFixture();
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto& gpu = MobilityTestGpu();
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto mobility = GpuArticulationMobility{aba};
			PR_EXPECT(midpoint.Upload(gpu.m_job, upload));
			auto const participants = std::array{0};
			PR_EXPECT(mobility.Upload(gpu.m_job, upload, participants));
			midpoint.Run(gpu.m_job, 0.02f, 1);

			// Capture both sides of the mobility dispatch in one submission so the same accepted resource is compared.
			auto* accelerations = midpoint.Output().m_accelerations;
			auto before = gpu.m_job.m_readback.Alloc<float>(1);
			auto after = gpu.m_job.m_readback.Alloc<float>(1);
			gpu.m_job.m_barriers.Transition(accelerations, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			gpu.m_job.m_cmd_list.CopyBufferRegion(before, accelerations, 0);
			gpu.m_job.m_barriers.Transition(accelerations, D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			mobility.Run(gpu.m_job);
			gpu.m_job.m_barriers.Transition(accelerations, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			gpu.m_job.m_cmd_list.CopyBufferRegion(after, accelerations, 0);
			gpu.m_job.Run();
			PR_EXPECT(FEqlAbsolute(*before.ptr<float>(), 3.0f, 1.0e-6f));
			PR_EXPECT(*after.ptr<float>() == *before.ptr<float>());
		}

		// An inactive coupled limit adds no impulse and must not change generalized or reconstructed link acceleration.
		PRUnitTestMethod(EngineZeroImpulseCouplingPreservesAcceleration, Extended)
		{
			auto [articulation, child] = BuildMobilityAccelerationFixture();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, child);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0].m_mode = EConstraintAxisMode::Limited;
			desc.m_linear[0].m_limits = {-10.0f, +10.0f};
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto forest = std::array{&articulation};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			config.constraint_warm_start_factor = 0.0f;
			engine.Config(config);

			// The closed-form constant-force trajectory remains strictly inside the limit for this complete frame.
			auto const dt = 0.02f;
			engine.Step(Engine::StepInput{.m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = dt});
			PR_EXPECT(engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count > 0);
			PR_EXPECT(FEqlAbsolute(articulation.JointPosition(child)[0], 0.5f * 3.0f * dt * dt, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.JointVelocity(child)[0], 3.0f * dt, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.JointAcceleration(child)[0], 3.0f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkAcceleration(child).lin, 3.0f * v4::XAxis(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkAcceleration(child).ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			ResetEngineForNextTest(engine);
		}
	};

	PRUnitTestClass(ArticulationMobilityGpuTests)
	{
		// Empty participation allocates no replay output even when unrelated articulations exist.
		PRUnitTestMethod(EmptyParticipationDoesNoWork, Quick)
		{
			auto articulation = BuildMobilityTree(EArticulationRootType::Floating, 1);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto runner = ArticulationMobilityInteropRunner{};
			runner.Run(upload, {});
			PR_EXPECT(runner.Ranges().empty());
			PR_EXPECT(runner.Mobilities().empty());
			PR_EXPECT(runner.Scratch().empty());
		}

		// Match fixed and floating production CPU mobilities with canonical compact participating ranges.
		PRUnitTestMethod(SelectedTreesMatchCpu, Quick)
		{
			auto fixed = BuildMobilityTree(EArticulationRootType::Fixed, 2);
			auto omitted = BuildMobilityTree(EArticulationRootType::Floating, 4);
			auto floating = BuildMobilityTree(EArticulationRootType::Floating, 5);
			auto forest = std::array{&fixed, &omitted, &floating};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{2, 0, 2};
			auto runner = ArticulationMobilityInteropRunner{};
			runner.Run(upload, participants);

			PR_EXPECT(runner.Ranges().size() == 2);
			PR_EXPECT(runner.Ranges()[0].articulation_index == 0);
			PR_EXPECT(runner.Ranges()[1].articulation_index == 2);
			PR_EXPECT(runner.Ranges()[0].mobility_offset == 0);
			PR_EXPECT(runner.Ranges()[1].mobility_offset == fixed.LinkCount());
			PR_EXPECT(runner.Mobilities().size() == static_cast<size_t>(fixed.LinkCount() + floating.LinkCount()));

			// Compare every full symmetric matrix even though the GPU stores only its upper triangle.
			auto const selected = std::array{&fixed, &floating};
			auto range_index = 0;
			for (auto* articulation : selected)
			{
				auto expected = std::vector<detail::SpatialMobility>(articulation->LinkCount());
				detail::ComputeArticulationLinkMobilities(*articulation, expected);
				auto const& range = runner.Ranges()[range_index++];
				for (int link_index = 0; link_index != articulation->LinkCount(); ++link_index)
					ExpectMobilityNear(runner.Mobilities()[range.mobility_offset + link_index], expected[link_index], 2.0e-3f);
			}
		}

		// Reject a rank-deficient active joint instead of publishing its regularized diagnostic factor.
		PRUnitTestMethod(SingularJointFailsExplicitly, Quick)
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];

			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MobilityLink(0));
			builder.AddLink(root, joint, MobilityLink(1));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto runner = ArticulationMobilityInteropRunner{};
			auto const participants = std::array{0};
			PR_THROWS(runner.Run(PackGpuArticulations(forest), participants), std::exception);
		}

		// Identical packed state and participation produce byte-identical compact replay output.
		PRUnitTestMethod(RepeatedReplayIsDeterministic, Quick)
		{
			auto articulation = BuildMobilityTree(EArticulationRootType::Floating, 3);
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{0};
			auto runner = ArticulationMobilityInteropRunner{};
			runner.Run(upload, participants);
			auto const first = std::vector<GpuArticulationSpatialMobility>{runner.Mobilities().begin(), runner.Mobilities().end()};

			runner.Run(upload, participants);
			PR_EXPECT(runner.Mobilities().size() == first.size());
			PR_EXPECT(std::memcmp(runner.Mobilities().data(), first.data(), first.size() * sizeof(first[0])) == 0);
		}

		// Match hardware and replay to the independent double oracle and preserve zero allocation when no tree participates.
		PRUnitTestMethod(HardwareMatchesOracleAndOptionalCost, Extended)
		{
			auto fixed = BuildMobilityTree(EArticulationRootType::Fixed, 4);
			auto floating = BuildMobilityTree(EArticulationRootType::Floating, 7);
			auto forest = std::array{&fixed, &floating};
			auto const upload = PackGpuArticulations(forest);
			auto const participants = std::array{1};
			auto replay = ArticulationMobilityInteropRunner{};
			replay.Run(upload, participants);

			auto aba = GpuArticulationForceAba{MobilityTestGpu()};
			auto solver = GpuArticulationMobility{aba};
			auto const hardware = solver.Solve(MobilityTestGpu().m_job, upload, participants);
			PR_EXPECT(hardware.AllValid());
			PR_EXPECT(hardware.m_ranges.size() == replay.Ranges().size());
			for (int range_index = 0; range_index != isize(hardware.m_ranges); ++range_index)
			{
				PR_EXPECT(hardware.m_ranges[range_index].articulation_index == replay.Ranges()[range_index].articulation_index);
				PR_EXPECT(hardware.m_ranges[range_index].link_count == replay.Ranges()[range_index].link_count);
				PR_EXPECT(hardware.m_ranges[range_index].mobility_offset == replay.Ranges()[range_index].mobility_offset);
			}
			PR_EXPECT(hardware.m_mobilities.size() == replay.Mobilities().size());

			// Independent double responses avoid treating rounded replay output as the exact reference for hardware accuracy.
			auto const reference_rows = std::array{articulation_oracle::ConstraintJacobianRow{
				.m_terms = {articulation_oracle::ConstraintJacobianTerm{
					.m_link = floating.LinkAt(0),
					.m_wrench = {1, 0, 0, 0, 0, 0},
				}},
				.m_term_count = 1,
			}};
			auto const oracle = articulation_oracle::BuildConstraintSystem(floating, reference_rows);
			auto cpu = std::vector<detail::SpatialMobility>(floating.LinkCount());
			detail::ComputeArticulationLinkMobilities(floating, cpu);
			auto max_hardware_oracle_error = 0.0;
			auto max_replay_oracle_error = 0.0;
			auto max_cpu_oracle_error = 0.0;
			for (int link_index = 0; link_index != isize(hardware.m_mobilities); ++link_index)
			for (int row = 0; row != 6; ++row)
			for (int column = 0; column != 6; ++column)
			{
				auto const hardware_value = MobilityComponent(hardware.m_mobilities[link_index], row, column);
				auto const replay_value = MobilityComponent(replay.Mobilities()[link_index], row, column);
				auto const oracle_value = oracle.m_link_response[link_index](row, column);
				auto const cpu_value = MobilityComponent(cpu[link_index].col(column), row);
				auto const hardware_oracle_error = std::abs(hardware_value - oracle_value) / std::max({1.0, std::abs(static_cast<double>(hardware_value)), std::abs(oracle_value)});
				auto const replay_oracle_error = std::abs(replay_value - oracle_value) / std::max({1.0, std::abs(static_cast<double>(replay_value)), std::abs(oracle_value)});
				auto const cpu_oracle_error = std::abs(cpu_value - oracle_value) / std::max({1.0, std::abs(static_cast<double>(cpu_value)), std::abs(oracle_value)});
				max_hardware_oracle_error = std::max(max_hardware_oracle_error, hardware_oracle_error);
				max_replay_oracle_error = std::max(max_replay_oracle_error, replay_oracle_error);
				max_cpu_oracle_error = std::max(max_cpu_oracle_error, cpu_oracle_error);
				PR_EXPECT(std::isfinite(hardware_value) && std::isfinite(replay_value));
				PR_EXPECT(std::isfinite(cpu_value) && std::isfinite(oracle_value));
			}

			// Each float implementation has its own rounding error, so apply the same accuracy limit directly against the double oracle.
			PR_EXPECT(max_hardware_oracle_error <= 1.0e-3);
			PR_EXPECT(max_replay_oracle_error <= 1.0e-3);
			PR_EXPECT(max_cpu_oracle_error <= 1.0e-3);
			PR_EXPECT(sizeof(GpuArticulationSpatialMobility) == 96);
			PR_EXPECT(hardware.m_mobilities.size() * sizeof(GpuArticulationSpatialMobility) == floating.LinkCount() * 96);

			// Identical hardware inputs retain byte-identical output independently of cross-implementation rounding.
			auto const repeated = solver.Solve(MobilityTestGpu().m_job, upload, participants);
			PR_EXPECT(repeated.m_mobilities.size() == hardware.m_mobilities.size());
			PR_EXPECT(std::memcmp(repeated.m_mobilities.data(), hardware.m_mobilities.data(), hardware.m_mobilities.size() * sizeof(hardware.m_mobilities[0])) == 0);

			// The hardware boundary reports singularity explicitly instead of publishing regularized values as valid physics.
			auto singular_joint = ArticulationJointDesc::Fixed();
			singular_joint.m_dof_count = 2;
			singular_joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			singular_joint.m_axes[1] = singular_joint.m_axes[0];
			auto singular_builder = ArticulationBuilder{};
			auto const singular_root = singular_builder.AddFixedRoot(MobilityLink(0));
			singular_builder.AddLink(singular_root, singular_joint, MobilityLink(1));
			auto singular = singular_builder.Build();
			auto singular_forest = std::array{&singular};
			auto const singular_participants = std::array{0};
			auto const singular_result = solver.Solve(MobilityTestGpu().m_job, PackGpuArticulations(singular_forest), singular_participants);
			PR_EXPECT(!singular_result.AllValid());

			auto const empty = solver.Solve(MobilityTestGpu().m_job, upload, {});
			PR_EXPECT(empty.m_ranges.empty());
			PR_EXPECT(empty.m_mobilities.empty());
			PR_EXPECT(solver.Stats().m_range_capacity == 0);
			PR_EXPECT(solver.Stats().m_mobility_capacity == 0);
			PR_EXPECT(solver.Stats().m_dispatch_count == 0);
			PR_EXPECT(solver.Stats().m_logical_bytes == 0);
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}
	};
}
#endif
