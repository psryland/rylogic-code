//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/interop/articulation_mobility_runner.h"

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
			static auto gpu = Gpu{};
			return gpu;
		}
	}

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

		// Match real D3D12 output with shared replay and preserve zero allocation when no tree participates.
		PRUnitTestMethod(HardwareMatchesReplayAndOptionalCost, Extended)
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
			auto max_relative_error = 0.0f;
			for (int link_index = 0; link_index != isize(hardware.m_mobilities); ++link_index)
			for (int row = 0; row != 6; ++row)
			for (int column = 0; column != 6; ++column)
			{
				auto const actual = MobilityComponent(hardware.m_mobilities[link_index], row, column);
				auto const expected = MobilityComponent(replay.Mobilities()[link_index], row, column);
				auto const scale = std::max({1.0f, Abs(actual), Abs(expected)});
				max_relative_error = std::max(max_relative_error, Abs(actual - expected) / scale);
			}

			// Shader contraction can accumulate across the factorization and recurrence while retaining solver-quality parity.
			PR_EXPECT(max_relative_error <= 1.0e-3f);
			PR_EXPECT(sizeof(GpuArticulationSpatialMobility) == 96);
			PR_EXPECT(hardware.m_mobilities.size() * sizeof(GpuArticulationSpatialMobility) == floating.LinkCount() * 96);

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
