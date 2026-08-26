//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_gpu_data.h"
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/interop/articulation_force_aba_runner.h"
#include "src/unittests/articulation_oracle.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric finite mass properties with an offset centre of mass.
		ArticulationLinkDesc ForceAbaLink(int seed, float mass = 1.0f)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.14f + 0.013f * scale, 0.23f + 0.009f * scale, 0.32f + 0.007f * scale, 0},
					mass,
					v4{0.011f * scale, -0.008f * scale, 0.006f * scale, 0}),
			};
		}

		// Return one well-conditioned serial joint with ordered mixed scalar axes.
		ArticulationJointDesc ForceAbaJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.037f * scale, v4{0.16f * scale, -0.09f, 0.07f, 1}),
				m4x4::Transform(v4::XAxis(), -0.026f * scale, v4{-0.05f, 0.08f * scale, -0.1f, 1}));
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
				joint.m_initial_position[axis_index] = 0.031f * scale * static_cast<float>(axis_index + 1);
				joint.m_initial_velocity[axis_index] = -0.27f + 0.071f * static_cast<float>(axis_index) + 0.012f * scale;
			}
			return joint;
		}

		// Return a deterministic link-frame wrench with observable angular and linear parts.
		v8force ForceAbaWrench(float scale)
		{
			return v8force{
				v4{+0.19f * scale, -0.13f * scale, +0.23f * scale, 0},
				v4{-0.29f * scale, +0.17f * scale, +0.11f * scale, 0},
			};
		}

		// Require two scalar values to agree under a mixed absolute and relative single-precision tolerance.
		void ExpectForceAbaNear(float actual, float expected, float tolerance)
		{
			auto const scale = std::max({1.0f, Abs(actual), Abs(expected)});
			PR_EXPECT(Abs(actual - expected) <= tolerance * scale);
		}

		// Require one phase-reused shared link acceleration to match the production CPU result.
		void ExpectForceAbaLinkNear(GpuArticulationSpatialVector const& actual, v8motion expected, float tolerance)
		{
			ExpectForceAbaNear(actual.ang.x, expected.ang.x, tolerance);
			ExpectForceAbaNear(actual.ang.y, expected.ang.y, tolerance);
			ExpectForceAbaNear(actual.ang.z, expected.ang.z, tolerance);
			ExpectForceAbaNear(actual.lin.x, expected.lin.x, tolerance);
			ExpectForceAbaNear(actual.lin.y, expected.lin.y, tolerance);
			ExpectForceAbaNear(actual.lin.z, expected.lin.z, tolerance);
		}

		// Require one phase-reused shared link acceleration to match an independent double-precision result.
		void ExpectForceAbaLinkNear(GpuArticulationSpatialVector const& actual, std::array<double, 6> const& expected, float tolerance)
		{
			ExpectForceAbaNear(actual.ang.x, static_cast<float>(expected[0]), tolerance);
			ExpectForceAbaNear(actual.ang.y, static_cast<float>(expected[1]), tolerance);
			ExpectForceAbaNear(actual.ang.z, static_cast<float>(expected[2]), tolerance);
			ExpectForceAbaNear(actual.lin.x, static_cast<float>(expected[3]), tolerance);
			ExpectForceAbaNear(actual.lin.y, static_cast<float>(expected[4]), tolerance);
			ExpectForceAbaNear(actual.lin.z, static_cast<float>(expected[5]), tolerance);
		}

		// Compare one packed shared-code replay with the production float ABA and independent double oracle for a forest.
		void ExpectForceAbaForest(std::span<Articulation* const> articulations, float tolerance)
		{
			auto upload = PackGpuArticulations(articulations);
			auto oracle_solutions = std::vector<articulation_oracle::DynamicsSolution>{};
			oracle_solutions.reserve(articulations.size());
			for (auto* articulation : articulations)
				oracle_solutions.push_back(articulation_oracle::SolveForwardDynamics(*articulation));

			auto runner = ArticulationForceAbaInteropRunner{};
			runner.Run(upload);

			// Solve the unchanged CPU trees after packing so both paths consume identical state and force inputs.
			for (auto* articulation : articulations)
				articulation->ForwardDynamics();

			PR_EXPECT(runner.Scratch().size() == upload.m_links.size());
			PR_EXPECT(runner.DofScratch().size() == upload.m_dofs.size());
			PR_EXPECT(runner.JointMatrixScratch().size() == static_cast<size_t>(upload.m_joint_matrix_scratch_count));
			for (auto const& scratch : runner.Scratch())
				PR_EXPECT(scratch.solve_valid != 0);

			// Generalized and link outputs retain the root-then-topological order encoded by each packed header.
			for (int articulation_index = 0; articulation_index != isize(articulations); ++articulation_index)
			{
				auto const& header = upload.m_articulations[articulation_index];
				auto const& articulation = *articulations[articulation_index];
				auto const& oracle = oracle_solutions[articulation_index];
				auto generalized_offset = header.velocity_offset;
				switch (articulation.RootType())
				{
					case EArticulationRootType::Fixed:
					{
						break;
					}
					case EArticulationRootType::Floating:
					{
						auto const acceleration = articulation.RootAcceleration();
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 0], acceleration.ang.x, tolerance);
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 1], acceleration.ang.y, tolerance);
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 2], acceleration.ang.z, tolerance);
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 3], acceleration.lin.x, tolerance);
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 4], acceleration.lin.y, tolerance);
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset + 5], acceleration.lin.z, tolerance);
						generalized_offset += 6;
						break;
					}
					default:
					{
						throw std::runtime_error("Articulation root type is invalid");
					}
				}

				for (int local_link_index = 0; local_link_index != articulation.LinkCount(); ++local_link_index)
				{
					auto const link = articulation.LinkAt(local_link_index);
					ExpectForceAbaLinkNear(
						runner.LinkAcceleration(header.link_offset + local_link_index),
						articulation.LinkAcceleration(link),
						tolerance);
					if (local_link_index == 0)
						continue;

					for (auto acceleration : articulation.JointAcceleration(link))
						ExpectForceAbaNear(upload.m_accelerations[generalized_offset++], acceleration, tolerance);
				}
				PR_EXPECT(generalized_offset == header.velocity_offset + header.velocity_count);

				// The same packed output must remain inside the envelope of the independent dense double-precision oracle.
				PR_EXPECT(oracle.m_acceleration.size() == static_cast<size_t>(header.velocity_count));
				for (int dof_index = 0; dof_index != header.velocity_count; ++dof_index)
					ExpectForceAbaNear(
						upload.m_accelerations[header.velocity_offset + dof_index],
						static_cast<float>(oracle.m_acceleration[dof_index]),
						tolerance);
				for (int local_link_index = 0; local_link_index != articulation.LinkCount(); ++local_link_index)
					ExpectForceAbaLinkNear(
						runner.LinkAcceleration(header.link_offset + local_link_index),
						oracle.m_link_acceleration[local_link_index],
						tolerance);
			}
		}

		// A deterministic xorshift generator keeps randomized forest coverage reproducible.
		struct ForceAbaRandomSource
		{
			uint64_t m_state;

			// Return one deterministic float in [-1,+1).
			float NextSigned()
			{
				m_state ^= m_state << 13;
				m_state ^= m_state >> 7;
				m_state ^= m_state << 17;
				auto const unit = static_cast<double>(m_state >> 11) * (1.0 / 9007199254740992.0);
				return static_cast<float>(2.0 * unit - 1.0);
			}
		};

		// Build one varied deterministic articulation and excite every generalized and link-force path.
		Articulation BuildRandomForceAbaArticulation(ForceAbaRandomSource& random, int trial, bool floating)
		{
			auto builder = ArticulationBuilder{};
			auto handles = std::vector<LinkHandle>{};
			auto const root_transform = m4x4{
				m3x3::Rotation(v3{
					0.2f * random.NextSigned(),
					0.2f * random.NextSigned(),
					0.2f * random.NextSigned()}),
				v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 1},
			};
			if (floating)
			{
				handles.push_back(builder.AddFloatingRoot(
					ForceAbaLink(trial, 1.5f + Abs(random.NextSigned())),
					root_transform,
					v8motion{
						v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0},
						v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0}}));
			}
			else
			{
				handles.push_back(builder.AddFixedRoot(ForceAbaLink(trial), root_transform));
			}

			// Parent selection yields both chains and branches while builder order remains topological.
			auto const child_count = 3 + trial % 4;
			for (int child_index = 0; child_index != child_count; ++child_index)
			{
				auto const dof_count = (trial + 2 * child_index) % 7;
				auto joint = ForceAbaJoint(dof_count, trial + child_index);
				for (int axis_index = 0; axis_index != dof_count; ++axis_index)
				{
					joint.m_initial_position[axis_index] = 0.2f * random.NextSigned();
					joint.m_initial_velocity[axis_index] = 0.6f * random.NextSigned();
				}
				auto const parent_index = child_index == 0 ? 0 : (trial + child_index) % isize(handles);
				handles.push_back(builder.AddLink(
					handles[parent_index],
					joint,
					ForceAbaLink(trial * 11 + child_index, 0.6f + Abs(random.NextSigned()))));
			}

			auto articulation = builder.Build();

			// Non-zero inputs prevent transform, bias, or generalized-force sign errors from hiding behind cancellation.
			if (floating)
				articulation.RootForce(ForceAbaWrench(random.NextSigned()));

			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				articulation.ExternalForce(link, ForceAbaWrench(random.NextSigned()));
				if (link_index == 0)
					continue;

				auto force = std::array<float, 6>{};
				for (int axis_index = 0; axis_index != articulation.JointDofCount(link); ++axis_index)
					force[axis_index] = random.NextSigned();
				articulation.JointForce(link, std::span{force}.first(articulation.JointDofCount(link)));
			}
			return articulation;
		}

		// Reuse one D3D12 device and command job across hardware articulation tests.
		Gpu& ForceAbaTestGpu()
		{
			static auto gpu = Gpu{};
			return gpu;
		}

		// Compare real D3D12 output against the exact shared-code replay, then reuse CPU and double-oracle acceptance checks.
		void ExpectForceAbaHardwareForest(std::span<Articulation* const> articulations, float hardware_tolerance, float reference_tolerance)
		{
			auto upload = PackGpuArticulations(articulations);
			auto replay_upload = upload;
			auto replay = ArticulationForceAbaInteropRunner{};
			replay.Run(replay_upload);

			// Production upload and all level passes execute in one submission owned only by the test readback surface.
			auto solver = GpuArticulationForceAba{ForceAbaTestGpu()};
			auto const hardware = solver.Solve(ForceAbaTestGpu().m_job, upload);
			PR_EXPECT(hardware.AllValid());
			PR_EXPECT(hardware.m_accelerations.size() == replay_upload.m_accelerations.size());
			PR_EXPECT(hardware.m_link_accelerations.size() == replay.Scratch().size());
			for (int index = 0; index != isize(hardware.m_accelerations); ++index)
				ExpectForceAbaNear(hardware.m_accelerations[index], replay_upload.m_accelerations[index], hardware_tolerance);
			for (int link_index = 0; link_index != isize(hardware.m_link_accelerations); ++link_index)
			{
				auto const& expected = replay.LinkAcceleration(link_index);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].ang.x, expected.ang.x, hardware_tolerance);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].ang.y, expected.ang.y, hardware_tolerance);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].ang.z, expected.ang.z, hardware_tolerance);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].lin.x, expected.lin.x, hardware_tolerance);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].lin.y, expected.lin.y, hardware_tolerance);
				ExpectForceAbaNear(hardware.m_link_accelerations[link_index].lin.z, expected.lin.z, hardware_tolerance);
			}

			// Existing acceptance checks compare that same replay result with production float ABA and an independent double solve.
			ExpectForceAbaForest(articulations, reference_tolerance);
		}
	}

	PRUnitTestClass(ArticulationForceAbaTests)
	{
		// Empty optional input performs no work and retains no replay storage.
		PRUnitTestMethod(EmptyForestDoesNoWork, Quick)
		{
			auto upload = PackGpuArticulations({});
			auto runner = ArticulationForceAbaInteropRunner{};
			runner.Run(upload);
			PR_EXPECT(upload.m_accelerations.empty());
			PR_EXPECT(runner.Scratch().empty());
			PR_EXPECT(runner.DofScratch().empty());
			PR_EXPECT(runner.JointMatrixScratch().empty());
		}

		// Persistent replay storage follows 336L + 64D + 4*sum(d_j^2) for mixed fixed, scalar, and full joints.
		PRUnitTestMethod(ScratchStorageFollowsActiveDofFormula, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(ForceAbaLink(0));
			builder.AddLink(root, ForceAbaJoint(0, 1), ForceAbaLink(1));
			builder.AddLink(root, ForceAbaJoint(1, 2), ForceAbaLink(2));
			builder.AddLink(root, ForceAbaJoint(6, 3), ForceAbaLink(3));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);

			// The zero-DOF block consumes no scalars; subsequent blocks occupy one and thirty-six contiguous floats.
			PR_EXPECT(upload.m_links[0].joint_matrix_offset == -1);
			PR_EXPECT(upload.m_links[1].joint_matrix_offset == 0);
			PR_EXPECT(upload.m_links[2].joint_matrix_offset == 0);
			PR_EXPECT(upload.m_links[3].joint_matrix_offset == 1);
			PR_EXPECT(upload.m_joint_matrix_scratch_count == 37);

			auto runner = ArticulationForceAbaInteropRunner{};
			runner.Run(upload);
			auto const persistent_bytes =
				runner.Scratch().size_bytes() +
				runner.DofScratch().size_bytes() +
				runner.JointMatrixScratch().size_bytes();
			PR_EXPECT(runner.Scratch().size() == 4);
			PR_EXPECT(runner.DofScratch().size() == 7);
			PR_EXPECT(runner.JointMatrixScratch().size() == 37);
			PR_EXPECT(persistent_bytes == 336u * 4u + 64u * 7u + 4u * 37u);
		}

		// Match fixed-root zero-to-six-DOF joints, including a force-loaded fixed joint with zero acceleration.
		PRUnitTestMethod(FixedRootOrderedJointsMatchCpu, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(ForceAbaLink(0));
			auto handles = std::vector<LinkHandle>{root};
			for (int dof_count = 0; dof_count != 7; ++dof_count)
				handles.push_back(builder.AddLink(root, ForceAbaJoint(dof_count, dof_count), ForceAbaLink(dof_count + 1, 0.8f + 0.2f * dof_count)));
			auto articulation = builder.Build();
			for (int link_index = 1; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				articulation.ExternalForce(link, ForceAbaWrench(0.2f * link_index));
				auto force = std::array<float, 6>{};
				for (int axis_index = 0; axis_index != articulation.JointDofCount(link); ++axis_index)
					force[axis_index] = -0.37f + 0.11f * axis_index + 0.03f * link_index;
				articulation.JointForce(link, std::span{force}.first(articulation.JointDofCount(link)));
			}

			auto forest = std::array{&articulation};
			ExpectForceAbaForest(forest, 4.0e-3f);
		}

		// Match a moving floating branch with fixed welds, gyroscopic bias, and distributed external loading.
		PRUnitTestMethod(FloatingBranchedTreeMatchesCpu, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(
				ForceAbaLink(0, 3.2f),
				m4x4::Transform(v4::ZAxis(), 0.31f, v4{1.3f, -0.8f, 2.1f, 1}),
				v8motion{v4{0.41f, -0.27f, 0.36f, 0}, v4{-0.16f, 0.24f, 0.29f, 0}});
			auto const branch_a = builder.AddLink(root, ForceAbaJoint(2, 1), ForceAbaLink(1, 1.3f));
			auto const branch_b = builder.AddLink(root, ForceAbaJoint(4, 2), ForceAbaLink(2, 0.9f));
			auto const welded = builder.AddLink(branch_a, ForceAbaJoint(0, 3), ForceAbaLink(3, 0.7f));
			auto const tip = builder.AddLink(branch_b, ForceAbaJoint(6, 4), ForceAbaLink(4, 0.5f));
			auto articulation = builder.Build();
			articulation.RootForce(ForceAbaWrench(+0.7f));
			articulation.JointForce(branch_a, std::array{+0.21f, -0.34f});
			articulation.JointForce(branch_b, std::array{-0.17f, +0.29f, +0.13f, -0.08f});
			articulation.JointForce(tip, std::array{+0.31f, -0.26f, +0.19f, -0.14f, +0.11f, -0.07f});
			articulation.ExternalForce(root, ForceAbaWrench(-0.4f));
			articulation.ExternalForce(welded, ForceAbaWrench(+0.8f));
			articulation.ExternalForce(tip, ForceAbaWrench(-0.6f));

			auto forest = std::array{&articulation};
			ExpectForceAbaForest(forest, 5.0e-3f);
		}

		// Match reproducible fixed/floating randomized forests with branches and ordered zero-to-six-DOF joints.
		PRUnitTestMethod(RandomizedForestsMatchCpu, Quick)
		{
			auto random = ForceAbaRandomSource{0xA0761D6478BD642Full};
			for (int trial = 0; trial != 8; ++trial)
			{
				auto fixed = BuildRandomForceAbaArticulation(random, 2 * trial + 0, false);
				auto floating = BuildRandomForceAbaArticulation(random, 2 * trial + 1, true);
				auto forest = std::array{&fixed, &floating};
				ExpectForceAbaForest(forest, 1.0e-2f);
			}
		}

		// Repeated replay of identical packed input produces exactly identical generalized and link outputs.
		PRUnitTestMethod(RepeatedReplayIsDeterministic, Quick)
		{
			auto random = ForceAbaRandomSource{0xE7037ED1A0B428DBull};
			auto articulation = BuildRandomForceAbaArticulation(random, 5, true);
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto runner = ArticulationForceAbaInteropRunner{};
			runner.Run(upload);
			auto const first_accelerations = upload.m_accelerations;
			auto first_link_accelerations = std::vector<GpuArticulationSpatialVector>{};
			first_link_accelerations.reserve(upload.m_links.size());
			for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
				first_link_accelerations.push_back(runner.LinkAcceleration(link_index));

			runner.Run(upload);
			PR_EXPECT(upload.m_accelerations == first_accelerations);
			PR_EXPECT(runner.Scratch().size() == first_link_accelerations.size());
			for (int link_index = 0; link_index != isize(first_link_accelerations); ++link_index)
			{
				auto const& lhs = runner.LinkAcceleration(link_index);
				auto const& rhs = first_link_accelerations[link_index];
				PR_EXPECT(lhs.ang.x == rhs.ang.x);
				PR_EXPECT(lhs.ang.y == rhs.ang.y);
				PR_EXPECT(lhs.ang.z == rhs.ang.z);
				PR_EXPECT(lhs.lin.x == rhs.lin.x);
				PR_EXPECT(lhs.lin.y == rhs.lin.y);
				PR_EXPECT(lhs.lin.z == rhs.lin.z);
			}
		}

		// Reject a rank-deficient reduced joint instead of exposing regularized replay diagnostics as physical output.
		PRUnitTestMethod(SingularJointFailsExplicitly, Quick)
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];

			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(ForceAbaLink(0));
			builder.AddLink(root, joint, ForceAbaLink(1));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto runner = ArticulationForceAbaInteropRunner{};
			PR_THROWS(runner.Run(upload), std::exception);
		}

		// Match real D3D12 and shared replay for a branched fixed/floating forest containing every zero-to-six-DOF case.
		PRUnitTestMethod(HardwareMixedForestMatchesReplayAndCpu, Extended)
		{
			auto random = ForceAbaRandomSource{0x8EBC6AF09C88C6E3ull};
			auto fixed = BuildRandomForceAbaArticulation(random, 6, false);
			auto floating = BuildRandomForceAbaArticulation(random, 7, true);
			auto forest = std::array{&fixed, &floating};
			ExpectForceAbaHardwareForest(forest, 2.0e-4f, 1.0e-2f);
		}

		// Preserve exact repeated hardware output and report singular reduced inertia through explicit per-link status.
		PRUnitTestMethod(HardwareDeterminismAndSingularStatus, Extended)
		{
			auto random = ForceAbaRandomSource{0x589965CC75374CC3ull};
			auto articulation = BuildRandomForceAbaArticulation(random, 9, true);
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto solver = GpuArticulationForceAba{ForceAbaTestGpu()};
			auto const first = solver.Solve(ForceAbaTestGpu().m_job, upload);
			auto const second = solver.Solve(ForceAbaTestGpu().m_job, upload);
			PR_EXPECT(first.AllValid());
			PR_EXPECT(second.AllValid());
			PR_EXPECT(first.m_accelerations == second.m_accelerations);
			PR_EXPECT(first.m_solve_valid == second.m_solve_valid);
			PR_EXPECT(first.m_link_accelerations.size() == second.m_link_accelerations.size());
			PR_EXPECT(std::memcmp(first.m_link_accelerations.data(), second.m_link_accelerations.data(), first.m_link_accelerations.size() * sizeof(GpuArticulationSpatialVector)) == 0);

			// Duplicate screw axes make the active two-dimensional joint inertia rank deficient.
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];
			auto singular_builder = ArticulationBuilder{};
			auto const singular_root = singular_builder.AddFixedRoot(ForceAbaLink(0));
			singular_builder.AddLink(singular_root, joint, ForceAbaLink(1));
			auto singular = singular_builder.Build();
			auto singular_forest = std::array{&singular};
			auto const singular_result = solver.Solve(ForceAbaTestGpu().m_job, PackGpuArticulations(singular_forest));
			PR_EXPECT(!singular_result.AllValid());
			PR_EXPECT(std::ranges::find(singular_result.m_solve_valid, 0) != singular_result.m_solve_valid.end());
		}

		// Empty forests retain no feature allocation or dispatch, while a root-only tree uses valid shared sentinels.
		PRUnitTestMethod(HardwareEmptyAndScratchFormula, Extended)
		{
			auto solver = GpuArticulationForceAba{ForceAbaTestGpu()};
			auto const empty_upload = PackGpuArticulations({});
			auto const empty_result = solver.Solve(ForceAbaTestGpu().m_job, empty_upload);
			auto const& empty_stats = solver.Stats();
			auto const empty_capacity =
				empty_stats.m_articulation_capacity +
				empty_stats.m_link_capacity +
				empty_stats.m_dof_capacity +
				empty_stats.m_position_capacity +
				empty_stats.m_velocity_capacity +
				empty_stats.m_force_capacity +
				empty_stats.m_external_force_capacity +
				empty_stats.m_child_capacity +
				empty_stats.m_level_link_capacity +
				empty_stats.m_acceleration_capacity +
				empty_stats.m_scratch_capacity +
				empty_stats.m_dof_scratch_capacity +
				empty_stats.m_joint_matrix_capacity;
			PR_EXPECT(empty_result.m_accelerations.empty());
			PR_EXPECT(empty_result.m_link_accelerations.empty());
			PR_EXPECT(empty_capacity == 0);
			PR_EXPECT(empty_stats.m_logical_scratch_bytes == 0);
			PR_EXPECT(empty_stats.m_logical_feature_bytes == 0);
			PR_EXPECT(empty_stats.m_allocated_feature_bytes == 0);
			PR_EXPECT(empty_stats.m_dispatch_count == 0);

			// A fixed root exercises zero-sized DOF, position, generalized-output, child, and inverse streams.
			auto root_builder = ArticulationBuilder{};
			root_builder.AddFixedRoot(ForceAbaLink(0));
			auto root_only = root_builder.Build();
			auto root_forest = std::array{&root_only};
			auto const root_upload = PackGpuArticulations(root_forest);
			auto const root_result = solver.Solve(ForceAbaTestGpu().m_job, root_upload);
			PR_EXPECT(root_result.AllValid());
			PR_EXPECT(root_result.m_accelerations.empty());
			PR_EXPECT(root_result.m_link_accelerations.size() == 1);
			PR_EXPECT(solver.Stats().m_logical_scratch_bytes == sizeof(GpuArticulationAbaScratch));
			PR_EXPECT(solver.Stats().m_dispatch_count == 3);

			// Mixed fixed, scalar, and full joints retain only the documented active-dimension scratch.
			auto mixed_builder = ArticulationBuilder{};
			auto const root = mixed_builder.AddFixedRoot(ForceAbaLink(0));
			mixed_builder.AddLink(root, ForceAbaJoint(0, 1), ForceAbaLink(1));
			mixed_builder.AddLink(root, ForceAbaJoint(1, 2), ForceAbaLink(2));
			mixed_builder.AddLink(root, ForceAbaJoint(6, 3), ForceAbaLink(3));
			auto mixed = mixed_builder.Build();
			auto mixed_forest = std::array{&mixed};
			auto const mixed_upload = PackGpuArticulations(mixed_forest);
			auto const mixed_result = solver.Solve(ForceAbaTestGpu().m_job, mixed_upload);
			PR_EXPECT(mixed_result.AllValid());
			PR_EXPECT(solver.Stats().m_logical_scratch_bytes == 336u * 4u + 64u * 7u + 4u * 37u);
			PR_EXPECT(solver.Stats().m_dispatch_count == 3 * isize(mixed_upload.m_levels));
		}
	};
}
#endif
