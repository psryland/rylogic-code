//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_gpu_data.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/interop/articulation_midpoint_runner.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return finite asymmetric mass properties that expose angular, linear, and centre-of-mass coupling.
		ArticulationLinkDesc MidpointLink(int seed, float mass = 1.0f)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.17f + 0.007f * scale, 0.24f + 0.005f * scale, 0.31f + 0.003f * scale, 0},
					mass,
					v4{0.006f * scale, -0.004f * scale, 0.003f * scale, 0}),
			};
		}

		// Return one well-conditioned ordered mixed-axis joint with non-zero initial state.
		ArticulationJointDesc MidpointJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::YAxis(), 0.021f * scale, v4{0.08f * scale, -0.04f, 0.03f, 1}),
				m4x4::Transform(v4::XAxis(), -0.017f * scale, v4{-0.03f, 0.05f * scale, -0.06f, 1}));
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
				joint.m_initial_velocity[axis_index] = -0.12f + 0.037f * static_cast<float>(axis_index) + 0.006f * scale;
			}
			return joint;
		}

		// Return one fixed or floating branched tree containing zero-, low-, and full-dimensional joints.
		Articulation BuildMidpointArticulation(bool floating)
		{
			auto builder = ArticulationBuilder{};
			auto root = LinkHandle{};
			if (floating)
			{
				root = builder.AddFloatingRoot(
					MidpointLink(0, 2.7f),
					m4x4::Transform(v4::ZAxis(), 0.27f, v4{0.7f, -0.4f, 1.1f, 1}),
					v8motion{v4{0.23f, -0.16f, 0.19f, 0}, v4{-0.08f, 0.13f, 0.17f, 0}});
			}
			else
			{
				root = builder.AddFixedRoot(MidpointLink(0), m4x4::Transform(v4::YAxis(), -0.19f, v4{-0.3f, 0.5f, 0.2f, 1}));
			}

			auto const branch_a = builder.AddLink(root, MidpointJoint(2, 1), MidpointLink(1, 1.2f));
			auto const branch_b = builder.AddLink(root, MidpointJoint(4, 2), MidpointLink(2, 0.9f));
			builder.AddLink(branch_a, MidpointJoint(0, 3), MidpointLink(3, 0.7f));
			builder.AddLink(branch_b, MidpointJoint(6, 4), MidpointLink(4, 0.6f));
			return builder.Build();
		}

		// Return one articulation whose duplicate motion axes make its joint-space inertia singular.
		Articulation BuildSingularMidpointArticulation()
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MidpointLink(0));
			builder.AddLink(root, joint, MidpointLink(1));
			return builder.Build();
		}

		// Reapply deterministic frame-constant generalized and link forces after CPU integration clears them.
		void ApplyMidpointForces(Articulation& articulation)
		{
			switch (articulation.RootType())
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					articulation.RootForce(v8force{
						v4{0.17f, -0.11f, 0.13f, 0},
						v4{-0.21f, 0.09f, 0.15f, 0}});
					break;
				}
				default:
				{
					throw std::runtime_error("Articulation root type is invalid");
				}
			}

			for (int link_index = 0; link_index != articulation.LinkCount(); ++link_index)
			{
				auto const link = articulation.LinkAt(link_index);
				auto const scale = static_cast<float>(link_index + 1);
				articulation.ExternalForce(link, v8force{
					v4{0.023f * scale, -0.017f * scale, 0.019f * scale, 0},
					v4{-0.029f * scale, 0.013f * scale, 0.011f * scale, 0}});
				if (link_index == 0)
					continue;

				auto force = std::array<float, 6>{};
				for (int axis_index = 0; axis_index != articulation.JointDofCount(link); ++axis_index)
					force[axis_index] = -0.14f + 0.031f * axis_index + 0.009f * link_index;
				articulation.JointForce(link, std::span{force}.first(articulation.JointDofCount(link)));
			}
		}

		// Require scalar agreement under a mixed absolute and relative single-precision tolerance.
		void ExpectMidpointNear(float actual, float expected, float tolerance)
		{
			auto const scale = std::max({1.0f, Abs(actual), Abs(expected)});
			PR_EXPECT(Abs(actual - expected) <= tolerance * scale);
		}

		// Require quaternion-and-position frames to agree while allowing equivalent opposite quaternion signs.
		void ExpectMidpointFrameNear(GpuConstraintFrame const& actual, GpuConstraintFrame const& expected, float tolerance)
		{
			auto const quaternion_dot = Abs(
				actual.rotation.x * expected.rotation.x +
				actual.rotation.y * expected.rotation.y +
				actual.rotation.z * expected.rotation.z +
				actual.rotation.w * expected.rotation.w);
			ExpectMidpointNear(quaternion_dot, 1.0f, tolerance);
			ExpectMidpointNear(actual.position.x, expected.position.x, tolerance);
			ExpectMidpointNear(actual.position.y, expected.position.y, tolerance);
			ExpectMidpointNear(actual.position.z, expected.position.z, tolerance);
		}

		// Require one link-frame spatial acceleration to agree component-wise.
		void ExpectMidpointSpatialNear(GpuArticulationSpatialVector const& actual, GpuArticulationSpatialVector const& expected, float tolerance)
		{
			ExpectMidpointNear(actual.ang.x, expected.ang.x, tolerance);
			ExpectMidpointNear(actual.ang.y, expected.ang.y, tolerance);
			ExpectMidpointNear(actual.ang.z, expected.ang.z, tolerance);
			ExpectMidpointNear(actual.lin.x, expected.lin.x, tolerance);
			ExpectMidpointNear(actual.lin.y, expected.lin.y, tolerance);
			ExpectMidpointNear(actual.lin.z, expected.lin.z, tolerance);
		}

		// Require packed primary and ABA output state to agree with a CPU articulation snapshot.
		void ExpectMidpointCpuNear(GpuArticulationUpload const& actual, std::span<GpuArticulation const> actual_articulations, ArticulationMidpointInteropRunner const& replay, std::span<Articulation* const> cpu_articulations, float tolerance)
		{
			auto expected = PackGpuArticulations(cpu_articulations);
			PR_EXPECT(actual.m_positions.size() == expected.m_positions.size());
			PR_EXPECT(actual.m_velocities.size() == expected.m_velocities.size());
			PR_EXPECT(actual.m_accelerations.size() == expected.m_accelerations.size());
			for (int index = 0; index != isize(actual.m_positions); ++index)
				ExpectMidpointNear(actual.m_positions[index], expected.m_positions[index], std::min(tolerance, 5.0e-4f));
			for (int index = 0; index != isize(actual.m_velocities); ++index)
				ExpectMidpointNear(actual.m_velocities[index], expected.m_velocities[index], std::min(tolerance, 5.0e-4f));
			for (int index = 0; index != isize(actual.m_accelerations); ++index)
				ExpectMidpointNear(actual.m_accelerations[index], expected.m_accelerations[index], tolerance);

			// Root transforms and phase-reused link acceleration retain packed articulation/topological order.
			for (int articulation_index = 0; articulation_index != isize(cpu_articulations); ++articulation_index)
			{
				ExpectMidpointFrameNear(actual_articulations[articulation_index].root_to_world, expected.m_articulations[articulation_index].root_to_world, std::min(tolerance, 5.0e-5f));
				auto const link_offset = expected.m_articulations[articulation_index].link_offset;
				for (int local_link_index = 0; local_link_index != cpu_articulations[articulation_index]->LinkCount(); ++local_link_index)
				{
					auto const acceleration = cpu_articulations[articulation_index]->LinkAcceleration(cpu_articulations[articulation_index]->LinkAt(local_link_index));
					auto const expected_spatial = GpuArticulationSpatialVector{
						.ang = acceleration.ang,
						.lin = acceleration.lin,
					};
					ExpectMidpointSpatialNear(replay.LinkAcceleration(link_offset + local_link_index), expected_spatial, tolerance);
				}
			}
		}

		// Reuse one D3D12 device and command job across hardware midpoint tests.
		Gpu& MidpointTestGpu()
		{
			static auto gpu = Gpu{};
			return gpu;
		}

		// Return root-only kinetic energy directly from packed body-frame inertia and generalized velocity.
		float MidpointRootKineticEnergy(GpuArticulationUpload const& upload, std::span<float const> velocities)
		{
			auto const& articulation = upload.m_articulations[0];
			auto const& link = upload.m_links[articulation.link_offset];
			auto const offset = articulation.velocity_offset;
			auto const angular = v4{velocities[offset + 0], velocities[offset + 1], velocities[offset + 2], 0};
			auto const linear = v4{velocities[offset + 3], velocities[offset + 4], velocities[offset + 5], 0};
			auto const centre = link.inertia_com_and_mass;
			auto const centre_velocity = linear + Cross(angular, centre);
			auto const inertia_angular = v4{
				link.inertia_diagonal.x * angular.x + link.inertia_products.x * angular.y + link.inertia_products.y * angular.z,
				link.inertia_products.x * angular.x + link.inertia_diagonal.y * angular.y + link.inertia_products.z * angular.z,
				link.inertia_products.y * angular.x + link.inertia_products.z * angular.y + link.inertia_diagonal.z * angular.z,
				0};
			auto const mass = link.inertia_com_and_mass.w;
			return 0.5f * mass * (Dot(centre_velocity, centre_velocity) + Dot(angular, inertia_angular));
		}

		// Return one long scalar-joint chain for bounded serial-lane scaling coverage.
		Articulation BuildMidpointChain(int link_count)
		{
			auto builder = ArticulationBuilder{};
			auto parent = builder.AddFixedRoot(MidpointLink(0));
			for (int link_index = 1; link_index != link_count; ++link_index)
				parent = builder.AddLink(parent, MidpointJoint(1, link_index % 5), MidpointLink(link_index % 7, 0.8f));
			return builder.Build();
		}
	}

	PRUnitTestClass(ArticulationMidpointTests)
	{
		// Shared replay matches one CPU midpoint step for a fixed/floating branched mixed-DOF forest.
		PRUnitTestMethod(ReplayMixedForestMatchesCpu, Quick)
		{
			auto fixed = BuildMidpointArticulation(false);
			auto floating = BuildMidpointArticulation(true);
			ApplyMidpointForces(fixed);
			ApplyMidpointForces(floating);
			auto forest = std::array{&fixed, &floating};
			auto replay_upload = PackGpuArticulations(forest);
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(replay_upload, 0.001f, 1);
			PR_EXPECT(std::ranges::all_of(replay.States(), [](GpuArticulationIntegrationState const& state)
			{
				return state.status == GpuArticulationIntegrationStatus_Success && state.iteration_count > 0 && state.iteration_count <= 12;
			}));

			fixed.Integrate(0.001f);
			floating.Integrate(0.001f);
			ExpectMidpointCpuNear(replay_upload, replay.Articulations(), replay, forest, 1.0e-2f);
		}

		// Zero time preserves every primary value and empty forests retain no replay allocation.
		PRUnitTestMethod(ReplayZeroTimeAndEmptyAreIdentity, Quick)
		{
			auto empty = PackGpuArticulations({});
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(empty, 0.0f, 3);
			PR_EXPECT(replay.Articulations().empty());
			PR_EXPECT(replay.States().empty());

			auto articulation = BuildMidpointArticulation(true);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto const positions = upload.m_positions;
			auto const velocities = upload.m_velocities;
			auto const root = upload.m_articulations[0].root_to_world;
			replay.Run(upload, 0.0f, 2);
			PR_EXPECT(upload.m_positions == positions);
			PR_EXPECT(upload.m_velocities == velocities);
			PR_EXPECT(std::memcmp(&replay.Articulations()[0].root_to_world, &root, sizeof(root)) == 0);
			PR_EXPECT(replay.States()[0].status == GpuArticulationIntegrationStatus_Success);
		}

		// Singular failure restores primary state and remains sticky across the next recorded substep.
		PRUnitTestMethod(ReplaySingularFailureIsTransactionalAndSticky, Quick)
		{
			auto articulation = BuildSingularMidpointArticulation();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto const positions = upload.m_positions;
			auto const velocities = upload.m_velocities;

			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(upload, 0.01f, 2);
			PR_EXPECT(replay.States()[0].status == GpuArticulationIntegrationStatus_Singular);
			PR_EXPECT(upload.m_positions == positions);
			PR_EXPECT(upload.m_velocities == velocities);
		}

		// Floating-root exponential-map composition and midpoint translation agree tightly with the CPU convention.
		PRUnitTestMethod(ReplayFloatingRootPoseMatchesCpu, Quick)
		{
			auto builder = ArticulationBuilder{};
			builder.AddFloatingRoot(
				MidpointLink(2, 2.3f),
				m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.61f, v4{1.2f, -0.7f, 0.9f, 1}),
				v8motion{v4{0.83f, -0.57f, 0.41f, 0}, v4{-0.36f, 0.28f, 0.47f, 0}});
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(upload, 0.01f, 1);
			PR_EXPECT(replay.States()[0].status == GpuArticulationIntegrationStatus_Success);

			articulation.Integrate(0.01f);
			auto const expected = PackGpuArticulations(forest);
			ExpectMidpointFrameNear(replay.Articulations()[0].root_to_world, expected.m_articulations[0].root_to_world, 2.0e-5f);
		}

		// A deliberately oversized nonlinear step exhausts the fixed CPU iteration bound and rolls back.
		PRUnitTestMethod(ReplayNonConvergenceIsExplicit, Quick)
		{
			auto articulation = BuildMidpointArticulation(true);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto const accepted = PackGpuArticulations(forest);
			auto found_nonconverged = false;
			for (auto dt : std::array{0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f})
			{
				auto upload = accepted;
				auto replay = ArticulationMidpointInteropRunner{};
				replay.Run(upload, dt, 1);
				if (replay.States()[0].status != GpuArticulationIntegrationStatus_NonConverged)
					continue;

				PR_EXPECT(upload.m_positions == accepted.m_positions);
				PR_EXPECT(upload.m_velocities == accepted.m_velocities);
				PR_EXPECT(std::memcmp(&replay.Articulations()[0].root_to_world, &accepted.m_articulations[0].root_to_world, sizeof(GpuConstraintFrame)) == 0);
				found_nonconverged = true;
				break;
			}
			PR_EXPECT(found_nonconverged);
		}

		// Extra persistent integration storage follows exactly 4P + 8V + 48A.
		PRUnitTestMethod(HardwareEmptyRootOnlyAndScratchFormula, Extended)
		{
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const empty = midpoint.Solve(MidpointTestGpu().m_job, PackGpuArticulations({}), 0.01f, 3);
			auto const& empty_stats = midpoint.Stats();
			PR_EXPECT(empty.m_dispatch_count == 0);
			PR_EXPECT(empty_stats.m_position_start_capacity == 0);
			PR_EXPECT(empty_stats.m_velocity_start_capacity == 0);
			PR_EXPECT(empty_stats.m_midpoint_velocity_capacity == 0);
			PR_EXPECT(empty_stats.m_integration_state_capacity == 0);
			PR_EXPECT(empty_stats.m_logical_scratch_bytes == 0);
			PR_EXPECT(empty_stats.m_allocated_feature_bytes == 0);

			auto root_builder = ArticulationBuilder{};
			root_builder.AddFixedRoot(MidpointLink(0));
			auto root_only = root_builder.Build();
			auto root_forest = std::array{&root_only};
			auto const root_result = midpoint.Solve(MidpointTestGpu().m_job, PackGpuArticulations(root_forest), 0.01f, 2);
			PR_EXPECT(root_result.AllSucceeded());
			PR_EXPECT(root_result.m_dispatch_count == 2);
			PR_EXPECT(root_result.m_logical_scratch_bytes == sizeof(GpuArticulationIntegrationState));

			auto mixed = BuildMidpointArticulation(true);
			ApplyMidpointForces(mixed);
			auto mixed_forest = std::array{&mixed};
			auto const upload = PackGpuArticulations(mixed_forest);
			auto const mixed_result = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.001f, 1);
			auto const expected_bytes = 4u * upload.m_positions.size() + 8u * upload.m_velocities.size() + sizeof(GpuArticulationIntegrationState) * upload.m_articulations.size();
			PR_EXPECT(mixed_result.AllSucceeded());
			PR_EXPECT(mixed_result.m_logical_scratch_bytes == expected_bytes);
			PR_EXPECT(mixed_result.m_dispatch_count == 1);
		}

		// Real D3D12 matches exact shared replay and CPU for state, root pose, qdd, and link acceleration.
		PRUnitTestMethod(HardwareMixedForestMatchesReplayAndCpu, Extended)
		{
			auto fixed = BuildMidpointArticulation(false);
			auto floating = BuildMidpointArticulation(true);
			ApplyMidpointForces(fixed);
			ApplyMidpointForces(floating);
			auto forest = std::array{&fixed, &floating};
			auto upload = PackGpuArticulations(forest);
			auto replay_upload = upload;
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(replay_upload, 0.001f, 1);

			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const hardware = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.001f, 1);
			PR_EXPECT(hardware.AllSucceeded());
			PR_EXPECT(hardware.m_dispatch_count == 1);
			for (int index = 0; index != isize(hardware.m_positions); ++index)
				ExpectMidpointNear(hardware.m_positions[index], replay_upload.m_positions[index], 2.0e-4f);
			for (int index = 0; index != isize(hardware.m_velocities); ++index)
				ExpectMidpointNear(hardware.m_velocities[index], replay_upload.m_velocities[index], 2.0e-4f);
			for (int index = 0; index != isize(hardware.m_accelerations); ++index)
				ExpectMidpointNear(hardware.m_accelerations[index], replay_upload.m_accelerations[index], 2.0e-4f);
			for (int articulation_index = 0; articulation_index != isize(hardware.m_articulations); ++articulation_index)
				ExpectMidpointFrameNear(hardware.m_articulations[articulation_index].root_to_world, replay.Articulations()[articulation_index].root_to_world, 2.0e-4f);
			for (int link_index = 0; link_index != isize(hardware.m_link_accelerations); ++link_index)
				ExpectMidpointSpatialNear(hardware.m_link_accelerations[link_index], replay.LinkAcceleration(link_index), 2.0e-4f);

			fixed.Integrate(0.001f);
			floating.Integrate(0.001f);
			ExpectMidpointCpuNear(replay_upload, replay.Articulations(), replay, forest, 1.0e-2f);
		}

		// Mixed-success readback remains detached so a later caller can reject the complete forest before publishing any tree.
		PRUnitTestMethod(HardwareMixedFailureReturnsDetachedForest, Extended)
		{
			auto valid = BuildMidpointArticulation(true);
			auto singular = BuildSingularMidpointArticulation();
			ApplyMidpointForces(valid);
			auto forest = std::array{&valid, &singular};
			auto const accepted = PackGpuArticulations(forest);

			// One late failure must make the aggregate status fail without hiding the successful lane's diagnostics.
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(MidpointTestGpu().m_job, accepted, 0.001f, 2);
			PR_EXPECT(!result.AllSucceeded());
			PR_EXPECT(result.m_states.size() == forest.size());
			PR_EXPECT(result.m_states[0].status == GpuArticulationIntegrationStatus_Success);
			PR_EXPECT(result.m_states[1].status == GpuArticulationIntegrationStatus_Singular);

			// Upload and readback operate on detached storage; no source object changes before the future validation-and-commit boundary.
			auto const unchanged = PackGpuArticulations(forest);
			PR_EXPECT(unchanged.m_positions == accepted.m_positions);
			PR_EXPECT(unchanged.m_velocities == accepted.m_velocities);
			PR_EXPECT(unchanged.m_accelerations == accepted.m_accelerations);
			PR_EXPECT(unchanged.m_forces == accepted.m_forces);
			PR_EXPECT(unchanged.m_external_forces.size() == accepted.m_external_forces.size());
			PR_EXPECT(std::memcmp(
				unchanged.m_external_forces.data(),
				accepted.m_external_forces.data(),
				accepted.m_external_forces.size() * sizeof(GpuArticulationSpatialVector)) == 0);
			for (int articulation_index = 0; articulation_index != isize(forest); ++articulation_index)
			{
				PR_EXPECT(std::memcmp(
					&unchanged.m_articulations[articulation_index].root_to_world,
					&accepted.m_articulations[articulation_index].root_to_world,
					sizeof(GpuConstraintFrame)) == 0);
			}
		}

		// Multiple substeps share immutable forces in one submission and remain bitwise deterministic across identical uploads.
		PRUnitTestMethod(HardwareMultipleSubstepsAndDeterminism, Extended)
		{
			auto articulation = BuildMidpointArticulation(true);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const first = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.0005f, 3);
			auto const second = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.0005f, 3);
			PR_EXPECT(first.AllSucceeded());
			PR_EXPECT(second.AllSucceeded());
			PR_EXPECT(first.m_dispatch_count == 3);
			PR_EXPECT(first.m_positions == second.m_positions);
			PR_EXPECT(first.m_velocities == second.m_velocities);
			PR_EXPECT(first.m_accelerations == second.m_accelerations);
			PR_EXPECT(std::memcmp(first.m_articulations.data(), second.m_articulations.data(), first.m_articulations.size() * sizeof(GpuArticulation)) == 0);

			// CPU clears forces after each accepted step, so reapply the same frame-constant inputs before every reference substep.
			for (int substep_index = 0; substep_index != 3; ++substep_index)
			{
				ApplyMidpointForces(articulation);
				articulation.Integrate(0.0005f);
			}
			auto const expected = PackGpuArticulations(forest);
			for (int index = 0; index != isize(first.m_positions); ++index)
				ExpectMidpointNear(first.m_positions[index], expected.m_positions[index], 1.0e-2f);
			for (int index = 0; index != isize(first.m_velocities); ++index)
				ExpectMidpointNear(first.m_velocities[index], expected.m_velocities[index], 1.0e-2f);
			ExpectMidpointFrameNear(first.m_articulations[0].root_to_world, expected.m_articulations[0].root_to_world, 1.0e-2f);
		}

		// Singular hardware output restores primary state and retains failure through a later dispatch.
		PRUnitTestMethod(HardwareSingularStatusIsSticky, Extended)
		{
			auto joint = ArticulationJointDesc::Fixed();
			joint.m_dof_count = 2;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = v4::XAxis()};
			joint.m_axes[1] = joint.m_axes[0];
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MidpointLink(0));
			builder.AddLink(root, joint, MidpointLink(1));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.01f, 2);
			PR_EXPECT(!result.AllSucceeded());
			PR_EXPECT(result.m_states[0].status == GpuArticulationIntegrationStatus_Singular);
			PR_EXPECT(result.m_positions == upload.m_positions);
			PR_EXPECT(result.m_velocities == upload.m_velocities);
			PR_EXPECT(result.m_dispatch_count == 2);
		}

		// A torque-free asymmetric floating root retains finite normalized pose and approximately conserved kinetic energy.
		PRUnitTestMethod(HardwareFreeFloatingConservationGate, Extended)
		{
			auto builder = ArticulationBuilder{};
			builder.AddFloatingRoot(
				MidpointLink(4, 3.1f),
				m4x4::Identity(),
				v8motion{v4{0.31f, 1.17f, -0.42f, 0}, v4{0, 0, 0, 0}});
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto const energy_start = MidpointRootKineticEnergy(upload, upload.m_velocities);
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(MidpointTestGpu().m_job, upload, 0.0005f, 64);
			auto const energy_end = MidpointRootKineticEnergy(upload, result.m_velocities);
			auto const& rotation = result.m_articulations[0].root_to_world.rotation;
			auto const rotation_length_sq = rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w;
			PR_EXPECT(result.AllSucceeded());
			PR_EXPECT(std::isfinite(energy_end));
			PR_EXPECT(Abs(energy_end - energy_start) <= 1.0e-3f * std::max(1.0f, energy_start));
			ExpectMidpointNear(rotation_length_sq, 1.0f, 2.0e-5f);
		}

		// Shallow-forest and 100-link-chain submissions provide a watchdog gate and report measured serial-lane scaling.
		PRUnitTestMethod(HardwareScalingEvidence, Extended)
		{
			auto shallow = std::vector<Articulation>{};
			shallow.reserve(24);
			for (int articulation_index = 0; articulation_index != 24; ++articulation_index)
			{
				shallow.push_back(BuildMidpointArticulation(false));
				ApplyMidpointForces(shallow.back());
			}
			auto shallow_ptrs = std::vector<Articulation*>{};
			for (auto& articulation : shallow)
				shallow_ptrs.push_back(&articulation);

			auto chain = BuildMidpointChain(100);
			auto chain_forest = std::array{&chain};
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const shallow_start = std::chrono::steady_clock::now();
			auto const shallow_result = midpoint.Solve(MidpointTestGpu().m_job, PackGpuArticulations(shallow_ptrs), 0.0001f, 1);
			auto const shallow_end = std::chrono::steady_clock::now();
			auto const chain_result = midpoint.Solve(MidpointTestGpu().m_job, PackGpuArticulations(chain_forest), 0.0001f, 1);
			auto const chain_end = std::chrono::steady_clock::now();
			auto const shallow_ms = std::chrono::duration<double, std::milli>(shallow_end - shallow_start).count();
			auto const chain_ms = std::chrono::duration<double, std::milli>(chain_end - shallow_end).count();
			std::cout << "[articulation-midpoint] shallow-24x5=" << shallow_ms << "ms, chain-100=" << chain_ms << "ms\n";
			PR_EXPECT(shallow_result.AllSucceeded());
			PR_EXPECT(chain_result.AllSucceeded());
			PR_EXPECT(shallow_ms < 15000.0);
			PR_EXPECT(chain_ms < 15000.0);
		}
	};
}
#endif
