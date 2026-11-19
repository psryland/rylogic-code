//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_gpu_data.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/frame_output_gpu.h"
#include "src/compute/interop/articulation_midpoint_runner.h"
#include "src/compute/interop/articulation_force_aba_runner.h"
#include "src/compute/interop/articulation_impulse_aba_runner.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/coupled_contact_gpu.h"
#include "src/compute/coupled_constraint_solver_gpu.h"
#include "src/compute/constraint_coordinates.hlsli"
#include "src/constraint/constraint_solver.h"
#include "src/unittests/shared_gpu.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// One typed readback whose destination remains valid across moves of the surrounding diagnostic snapshot.
		template <typename Type>
		struct CoupledTransactionReadback
		{
			std::vector<Type> m_values;
			ReadbackAlloc m_allocation;

			// Record a logical stream without reading binding sentinels.
			void Record(GpuJob& job, ID3D12Resource* resource, int count)
			{
				// Empty arrays have no physical elements to copy.
				m_values.resize(count);
				if (count == 0)
					return;

				m_allocation = job.m_readback.Alloc<Type>(count);
				job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				job.m_cmd_list.CopyBufferRegion(m_allocation, resource, 0);
			}

			// Decode only after the recorded submission has completed.
			void Read()
			{
				// The upload/readback allocator remains owned by the completed test job.
				if (!m_values.empty())
					memcpy(m_values.data(), m_allocation.ptr<Type>(), m_values.size() * sizeof(Type));
			}
		};

		// Capture every primary, owner, and sleep field relevant to a coupled admission boundary.
		struct CoupledTransactionSnapshot
		{
			CoupledTransactionReadback<GpuRigidBody> m_bodies;
			CoupledTransactionReadback<GpuConstraintRow> m_rows;
			CoupledTransactionReadback<GpuConstraintBlock> m_blocks;
			CoupledTransactionReadback<float> m_velocities;
			CoupledTransactionReadback<GpuArticulationAbaScratch> m_scratch;
			CoupledTransactionReadback<GpuConstraintAcceptance> m_acceptance;
			CoupledTransactionReadback<GpuConstraintAcceptedRows> m_accepted_rows;
			CoupledTransactionReadback<GpuArticulationSpatialVector> m_rigid_response;
			CoupledTransactionReadback<float> m_velocity_response;
			CoupledTransactionReadback<uint32_t> m_results;
			CoupledTransactionReadback<GpuCoupledConstraintIslandState> m_islands;
			CoupledTransactionReadback<GpuConstraintRigidRecord> m_ordinary_records;
			CoupledTransactionReadback<GpuConstraintSolverState> m_solver_state;

			// Decode the complete capture after its owning job completes.
			void Read()
			{
				// Prepared rows are diagnostic scratch; acceptance and represented responses are independent streams.
				m_bodies.Read();
				m_rows.Read();
				m_blocks.Read();
				m_velocities.Read();
				m_scratch.Read();
				m_acceptance.Read();
				m_accepted_rows.Read();
				m_rigid_response.Read();
				m_velocity_response.Read();
				m_results.Read();
				m_islands.Read();
				m_ordinary_records.Read();
				m_solver_state.Read();
			}
		};

		// Direct production owners sharing one physical buffer; no old warm-publication call is used by this fixture.
		struct CoupledTransactionFixture
		{
			Gpu& m_gpu;
			GpuConstraintSolver m_constraints;
			GpuArticulationForceAba m_aba;
			GpuArticulationMobility m_mobility;
			GpuArticulationImpulseAba m_impulse;
			GpuCoupledConstraintPrepare m_prepare;
			GpuCoupledConstraintVelocity m_velocity;
			D3DPtr<ID3D12Resource> m_bodies;
			D3DPtr<ID3D12Resource> m_frames;
			D3DPtr<ID3D12Resource> m_selection;
			D3DPtr<ID3D12Resource> m_tree_results;
			int m_body_count = 0;
			int m_velocity_count = 0;
			int m_link_count = 0;

			// Keep all runtime owners alive across submissions and same-epoch preparations.
			CoupledTransactionFixture(Gpu& gpu, EngineConfig const& config)
				:m_gpu(gpu)
				,m_constraints(gpu, config)
				,m_aba(gpu)
				,m_mobility(m_aba)
				,m_impulse(gpu, m_aba, m_mobility)
				,m_prepare(m_constraints, config)
				,m_velocity(m_prepare, m_impulse, config)
			{
				// Physical and accepted storage are allocated only by explicit preparation.
			}

			// Replace one fixture-owned input while retaining earlier recorded users.
			template <typename Type>
			void Input(D3DPtr<ID3D12Resource>& resource, std::span<Type const> values)
			{
				// This is diagnostic input transport, not a solver cache or response record.
				auto& job = m_gpu.m_job;
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);

				resource = m_gpu.CreateResource(compute::ResDesc::Buf<Type>(std::max(1, isize(values)), {}).usage(compute::EUsage::UnorderedAccess), job, "Physics:CoupledTransactionFixtureInput");
				auto data = job.m_upload.Alloc<Type>(std::max(1, isize(values)));
				std::fill_n(data.ptr<Type>(), std::max(1, isize(values)), Type{});
				if (!values.empty())
					memcpy(data.ptr<Type>(), values.data(), values.size_bytes());

				job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				job.m_cmd_list.CopyBufferRegion(resource.get(), 0, data);
			}

			// Record only input transport, fixed-configuration preparation, and both owners' explicit transaction preparation.
			void Prepare(GpuConstraintEpoch epoch, GpuConstraintUpload const& constraints, GpuArticulationUpload const& forest, std::span<GpuRigidBody const> bodies, std::span<GpuConstraintFrame const> frames)
			{
				// No implicit solve or physical warm guess is hidden in this setup.
				auto& job = m_gpu.m_job;
				m_body_count = isize(bodies);
				m_velocity_count = isize(forest.m_velocities);
				m_link_count = isize(forest.m_links);
				m_constraints.Upload(job, constraints);
				m_aba.Upload(job, forest);
				m_mobility.Upload(job, forest, constraints.m_coupled_articulation_indices);
				m_prepare.Upload(job, constraints);
				m_velocity.Upload(job, constraints);
				Input(m_bodies, bodies);
				Input(m_frames, frames);
				m_mobility.Run(job);
				m_impulse.Prepare(job);

				// Reconstruct the uploaded link velocities explicitly, without a zero-impulse physical application.
				auto selection = std::vector<uint32_t>(constraints.m_coupled_articulation_indices.size(), 1u);
				Input<uint32_t>(m_selection, selection);
				Input<uint32_t>(m_tree_results, selection);
				m_impulse.RefreshVelocities(job, m_selection.get(), m_tree_results.get(), m_impulse.VelocityBuffers());
				m_constraints.PrepareVelocityTransactions(job, epoch);
				m_constraints.CompileRows(job, epoch.timestep, m_body_count, m_bodies);
				m_constraints.PrepareVelocitySolve(job, epoch.timestep, m_body_count, m_bodies);
				m_prepare.Run(job, epoch.timestep, m_body_count, m_bodies.get(), m_frames.get(), m_mobility);
				m_velocity.PrepareVelocityTransactions(job, epoch, m_body_count);
			}

			// Record all relevant state before or after an explicit production owner commit.
			CoupledTransactionSnapshot Capture()
			{
				// Each snapshot is readback evidence only; it never participates in solving or rollback.
				auto& job = m_gpu.m_job;
				auto result = CoupledTransactionSnapshot{};
				auto output = m_velocity.TransactionOutput();
				auto ordinary = m_constraints.TransactionOutput();
				auto physical = m_impulse.VelocityBuffers();
				result.m_bodies.Record(job, m_bodies.get(), m_body_count);
				result.m_rows.Record(job, output.m_prepared_rows, GpuConstraintRowsPerBlock * output.m_slot_count);
				result.m_blocks.Record(job, output.m_blocks, output.m_slot_count);
				result.m_velocities.Record(job, physical.m_velocities, m_velocity_count);
				result.m_scratch.Record(job, physical.m_scratch, m_link_count);
				result.m_acceptance.Record(job, output.m_acceptance, output.m_island_count);
				result.m_accepted_rows.Record(job, output.m_rows, output.m_slot_count);
				result.m_rigid_response.Record(job, output.m_rigid_response, m_body_count);
				result.m_velocity_response.Record(job, output.m_velocity_response, m_velocity_count);
				result.m_results.Record(job, output.m_results, output.m_island_count);
				result.m_islands.Record(job, output.m_island_states, output.m_island_count);
				result.m_ordinary_records.Record(job, ordinary.m_records, ordinary.m_slot_count);
				result.m_solver_state.Record(job, m_constraints.FrameState(), 1);
				return result;
			}

			// Run a caller-written phase sequence and return its final captured state.
			template <typename Record>
			CoupledTransactionSnapshot Run(Record&& record)
			{
				// Failed recording retires while all fixture and production resource owners are still alive.
				auto result = CoupledTransactionSnapshot{};
				try
				{
					// The caller can capture intermediate gates and interleave ordinary commits outside coupled attempts.
					record();
					result = Capture();
				}
				catch (...)
				{
					// This cleanup does not reclassify a native failure or promise recovery from device execution failure.
					m_gpu.m_job.RetireRecordedWork();
					throw;
				}
				m_gpu.m_job.Run();
				result.Read();
				return result;
			}

			// Attempt one staged coupled replacement against the shared live physical buffer.
			void Coupled()
			{
				// The complete private stage/admit/commit sequence runs in the real persistent owner.
				m_velocity.CommitVelocityTransactions(m_gpu.m_job, m_body_count, m_bodies.get());
			}

			// Commit the ordinary owner without replacing or copying the shared physical buffer.
			void Ordinary()
			{
				// This write remains outside the coupled atomic attempt.
				m_constraints.CommitVelocityTransactions(m_gpu.m_job, m_body_count, m_bodies);
			}
		};

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

		// One articulation and stable handles used to compare hidden-proxy force routing with direct CPU link loading.
		struct ProxyMidpointFixture
		{
			Articulation m_articulation;
			LinkHandle m_root;
			LinkHandle m_child;
		};

		// Build a floating two-link tree whose non-identity proxy frame exposes force-frame and centre-of-mass conversion errors.
		ProxyMidpointFixture BuildProxyMidpointFixture()
		{
			auto builder = ArticulationBuilder{};
			auto root_desc = MidpointLink(0, 2.1f);
			root_desc.m_shape_to_link = m4x4::Transform(v4::YAxis(), 0.23f, v4{0.08f, -0.04f, 0.05f, 1});
			auto child_desc = MidpointLink(1, 1.3f);
			child_desc.m_shape_to_link = m4x4::Transform(Normalise(v4{1, -1, 2, 0}), -0.31f, v4{-0.06f, 0.09f, 0.03f, 1});
			auto const root = builder.AddFloatingRoot(
				root_desc,
				m4x4::Transform(v4::ZAxis(), 0.37f, v4{0.4f, -0.2f, 0.8f, 1}),
				v8motion{v4{0.13f, -0.09f, 0.07f, 0}, v4{-0.04f, 0.06f, 0.08f, 0}});
			auto const child = builder.AddLink(root, MidpointJoint(2, 2), child_desc);
			return ProxyMidpointFixture{
				.m_articulation = builder.Build(),
				.m_root = root,
				.m_child = child,
			};
		}

		// Return the link-frame wrench produced by a world-space force applied at the physical centre of mass.
		v8force LinkWrenchAtCentreOfMass(Articulation const& articulation, LinkHandle link, v4 force_ws)
		{
			auto const force_link = InvertOrthonormal(articulation.LinkToWorld(link).rot) * force_ws;
			auto const centre_link = articulation.LinkDescription(link).m_inertia.CoM();
			return v8force{Cross(centre_link, force_link), force_link};
		}

		// Overwrite one GPU body's world-space force at its centre of mass during the external-force recording phase.
		void WriteBodyLinearForce(Engine::ExternalForceArgs const& args, int body_index, v4 force_ws)
		{
			if (body_index < 0 || body_index >= args.m_body_count)
				throw std::out_of_range("External-force test body index is out of range");

			auto upload = args.m_job.m_upload.Alloc<v4>(1);
			*upload.ptr<v4>() = force_ws;
			args.m_job.m_barriers.Transition(args.m_bodies, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			args.m_job.m_cmd_list.CopyBufferRegion(
				args.m_bodies,
				static_cast<uint64_t>(body_index) * sizeof(GpuRigidBody) + offsetof(GpuRigidBody, force_lin),
				upload);
			args.m_job.m_barriers.Transition(args.m_bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}

		// Build a unit-mass pendulum with COM distance one and central angular inertia 0.1.
		std::pair<Articulation, LinkHandle> BuildGravityPendulum(float position = 0.1f, float velocity = 0.0f)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Revolute(v4::ZAxis());
			joint.m_initial_position[0] = position;
			joint.m_initial_velocity[0] = velocity;
			auto const inertia = Inertia{0.1f, 1.0f, -v4::YAxis()};
			PR_EXPECT(FEql(inertia.CoM(), -v4::YAxis()));
			PR_EXPECT(FEqlAbsolute(inertia.Ic3x3().z.z, 0.1f, 1.0e-7f));
			auto const child = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
			return {builder.Build(), child};
		}

		// Solve the scalar implicit-midpoint pendulum equations independently of spatial algebra and articulation code.
		void StepAnalyticalPendulum(std::array<double, 2>& state, double dt, double follower_torque = 0.0)
		{
			auto const position_start = state[0];
			auto const velocity_start = state[1];
			auto position_end = position_start + dt * velocity_start;
			for (int iteration = 0; iteration != 8; ++iteration)
			{
				auto const midpoint = 0.5 * (position_start + position_end);
				auto const acceleration = (follower_torque - 9.81 * std::sin(midpoint)) / 1.1;
				auto const residual = position_end - position_start - dt * velocity_start - 0.5 * dt * dt * acceleration;
				auto const derivative = 1.0 + 0.25 * dt * dt * 9.81 * std::cos(midpoint) / 1.1;
				position_end -= residual / derivative;
			}

			// The accepted velocity uses the same converged midpoint, not the beginning-of-step gravity direction.
			auto const acceleration = (follower_torque - 9.81 * std::sin(0.5 * (position_start + position_end))) / 1.1;
			state = {position_end, velocity_start + dt * acceleration};
		}

		// Measure mechanical energy above the downward equilibrium without subtracting nearly equal cosines.
		double GravityPendulumEnergy(double position, double velocity)
		{
			auto const half_sine = std::sin(0.5 * position);
			return 0.55 * velocity * velocity + 19.62 * half_sine * half_sine;
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

		// Require the generalized primary state and floating-root pose of two completed articulations to agree.
		void ExpectArticulationPrimaryNear(Articulation& actual, Articulation& expected, float tolerance)
		{
			auto actual_sources = std::array{&actual};
			auto expected_sources = std::array{&expected};
			auto const actual_upload = PackGpuArticulations(actual_sources);
			auto const expected_upload = PackGpuArticulations(expected_sources);
			PR_EXPECT(actual_upload.m_positions.size() == expected_upload.m_positions.size());
			PR_EXPECT(actual_upload.m_velocities.size() == expected_upload.m_velocities.size());
			for (int index = 0; index != isize(actual_upload.m_positions); ++index)
				ExpectMidpointNear(actual_upload.m_positions[index], expected_upload.m_positions[index], tolerance);
			for (int index = 0; index != isize(actual_upload.m_velocities); ++index)
				ExpectMidpointNear(actual_upload.m_velocities[index], expected_upload.m_velocities[index], tolerance);
			ExpectMidpointFrameNear(actual_upload.m_articulations[0].root_to_world, expected_upload.m_articulations[0].root_to_world, tolerance);
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

		// Reuse one D3D12 device and command job across hardware midpoint tests and their Engine fixtures.
		Gpu& MidpointTestGpu()
		{
			return SharedTestGpu();
		}

		// Borrow the class fixture's device and queue without changing process-wide diagnostics during Engine construction.
		Engine MidpointEngine(EngineConfig const& config)
		{
			// The shared GPU fixture outlives this local Engine and already owns the test device's diagnostics.
			auto& gpu = MidpointTestGpu();
			return Engine{config, &SharedTestShaderCache(), gpu.m_gpu.device(), gpu.m_gpu.queue()};
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

		// Require all CPU-owned primary, acceleration, and applied-force streams to remain bitwise unchanged.
		void ExpectPackedArticulationStateExact(GpuArticulationUpload const& actual, GpuArticulationUpload const& expected)
		{
			PR_EXPECT(actual.m_articulations.size() == expected.m_articulations.size());
			PR_EXPECT(actual.m_positions == expected.m_positions);
			PR_EXPECT(actual.m_velocities == expected.m_velocities);
			PR_EXPECT(actual.m_forces == expected.m_forces);
			PR_EXPECT(actual.m_accelerations == expected.m_accelerations);
			PR_EXPECT(actual.m_external_forces.size() == expected.m_external_forces.size());
			for (int articulation_index = 0; articulation_index != isize(actual.m_articulations); ++articulation_index)
			{
				PR_EXPECT(actual.m_articulations[articulation_index].identity_low == expected.m_articulations[articulation_index].identity_low);
				PR_EXPECT(actual.m_articulations[articulation_index].identity_high == expected.m_articulations[articulation_index].identity_high);
				PR_EXPECT(std::memcmp(
					&actual.m_articulations[articulation_index].root_to_world,
					&expected.m_articulations[articulation_index].root_to_world,
					sizeof(GpuConstraintFrame)) == 0);
			}
			for (int link_index = 0; link_index != isize(actual.m_external_forces); ++link_index)
			{
				PR_EXPECT(FEql(actual.m_external_forces[link_index].force_ang, expected.m_external_forces[link_index].force_ang));
				PR_EXPECT(FEql(actual.m_external_forces[link_index].force_lin, expected.m_external_forces[link_index].force_lin));
			}
		}

		// Require successful Engine publication to consume every caller-applied generalized and link force exactly once.
		void ExpectArticulationForcesCleared(GpuArticulationUpload const& upload)
		{
			PR_EXPECT(std::ranges::all_of(upload.m_forces, [](float value) { return value == 0.0f; }));
			PR_EXPECT(std::ranges::all_of(upload.m_external_forces, [](GpuFrameForce const& force)
			{
				return FEql(force.force_ang, v4{}) && FEql(force.force_lin, v4{});
			}));
		}

		// Return a deterministic Engine configuration that retains core GPU work while avoiding unrelated selective passes.
		EngineConfig MidpointEngineConfig()
		{
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			return config;
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

	// A rejected pseudo-position transaction must not expose a partially drifted configuration or wake its tree.
	PRUnitTestClass(SphericalPositionTransactionTests)
	{
		// Reject a late spherical failure and a floating-root failure, then apply an ordinary mixed-joint correction.
		PRUnitTestMethod(RejectThenRecoverWithoutPartialState, Quick)
		{
			// A scalar before and after the quaternion exposes partial writes as well as independent coordinate offsets.
			for (auto floating : {false, true})
			{
				for (auto sleeping : {false, true})
				{
					auto builder = ArticulationBuilder{};
					auto root = floating ? builder.AddFloatingRoot(MidpointLink(0)) : builder.AddFixedRoot(MidpointLink(0));
					auto slide = ArticulationJointDesc::Prismatic(v4::XAxis());
					slide.m_initial_velocity[0] = 0.1f;
					auto first = builder.AddLink(root, slide, MidpointLink(1));
					auto sphere = ArticulationJointDesc::Spherical(m4x4::Identity(), m4x4::Transform(v4::YAxis(), -0.4f, v4::Origin()));
					auto const q = ExpMap<quat>(v3{0.2f, -0.1f, 0.35f});
					sphere.m_initial_position = {q.x, q.y, q.z, q.w};
					sphere.m_initial_velocity = {0.2f, -0.1f, 0.3f};
					auto ball = builder.AddLink(first, sphere, MidpointLink(2));
					auto last = builder.AddLink(ball, ArticulationJointDesc::Revolute(v4::ZAxis()), MidpointLink(3));
					auto tree = builder.Build();
					tree.Sleeping(sleeping);
					auto forest = std::array{&tree};
					auto const before = PackGpuArticulations(forest);
					auto frames = std::vector<m4x4>{};
					for (int index = 0; index != tree.LinkCount(); ++index)
						frames.push_back(tree.LinkToWorld(tree.LinkAt(index)));

					// All inputs are finite, but a later quaternion's squared displacement cannot be represented in FP32.
					auto const offset = floating ? 6 : 0;
					auto pseudo = std::vector<float>(tree.DofCount(), 0.0f);
					pseudo[offset] = 0.25f;
					pseudo[offset + 1] = 1.0e20f;
					PR_THROWS(detail::ApplyArticulationPositionCorrection(tree, pseudo, 1.0f), std::runtime_error);
					auto assert_unchanged = [&]
					{
						// Query both public configuration and cached transforms; neither may publish any part of a rejected drift.
						auto const after = PackGpuArticulations(forest);
						PR_EXPECT(after.m_positions == before.m_positions);
						PR_EXPECT(after.m_velocities == before.m_velocities);
						PR_EXPECT(tree.Sleeping() == sleeping);
						for (int index = 0; index != tree.LinkCount(); ++index)
							PR_EXPECT(FEqlAbsolute(tree.LinkToWorld(tree.LinkAt(index)), frames[index], 1.0e-7f));
					};
					assert_unchanged();
					pseudo[offset + 1] = 0.3f;
					pseudo[offset + 2] = -0.2f;
					pseudo[offset + 3] = 0.4f;
					pseudo[offset + 4] = 0.12f;
					if (floating)
					{
						// Root rejection happens after all joint candidates are finite; it must still reject the complete tree.
						pseudo[0] = 1.0e20f;
						PR_THROWS(detail::ApplyArticulationPositionCorrection(tree, pseudo, 1.0f), std::runtime_error);
						assert_unchanged();
						pseudo[0] = 0.0f;
					}

					// Recovery uses a child-frame right rotation, advances the later scalar once, and leaves physical speeds untouched.
					detail::ApplyArticulationPositionCorrection(tree, pseudo, 1.0f);
					PR_EXPECT(!tree.Sleeping());
					ExpectMidpointNear(tree.JointPosition(first)[0], 0.25f, 1.0e-6f);
					ExpectMidpointNear(tree.JointPosition(last)[0], 0.12f, 1.0e-6f);
					auto const expected_q = q * ExpMap<quat>(v3{0.15f, -0.1f, 0.2f});
					auto const expected_frame = m4x4::Translation(0.25f, 0.0f, 0.0f) * m4x4::Transform(expected_q, v4::Origin()) * InvertOrthonormal(sphere.m_joint_to_child);
					PR_EXPECT(FEqlAbsolute(tree.LinkToWorld(ball), expected_frame, 2.0e-6f));
					PR_EXPECT(PackGpuArticulations(forest).m_velocities == before.m_velocities);
				}
			}
		}
	};

	// Spherical state keeps quaternion configuration separate from three child-joint angular velocities on CPU and GPU.
	PRUnitTestClass(SphericalArticulationTests)
	{
		// Build a nonzero mixed-joint forest member that exposes every position/velocity offset after a quaternion.
		static Articulation MixedTree(bool floating)
		{
			// Alternate spherical and scalar joints, with a fixed branch, so no common scalar offset can accidentally work.
			auto builder = ArticulationBuilder{};
			auto root = floating ? builder.AddFloatingRoot(MidpointLink(0, 2.7f)) : builder.AddFixedRoot(MidpointLink(0));
			auto sphere = ArticulationJointDesc::Spherical(
				m4x4::Transform(v4::YAxis(), 0.32f, v4{0.12f, -0.07f, 0.04f, 1}),
				m4x4::Transform(v4::XAxis(), -0.21f, v4{-0.03f, 0.05f, -0.08f, 1}));
			auto const q = ExpMap<quat>(v3{0.3f, -0.5f, 0.8f});
			sphere.m_initial_position = {q.x, q.y, q.z, q.w};
			sphere.m_initial_velocity = {0.13f, -0.09f, 0.17f};
			auto const first = builder.AddLink(root, sphere, MidpointLink(1));
			auto const second = builder.AddLink(first, MidpointJoint(1, 2), MidpointLink(2));
			builder.AddLink(root, ArticulationJointDesc::Fixed(), MidpointLink(3));
			builder.AddLink(second, sphere, MidpointLink(4));
			builder.AddLink(first, MidpointJoint(2, 3), MidpointLink(5));
			return builder.Build();
		}

		// Quaternion sign, nonidentity joint frames, and child-frame angular velocity describe one physical configuration.
		PRUnitTestMethod(QuaternionFramesAndSign, Quick)
		{
			// Compare with direct frame composition, not another scalar-joint parameterization.
			auto tree = MixedTree(false);
			auto const link = tree.LinkAt(1);
			auto const& joint = tree.JointDescription(link);
			auto position = std::array<float, 4>{};
			std::ranges::copy(tree.JointPosition(link), position.begin());
			auto const q = quat{position[0], position[1], position[2], position[3]};
			auto const expected = joint.m_joint_to_parent * m4x4::Transform(q, v4::Origin()) * InvertOrthonormal(joint.m_joint_to_child);
			auto const velocity = joint.m_joint_to_child * v8motion{v4{0.13f, -0.09f, 0.17f, 0}, v4{}};
			PR_EXPECT(FEqlAbsolute(tree.LinkToWorld(link), expected, 2.0e-6f));
			PR_EXPECT(FEqlAbsolute(tree.LinkVelocity(link).ang, velocity.ang, 2.0e-6f));
			PR_EXPECT(FEqlAbsolute(tree.LinkVelocity(link).lin, velocity.lin, 2.0e-6f));
			for (auto& coordinate : position)
				coordinate = -coordinate;

			tree.JointPosition(link, position);
			PR_EXPECT(FEqlAbsolute(tree.LinkToWorld(link), expected, 2.0e-6f));
			position[3] *= 0.5f;
			PR_THROWS(tree.JointPosition(link, position), std::invalid_argument);
		}

		// A centred asymmetric link obeys the body-frame Euler equation at an arbitrary spherical orientation.
		PRUnitTestMethod(BodyFrameEulerEquation, Quick)
		{
			// Use the physical inertia equation as an oracle independent of articulation factorization.
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Spherical();
			auto q = ExpMap<quat>(v3{0.43f, -0.76f, 0.27f});
			joint.m_initial_position = {q.x, q.y, q.z, q.w};
			joint.m_initial_velocity = {0.7f, -0.4f, 1.2f};
			auto inertia = Inertia::Box(v4{0.3f, 0.4f, 0.5f, 0}, 1.0f);
			auto link = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
			auto tree = builder.Build();
			auto torque = std::array{0.11f, -0.09f, 0.04f};
			tree.JointForce(link, torque);
			auto const omega = v4{0.7f, -0.4f, 1.2f, 0};
			auto const moment = inertia.Ic3x3();
			auto const expected = Invert(moment) * (v4{torque[0], torque[1], torque[2], 0} - Cross(omega, moment * omega));
			auto forest = std::array{&tree};
			auto upload = PackGpuArticulations(forest);
			auto replay = ArticulationForceAbaInteropRunner{};
			replay.Run(upload);
			tree.ForwardDynamics();
			for (int axis = 0; axis != 3; ++axis)
			{
				ExpectMidpointNear(tree.JointAcceleration(link)[axis], expected[axis], 2.0e-5f);
				ExpectMidpointNear(upload.m_accelerations[axis], expected[axis], 2.0e-5f);
			}
		}

		// At rest, an external wrench and the same spatial impulse must produce the same generalized response.
		PRUnitTestMethod(MixedForceAndImpulseAgreement, Quick)
		{
			// Excite all links of a floating mixed tree so incorrect spherical frames or later scalar offsets cannot cancel.
			auto tree = MixedTree(true);
			auto requests = std::vector<ArticulationImpulse>{};
			auto packed_impulses = std::vector<GpuArticulationSpatialVector>{};
			for (int index = 0; index != tree.LinkCount(); ++index)
			{
				auto const link = tree.LinkAt(index);
				auto const wrench = v8force{v4{0.011f * (index + 1), -0.02f, 0.03f, 0}, v4{0.04f, -0.03f * (index + 1), 0.02f, 0}};
				tree.ExternalForce(link, wrench);
				requests.push_back(ArticulationImpulse{.m_link = link, .m_impulse = wrench});
				packed_impulses.push_back(GpuArticulationSpatialVector{.ang = wrench.ang, .lin = wrench.lin});
				if (index != 0)
				{
					auto zero = std::array<float, 6>{};
					tree.JointVelocity(link, std::span{zero}.first(tree.JointDofCount(link)));
				}
			}
			auto forest = std::array{&tree};
			auto const initial = PackGpuArticulations(forest);
			PR_EXPECT(initial.m_positions.size() == 11);
			PR_EXPECT(initial.m_velocities.size() == 15);
			auto replay_upload = initial;
			auto force_replay = ArticulationForceAbaInteropRunner{};
			force_replay.Run(replay_upload);
			tree.ForwardDynamics();
			auto const cpu_force = PackGpuArticulations(forest);
			tree.ApplyImpulses(requests);
			auto const cpu_impulse = PackGpuArticulations(forest);
			auto const indices = std::array{0};
			auto impulse_replay = ArticulationImpulseAbaInteropRunner{};
			impulse_replay.Run(initial, indices, packed_impulses);

			// Run both actual GPU paths, not only HLSL-as-C++ replay.
			auto& gpu = MidpointTestGpu();
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto const hardware_force = aba.Solve(gpu.m_job, initial);
			auto const hardware_impulse = impulse.Apply(gpu.m_job, initial, indices, packed_impulses);
			PR_EXPECT(hardware_force.AllValid() && hardware_impulse.AllValid());
			for (int index = 0; index != isize(initial.m_velocities); ++index)
			{
				auto const expected = cpu_force.m_accelerations[index];
				ExpectMidpointNear(cpu_impulse.m_velocities[index], expected, 3.0e-4f);
				ExpectMidpointNear(replay_upload.m_accelerations[index], expected, 3.0e-4f);
				ExpectMidpointNear(impulse_replay.Velocities()[index], expected, 3.0e-4f);
				ExpectMidpointNear(hardware_force.m_accelerations[index], expected, 3.0e-4f);
				ExpectMidpointNear(hardware_impulse.m_velocities[index], expected, 3.0e-4f);
			}
		}

		// A torque-free spherical rotor preserves its spin and orientation under either quaternion sign.
		PRUnitTestMethod(IsotropicSpinAndQuaternionSign, Quick)
		{
			// A centred sphere has constant body angular velocity, giving an independent matrix-exponential orientation oracle.
			auto initial_q = ExpMap<quat>(v3{0.4f, -0.7f, 0.2f});
			auto angular_velocity = v4{0.3f, -0.2f, 0.5f, 0};
			auto trees = std::vector<Articulation>{};
			for (auto sign : {1.0f, -1.0f})
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFixedRoot(MidpointLink(0));
				auto joint = ArticulationJointDesc::Spherical();
				joint.m_initial_position = {sign * initial_q.x, sign * initial_q.y, sign * initial_q.z, sign * initial_q.w};
				joint.m_initial_velocity = {angular_velocity.x, angular_velocity.y, angular_velocity.z};
				builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.5f, 2.0f)});
				trees.push_back(builder.Build());
			}
			auto forest = std::array{&trees[0], &trees[1]};
			auto initial = PackGpuArticulations(forest);
			auto const expected = m4x4::Transform(initial_q, v4::Origin()) * m4x4{m3x3::Rotation(0.1f * angular_velocity.xyz), v4::Origin()};
			auto& gpu = MidpointTestGpu();
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(gpu.m_job, initial, 0.01f, 10);
			PR_EXPECT(result.AllSucceeded());
			for (int index = 0; index != 2; ++index)
			{
				auto const initial_energy = trees[index].KineticEnergy();
				for (int step = 0; step != 10; ++step)
					trees[index].Integrate(0.01f);

				PR_EXPECT(FEqlAbsolute(trees[index].LinkToWorld(trees[index].LinkAt(1)), expected, 2.0e-5f));
				ExpectMidpointNear(trees[index].KineticEnergy(), initial_energy, 2.0e-6f);
				auto const* q = result.m_positions.data() + index * 4;
				PR_EXPECT(FEqlAbsolute(m4x4::Transform(quat{q[0], q[1], q[2], q[3]}, v4::Origin()), expected, 2.0e-5f));
				auto reverse_velocity = std::array{-angular_velocity.x, -angular_velocity.y, -angular_velocity.z};
				trees[index].JointVelocity(trees[index].LinkAt(1), reverse_velocity);
				for (int step = 0; step != 10; ++step)
					trees[index].Integrate(0.01f);

				PR_EXPECT(FEqlAbsolute(trees[index].LinkToWorld(trees[index].LinkAt(1)), m4x4::Transform(initial_q, v4::Origin()), 3.0e-5f));
			}
		}

		// Sustained asymmetric free spin measures conservation error and its refinement, not exact discrete invariants.
		PRUnitTestMethod(AsymmetricTorqueFreeSpin, Extended)
		{
			// A centred anisotropic body has no applied torque, so physical energy and world angular momentum are constant.
			auto momentum_errors = std::array<float, 2>{};
			for (int refinement = 0; refinement != 2; ++refinement)
			{
				auto const steps = 120 << refinement;
				auto const dt = 2.0f / steps;
				auto builder = ArticulationBuilder{};
				auto root = builder.AddFixedRoot(MidpointLink(0));
				auto joint = ArticulationJointDesc::Spherical();
				auto q = ExpMap<quat>(v3{0.43f, -0.76f, 0.27f});
				joint.m_initial_position = {q.x, q.y, q.z, q.w};
				joint.m_initial_velocity = {0.7f, -0.4f, 1.2f};
				auto inertia = Inertia::Box(v4{0.3f, 0.4f, 0.5f, 0}, 1.0f);
				auto link = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
				auto tree = builder.Build();
				auto forest = std::array{&tree};
				auto initial = PackGpuArticulations(forest);
				auto const initial_energy = tree.KineticEnergy();
				auto const angular_momentum = tree.LinkToWorld(link).rot * (inertia.Ic3x3() * v4{0.7f, -0.4f, 1.2f, 0});
				auto max_energy_error = 0.0f;
				for (int step = 0; step != steps; ++step)
				{
					// Quaternion exponential midpoint has ordinary time-discretization error; track its maximum, not only its final value.
					tree.Integrate(dt);
					auto speed = tree.JointVelocity(link);
					auto const current_momentum = tree.LinkToWorld(link).rot * (inertia.Ic3x3() * v4{speed[0], speed[1], speed[2], 0});
					max_energy_error = std::max(max_energy_error, std::abs(tree.KineticEnergy() / initial_energy - 1.0f));
					momentum_errors[refinement] = std::max(momentum_errors[refinement], Length(current_momentum - angular_momentum) / Length(angular_momentum));
				}

				// The actual fused GPU path must follow the same sustained motion, not merely its instantaneous acceleration.
				auto& gpu = MidpointTestGpu();
				auto aba = GpuArticulationForceAba{gpu};
				auto midpoint = GpuArticulationMidpoint{aba};
				auto result = midpoint.Solve(gpu.m_job, initial, dt, steps);
				auto final = PackGpuArticulations(forest);
				PR_EXPECT(result.AllSucceeded());
				for (int index = 0; index != 4; ++index)
					ExpectMidpointNear(result.m_positions[index], final.m_positions[index], 3.0e-5f);

				for (int index = 0; index != 3; ++index)
					ExpectMidpointNear(result.m_velocities[index], final.m_velocities[index], 3.0e-5f);

				PR_EXPECT(max_energy_error < 2.0e-4f);
				PR_EXPECT(momentum_errors[refinement] < 2.0e-3f);
				printf("asymmetric free spin: steps=%d seconds=2 max_relative_energy=%.9g max_relative_world_momentum=%.9g\n", steps, max_energy_error, momentum_errors[refinement]);
			}
			PR_EXPECT(momentum_errors[1] <= 0.45f * momentum_errors[0] + 2.0e-5f);
		}

		// Mixed quaternion/scalar configurations advance consistently through physical GPU integration and public Engine publication.
		PRUnitTestMethod(MixedMidpointAndPublicEngine, Quick)
		{
			// Pack two trees so four-coordinate offsets differ from velocity offsets both within and between trees.
			auto fixed = MixedTree(false);
			auto floating = MixedTree(true);
			auto forest = std::array{&fixed, &floating};
			for (auto* tree : forest)
				ApplyMidpointForces(*tree);

			auto const initial = PackGpuArticulations(forest);
			auto expected = initial;
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(expected, 0.001f, 3);
			for (auto const& state : replay.States())
				PR_EXPECT(state.status == GpuArticulationIntegrationStatus_Success);

			auto& gpu = MidpointTestGpu();
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(gpu.m_job, initial, 0.001f, 3);
			PR_EXPECT(result.AllSucceeded());
			for (int index = 0; index != isize(expected.m_positions); ++index)
				ExpectMidpointNear(result.m_positions[index], expected.m_positions[index], 5.0e-5f);

			for (int index = 0; index != isize(expected.m_velocities); ++index)
				ExpectMidpointNear(result.m_velocities[index], expected.m_velocities[index], 5.0e-5f);

			// Engine validates and commits quaternion coordinates rather than treating the fourth entry as another scalar velocity.
			auto engine = MidpointEngine(MidpointEngineConfig());
			engine.Step(Engine::StepInput{.m_articulations = forest, .m_elapsed_seconds = 0.003f, .m_substep_count = 3});
			ExpectMidpointCpuNear(expected, replay.Articulations(), replay, forest, 3.0e-4f);
			auto cpu_fixed = MixedTree(false);
			auto cpu_floating = MixedTree(true);
			auto cpu_forest = std::array{&cpu_fixed, &cpu_floating};
			for (int step = 0; step != 3; ++step)
			{
				for (auto* tree : cpu_forest)
				{
					ApplyMidpointForces(*tree);
					tree->Integrate(0.001f);
				}
			}
			ExpectMidpointCpuNear(expected, replay.Articulations(), replay, cpu_forest, 3.0e-4f);
		}
	};

	// Exercise nonzero quaternion drift through both production position-application owners.
	PRUnitTestClass(SphericalPositionGpuTests)
	{
		// A quaternion followed by a scalar joint exposes offset aliasing and wrong-frame angular integration.
		static Articulation BuildTree()
		{
			// Rotated joint attachments and a nonzero quaternion prevent left and right drift from agreeing accidentally.
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Spherical(
				m4x4::Transform(v4::YAxis(), 0.3f, v4{0.1f, -0.2f, 0.1f, 1}),
				m4x4::Transform(v4::XAxis(), -0.2f, v4{-0.03f, 0.04f, 0.05f, 1}));
			auto q = ExpMap<quat>(v3{0.2f, -0.3f, 0.1f});
			joint.m_initial_position = {q.x, q.y, q.z, q.w};
			joint.m_initial_velocity = {0.17f, -0.09f, 0.11f};
			auto ball = builder.AddLink(root, joint, MidpointLink(1));
			auto hinge = ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(0.2f, 0.1f, 0.0f));
			hinge.m_initial_position[0] = 0.23f;
			hinge.m_initial_velocity[0] = -0.05f;
			builder.AddLink(ball, hinge, MidpointLink(2));
			return builder.Build();
		}

		// Constrain the last link's orientation without duplicating any of the articulation's translational constraints.
		static ConstraintDesc MakeConstraint(Articulation const& tree)
		{
			// The current nonzero orientation provides a genuine angular position error in all three rows.
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(tree, tree.LinkAt(2));
			for (auto& axis : desc.D6().m_angular)
				axis.m_mode = EConstraintAxisMode::Locked;

			return desc;
		}

		// Compare application with a direct rotation-matrix exponential, independent of the shared quaternion drift helper.
		static void CheckDrift(GpuArticulationUpload const& initial, std::span<float const> positions, std::span<float const> velocities, std::span<float const> pseudo, float dt)
		{
			// Four quaternion coordinates and three angular speeds must leave the following scalar in its own slot.
			PR_EXPECT(positions.size() == 5 && velocities.size() == 4 && pseudo.size() == 4);
			auto const* q0 = initial.m_positions.data();
			auto const* q1 = positions.data();
			auto const expected = m4x4::Transform(quat{q0[0], q0[1], q0[2], q0[3]}, v4::Origin()) *
				m4x4{m3x3::Rotation(dt * v3{pseudo[0], pseudo[1], pseudo[2]}), v4::Origin()};
			auto const actual = m4x4::Transform(quat{q1[0], q1[1], q1[2], q1[3]}, v4::Origin());
			PR_EXPECT(FEqlAbsolute(actual, expected, 2.0e-5f));
			ExpectMidpointNear(positions[4], initial.m_positions[4] + dt * pseudo[3], 2.0e-5f);
			PR_EXPECT(std::ranges::equal(velocities, initial.m_velocities));
			PR_EXPECT(Length(v3{pseudo[0], pseudo[1], pseudo[2]}) > 1.0e-3f);
			PR_EXPECT(std::abs(pseudo[3]) > 1.0e-3f);
		}

		// Exercise the shared application owner, optionally interrupting a valid recorded prefix to check failure cleanup.
		static void CheckSharedDrift(bool interrupt_recording)
		{
			// Inject a known detached correction to isolate application and ownership from the contact/constraint solve.
			auto tree = BuildTree();
			auto forest = std::array{&tree};
			auto initial = PackGpuArticulations(forest);
			auto bodies = PackGpuArticulationProxies(initial, forest, std::array{-1, -1, -1}, 0);
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			auto integrator = GpuIntegrator{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto proxies = GpuArticulationLinkProxies{aba, config};
			auto contact = GpuCoupledContactSolver{gpu, aba, proxies, config};
			auto constraints = GpuConstraintSolver{gpu, config};
			auto coupled = GpuCoupledConstraintSolver{gpu, constraints, aba, proxies, config};
			auto source = ConstraintSet{};
			source.Add(MakeConstraint(tree));
			auto remap = BodyRemap{{}, forest};
			auto constraint_upload = PackGpuConstraints(source, remap);
			auto counters = D3DPtr<ID3D12Resource>{};
			auto contacts = D3DPtr<ID3D12Resource>{};
			auto materials = D3DPtr<ID3D12Resource>{};
			auto recording = true;
			try
			{
				// Keep every referenced resource outside this recovery scope until its recorded commands have retired.
				PR_EXPECT(aba.Upload(gpu.m_job, initial));
				integrator.Upload(gpu.m_job, bodies);
				proxies.Upload(gpu.m_job);
				proxies.Refresh(gpu.m_job, integrator, 0);
				PR_EXPECT(contact.Upload(gpu.m_job, initial));
				constraints.Upload(gpu.m_job, constraint_upload);
				PR_EXPECT(coupled.Upload(gpu.m_job, constraint_upload, initial));
				coupled.PrepareVelocity(gpu.m_job, physics::tests::FreshConstraintTestEpoch(0.025f, config.constraint_warm_start_factor), 0, integrator.Bodies().get());

				// A zero-count manifold initializes the real contact application route without solving an unrelated collision.
				auto upload_value = [&]<typename T>(D3DPtr<ID3D12Resource>& buffer, T const& value, char const* name)
				{
					// Publish ownership before recording any command that references the resource.
					buffer = gpu.CreateResource(compute::ResDesc::Buf<T>(1, {}).usage(compute::EUsage::UnorderedAccess), gpu.m_job, name);
					auto allocation = gpu.m_job.m_upload.Alloc<T>(1);
					*allocation.template ptr<T>() = value;
					gpu.m_job.m_barriers.Transition(buffer.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
					gpu.m_job.m_cmd_list.CopyBufferRegion(buffer.get(), 0, allocation);
				};
				upload_value(counters, GpuCollisionCounters{}, "Test:SphericalPositionCounters");
				upload_value(contacts, GpuResolveContact{}, "Test:SphericalPositionContacts");
				upload_value(materials, GpuMaterial{}, "Test:SphericalPositionMaterials");
				contact.PrepareVelocity(gpu.m_job, 0.025f, isize(bodies), 0, 1, counters, contacts, integrator.Bodies(), materials, 0.0f);
				auto rigid_pseudo = coupled.RigidPseudoVelocityStorage(gpu.m_job, 0);
				PR_EXPECT(contact.PreparePosition(gpu.m_job, 1, rigid_pseudo));
				auto shared = contact.PseudoState();
				auto pseudo = std::array{0.7f, -0.4f, 0.3f, 0.2f};
				auto allocation = gpu.m_job.m_upload.Alloc<float>(4);
				std::ranges::copy(pseudo, allocation.ptr<float>());
				gpu.m_job.m_barriers.Transition(shared.m_generalized_velocities.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(shared.m_generalized_velocities.get(), 0, allocation);
				PR_EXPECT(coupled.PreparePosition(gpu.m_job, 0.025f, 0, integrator.Bodies().get(), shared, 1));
				if (interrupt_recording)
					throw std::runtime_error("Intentional spherical position recording interruption");

				PR_THROWS(coupled.ApplyPosition(gpu.m_job, integrator.Bodies().get(), 0), std::logic_error);

				// Observe unchanged coordinates before their sole owner's apply, then compare the actual production shader output.
				auto buffers = midpoint.Output();
				auto before = gpu.m_job.m_readback.Alloc<float>(5);
				gpu.m_job.m_barriers.Transition(buffers.m_positions, D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(before, buffers.m_positions, 0);
				contact.ApplyPosition(gpu.m_job);
				auto positions = gpu.m_job.m_readback.Alloc<float>(5);
				auto velocities = gpu.m_job.m_readback.Alloc<float>(4);
				auto failure = gpu.m_job.m_readback.Alloc<GpuCoupledContactState>(1);
				gpu.m_job.m_barriers.Transition(buffers.m_positions, D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(buffers.m_velocities, D3D12_RESOURCE_STATE_COPY_SOURCE);
				gpu.m_job.m_barriers.Transition(contact.FrameState(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(positions, buffers.m_positions, 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(velocities, buffers.m_velocities, 0);
				gpu.m_job.m_cmd_list.CopyBufferRegion(failure, contact.FrameState(), 0);
				recording = false;
				gpu.m_job.Run();
				PR_EXPECT(std::ranges::equal(std::span{before.ptr<float>(), 5}, initial.m_positions));
				PR_EXPECT(failure.ptr<GpuCoupledContactState>()->frame_failure_flags == 0u);
				CheckDrift(initial, {positions.ptr<float>(), 5}, {velocities.ptr<float>(), 4}, pseudo, 0.025f);
			}
			catch (...)
			{
				// Retire the valid prefix before releasing its resources, then preserve the fixture's original failure.
				auto const error = std::current_exception();
				if (recording)
					gpu.m_job.RetireRecordedWork();

				std::rethrow_exception(error);
			}
		}

		// The contact owner applies known shared pseudo state once; preparing persistent constraints must not clear or apply it.
		PRUnitTestMethod(ContactOwnsSharedDrift, Extended)
		{
			// Exercise the successful recording and application path.
			CheckSharedDrift(false);
		}

		// A failed recording must report its original exception and leave the shared GPU fixture usable.
		PRUnitTestMethod(InterruptedRecordingReportsFailure, Extended)
		{
			// Retirement must preserve the original failure instead of making the failed fixture appear successful.
			auto reported = false;
			try
			{
				// Interrupt after shared pseudo preparation has recorded commands and reserved transfer allocations.
				CheckSharedDrift(true);
			}
			catch (std::runtime_error const& error)
			{
				// The exact diagnostic distinguishes the intended failure from a replacement cleanup error.
				PR_EXPECT(std::string_view(error.what()) == "Intentional spherical position recording interruption");
				reported = true;
			}
			PR_EXPECT(reported);

			// Reusing the same recording owner proves its completed fence and command-list state are usable.
			CheckSharedDrift(false);
		}

		// Standalone persistent constraints generate and apply their own nonzero spherical and later-scalar correction.
		PRUnitTestMethod(ConstraintDriftMatchesImpulseResponse, Extended)
		{
			// Use a real position solve and recover its total pseudo speed independently from the committed row impulses.
			auto tree = BuildTree();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			source.Add(MakeConstraint(tree));
			auto initial = PackGpuArticulations(forest);
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto frames = std::vector<GpuConstraintFrame>{};
			for (int index = 0; index != tree.LinkCount(); ++index)
				frames.push_back(PackGpuTransform(tree.LinkToWorld(tree.LinkAt(index))));

			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.push_out_iterations = 1;
			config.constraint_coupled_relaxation = 1.0f;
			auto constraints = GpuConstraintSolver{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse, config};
			auto position = GpuCoupledConstraintPosition{velocity, config};
			auto result = position.Solve(gpu.m_job, 0.025f, upload, initial, {}, frames);
			PR_EXPECT(result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);

			// An impulse ABA response is the same linear map used for pseudo velocity, without advancing physical coordinates.
			auto wrench = v8force::Zero();
			for (int axis = 0; axis != 3; ++axis)
			{
				auto const& row = result.m_rows[3 + axis];
				wrench += row.bounds.w * v8force{row.jacobian_a_ang, row.jacobian_a_lin};
			}
			tree.ApplyImpulses(std::array{ArticulationImpulse{.m_link = tree.LinkAt(2), .m_impulse = wrench}});
			auto response = PackGpuArticulations(forest);
			auto pseudo = std::array<float, 4>{};
			for (int index = 0; index != 4; ++index)
				pseudo[index] = response.m_velocities[index] - initial.m_velocities[index];

			CheckDrift(initial, result.m_positions, result.m_articulation_velocities, pseudo, 0.025f);
		}
	};

	// Check the typed primitive against physical momentum and independently differentiated angular coordinates.
	PRUnitTestClass(SphericalAngularPrimitiveTests)
	{
		// Select a damping-only, simultaneous ellipse/twist, or fully locked six-row obligation.
		enum class ECase { DampingOnly, EllipseTwist, Locked, QuarterCorner, SixUnilateral };

		// Evaluate the public split in double precision without calling production coordinate or row preparation.
		static std::array<double, 3> Coordinates(std::array<double, 4> const& q)
		{
			// These fixtures remain inside one regular, positive-w chart, so no branch-cut differencing is needed.
			auto h = std::hypot(q[1], q[3]);
			auto x = (q[3] * q[0] + q[1] * q[2]) / h;
			auto z = (q[3] * q[2] - q[0] * q[1]) / h;
			auto sine = std::hypot(x, z);
			auto scale = sine != 0.0 ? 2.0 * std::atan2(sine, h) / sine : 2.0;
			return {scale * x, scale * z, 2.0 * std::atan2(q[1], q[3])};
		}

		// Independently differentiate left quaternion motion along each physical parent-frame angular axis.
		static std::array<v4, 3> Gradients(quat orientation)
		{
			// Double quaternion products separate the oracle from FP32 production covectors and matrix conversion.
			auto q = std::array<double, 4>{orientation.x, orientation.y, orientation.z, orientation.w};
			auto gradients = std::array<v4, 3>{};
			auto step = 1.0e-5;
			for (int axis = 0; axis != 3; ++axis)
			{
				// Central differencing measures the three coordinate rates for one physical angular velocity.
				auto values = std::array<std::array<double, 3>, 2>{};
				for (int side = 0; side != 2; ++side)
				{
					// Left Hamilton multiplication advances a parent-frame rather than child-frame angular speed.
					auto a = std::array<double, 3>{};
					a[axis] = std::sin((side == 0 ? 0.5 : -0.5) * step);
					auto w = std::cos(0.5 * step);
					values[side] = Coordinates({
						w*q[0] + a[0]*q[3] + a[1]*q[2] - a[2]*q[1],
						w*q[1] + a[1]*q[3] + a[2]*q[0] - a[0]*q[2],
						w*q[2] + a[2]*q[3] + a[0]*q[1] - a[1]*q[0],
						w*q[3] - a[0]*q[0] - a[1]*q[1] - a[2]*q[2],
					});
				}
				for (int coordinate = 0; coordinate != 3; ++coordinate)
				{
					// Store covectors in the order swing X, swing Z, twist Y.
					gradients[coordinate][axis] = static_cast<float>((values[0][coordinate] - values[1][coordinate]) / (2.0 * step));
				}
			}
			return gradients;
		}

		// Exercise the actual coupled GPU preparation, block solve and impulse-ABA application for one spherical joint.
		void Check(ECase test_case, bool check_position_application = false)
		{
			// Construct a known point outside a noncircular ellipse along its analytic normal, fixing its closest-point oracle.
			auto theta = 0.7;
			auto normal = Normalise(v3{static_cast<float>(std::cos(theta) / 0.7), 0, static_cast<float>(std::sin(theta) / 0.4)});
			auto swing = v3{static_cast<float>(0.7 * std::cos(theta)), 0, static_cast<float>(0.4 * std::sin(theta))} + 0.1f * normal;
			auto orientation = ExpMap<quat>(0.5f * swing) * ExpMap<quat>(v3{0, 0.2f, 0});
			auto parameters = SphericalAngularConstraintParameters{.m_damping = 6.0f};
			auto expected_rows = 3u;
			auto initial_parent_speed = v4{2.0f, 0.8f, 1.2f, 0};
			switch (test_case)
			{
				case ECase::DampingOnly: { break; }
				case ECase::EllipseTwist:
				{
					// Both inequalities are geometrically active, alongside all three physical dampers.
					parameters.m_swing_x = {-0.7f, 0.7f};
					parameters.m_swing_z = {-0.4f, 0.4f};
					parameters.m_twist = {-0.2f, 0.2f};
					expected_rows = 5u;
					break;
				}
				case ECase::Locked:
				{
					// Two collapsed swing axes plus locked twist and three dampers occupy every slot.
					parameters.m_swing_x = {0, 0};
					parameters.m_swing_z = {0, 0};
					parameters.m_twist = {0, 0};
					orientation = quat::Identity();
					expected_rows = 6u;
					break;
				}
				case ECase::QuarterCorner:
				case ECase::SixUnilateral:
				{
					// A quadrant's two collapsed sides need two face equations, not one combined distance normal.
					parameters.m_swing_x = {0, 0.7f};
					parameters.m_swing_z = {0, 0.4f};
					orientation = ExpMap<quat>(v3{-0.1f, 0, -0.05f}) * ExpMap<quat>(v3{0, 0.2f, 0});
					initial_parent_speed = v4{-2.0f, 0.8f, -1.2f, 0};
					expected_rows = 5u;
					break;
				}
				default: { throw std::invalid_argument("Unknown spherical primitive test case"); }
			}

			// A geometrically active but separating twist keeps all six rows present without eliminating physical damping motion.
			switch (test_case)
			{
				case ECase::SixUnilateral:
				{
					// Negative twist speed separates from the upper twist boundary while the two lower swing faces require support.
					parameters.m_twist = {-0.2f, 0.2f};
					initial_parent_speed.y = -2.0f;
					expected_rows = 6u;
					break;
				}
				case ECase::DampingOnly:
				case ECase::EllipseTwist:
				case ECase::Locked:
				case ECase::QuarterCorner: { break; }
				default: { throw std::invalid_argument("Unknown spherical primitive test case"); }
			}

			// Nonidentity parent and child attachments prevent accidental agreement between world, link and joint coordinates.
			auto parent_frame = m4x4::Transform(v4::ZAxis(), 0.37f, v4::Origin());
			auto child_frame = m4x4::Transform(v4::XAxis(), -0.23f, v4::Origin());
			auto rotation = m4x4::Transform(orientation, v4::Origin()).rot;
			auto initial_child_speed = Transpose(rotation) * initial_parent_speed;
			auto inertia = Inertia::Box(v4{0.7f, 0.5f, 0.3f, 0}, 2.0f);
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0), parent_frame);
			auto joint = ArticulationJointDesc::Spherical(m4x4::Identity(), child_frame);
			joint.m_initial_position = {orientation.x, orientation.y, orientation.z, orientation.w};
			joint.m_initial_velocity = {initial_child_speed.x, initial_child_speed.y, initial_child_speed.z};
			auto child = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			source.Add(ConstraintDesc{
				.m_frame_a = BodyFrame{.m_body = BodyRef::World(), .m_constraint_to_body = parent_frame},
				.m_frame_b = BodyFrame{.m_body = BodyRef::Link(tree, child), .m_constraint_to_body = child_frame},
				.m_parameters = parameters,
			});
			auto remap = BodyRemap{{}, forest};
			auto compiled = CompileConstraints(source, remap);
			PR_EXPECT(compiled.m_blocks.size() == 1 && compiled.m_rows.size() == expected_rows);
			auto initial = PackGpuArticulations(forest);
			auto upload = PackGpuConstraints(source, remap);
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};

			// One exact-self block has no omitted cross-tree terms; use the existing unit-relaxation single-sweep diagnostic route.
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1.0f;
			config.push_out_iterations = 0;
			auto constraints = GpuConstraintSolver{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse, config};
			auto dt = 1.0f / 60.0f;
			auto result = velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, initial, {}, frames);
			PR_EXPECT(result.m_island_states.size() == 1 && result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			auto speed = result.m_articulation_velocities;
			auto final_child_speed = v4{speed[0], speed[1], speed[2], 0};
			auto final_parent_speed = rotation * final_child_speed;
			auto link_to_parent = rotation * Transpose(child_frame.rot);
			auto initial_link_speed = child_frame.rot * initial_child_speed;
			auto final_link_speed = child_frame.rot * final_child_speed;
			auto total_impulse = v4::Zero();
			auto damper_impulse = v4::Zero();
			for (int slot = 0; slot != 6; ++slot)
			{
				// Reconstruct physical impulse from actual committed GPU rows, not their reported scalar residual.
				auto const& row = result.m_rows[slot];
				total_impulse += row.bounds.z * row.jacobian_b_ang;
				if (slot >= 3)
					damper_impulse += row.bounds.z * row.jacobian_b_ang;
			}

			// The independently known centred inertia and orthonormal viscous law must both hold after application.
			auto momentum_error = Length(inertia.Ic3x3() * (final_link_speed - initial_link_speed) - total_impulse);
			auto damping_error = Length(damper_impulse + (dt * parameters.m_damping) * final_link_speed);
			PR_EXPECT(momentum_error < 2.0e-5f);
			PR_EXPECT(damping_error < 2.0e-5f);
			auto gradients = Gradients(orientation);
			auto limit_error = 0.0f;
			switch (test_case)
			{
				case ECase::DampingOnly:
				{
					// With isotropic damping, every principal inertia component has a closed-form implicit decay.
					auto moment = inertia.Ic3x3();
					for (int axis = 0; axis != 3; ++axis)
					{
						// No coordinate Jacobian appears in this physical equation.
						auto expected = moment[axis][axis] / (moment[axis][axis] + dt * parameters.m_damping) * initial_link_speed[axis];
						ExpectMidpointNear(final_link_speed[axis], expected, 2.0e-5f);
					}
					break;
				}
				case ECase::EllipseTwist:
				case ECase::QuarterCorner:
				case ECase::SixUnilateral:
				{
					// Known ellipse normal and independently differentiated split coordinates supply the hard-law oracle.
					auto hard_gradients = std::array{normal.x * gradients[0] + normal.z * gradients[1], gradients[2]};
					auto slots = std::array{0, 2};
					auto impulse_sign = -1.0f;
					switch (test_case)
					{
						case ECase::EllipseTwist: { break; }
						case ECase::QuarterCorner:
						case ECase::SixUnilateral:
						{
							// Lower coordinate faces oppose negative speed with positive impulse.
							hard_gradients = {gradients[0], gradients[1]};
							slots = {0, 1};
							impulse_sign = 1.0f;
							break;
						}
						default: { throw std::invalid_argument("Unknown unilateral spherical case"); }
					}
					auto active_support = false;
					for (int index = 0; index != 2; ++index)
					{
						// Upper limits require inward speed, nonpositive impulse, and complementarity in angular-velocity units.
						auto slot = slots[index];
						auto gradient_link = Transpose(link_to_parent) * hard_gradients[index];
						auto lambda = result.m_rows[slot].bounds.z;
						auto rate = Dot3(hard_gradients[index], final_parent_speed);
						auto response = Dot3(gradient_link, Invert(inertia.Ic3x3()) * gradient_link);
						auto support = impulse_sign * lambda * response;
						auto separation = impulse_sign * rate;
						limit_error = std::max({limit_error, -separation, -support, std::abs(std::min(support, separation))});
						PR_EXPECT(FEqlAbsolute(result.m_rows[slot].jacobian_b_ang, gradient_link, 2.0e-5f));
						active_support |= impulse_sign * lambda > 1.0e-3f;
					}
					PR_EXPECT(active_support && Length(damper_impulse) > 1.0e-3f);
					break;
				}
				case ECase::Locked:
				{
					// All three hard equations coexist with the viscous equations in the maximum six-row block.
					limit_error = Length(final_parent_speed);
					break;
				}
				default: { throw std::invalid_argument("Unknown spherical primitive test case"); }
			}
			PR_EXPECT(limit_error < 1.0e-4f);

			// Unlike the fully locked case, every damper must carry a nonzero impulse in the six-unilateral-row fixture.
			switch (test_case)
			{
				case ECase::SixUnilateral:
				{
					// The unused upper twist support must release while both lower faces and all physical damping components remain observable.
					PR_EXPECT(Dot3(gradients[2], final_parent_speed) < -0.1f);
					PR_EXPECT(result.m_rows[2].bounds.z == 0.0f);
					PR_EXPECT(FEqlAbsolute(result.m_rows[2].jacobian_b_ang, Transpose(link_to_parent) * gradients[2], 2.0e-5f));
					PR_EXPECT(result.m_rows[0].bounds.z > 1.0e-3f && result.m_rows[1].bounds.z > 1.0e-3f);
					for (int slot = 3; slot != 6; ++slot)
					{
						// Missing any orthonormal damper changes this observed physical result, rather than satisfying a zero-speed equation.
						PR_EXPECT(std::abs(result.m_rows[slot].bounds.z) > 1.0e-3f);
						ExpectMidpointNear(result.m_rows[slot].solve.w, 1.0f / (dt * parameters.m_damping), 1.0e-5f);
					}

					// Observe the real coupled masks too: three unilateral obligations and three viscous obligations occupy the full block.
					auto mobilities = mobility.Solve(gpu.m_job, initial, upload.m_coupled_articulation_indices);
					PR_EXPECT(mobilities.AllValid());
					auto prepared = prepare.Solve(gpu.m_job, dt, upload, {}, frames, mobilities.m_mobilities, result.m_articulation_scratch);
					PR_EXPECT(prepared.m_blocks[0].velocity_mask == 63u && prepared.m_blocks[0].position_mask == 7u);
					log_stream() << FmtS("six-row GPU obligations: velocity-mask=%u position-mask=%u impulses=(%g,%g,%g,%g,%g,%g)\n",
						prepared.m_blocks[0].velocity_mask, prepared.m_blocks[0].position_mask, result.m_rows[0].bounds.z, result.m_rows[1].bounds.z,
						result.m_rows[2].bounds.z, result.m_rows[3].bounds.z, result.m_rows[4].bounds.z, result.m_rows[5].bounds.z);
					break;
				}
				case ECase::DampingOnly:
				case ECase::EllipseTwist:
				case ECase::Locked:
				case ECase::QuarterCorner: { break; }
				default: { throw std::invalid_argument("Unknown spherical primitive test case"); }
			}

			// The ordinary rigid GPU route must enforce the same physical equations using world rather than link-coordinate rows.
			auto rigid = RigidBody{};
			rigid.SetMassProperties(inertia);
			rigid.O2W(tree.LinkToWorld(child));
			rigid.VelocityOS(v8motion{initial_link_speed, v4::Zero()});
			auto rigid_source = ConstraintSet{};
			rigid_source.Add(ConstraintDesc{
				.m_frame_a = BodyFrame{.m_body = BodyRef::World(), .m_constraint_to_body = parent_frame},
				.m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(rigid), .m_constraint_to_body = child_frame},
				.m_parameters = parameters,
			});
			auto rigid_ptrs = std::array{&rigid};
			auto rigid_upload = PackGpuConstraints(rigid_source, BodyRemap{rigid_ptrs});
			auto rigid_bodies = std::array{PackDynamics(rigid, -1)};
			auto initial_momentum = rigid_bodies[0].momentum_ang;
			auto rigid_rows = std::array<GpuConstraintRow, 6>{};
			auto rigid_blocks = std::array<GpuConstraintBlock, 1>{};
			constraints.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), rigid_upload, rigid_bodies, rigid_blocks, rigid_rows);
			PR_EXPECT(std::cmp_equal(std::popcount(rigid_blocks[0].velocity_mask), expected_rows));
			auto rigid_impulse = v4::Zero();
			auto rigid_damping = v4::Zero();
			for (int slot = 0; slot != 6; ++slot)
			{
				// Retained impulses must correspond to the same covectors after converting from the articulation link frame.
				rigid_impulse += rigid_rows[slot].bounds.z * rigid_rows[slot].jacobian_b_ang;
				if (slot >= 3)
					rigid_damping += rigid_rows[slot].bounds.z * rigid_rows[slot].jacobian_b_ang;

				ExpectMidpointNear(rigid_rows[slot].bounds.z, result.m_rows[slot].bounds.z, 2.0e-5f);
			}
			UnpackDynamics(rigid_bodies[0], rigid);
			auto rigid_speed_error = Length(rigid.VelocityOS().ang - final_link_speed);
			auto rigid_momentum_error = Length(rigid_bodies[0].momentum_ang - initial_momentum - rigid_impulse);
			auto rigid_damping_error = Length(rigid_damping + (dt * parameters.m_damping) * rigid.VelocityWS().ang);
			PR_EXPECT(rigid_speed_error < 2.0e-5f);
			PR_EXPECT(rigid_momentum_error < 2.0e-5f);
			PR_EXPECT(rigid_damping_error < 2.0e-5f);

			// The public typed CPU compilation and common CPU solver must reach the same physical velocity.
			auto cpu = CpuConstraintSolver{};
			auto cpu_config = CpuConstraintSolverConfig{};
			cpu_config.m_position_iterations = 0;
			cpu_config.m_warm_start_factor = 0;
			cpu_config.m_coupled_relaxation = 1;
			cpu.Solve(compiled, remap, dt, cpu_config);
			auto cpu_speed_error = 0.0f;
			for (int axis = 0; axis != 3; ++axis)
			{
				// Compare generalized speeds only after the independent physical-law checks.
				cpu_speed_error = std::max(cpu_speed_error, std::abs(tree.JointVelocity(child)[axis] - final_child_speed[axis]));
				ExpectMidpointNear(tree.JointVelocity(child)[axis], final_child_speed[axis], 2.0e-5f);
			}
			log_stream() << FmtS("spherical primitive case=%d rows=%u: momentum=%g damping=%g hard-law=%g omega_parent=(%g,%g,%g); rigid momentum=%g damping=%g speed-difference=%g; CPU speed-difference=%g\n",
				static_cast<int>(test_case), expected_rows, momentum_error, damping_error, limit_error, final_parent_speed.x, final_parent_speed.y, final_parent_speed.z,
				rigid_momentum_error, rigid_damping_error, rigid_speed_error, cpu_speed_error);

			if (check_position_application)
			{
				// A real spherical-limit position solve must apply only pseudo velocity, without changing the uploaded physical state.
				config.push_out_iterations = 1;
				auto position = GpuCoupledConstraintPosition{velocity, config};
				auto corrected = position.Solve(gpu.m_job, dt, upload, initial, {}, frames);
				PR_EXPECT(corrected.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
				PR_EXPECT(std::ranges::equal(corrected.m_articulation_velocities, initial.m_velocities));
				auto pseudo_wrench = v8force::Zero();
				for (int slot = 0; slot != 6; ++slot)
				{
					// Soft damping has no position impulse; only the actual hard rows contribute to pseudo motion.
					auto const& row = corrected.m_rows[slot];
					pseudo_wrench += row.bounds.w * v8force{row.jacobian_b_ang, row.jacobian_b_lin};
					if (slot >= 3)
						PR_EXPECT(row.bounds.w == 0.0f);
				}

				// Recover the physical linear response independently through CPU impulse ABA at the original configuration.
				tree.JointVelocity(child, initial.m_velocities);
				tree.ApplyImpulses(std::array{ArticulationImpulse{.m_link = child, .m_impulse = pseudo_wrench}});
				auto pseudo = v3::Zero();
				for (int axis = 0; axis != 3; ++axis)
				{
					// The response is in child joint angular coordinates, so it multiplies the quaternion on the right.
					pseudo[axis] = tree.JointVelocity(child)[axis] - initial.m_velocities[axis];
				}
				PR_EXPECT(Length(pseudo) > 0.1f);
				auto p = corrected.m_positions;
				auto actual_q = quat{p[0], p[1], p[2], p[3]};
				auto expected_rotation = rotation * m3x3::Rotation(dt * pseudo);
				auto actual_rotation = m4x4::Transform(actual_q, v4::Origin()).rot;
				PR_EXPECT(FEqlAbsolute(actual_rotation, expected_rotation, 2.0e-5f));

				// Independent split values show improvement, not a claim of final angular tolerance after one biased correction.
				auto before = Coordinates({orientation.x, orientation.y, orientation.z, orientation.w});
				auto after = Coordinates({actual_q.x, actual_q.y, actual_q.z, actual_q.w});
				auto ellipse_before = before[0] * before[0] / (0.7 * 0.7) + before[1] * before[1] / (0.4 * 0.4) - 1.0;
				auto ellipse_after = after[0] * after[0] / (0.7 * 0.7) + after[1] * after[1] / (0.4 * 0.4) - 1.0;
				PR_EXPECT(ellipse_after < ellipse_before && std::abs(after[2]) < std::abs(before[2]));
				log_stream() << FmtS("spherical limit position: ellipse normalized-square excess %g -> %g; twist %g -> %g rad; pseudo child=(%g,%g,%g); physical speeds unchanged\n",
					ellipse_before, ellipse_after, before[2], after[2], pseudo.x, pseudo.y, pseudo.z);
			}
		}

		// Damping is an isotropic physical torque at a rotated, nonzero spherical configuration.
		PRUnitTestMethod(DampingOnlyPhysicalEquation, Extended)
		{
			// Exercise the three physical viscous rows without any angular limit.
			Check(ECase::DampingOnly);
		}

		// Ellipse and twist inequalities coexist with the distinct physical dampers.
		PRUnitTestMethod(ActiveLimitsAndDamping, Extended)
		{
			// Remeasure both hard laws and all viscous equations after committed impulse application.
			Check(ECase::EllipseTwist);
		}

		// Two locked swing coordinates, locked twist and three dampers fill, but do not exceed, the shared row capacity.
		PRUnitTestMethod(LockedSixRowObligations, Extended)
		{
			// No hard or soft obligation may be silently dropped from the full block.
			Check(ECase::Locked);
		}

		// One-sided zero-width X and Z limits retain both physical face obligations.
		PRUnitTestMethod(QuarterCornerAndDamping, Extended)
		{
			// This non-smooth corner requires two independently measured unilateral laws.
			Check(ECase::QuarterCorner);
		}

		// Six velocity rows coexist with nonzero damping and a separating upper twist limit.
		PRUnitTestMethod(SixRowsWithObservableDamping, Extended)
		{
			// Make omission of every individual damper observable while exercising both one-sided swing faces.
			Check(ECase::SixUnilateral);
		}

		// A real spherical-limit correction uses the quaternion application route without applying damper pseudo impulses.
		PRUnitTestMethod(NonzeroLimitPositionApplication, Extended)
		{
			// Check the actual committed correction through an independent impulse response and right matrix drift.
			Check(ECase::EllipseTwist, true);
		}

		// Two finite-inertia endpoints exchange equal-and-opposite physical viscous impulse.
		PRUnitTestMethod(TwoMovingEndpointDamping, Extended)
		{
			// Unequal anisotropic inertias and unrelated body/joint frames distinguish relative physical damping from one-body or coordinate damping.
			auto inertias = std::array{Inertia::Box(v4{0.8f, 0.6f, 0.4f, 0}, 3.0f), Inertia::Box(v4{0.7f, 0.5f, 0.3f, 0}, 2.0f)};
			auto poses = std::array{m4x4::Transform(v4::ZAxis(), 0.37f, v4::Origin()), m4x4::Transform(ExpMap<quat>(v3{0.2f, -0.25f, 0.1f}), v4::Origin())};
			auto joints = std::array{m4x4::Transform(v4::XAxis(), -0.23f, v4::Origin()), m4x4::Transform(v4::YAxis(), 0.18f, v4::Origin())};
			auto initial_ws = std::array{v4{0.2f, -0.7f, 0.6f, 0}, v4{1.5f, 0.4f, -0.3f, 0}};
			auto initial_os = std::array{Transpose(poses[0].rot) * initial_ws[0], Transpose(poses[1].rot) * initial_ws[1]};
			auto builder_a = ArticulationBuilder{};
			auto builder_b = ArticulationBuilder{};
			auto root_a = builder_a.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertias[0]}, poses[0], v8motion{initial_os[0], v4::Zero()});
			auto root_b = builder_b.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = inertias[1]}, poses[1], v8motion{initial_os[1], v4::Zero()});
			auto tree_a = builder_a.Build();
			auto tree_b = builder_b.Build();
			auto forest = std::array{&tree_a, &tree_b};
			auto initial = PackGpuArticulations(forest);
			auto source = ConstraintSet{};
			source.Add(ConstraintDesc{
				.m_frame_a = BodyFrame{.m_body = BodyRef::Link(tree_a, root_a), .m_constraint_to_body = joints[0]},
				.m_frame_b = BodyFrame{.m_body = BodyRef::Link(tree_b, root_b), .m_constraint_to_body = joints[1]},
				.m_parameters = SphericalAngularConstraintParameters{.m_damping = 6},
			});
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto frames = std::array{PackGpuTransform(poses[0]), PackGpuTransform(poses[1])};
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1;
			config.push_out_iterations = 0;
			auto constraints = GpuConstraintSolver{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse, config};
			auto dt = 1.0f / 60.0f;
			auto result = velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, initial, {}, frames);
			PR_EXPECT(result.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);

			// Reconstruct both world impulses from committed rows, then compare each independently with its physical inertia change.
			auto final_ws = std::array<v4, 2>{};
			auto applied_ws = std::array<v4, 2>{};
			auto momentum_error = 0.0f;
			for (int body = 0; body != 2; ++body)
			{
				// Root speeds and row wrenches are local to their own body, not the other endpoint's joint frame.
				auto offset = initial.m_articulations[body].velocity_offset;
				auto speed = result.m_articulation_velocities;
				auto final_os = v4{speed[offset], speed[offset + 1], speed[offset + 2], 0};
				final_ws[body] = poses[body].rot * final_os;
				auto applied_os = v4::Zero();
				for (int slot = 3; slot != 6; ++slot)
				{
					// The three physical damping rows are the only active impulses in this fixture.
					auto const& row = result.m_rows[slot];
					applied_os += row.bounds.z * (body == 0 ? row.jacobian_a_ang : row.jacobian_b_ang);
				}
				applied_ws[body] = poses[body].rot * applied_os;
				momentum_error = std::max(momentum_error, Length(inertias[body].Ic3x3() * (final_os - initial_os[body]) - applied_os));
				PR_EXPECT(speed[offset + 3] == 0 && speed[offset + 4] == 0 && speed[offset + 5] == 0);
			}
			auto reaction_error = Length(applied_ws[0] + applied_ws[1]);
			auto damping_error = Length(applied_ws[1] + (dt * 6.0f) * (final_ws[1] - final_ws[0]));
			PR_EXPECT(momentum_error < 2.0e-5f && reaction_error < 2.0e-5f && damping_error < 2.0e-5f);
			PR_EXPECT(Length(applied_ws[1]) > 1.0e-2f);

			// A separate three-dimensional physical-inertia equation supplies the implicit relative-speed oracle.
			auto inverse_a = poses[0].rot * Invert(inertias[0].Ic3x3()) * Transpose(poses[0].rot);
			auto inverse_b = poses[1].rot * Invert(inertias[1].Ic3x3()) * Transpose(poses[1].rot);
			auto expected_relative = Invert(m3x3::Identity() + (dt * 6.0f) * (inverse_a + inverse_b)) * (initial_ws[1] - initial_ws[0]);
			auto reference_error = Length(final_ws[1] - final_ws[0] - expected_relative);
			PR_EXPECT(reference_error < 2.0e-5f);

			// The ordinary two-rigid-body route starts from the same free physical velocities and must exchange the same impulses.
			auto rigid_a = RigidBody{};
			auto rigid_b = RigidBody{};
			auto rigids = std::array{&rigid_a, &rigid_b};
			auto bodies = std::array<GpuRigidBody, 2>{};
			for (int body = 0; body != 2; ++body)
			{
				// Keep each body's centre fixed so only angular response enters the reference.
				rigids[body]->SetMassProperties(inertias[body]);
				rigids[body]->O2W(poses[body]);
				rigids[body]->VelocityOS(v8motion{initial_os[body], v4::Zero()});
				bodies[body] = PackDynamics(*rigids[body], -1);
			}
			auto rigid_source = ConstraintSet{};
			rigid_source.Add(ConstraintDesc{
				.m_frame_a = BodyFrame{.m_body = BodyRef::Rigid(rigid_a), .m_constraint_to_body = joints[0]},
				.m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(rigid_b), .m_constraint_to_body = joints[1]},
				.m_parameters = SphericalAngularConstraintParameters{.m_damping = 6},
			});
			auto rigid_upload = PackGpuConstraints(rigid_source, BodyRemap{rigids});
			auto rows = std::array<GpuConstraintRow, 6>{};
			constraints.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), rigid_upload, bodies, {}, rows);
			auto parity_error = 0.0f;
			auto rigid_applied = std::array<v4, 2>{};
			for (int body = 0; body != 2; ++body)
			{
				// Both endpoints must agree, rather than only their relative speed.
				UnpackDynamics(bodies[body], *rigids[body]);
				parity_error = std::max(parity_error, Length(rigids[body]->VelocityWS().ang - final_ws[body]));
				for (int slot = 3; slot != 6; ++slot)
				{
					// Rigid runtime wrenches are already in world coordinates.
					rigid_applied[body] += rows[slot].bounds.z * (body == 0 ? rows[slot].jacobian_a_ang : rows[slot].jacobian_b_ang);
				}
				PR_EXPECT(Length(rigid_applied[body] - applied_ws[body]) < 2.0e-5f);
			}
			PR_EXPECT(parity_error < 2.0e-5f);
			PR_EXPECT(Length(rigid_applied[0] + rigid_applied[1]) < 2.0e-5f);
			PR_EXPECT(Length(rigid_applied[1] + (dt * 6.0f) * (rigid_b.VelocityWS().ang - rigid_a.VelocityWS().ang)) < 2.0e-5f);
			log_stream() << FmtS("two-moving damping: momentum=%g reaction=%g viscous=%g direct-relative=%g rigid-speed=%g; impulse-B=(%g,%g,%g)\n",
				momentum_error, reaction_error, damping_error, reference_error, parity_error, applied_ws[1].x, applied_ws[1].y, applied_ws[1].z);
		}

		// Undefined limited coordinates reject the whole solve, without preventing physical damping at unrestricted orientations.
		PRUnitTestMethod(LimitedAntipodeRejectionAndRecovery, Extended)
		{
			// Exact X half-turns produce y=w=0 without relying on a rounded near-cut configuration.
			auto inertia = Inertia::Box(v4{0.7f, 0.5f, 0.3f, 0}, 2.0f);
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Spherical(m4x4::Identity(), m4x4::Identity());
			joint.m_initial_position = {1, 0, 0, 0};
			joint.m_initial_velocity = {0.4f, 0.8f, 1.2f};
			auto child = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto limited = ConstraintSet{};
			auto unrestricted = ConstraintSet{};
			auto frame = BodyFrame{.m_body = BodyRef::Link(tree, child)};
			limited.Add(ConstraintDesc{
				.m_frame_b = frame,
				.m_parameters = SphericalAngularConstraintParameters{.m_swing_x = {-0.7f, 0.7f}, .m_swing_z = {-0.4f, 0.4f}, .m_twist = {-0.2f, 0.2f}, .m_damping = 6},
			});
			unrestricted.Add(ConstraintDesc{.m_frame_b = frame, .m_parameters = SphericalAngularConstraintParameters{.m_damping = 6}});
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1;
			config.push_out_iterations = 0;
			auto constraints = GpuConstraintSolver{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse, config};
			auto dt = 1.0f / 60.0f;
			auto run = [&](ConstraintSet const& source, quat orientation)
			{
				// Pack valid configuration inputs; only the limited coordinate chart, not the quaternion itself, is undefined.
				tree.JointPosition(child, std::array{orientation.x, orientation.y, orientation.z, orientation.w});
				auto initial = PackGpuArticulations(forest);
				auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
				auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
				return velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, initial, {}, frames);
			};
			auto rejected = run(limited, quat{1, 0, 0, 0});
			PR_EXPECT(rejected.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(std::ranges::equal(rejected.m_articulation_velocities, tree.JointVelocity(child)));
			for (auto const& row : rejected.m_rows)
			{
				// No subset of physical dampers may commit when a limit's preparation rejects.
				PR_EXPECT(row.bounds.z == 0.0f);
			}

			// Reuse the failed lane at an ordinary limited configuration, then at the same unrestricted antipode.
			auto recovered = run(limited, ExpMap<quat>(v3{0.05f, 0.025f, -0.04f}));
			auto allowed = run(unrestricted, quat{1, 0, 0, 0});
			for (auto const* result : {&recovered, &allowed})
			{
				// Both successful cases must actually damp, with no coordinate-gradient factor in their physical torque.
				PR_EXPECT(result->m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
				auto const& speed = result->m_articulation_velocities;
				auto final = v4{speed[0], speed[1], speed[2], 0};
				auto initial = v4{0.4f, 0.8f, 1.2f, 0};
				auto applied = v4::Zero();
				for (int slot = 3; slot != 6; ++slot)
				{
					// Child joint and link frames coincide in this rejection fixture.
					applied += result->m_rows[slot].bounds.z * result->m_rows[slot].jacobian_b_ang;
				}
				auto momentum_error = Length(inertia.Ic3x3() * (final - initial) - applied);
				auto damping_error = Length(applied + (dt * 6.0f) * final);
				PR_EXPECT(Length(final - initial) > 0.1f);
				PR_EXPECT(momentum_error < 2.0e-5f && damping_error < 2.0e-5f);
				log_stream() << FmtS("chart success control: momentum=%g damping=%g speed=(%g,%g,%g)\n", momentum_error, damping_error, final.x, final.y, final.z);
			}
			log_stream() << "limited exact antipode rejected with unchanged physical speeds and zero committed row impulses; ordinary recovery and unrestricted antipode committed\n";
		}

		// Fresh coupled warm starts discard changed-feature support while retaining unchanged physical damping.
		PRUnitTestMethod(CoupledFeatureSwitchWarmStartAndRelease, Extended)
		{
			// The same generational owner moves from an upper X side to an upper ellipse in slot zero.
			auto orientation = ExpMap<quat>(v3{0.05f, 0, 0.025f});
			auto child_speed = Transpose(m4x4::Transform(orientation, v4::Origin()).rot) * v4{2.0f, 0.8f, 0.6f, 0};
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Spherical(m4x4::Identity(), m4x4::Identity());
			joint.m_initial_position = {orientation.x, orientation.y, orientation.z, orientation.w};
			joint.m_initial_velocity = {child_speed.x, child_speed.y, child_speed.z};
			auto child = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = Inertia::Box(v4{0.7f, 0.5f, 0.3f, 0}, 2.0f)});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			source.Add(ConstraintDesc{
				.m_frame_b = BodyFrame{.m_body = BodyRef::Link(tree, child)},
				.m_parameters = SphericalAngularConstraintParameters{.m_swing_x = {-0.7f, 0}, .m_swing_z = {-0.4f, 0.4f}, .m_damping = 6},
			});
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			PR_EXPECT((upload.m_endpoints[0].flags & GpuConstraintEndpointFlags_ResetWarmStart) == 0u);
			auto initial = PackGpuArticulations(forest);
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1;
			config.push_out_iterations = 0;
			auto constraints = GpuConstraintSolver{gpu, config};
			auto aba = GpuArticulationForceAba{gpu};
			auto mobility = GpuArticulationMobility{aba};
			auto impulse = GpuArticulationImpulseAba{gpu, aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto velocity = GpuCoupledConstraintVelocity{prepare, impulse, config};
			auto dt = 1.0f / 60.0f;
			auto solved = velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, initial, {}, frames);
			PR_EXPECT(solved.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			auto old_mobility = mobility.Solve(gpu.m_job, initial, upload.m_coupled_articulation_indices);
			PR_EXPECT(old_mobility.AllValid());
			auto old = prepare.Solve(gpu.m_job, dt, upload, {}, frames, old_mobility.m_mobilities, solved.m_articulation_scratch);
			PR_EXPECT(old.m_rows[0].bounds.z < -1.0e-3f && std::abs(old.m_rows[4].bounds.z) > 1.0e-3f);

			// A separate cold lane supplies new-configuration factors without overwriting the warm owner's retained rows.
			orientation = ExpMap<quat>(v3{-0.15f, 0, 0.225f});
			tree.JointPosition(child, std::array{orientation.x, orientation.y, orientation.z, orientation.w});
			tree.JointVelocity(child, std::array{0.0f, 0.0f, 0.0f});
			auto changed = PackGpuArticulations(forest);
			frames = {PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
			auto cold_constraints = GpuConstraintSolver{gpu, config};
			auto cold_aba = GpuArticulationForceAba{gpu};
			auto cold_mobility = GpuArticulationMobility{cold_aba};
			auto cold_impulse = GpuArticulationImpulseAba{gpu, cold_aba, cold_mobility};
			auto cold_prepare = GpuCoupledConstraintPrepare{cold_constraints, config};
			auto cold_velocity = GpuCoupledConstraintVelocity{cold_prepare, cold_impulse, config};
			auto cold = cold_velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, changed, {}, frames);
			auto new_mobility = cold_mobility.Solve(gpu.m_job, changed, upload.m_coupled_articulation_indices);
			PR_EXPECT(cold.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed && new_mobility.AllValid());

			// This diagnostic uses fresh-solve preparation, not retain_current_impulses recompilation or already warm-applied input.
			auto replay = prepare.Solve(gpu.m_job, dt, upload, {}, frames, new_mobility.m_mobilities, cold.m_articulation_scratch);
			PR_EXPECT((old.m_blocks[0].row_states & 3u) == (replay.m_blocks[0].row_states & 3u));
			PR_EXPECT((old.m_blocks[0].row_identities & 15u) == 1u && (replay.m_blocks[0].row_identities & 15u) == 3u);
			PR_EXPECT(replay.m_rows[0].bounds.z == 0.0f);
			ExpectMidpointNear(replay.m_rows[4].bounds.z, config.constraint_warm_start_factor * old.m_rows[4].bounds.z, 1.0e-6f);
			PR_EXPECT(std::abs(replay.m_rows[4].bounds.z) > 1.0e-3f);

			// A complete warm solve must release support and match the cold solve from the identical zero free input.
			auto warm = velocity.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, changed, {}, frames);
			PR_EXPECT(warm.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			auto speed_error = 0.0f;
			for (int index = 0; index != isize(changed.m_velocities); ++index)
			{
				// Compare authoritative generalized speeds, not only cache contents.
				speed_error = std::max(speed_error, std::abs(warm.m_articulation_velocities[index] - cold.m_articulation_velocities[index]));
				PR_EXPECT(std::abs(warm.m_articulation_velocities[index]) < 2.0e-5f);
			}
			for (int slot = 0; slot != 6; ++slot)
			{
				// No physical normal or viscous support is required at zero free speed.
				ExpectMidpointNear(warm.m_rows[slot].bounds.z, cold.m_rows[slot].bounds.z, 2.0e-5f);
				PR_EXPECT(std::abs(warm.m_rows[slot].bounds.z) < 2.0e-5f);
			}
			PR_EXPECT(speed_error < 2.0e-5f);
			log_stream() << FmtS("coupled fresh feature replay: state=%u identity=%u->%u support=%g->%g retained damper=%g; warm/cold speed error=%g\n",
				old.m_blocks[0].row_states & 3u, old.m_blocks[0].row_identities & 15u, replay.m_blocks[0].row_identities & 15u,
				old.m_rows[0].bounds.z, replay.m_rows[0].bounds.z, replay.m_rows[4].bounds.z, speed_error);
		}

		// Retained support follows a boundary feature, not merely a slot number and upper/lower state.
		PRUnitTestMethod(FeatureSwitchWarmStartAndRelease, Extended)
		{
			// An upper X side initially occupies slot zero; an upper curved boundary later reuses the same slot and limit state.
			auto rigid = RigidBody{};
			rigid.SetMassProperties(Inertia::Box(v4{0.7f, 0.5f, 0.3f, 0}, 2.0f));
			rigid.O2W(m4x4::Transform(ExpMap<quat>(v3{0.05f, 0, 0.025f}), v4::Origin()));
			rigid.VelocityWS(v8motion{v4{2.0f, 0.8f, 0.6f, 0}, v4::Zero()});
			auto source = ConstraintSet{};
			source.Add(ConstraintDesc{
				.m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(rigid)},
				.m_parameters = SphericalAngularConstraintParameters{
					.m_swing_x = {-0.7f, 0},
					.m_swing_z = {-0.4f, 0.4f},
					.m_damping = 6,
				},
			});
			auto rigid_ptrs = std::array{&rigid};
			auto upload = PackGpuConstraints(source, BodyRemap{rigid_ptrs});
			auto initial = std::array{PackDynamics(rigid, -1)};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			auto& gpu = MidpointTestGpu();
			auto solver = GpuConstraintSolver{gpu, config};
			auto rows = std::array<GpuConstraintRow, 6>{};
			auto blocks = std::array<GpuConstraintBlock, 1>{};
			auto dt = 1.0f / 60.0f;
			solver.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, initial, blocks, rows);
			auto old_rows = rows;
			auto old_block = blocks[0];
			PR_EXPECT(rows[0].bounds.z < -1.0e-3f && std::abs(rows[4].bounds.z) > 1.0e-3f);

			// Model an ordinary configuration change without the teleport/reset flag, leaving the descriptor and generational owner unchanged.
			auto changed = std::array{PackDynamics(rigid, -1)};
			changed[0].o2w = m4x4::Transform(ExpMap<quat>(v3{-0.15f, 0, 0.225f}), v4::Origin());
			changed[0].momentum_ang = float4{};
			changed[0].state_flags &= ~ERigidBodyStateFlags_ResetWarmStart;
			auto new_input = changed;
			config.solver_iterations = 0;
			solver.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, changed, blocks, rows);
			PR_EXPECT((old_block.row_states & 3u) == (blocks[0].row_states & 3u));
			PR_EXPECT((old_block.row_identities & 15u) != (blocks[0].row_identities & 15u));
			PR_EXPECT(rows[0].bounds.z == 0.0f);
			ExpectMidpointNear(rows[4].bounds.z, config.constraint_warm_start_factor * old_rows[4].bounds.z, 1.0e-6f);
			PR_EXPECT(std::abs(rows[4].bounds.z) > 1.0e-3f);

			// With zero free speed, a complete solve must release old support and agree with a cold solve at the changed configuration.
			config.solver_iterations = 8;
			auto warm = new_input;
			solver.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, warm, blocks, rows);
			auto cold = new_input;
			auto cold_solver = GpuConstraintSolver{gpu, config};
			auto cold_rows = std::array<GpuConstraintRow, 6>{};
			cold_solver.Solve(gpu.m_job, physics::tests::FreshConstraintTestEpoch(dt, config.constraint_warm_start_factor), upload, cold, {}, cold_rows);
			PR_EXPECT(Length(warm[0].momentum_ang - cold[0].momentum_ang) < 2.0e-5f);
			PR_EXPECT(Length(warm[0].momentum_ang) < 2.0e-5f);
			for (int slot = 0; slot != 6; ++slot)
			{
				// Zero input requires no retained physical support or viscous impulse.
				ExpectMidpointNear(rows[slot].bounds.z, cold_rows[slot].bounds.z, 2.0e-5f);
				PR_EXPECT(std::abs(rows[slot].bounds.z) < 2.0e-5f);
			}
			log_stream() << FmtS("spherical feature switch: state=%u, identity=%u -> %u, old support=%g; warm/cold angular momentum difference=%g\n",
				old_block.row_states & 3u, old_block.row_identities & 15u, blocks[0].row_identities & 15u,
				old_rows[0].bounds.z, Length(warm[0].momentum_ang - cold[0].momentum_ang));
		}
	};

	// Current-frame relinearization must preserve physical impulse accounting, independently of fresh warm starts.
	// Actual coupled publication preserves the distinct physical, context, and existing numerical rejection boundaries.
	PRUnitTestClass(CoupledTransactionOutcomeTests)
	{
		// A valid new-epoch scale overflow rejects the provisional attempt, while changed accepted-epoch dt remains fatal.
		PRUnitTestMethod(WarmScaleOverflowPreservesOwner, Extended)
		{
			// Include both a native tree and a rigid target so rejection must preserve both physical representations and their records.
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 1)});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto rigid = RigidBody{};
			rigid.SetMassProperties(Inertia::Sphere(1, 1));
			rigid.O2W(m4x4::Identity());
			auto native_bodies = std::array{&rigid};
			auto bodies = std::array{PackDynamics(rigid, 0)};
			bodies[0].state_flags &= ~ERigidBodyStateFlags_ResetWarmStart;
			auto desc = ConstraintDesc{.m_frame_a = {.m_body = BodyRef::Link(tree, root)}, .m_frame_b = {.m_body = BodyRef::Rigid(rigid)}};
			desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
			auto source = ConstraintSet{};
			source.Add(desc);
			auto upload = PackGpuConstraints(source, BodyRemap{native_bodies, forest});
			auto packed = PackGpuArticulations(forest);
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root))};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_coupled_relaxation = 1;
			auto& gpu = MidpointTestGpu();
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 32), .timestep = 1.0f / 60.0f, .warm_start_factor = 1};
			auto accepted = fixture.Run([&]
			{
				// A physical solve, not a fabricated cache record, establishes the historical warm provenance.
				fixture.Prepare(epoch, upload, packed, bodies, frames);
				fixture.Coupled();
			});
			PR_EXPECT(accepted.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(std::abs(accepted.m_accepted_rows.m_values[0].impulses[0] - 0.5f) < 1.0e-5f);

			// Changing only epoch identity distinguishes valid provisional overflow from invalid accepted-dt context.
			packed.m_velocities = accepted.m_velocities.m_values;
			epoch.timestep *= 2;
			epoch.warm_start_factor = 2.0e38f;
			for (int phase = 0; phase != 2; ++phase)
			{
				// Phase zero is a fresh epoch; phase one reuses the original accepted identity with the wrong dt.
				epoch.identity.x = phase == 0 ? 2u : 1u;
				auto before = CoupledTransactionSnapshot{};
				auto rejected = fixture.Run([&]
				{
					// The snapshot is taken after preparation, immediately before the atomic attempt.
					fixture.Prepare(epoch, upload, packed, accepted.m_bodies.m_values, frames);
					before = fixture.Capture();
					fixture.Coupled();
				});
				before.Read();
				auto expected = phase == 0 ? GpuConstraintStage_AdmissionRejected : GpuConstraintStage_NumericalFailure;
				auto failure = rejected.m_solver_state.m_values[0].failure_slot_plus_one;
				log_stream() << "coupled warm-scale phase=" << phase << ": outcome=" << rejected.m_results.m_values[0] << ", fatal_slot=" << failure << '\n';
				PR_EXPECT(rejected.m_results.m_values[0] == expected && failure == (phase == 0 ? 0u : 1u));
				PR_EXPECT(memcmp(before.m_bodies.m_values.data(), rejected.m_bodies.m_values.data(), sizeof(GpuRigidBody)) == 0);
				PR_EXPECT(before.m_velocities.m_values == rejected.m_velocities.m_values);
				PR_EXPECT(memcmp(before.m_scratch.m_values.data(), rejected.m_scratch.m_values.data(), sizeof(GpuArticulationAbaScratch)) == 0);
				PR_EXPECT(memcmp(before.m_acceptance.m_values.data(), rejected.m_acceptance.m_values.data(), sizeof(GpuConstraintAcceptance)) == 0);
				PR_EXPECT(memcmp(before.m_accepted_rows.m_values.data(), rejected.m_accepted_rows.m_values.data(), sizeof(GpuConstraintAcceptedRows)) == 0);
				PR_EXPECT(memcmp(before.m_rigid_response.m_values.data(), rejected.m_rigid_response.m_values.data(), sizeof(GpuArticulationSpatialVector)) == 0);
				PR_EXPECT(before.m_velocity_response.m_values == rejected.m_velocity_response.m_values);
				PR_EXPECT(memcmp(before.m_ordinary_records.m_values.data(), rejected.m_ordinary_records.m_values.data(), sizeof(GpuConstraintRigidRecord)) == 0);
			}
		}

		// Ordinary valid drives can trigger the existing merit rejection without any injected device or solver fault.
		PRUnitTestMethod(MeritRejectionPreservesOwner, Extended)
		{
			// Three separate unit-mass X drives share one floating link, so their simultaneous responses add.
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 1)});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			for (int index = 0; index != 3; ++index)
			{
				// Each block connects world to the link; no unsupported self-constraint or softness is introduced.
				auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Link(tree, root)}};
				desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = 1};
				source.Add(desc);
			}
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			PR_EXPECT(upload.m_coupled_islands.size() == 1 && upload.m_endpoints.size() == 3);
			auto packed = PackGpuArticulations(forest);
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root))};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_coupled_relaxation = 0.25f;
			config.constraint_coupled_backtrack_limit = 0;
			auto& gpu = MidpointTestGpu();
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 33), .timestep = 1.0f / 60.0f, .warm_start_factor = 1};
			auto accepted = fixture.Run([&]
			{
				// Three quarter-impulses produce speed .75 and an independently negative quadratic merit change.
				fixture.Prepare(epoch, upload, packed, {}, frames);
				fixture.Coupled();
			});
			PR_EXPECT(accepted.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(std::abs(accepted.m_velocities.m_values[3] - 0.75f) < 1.0e-5f);
			PR_EXPECT(std::abs(accepted.m_islands.m_values[0].merit_change + 0.46875f) < 1.0e-5f);
			auto accepted_speed = accepted.m_velocities.m_values[3];
			auto accepted_merit = accepted.m_islands.m_values[0].merit_change;
			for (auto const& row : accepted.m_accepted_rows.m_values)
				PR_EXPECT(std::abs(row.impulses[0] - 0.25f) < 1.0e-5f);

			// With no new input or preparation, full relaxation proposes another .75: speed 1.5 and merit +.09375.
			config.constraint_coupled_relaxation = 1;
			auto before = CoupledTransactionSnapshot{};
			auto rejected = fixture.Run([&]
			{
				// Reuse the same actual buffers and accepted epoch, so only the numerical admission decision changes.
				before = fixture.Capture();
				fixture.Coupled();
			});
			before.Read();
			auto const& island = rejected.m_islands.m_values[0];
			log_stream() << "merit rejection: accepted speed=" << accepted_speed << ", accepted merit=" << accepted_merit
				<< ", rejected merit=" << island.merit_change << ", outcome=" << rejected.m_results.m_values[0] << ", flags=" << island.failure_flags
				<< ", fatal_slot=" << rejected.m_solver_state.m_values[0].failure_slot_plus_one << '\n';
			PR_EXPECT(rejected.m_results.m_values[0] == GpuConstraintStage_SolveRejected);
			PR_EXPECT(island.status == GpuCoupledConstraintIslandStatus_Rejected && island.failure_flags == GpuCoupledConstraintFailure_Merit);
			PR_EXPECT(std::abs(island.merit_change - 0.09375f) < 1.0e-5f);
			PR_EXPECT(island.merit_change > 64.0f * std::numeric_limits<float>::epsilon() * (1.0f + std::abs(island.merit_change)));
			PR_EXPECT(rejected.m_solver_state.m_values[0].failure_slot_plus_one == 0u);
			PR_EXPECT(before.m_velocities.m_values == rejected.m_velocities.m_values);
			PR_EXPECT(memcmp(before.m_scratch.m_values.data(), rejected.m_scratch.m_values.data(), sizeof(GpuArticulationAbaScratch)) == 0);
			PR_EXPECT(memcmp(before.m_acceptance.m_values.data(), rejected.m_acceptance.m_values.data(), sizeof(GpuConstraintAcceptance)) == 0);
			PR_EXPECT(memcmp(before.m_accepted_rows.m_values.data(), rejected.m_accepted_rows.m_values.data(), 3 * sizeof(GpuConstraintAcceptedRows)) == 0);
			PR_EXPECT(before.m_velocity_response.m_values == rejected.m_velocity_response.m_values);
			PR_EXPECT(memcmp(before.m_ordinary_records.m_values.data(), rejected.m_ordinary_records.m_values.data(), 3 * sizeof(GpuConstraintRigidRecord)) == 0);
		}
	};

	PRUnitTestClass(SphericalContinuationTests)
	{
		// Zero net response still owns coefficients, and a bad child factor is distinct from provisional physical overflow.
		PRUnitTestMethod(NullResponseRetirementAndInvalidChildContext, Extended)
		{
			// Two different valid bounded motors on the same link produce cancelling support with nonzero per-block coefficients.
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto child = builder.AddLink(root, ArticulationJointDesc::Spherical(), ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 2.5f)});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			auto handles = std::array<ConstraintHandle, 2>{};
			for (int index = 0; index != 2; ++index)
			{
				// This does not introduce a same-endpoint self-constraint: each block connects world to one link.
				auto desc = ConstraintDesc{.m_frame_b = {.m_body = BodyRef::Link(tree, child)}};
				desc.D6().m_angular[2] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = index == 0 ? 1.0f : -1.0f, .m_max_force = 6.0f};
				handles[index] = source.Add(desc);
			}
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto packed = PackGpuArticulations(forest);
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_coupled_relaxation = 1;
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 24), .timestep = 1.0f / 60.0f, .warm_start_factor = 1};
			auto supported = fixture.Run([&]
			{
				// The actual coupled reduction cancels the two motor responses before applying them.
				fixture.Prepare(epoch, upload, packed, {}, frames);
				fixture.Coupled();
			});
			PR_EXPECT(supported.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(supported.m_accepted_rows.m_values[0].impulses[5] > 0.09f && supported.m_accepted_rows.m_values[1].impulses[5] < -0.09f);
			for (auto response : supported.m_velocity_response.m_values)
				PR_EXPECT(response == 0.0f);

			// Clear row activity while keeping enabled slots, native handles, and the island membership stable.
			for (auto handle : handles)
			{
				// An accepted visit must retire cache/load state even though its old response record is all zero.
				auto desc = source.Get(handle);
				desc.D6().m_angular[2] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Limited, .m_limits = Range<float>{-1, 1}};
				source.Update(handle, desc);
			}
			upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto retired = fixture.Run([&]
			{
				// No response-magnitude shortcut is allowed to omit this visit.
				fixture.Prepare(epoch, upload, packed, {}, frames);
				fixture.Coupled();
			});
			PR_EXPECT(retired.m_results.m_values[0] == GpuConstraintStage_Accepted);
			for (auto const& row : retired.m_accepted_rows.m_values)
			{
				// Coefficients and the detector's accepted load definition are both retired.
				PR_EXPECT(row.velocity_mask == 0u && row.linear_impulse_norm == 0.0f && row.angular_impulse_norm == 0.0f);
				for (auto value : row.impulses)
					PR_EXPECT(value == 0.0f);
			}

			// Inject only a child factor-validity failure after normal preparation; this is a scoped diagnostic fault, not an API input contract.
			auto before = CoupledTransactionSnapshot{};
			auto invalid = fixture.Run([&]
			{
				// The root remains valid, so a root-only context check would miss the fault.
				fixture.Prepare(epoch, upload, packed, {}, frames);
				auto scratch = fixture.m_impulse.VelocityBuffers().m_scratch;
				auto data = gpu.m_job.m_upload.Alloc<int>(1);
				*data.ptr<int>() = 0;
				gpu.m_job.m_barriers.Transition(scratch, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				gpu.m_job.m_cmd_list.CopyBufferRegion(scratch, sizeof(GpuArticulationAbaScratch) + offsetof(GpuArticulationAbaScratch, solve_valid), data);
				before = fixture.Capture();
				fixture.Coupled();
			});
			before.Read();
			PR_EXPECT(before.m_scratch.m_values[0].solve_valid != 0 && before.m_scratch.m_values[1].solve_valid == 0);
			PR_EXPECT(invalid.m_results.m_values[0] == GpuConstraintStage_NumericalFailure);
			PR_EXPECT(invalid.m_solver_state.m_values[0].failure_slot_plus_one != 0u);
			PR_EXPECT(memcmp(before.m_scratch.m_values.data(), invalid.m_scratch.m_values.data(), 2 * sizeof(GpuArticulationAbaScratch)) == 0);
			PR_EXPECT(memcmp(before.m_accepted_rows.m_values.data(), invalid.m_accepted_rows.m_values.data(), 2 * sizeof(GpuConstraintAcceptedRows)) == 0);
			PR_EXPECT(memcmp(before.m_acceptance.m_values.data(), invalid.m_acceptance.m_values.data(), sizeof(GpuConstraintAcceptance)) == 0);
			PR_EXPECT(before.m_velocities.m_values == invalid.m_velocities.m_values && before.m_velocity_response.m_values == invalid.m_velocity_response.m_values);
		}

		// Independent coupled islands share rigid endpoints with an ordinary bridge without sharing their acceptance decisions.
		PRUnitTestMethod(MixedOwnersRejectRetryAndPreserveSleep, Extended)
		{
			// Two one-link floating trees and two unit-mass rigid bodies make every X response independently calculable.
			auto build = []
			{
				// Each tree is a distinct validated owner, not a synthetic same-body constraint.
				auto builder = ArticulationBuilder{};
				auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(1, 1)});
				return std::pair{builder.Build(), root};
			};
			auto [tree_a, root_a] = build();
			auto [tree_b, root_b] = build();
			auto forest = std::array{&tree_a, &tree_b};
			auto rigid_a = RigidBody{};
			auto rigid_b = RigidBody{};
			rigid_a.SetMassProperties(Inertia::Sphere(1, 1));
			rigid_b.SetMassProperties(Inertia::Sphere(1, 1));
			rigid_a.O2W(m4x4::Identity());
			rigid_b.O2W(m4x4::Identity());
			auto native_bodies = std::array{&rigid_a, &rigid_b};
			auto bodies = std::array{PackDynamics(rigid_a, 0), PackDynamics(rigid_b, 1)};
			for (auto& body : bodies)
			{
				// No descriptor/body invalidation hides use of the historical coupled cache in the rejection control.
				body.state_flags &= ~ERigidBodyStateFlags_ResetWarmStart;
			}
			auto drive = [](BodyRef a, BodyRef b, float target)
			{
				// Each valid block has distinct endpoints; shared-body overlap occurs only across different blocks.
				auto desc = ConstraintDesc{.m_frame_a = {.m_body = a}, .m_frame_b = {.m_body = b}};
				desc.D6().m_linear[0] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Driven, .m_target_velocity = target};
				return desc;
			};
			auto source = ConstraintSet{};
			source.Add(drive(BodyRef::Link(tree_a, root_a), BodyRef::Rigid(rigid_a), 8));
			source.Add(drive(BodyRef::Rigid(rigid_a), BodyRef::Rigid(rigid_b), 2));
			source.Add(drive(BodyRef::Link(tree_b, root_b), BodyRef::Rigid(rigid_b), 0));
			auto upload = PackGpuConstraints(source, BodyRemap{native_bodies, forest});
			PR_EXPECT(upload.m_coupled_islands.size() == 2);
			PR_EXPECT(upload.m_coupled_block_topology[0].island_idx == 0 && upload.m_coupled_block_topology[1].island_idx == -1 && upload.m_coupled_block_topology[2].island_idx == 1);
			auto packed_forest = PackGpuArticulations(forest);
			auto frames = std::array{PackGpuTransform(tree_a.LinkToWorld(root_a)), PackGpuTransform(tree_b.LinkToWorld(root_b))};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_relaxation = 1;
			config.constraint_coupled_relaxation = 1;
			auto& gpu = MidpointTestGpu();
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 23), .timestep = 1.0f / 60.0f, .warm_start_factor = 1};
			auto first = fixture.Run([&]
			{
				// C0 applies +4 to R0; the ordinary bridge then applies (-3,+3) on the very same physical buffer.
				fixture.Prepare(epoch, upload, packed_forest, bodies, frames);
				fixture.Coupled();
				fixture.Ordinary();
			});
			PR_EXPECT(first.m_bodies.m_values[0].momentum_lin.x == 1.0f && first.m_bodies.m_values[1].momentum_lin.x == 3.0f);
			PR_EXPECT(first.m_accepted_rows.m_values[0].impulses[0] == 4.0f && first.m_accepted_rows.m_values[2].impulses[0] == 0.0f);
			PR_EXPECT(first.m_ordinary_records.m_values[1].response_a.lin.x == -3.0f && first.m_ordinary_records.m_values[1].response_b.lin.x == 3.0f);

			// A finite caller-supplied warm factor overflows C0's nonzero historical guess, while C1's zero guess remains valid.
			++epoch.identity.x;
			epoch.warm_start_factor = 1.0e38f;
			packed_forest.m_velocities = first.m_velocities.m_values;
			auto before = CoupledTransactionSnapshot{};
			auto rejected = CoupledTransactionSnapshot{};
			auto bridge = CoupledTransactionSnapshot{};
			auto final = fixture.Run([&]
			{
				// Capture before the rejectable attempt; no live field or accepted member of C0 may change.
				fixture.Prepare(epoch, upload, packed_forest, first.m_bodies.m_values, frames);
				before = fixture.Capture();
				fixture.Coupled();
				rejected = fixture.Capture();

				// An ordinary accepted write interleaves after rejection, without replacing the physical resource.
				epoch.warm_start_factor = 0;
				fixture.m_constraints.PrepareVelocityTransactions(gpu.m_job, epoch);
				fixture.m_constraints.CompileRows(gpu.m_job, epoch.timestep, 2, fixture.m_bodies);
				fixture.m_constraints.PrepareVelocitySolve(gpu.m_job, epoch.timestep, 2, fixture.m_bodies);
				fixture.Ordinary();
				fixture.m_prepare.Run(gpu.m_job, epoch.timestep, 2, fixture.m_bodies.get(), fixture.m_frames.get(), fixture.m_mobility);
				fixture.m_velocity.PrepareVelocityTransactions(gpu.m_job, epoch, 2);
				bridge = fixture.Capture();
				fixture.Coupled();
			});
			before.Read();
			rejected.Read();
			bridge.Read();
			PR_EXPECT(rejected.m_results.m_values[0] == GpuConstraintStage_AdmissionRejected && rejected.m_results.m_values[1] == GpuConstraintStage_Accepted);
			PR_EXPECT(rejected.m_solver_state.m_values[0].failure_slot_plus_one == 0u);
			PR_EXPECT(memcmp(&before.m_bodies.m_values[0], &rejected.m_bodies.m_values[0], sizeof(GpuRigidBody)) == 0);
			PR_EXPECT(memcmp(before.m_velocities.m_values.data(), rejected.m_velocities.m_values.data(), 6 * sizeof(float)) == 0);
			PR_EXPECT(memcmp(&before.m_scratch.m_values[0], &rejected.m_scratch.m_values[0], sizeof(GpuArticulationAbaScratch)) == 0);
			PR_EXPECT(memcmp(&before.m_acceptance.m_values[0], &rejected.m_acceptance.m_values[0], sizeof(GpuConstraintAcceptance)) == 0);
			PR_EXPECT(memcmp(&before.m_accepted_rows.m_values[0], &rejected.m_accepted_rows.m_values[0], sizeof(GpuConstraintAcceptedRows)) == 0);
			PR_EXPECT(memcmp(&before.m_rigid_response.m_values[0], &rejected.m_rigid_response.m_values[0], sizeof(GpuArticulationSpatialVector)) == 0);
			PR_EXPECT(memcmp(before.m_velocity_response.m_values.data(), rejected.m_velocity_response.m_values.data(), 6 * sizeof(float)) == 0);
			PR_EXPECT(memcmp(&before.m_ordinary_records.m_values[1], &rejected.m_ordinary_records.m_values[1], sizeof(GpuConstraintRigidRecord)) == 0);
			PR_EXPECT(rejected.m_bodies.m_values[1].momentum_lin.x == 1.5f && rejected.m_velocities.m_values[9] == 1.5f);
			PR_EXPECT(All(rejected.m_acceptance.m_values[0].epoch == first.m_acceptance.m_values[0].epoch));
			PR_EXPECT(All(rejected.m_acceptance.m_values[1].epoch == epoch.identity));

			// The bridge adds (-.75,+.75); C0 then accepts (+1.875,-1.875), while C1 replaces -1.5 with -1.875.
			PR_EXPECT(bridge.m_bodies.m_values[0].momentum_lin.x == 0.25f && bridge.m_bodies.m_values[1].momentum_lin.x == 2.25f);
			PR_EXPECT(final.m_results.m_values[0] == GpuConstraintStage_Accepted && final.m_results.m_values[1] == GpuConstraintStage_Accepted);
			PR_EXPECT(final.m_bodies.m_values[0].momentum_lin.x == 2.125f && final.m_bodies.m_values[1].momentum_lin.x == 1.875f);
			PR_EXPECT(final.m_velocities.m_values[3] == -5.875f && final.m_velocities.m_values[9] == 1.875f);
			PR_EXPECT(final.m_rigid_response.m_values[0].lin.x == 1.875f && final.m_rigid_response.m_values[1].lin.x == -1.875f);
			PR_EXPECT(memcmp(&bridge.m_ordinary_records.m_values[1], &final.m_ordinary_records.m_values[1], sizeof(GpuConstraintRigidRecord)) == 0);
			PR_EXPECT(final.m_solver_state.m_values[0].failure_slot_plus_one == 0u);
			log_stream() << "mixed owner: rejection=(" << rejected.m_results.m_values[0] << ',' << rejected.m_results.m_values[1]
				<< "); after bridge=(" << bridge.m_bodies.m_values[0].momentum_lin.x << ',' << bridge.m_bodies.m_values[1].momentum_lin.x
				<< "); retry=(" << final.m_bodies.m_values[0].momentum_lin.x << ',' << final.m_bodies.m_values[1].momentum_lin.x << ")\n";

			// External input brings R1/tree B to rest; replaying its nonzero current-epoch support must not wake R1.
			auto resting_bodies = final.m_bodies.m_values;
			resting_bodies[1].momentum_lin.x = 0;
			resting_bodies[1].state_flags |= ERigidBodyStateFlags_Sleeping;
			resting_bodies[1].sleep.island_id = 37;
			resting_bodies[1].sleep.generation = 11;
			resting_bodies[1].sleep.timer_s = 3.5f;
			packed_forest.m_velocities = final.m_velocities.m_values;
			packed_forest.m_velocities[9] = 0;
			auto resting = fixture.Run([&]
			{
				// Preparation does not warm-apply, and no ordinary solve is requested for this controlled replay.
				fixture.Prepare(epoch, upload, packed_forest, resting_bodies, frames);
				fixture.Coupled();
			});
			PR_EXPECT(resting.m_results.m_values[1] == GpuConstraintStage_Accepted);
			PR_EXPECT(resting.m_accepted_rows.m_values[2].impulses[0] == -1.875f);
			PR_EXPECT(memcmp(&resting.m_bodies.m_values[1], &resting_bodies[1], sizeof(GpuRigidBody)) == 0);

			// A different dt for an already accepted epoch is a fatal context error, not the provisional-overflow rejection above.
			epoch.timestep *= 0.5f;
			auto context_before = CoupledTransactionSnapshot{};
			auto invalid = fixture.Run([&]
			{
				// The owner checks its actual accepted dt on the device; preparation alone is not acceptance metadata.
				fixture.Prepare(epoch, upload, packed_forest, resting_bodies, frames);
				context_before = fixture.Capture();
				fixture.Coupled();
			});
			context_before.Read();
			PR_EXPECT(invalid.m_results.m_values[0] == GpuConstraintStage_NumericalFailure && invalid.m_results.m_values[1] == GpuConstraintStage_NumericalFailure);
			PR_EXPECT(invalid.m_solver_state.m_values[0].failure_slot_plus_one != 0u);
			PR_EXPECT(memcmp(context_before.m_bodies.m_values.data(), invalid.m_bodies.m_values.data(), 2 * sizeof(GpuRigidBody)) == 0);
			PR_EXPECT(memcmp(context_before.m_acceptance.m_values.data(), invalid.m_acceptance.m_values.data(), 2 * sizeof(GpuConstraintAcceptance)) == 0);
			PR_EXPECT(memcmp(context_before.m_accepted_rows.m_values.data(), invalid.m_accepted_rows.m_values.data(), 3 * sizeof(GpuConstraintAcceptedRows)) == 0);
			PR_EXPECT(context_before.m_velocities.m_values == invalid.m_velocities.m_values && context_before.m_velocity_response.m_values == invalid.m_velocity_response.m_values);
		}

		// Rebase an actual configuration-dependent two-joint response without removing a later independently applied impulse.
		PRUnitTestMethod(ChangedMobilityPreservesInterleavedImpulse, Extended)
		{
			// Two unit-length planar links have M11=3.8+2*cos(q2), M12=1.4+cos(q2), M22=1.4.
			auto inertia = Inertia::Sphere(1.0f, 1.0f);
			inertia.CoM(v4{1, 0, 0, 0});
			auto link = ArticulationLinkDesc{.m_inertia = inertia};
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Revolute(v4::ZAxis());
			joint.m_initial_velocity[0] = 1.0f;
			auto first_link = builder.AddLink(root, joint, link);
			joint = ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1, 0, 0));
			joint.m_initial_velocity[0] = 2.0f;
			auto last_link = builder.AddLink(first_link, joint, link);
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			auto handles = std::array<ConstraintHandle, 2>{};
			auto links = std::array{first_link, last_link};
			for (int index = 0; index != 2; ++index)
			{
				// Isotropic angular damping on both links gives C/6 = [[2,1],[1,1]] in generalized coordinates.
				handles[index] = source.Add(ConstraintDesc{
					.m_frame_b = {.m_body = BodyRef::Link(tree, links[index])},
					.m_parameters = SphericalAngularConstraintParameters{.m_damping = 6.0f}});
			}
			auto frames = [&]
			{
				// Geometry comes from the same native tree, independently of the response ledger.
				return std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(first_link)), PackGpuTransform(tree.LinkToWorld(last_link))};
			};
			auto mass = [](double angle)
			{
				// Closed-form rigid-link kinetic energy supplies the independent mass matrix.
				auto cosine = std::cos(angle);
				return std::array{3.8 + 2.0 * cosine, 1.4 + cosine, 1.4};
			};
			auto solve = [](std::array<double, 3> matrix, std::array<double, 2> rhs)
			{
				// Solve the independent two-by-two physical equation without production factors or compiled rows.
				auto determinant = matrix[0] * matrix[2] - matrix[1] * matrix[1];
				return std::array{(matrix[2] * rhs[0] - matrix[1] * rhs[1]) / determinant, (matrix[0] * rhs[1] - matrix[1] * rhs[0]) / determinant};
			};
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_coupled_relaxation = 1.0f;
			auto& gpu = MidpointTestGpu();
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 22), .timestep = 1.0f / 60.0f, .warm_start_factor = 0.25f};
			auto first = fixture.Run([&]
			{
				// Twelve controlled sweeps resolve the two coupled damper blocks; this is not a default-budget performance claim.
				fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames());
				for (int iteration = 0; iteration != 12; ++iteration)
					fixture.Coupled();
			});
			auto initial_mass = mass(0);
			auto gamma = static_cast<double>(epoch.timestep) * 6.0;
			auto first_expected = solve({initial_mass[0] + 2 * gamma, initial_mass[1] + gamma, initial_mass[2] + gamma},
				{initial_mass[0] + 2 * initial_mass[1], initial_mass[1] + 2 * initial_mass[2]});
			auto expect = [&](double actual, double expected)
			{
				// Keep the independent calculation in double precision and print a discriminating mismatch before the assertion.
				if (std::abs(actual - expected) > 2.0e-5)
					log_stream() << FmtS("physical oracle mismatch: actual=%.12g, expected=%.12g, error=%.12g\n", actual, expected, actual - expected);

				PR_EXPECT(std::abs(actual - expected) <= 2.0e-5);
			};
			log_stream() << FmtS("initial two-link damping: actual=(%.9g,%.9g), independent=(%.9g,%.9g)\n",
				first.m_velocities.m_values[0], first.m_velocities.m_values[1], first_expected[0], first_expected[1]);
			PR_EXPECT(first.m_results.m_values[0] == GpuConstraintStage_Accepted);
			expect(first.m_velocities.m_values[0], first_expected[0]);
			expect(first.m_velocities.m_values[1], first_expected[1]);
			auto old_response = std::array{first.m_velocities.m_values[0] - 1.0, first.m_velocities.m_values[1] - 2.0};
			auto old_impulse = std::array{initial_mass[0] * old_response[0] + initial_mass[1] * old_response[1], initial_mass[1] * old_response[0] + initial_mass[2] * old_response[1]};
			expect(old_impulse[0], -gamma * (2 * first_expected[0] + first_expected[1]));
			expect(old_impulse[1], -gamma * (first_expected[0] + first_expected[1]));

			// Change actual articulated mobility through the distal joint pose, while retaining the previously published generalized speeds.
			auto angle = 0.6f;
			tree.JointPosition(last_link, std::array{angle});
			tree.JointVelocity(first_link, std::array{first.m_velocities.m_values[0]});
			tree.JointVelocity(last_link, std::array{first.m_velocities.m_values[1]});
			auto current_mass = mass(angle);
			auto external_delta = solve(current_mass, {0.3, -0.2});
			auto external = CoupledTransactionSnapshot{};
			auto final = fixture.Run([&]
			{
				// Apply world-Z impulses through the real independent impulse owner after transaction preparation and before its attempt.
				fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames());
				auto impulses = std::array<GpuArticulationSpatialVector, 3>{};
				impulses[1].ang.z = 0.5f;
				impulses[2].ang.z = -0.2f;
				fixture.m_impulse.Upload(gpu.m_job, impulses);
				fixture.m_impulse.Run(gpu.m_job);
				external = fixture.Capture();
				for (int iteration = 0; iteration != 12; ++iteration)
					fixture.Coupled();
			});
			external.Read();
			auto expected = solve({current_mass[0] + 2 * gamma, current_mass[1] + gamma, current_mass[2] + gamma},
				{current_mass[0] + 2 * current_mass[1] + 0.3, current_mass[1] + 2 * current_mass[2] - 0.2});
			PR_EXPECT(final.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(final.m_solver_state.m_values[0].failure_slot_plus_one == 0u);
			for (int index = 0; index != 2; ++index)
			{
				// External response, old response, and the subsequent actual delta are reconstructed separately.
				expect(external.m_velocities.m_values[index] - first.m_velocities.m_values[index], external_delta[index]);
				expect(final.m_velocities.m_values[index], expected[index]);
				expect(final.m_velocity_response.m_values[index], expected[index] - (index + 1.0 + external_delta[index]));
				expect(final.m_velocities.m_values[index] - external.m_velocities.m_values[index],
					final.m_velocity_response.m_values[index] - first.m_velocity_response.m_values[index]);
				PR_EXPECT(final.m_accepted_rows.m_values[index].invalidation_receipt == first.m_accepted_rows.m_values[index].invalidation_receipt);
			}
			log_stream() << FmtS("changed mobility: old impulse=(%g,%g); external delta=(%g,%g); final=(%g,%g), independent=(%g,%g); actual commit delta=(%g,%g)\n",
				old_impulse[0], old_impulse[1], external_delta[0], external_delta[1], final.m_velocities.m_values[0], final.m_velocities.m_values[1],
				expected[0], expected[1], final.m_velocities.m_values[0] - external.m_velocities.m_values[0], final.m_velocities.m_values[1] - external.m_velocities.m_values[1]);

			// Enabled zero-row visits release this epoch's support and retire accepted coefficients without removing the external response.
			for (auto handle : handles)
			{
				// Descriptor invalidation remains per slot; island membership is unchanged.
				auto desc = source.Get(handle);
				desc.m_parameters = D6ConstraintParameters{};
				desc.D6().m_angular[2] = ConstraintAxisDesc{.m_mode = EConstraintAxisMode::Limited, .m_limits = Range<float>{-1, 1}};
				source.Update(handle, desc);
			}
			tree.JointVelocity(first_link, std::array{final.m_velocities.m_values[0]});
			tree.JointVelocity(last_link, std::array{final.m_velocities.m_values[1]});
			upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto released = fixture.Run([&]
			{
				// Zero current rows still visit the existing owner instead of testing response magnitude as eligibility.
				fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames());
				fixture.Coupled();
			});
			PR_EXPECT(released.m_results.m_values[0] == GpuConstraintStage_Accepted);
			for (int index = 0; index != 2; ++index)
			{
				// Both physical external response and empty accepted caches are observable at the actual publication boundary.
				expect(released.m_velocities.m_values[index], index + 1.0 + external_delta[index]);
				expect(released.m_velocity_response.m_values[index], 0.0f);
				auto const& row = released.m_accepted_rows.m_values[index];
				PR_EXPECT(row.velocity_mask == 0u && row.linear_impulse_norm == 0.0f && row.angular_impulse_norm == 0.0f);
				for (auto value : row.impulses)
					PR_EXPECT(value == 0.0f);
			}
		}

		// Isotropic viscous response is invariant under a configuration-only rotation at unchanged child-joint speeds.
		PRUnitTestMethod(IsotropicDamperPoseContinuation, Extended)
		{
			// Unit isotropic inertia gives the independent implicit solution omega = omega_free / (1 + dt*c).
			auto const inertia = Inertia::Sphere(1.0f, 2.5f);
			auto const free_speed = v4{1, 2, 3, 0};
			auto const dt = 1.0f / 60.0f;
			auto const viscosity = 6.0f;
			auto const expected_speed = free_speed / (1.0f + dt * viscosity);
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFixedRoot(MidpointLink(0));
			auto joint = ArticulationJointDesc::Spherical();
			joint.m_initial_velocity = {free_speed.x, free_speed.y, free_speed.z};
			auto child = builder.AddLink(root, joint, ArticulationLinkDesc{.m_inertia = inertia});
			auto tree = builder.Build();
			auto forest = std::array{&tree};
			auto source = ConstraintSet{};
			source.Add(ConstraintDesc{
				.m_frame_b = BodyFrame{.m_body = BodyRef::Link(tree, child)},
				.m_parameters = SphericalAngularConstraintParameters{.m_damping = viscosity},
			});
			auto upload = PackGpuConstraints(source, BodyRemap{{}, forest});
			auto frames = std::array{PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
			auto& gpu = MidpointTestGpu();
			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 1;
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.25f;
			PR_EXPECT((upload.m_endpoints[0].flags & GpuConstraintEndpointFlags_ResetWarmStart) == 0u);
			auto fixture = CoupledTransactionFixture{gpu, config};
			auto epoch = GpuConstraintEpoch{.identity = uint2(1, 21), .timestep = dt, .warm_start_factor = config.constraint_warm_start_factor};
			auto unattempted = fixture.Run([&]
			{
				// Preparation must not publish a physical guess or fabricate acceptance.
				fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames);
			});
			PR_EXPECT(unattempted.m_results.m_values[0] == 0xFFFFFFFFu && unattempted.m_acceptance.m_values[0].valid == 0u);
			PR_EXPECT(unattempted.m_velocities.m_values == std::vector<float>({1, 2, 3}));
			auto solve = [&]
			{
				// Every call uses persistent accepted storage and the actual private stage/admit/commit sequence.
				return fixture.Run([&]
				{
					// Geometry preparation is not allowed to publish a warm impulse.
					fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames);
					fixture.Coupled();
				});
			};
			auto solved = solve();
			PR_EXPECT(solved.m_results.m_values[0] == GpuConstraintStage_Accepted);
			auto const solved_speed = v4{solved.m_velocities.m_values[0], solved.m_velocities.m_values[1], solved.m_velocities.m_values[2], 0};
			PR_EXPECT(Length(solved_speed - expected_speed) < 2.0e-5f);

			// The continuation consumes already-solved speeds; unchanged geometry must not apply viscosity twice.
			tree.JointVelocity(child, solved.m_velocities.m_values);
			auto control = solve();
			PR_EXPECT(control.m_results.m_values[0] == GpuConstraintStage_Accepted);
			auto const control_speed = v4{control.m_velocities.m_values[0], control.m_velocities.m_values[1], control.m_velocities.m_values[2], 0};
			PR_EXPECT(Length(control_speed - expected_speed) < 2.0e-5f);

			// Rotate only the pose by an ordinary angle, as detached position application can do; physical generalized speeds remain untouched.
			auto const orientation = ExpMap<quat>(v3{0, 0, 0.3f});
			tree.JointPosition(child, std::array{orientation.x, orientation.y, orientation.z, orientation.w});
			tree.JointVelocity(child, control.m_velocities.m_values);
			frames = {PackGpuTransform(tree.LinkToWorld(root)), PackGpuTransform(tree.LinkToWorld(child))};
			auto continued = solve();
			PR_EXPECT(continued.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(continued.m_solver_state.m_values[0].failure_slot_plus_one == 0u);
			PR_EXPECT(continued.m_accepted_rows.m_values[0].invalidation_receipt == solved.m_accepted_rows.m_values[0].invalidation_receipt);
			PR_EXPECT(All(continued.m_acceptance.m_values[0].epoch == epoch.identity));
			auto const final_speed = v4{continued.m_velocities.m_values[0], continued.m_velocities.m_values[1], continued.m_velocities.m_values[2], 0};
			auto const physical_impulse = inertia.Ic3x3() * (final_speed - free_speed);
			auto const prior_physical_impulse = inertia.Ic3x3() * (control_speed - free_speed);
			auto const actual_delta = inertia.Ic3x3() * (final_speed - control_speed);
			auto prior_row_impulse = v4::Zero();
			auto compiled_delta = v4::Zero();
			auto row_impulse = v4::Zero();
			for (int slot = 3; slot != 6; ++slot)
			{
				// Compare the actual old response and new response delta separately; new-J times retained scalars is not the physical total.
				auto old_impulse = control.m_accepted_rows.m_values[0].impulses[slot] * control.m_rows.m_values[slot].jacobian_b_ang;
				auto new_impulse = continued.m_accepted_rows.m_values[0].impulses[slot] * continued.m_rows.m_values[slot].jacobian_b_ang;
				prior_row_impulse += old_impulse;
				compiled_delta += new_impulse - old_impulse;
				row_impulse += new_impulse;
			}
			auto const row_law_error = Length(row_impulse + (dt * viscosity) * final_speed);
			auto const physical_law_error = Length(physical_impulse + (dt * viscosity) * final_speed);
			auto const speed_error = Length(final_speed - expected_speed);
			log_stream() << FmtS("prior physical impulse=(%g,%g,%g); actual continuation delta=(%g,%g,%g); prior-row error=%g, delta-row error=%g\n",
				prior_physical_impulse.x, prior_physical_impulse.y, prior_physical_impulse.z, actual_delta.x, actual_delta.y, actual_delta.z,
				Length(prior_row_impulse - prior_physical_impulse), Length(compiled_delta - actual_delta));
			log_stream() << FmtS("same-pose error=%g; rotated continuation=(%g,%g,%g), expected=(%g,%g,%g); speed error=%g, row law=%g, physical law=%g\n",
				Length(control_speed - expected_speed), final_speed.x, final_speed.y, final_speed.z,
				expected_speed.x, expected_speed.y, expected_speed.z, speed_error, row_law_error, physical_law_error);
			PR_EXPECT(Length(prior_row_impulse - prior_physical_impulse) < 2.0e-5f);
			PR_EXPECT(Length(compiled_delta - actual_delta) < 2.0e-5f);
			PR_EXPECT(row_law_error < 2.0e-5f);
			PR_EXPECT(physical_law_error < 2.0e-5f);
			PR_EXPECT(speed_error < 2.0e-5f);

			// A cold owner at the rotated pose must converge to the same physical law, independently of any retained basis.
			tree.JointVelocity(child, std::array{free_speed.x, free_speed.y, free_speed.z});
			auto cold_fixture = CoupledTransactionFixture{gpu, config};
			auto cold = cold_fixture.Run([&]
			{
				// This owner has no accepted coefficients or represented response.
				cold_fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames);
				cold_fixture.Coupled();
			});
			auto cold_speed = v4{cold.m_velocities.m_values[0], cold.m_velocities.m_values[1], cold.m_velocities.m_values[2], 0};
			PR_EXPECT(cold.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(Length(cold_speed - final_speed) < 2.0e-5f);

			// A fresh substep retains old physical history rather than subtracting last substep's damping again.
			tree.JointVelocity(child, continued.m_velocities.m_values);
			++epoch.identity.x;
			auto no_iterations = fixture.Run([&]
			{
				// This cache is nonzero, so an accidental physical warm publication would change the uploaded speeds.
				fixture.Prepare(epoch, upload, PackGpuArticulations(forest), {}, frames);
			});
			PR_EXPECT(no_iterations.m_results.m_values[0] == 0xFFFFFFFFu);
			PR_EXPECT(no_iterations.m_velocities.m_values == continued.m_velocities.m_values);
			PR_EXPECT(memcmp(&no_iterations.m_acceptance.m_values[0], &continued.m_acceptance.m_values[0], sizeof(GpuConstraintAcceptance)) == 0);
			auto fresh = solve();
			auto fresh_speed = v4{fresh.m_velocities.m_values[0], fresh.m_velocities.m_values[1], fresh.m_velocities.m_values[2], 0};
			PR_EXPECT(fresh.m_results.m_values[0] == GpuConstraintStage_Accepted);
			PR_EXPECT(Length(fresh_speed - final_speed / (1.0f + dt * viscosity)) < 2.0e-5f);
			auto recorded = v4{fresh.m_velocity_response.m_values[0], fresh.m_velocity_response.m_values[1], fresh.m_velocity_response.m_values[2], 0};
			PR_EXPECT(Length(recorded - (fresh_speed - final_speed)) < 2.0e-5f);
		}
	};

	// World-fixed loads follow trial configurations while explicit link-frame follower loads retain their frame.
	PRUnitTestClass(ArticulationWorldForceRegressionTests)
	{
		// CPU gravity follows every midpoint trial and agrees with independent scalar equations over ten seconds.
		PRUnitTestMethod(CpuPendulumMatchesAnalyticalMidpoint, Quick)
		{
			auto [articulation, child] = BuildGravityPendulum();
			articulation.GravityWS(child, -9.81f * v4::YAxis());
			articulation.ForwardDynamics();
			PR_EXPECT(FEqlAbsolute(articulation.JointAcceleration(child)[0], -9.81f * Sin(0.1f) / 1.1f, 1.0e-6f));
			auto reference = std::array<double, 2>{static_cast<double>(0.1f), 0.0};
			auto const dt = 1.0f / 60.0f;
			auto const initial_energy = GravityPendulumEnergy(reference[0], reference[1]);
			auto max_energy_error = 0.0;
			auto max_state_error = 0.0;

			// A frozen beginning-of-step wrench grows this pendulum's energy to about 2.10 times its initial value.
			for (int step = 0; step != 600; ++step)
			{
				StepAnalyticalPendulum(reference, dt);
				articulation.Integrate(dt);
				auto const position = articulation.JointPosition(child)[0];
				auto const velocity = articulation.JointVelocity(child)[0];
				max_state_error = std::max({max_state_error, std::abs(position - reference[0]), std::abs(velocity - reference[1])});
				max_energy_error = std::max(max_energy_error, std::abs(GravityPendulumEnergy(position, velocity) / initial_energy - 1.0));
			}
			PR_EXPECT(max_state_error < 1.0e-3);
			PR_EXPECT(max_energy_error < 5.0e-3);
		}

		// Standalone hardware and replay retain world-fixed gravity without requiring hidden proxy resources.
		PRUnitTestMethod(StandalonePendulumUsesWorldGravity, Extended)
		{
			auto [articulation, child] = BuildGravityPendulum();
			articulation.GravityWS(child, -9.81f * v4::YAxis());
			auto forest = std::array{&articulation};
			auto const upload = PackGpuArticulations(forest);
			auto replay_upload = upload;
			auto replay = ArticulationMidpointInteropRunner{};
			auto const dt = 1.0f / 60.0f;
			replay.Run(replay_upload, dt, 600);
			auto aba = GpuArticulationForceAba{SharedTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const hardware = midpoint.Solve(SharedTestGpu().m_job, upload, dt, 600);
			PR_EXPECT(hardware.AllSucceeded());

			// The independent scalar solution and mechanical energy guard both paths rather than comparing only shader implementations.
			auto reference = std::array<double, 2>{static_cast<double>(0.1f), 0.0};
			auto const initial_energy = GravityPendulumEnergy(reference[0], reference[1]);
			for (int step = 0; step != 600; ++step)
				StepAnalyticalPendulum(reference, dt);

			// Both final states must remain on the bounded-energy trajectory.
			for (auto const state : {std::array{replay_upload.m_positions[0], replay_upload.m_velocities[0]}, std::array{hardware.m_positions[0], hardware.m_velocities[0]}})
			{
				PR_EXPECT(std::abs(state[0] - reference[0]) < 1.0e-3);
				PR_EXPECT(std::abs(state[1] - reference[1]) < 1.0e-3);
				PR_EXPECT(std::abs(GravityPendulumEnergy(state[0], state[1]) / initial_energy - 1.0) < 5.0e-3);
			}
		}

		// Engine gravity remains world-fixed throughout each trial without adding submissions or readbacks.
		PRUnitTestMethod(EnginePendulumGravityDoesNotPumpEnergy, Extended)
		{
			auto [articulation, child] = BuildGravityPendulum();
			articulation.GravityWS(child, -9.81f * v4::YAxis());
			auto forest = std::array{&articulation};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Config(MidpointEngineConfig());
			auto reference = std::array<double, 2>{static_cast<double>(0.1f), 0.0};
			auto const dt = 1.0f / 60.0f;
			auto const initial_energy = GravityPendulumEnergy(reference[0], reference[1]);
			auto max_energy_error = 0.0;
			auto max_state_error = 0.0;

			// Long-run energy and the scalar solution independently expose a force frozen in the starting link frame.
			for (int step = 0; step != 600; ++step)
			{
				StepAnalyticalPendulum(reference, dt);
				engine.Step(Engine::StepInput{.m_articulations = forest, .m_elapsed_seconds = dt});
				auto const position = articulation.JointPosition(child)[0];
				auto const velocity = articulation.JointVelocity(child)[0];
				max_state_error = std::max({max_state_error, std::abs(position - reference[0]), std::abs(velocity - reference[1])});
				max_energy_error = std::max(max_energy_error, std::abs(GravityPendulumEnergy(position, velocity) / initial_energy - 1.0));
			}
			PR_EXPECT(max_state_error < 1.0e-3);
			PR_EXPECT(max_energy_error < 5.0e-3);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			ResetEngineForNextTest(engine);
		}

		// A GPU force at the moving COM rotates per trial while an explicit link-origin follower wrench remains local.
		PRUnitTestMethod(EngineWorldForceKeepsLocalFollowerSeparate, Extended)
		{
			auto [articulation, child] = BuildGravityPendulum(0.7f, 0.8f);
			auto forest = std::array{&articulation};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Config(MidpointEngineConfig());
			auto callback_count = 0;
			engine.ExternalForces += [&](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				auto const proxy_index = sender.ArticulationLinkStepIndex(articulation.Id(), child);
				WriteBodyLinearForce(args, proxy_index, -9.81f * v4::YAxis());
				++callback_count;
			};
			auto reference = std::array<double, 2>{static_cast<double>(0.7f), static_cast<double>(0.8f)};
			auto const dt = 1.0f / 30.0f;
			auto const follower_torque = 0.3f;
			auto max_state_error = 0.0;

			// Link-local force is already about the link origin; applying a second COM shift changes its generalized torque.
			for (int step = 0; step != 120; ++step)
			{
				articulation.ExternalForce(child, v8force{follower_torque * v4::ZAxis(), 0.4f * v4::XAxis()});
				StepAnalyticalPendulum(reference, dt, follower_torque);
				engine.Step(Engine::StepInput{.m_articulations = forest, .m_elapsed_seconds = dt});
				max_state_error = std::max({
					max_state_error,
					std::abs(articulation.JointPosition(child)[0] - reference[0]),
					std::abs(articulation.JointVelocity(child)[0] - reference[1])});
			}
			PR_EXPECT(max_state_error < 2.0e-3);
			PR_EXPECT(callback_count == 120);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			ResetEngineForNextTest(engine);
		}
	};

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

		// Begin/Complete keeps an articulation-only mixed forest detached until one-copy publication and matches replay across internal substeps.
		PRUnitTestMethod(EngineArticulationOnlyBeginCompleteMatchesReplay, Quick)
		{
			auto fixed = BuildMidpointArticulation(false);
			auto floating = BuildMidpointArticulation(true);
			ApplyMidpointForces(fixed);
			ApplyMidpointForces(floating);
			auto forest = std::array{&fixed, &floating};
			auto const accepted = PackGpuArticulations(forest);
			auto expected = accepted;
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(expected, 0.0005f, 3);

			auto engine = MidpointEngine(MidpointEngineConfig());
			engine.BeginStep(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.0015f,
				.m_substep_count = 3,
			});

			// Submission never consumes caller state; forces and accepted generalized values remain available until validation completes.
			ExpectPackedArticulationStateExact(PackGpuArticulations(forest), accepted);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 3);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 0);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			engine.CompleteStep();

			ExpectMidpointCpuNear(expected, replay.Articulations(), replay, forest, 1.0e-2f);
			ExpectArticulationForcesCleared(PackGpuArticulations(forest));
			PR_EXPECT(engine.LastStepProfile().m_substep_count == 3);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 3);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_articulations.m_articulation_count == 2);
			PR_EXPECT(stats.m_articulations.m_link_count == isize(accepted.m_links));
			PR_EXPECT(stats.m_articulations.m_dof_count == isize(accepted.m_dofs));
			PR_EXPECT(stats.m_articulations.m_articulation_capacity >= 2);
			PR_EXPECT(stats.m_articulations.m_link_capacity >= isize(accepted.m_links));
			PR_EXPECT(stats.m_articulations.m_resources.m_dispatch_count > 0);
			PR_EXPECT(stats.m_articulations.m_resources.m_logical_bytes > 0);
			PR_EXPECT(stats.m_articulations.m_resources.m_allocated_bytes >= stats.m_articulations.m_resources.m_logical_bytes);
			PR_EXPECT(stats.m_constraints.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(stats.m_frame_output.m_articulation_count == 2);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 1);
			PR_EXPECT(!stats.m_failure.HasTerminalFailure());

			// An empty frame releases articulation-dependent buffers while retaining only the reusable packed-output allocation.
			auto const retained_frame_output_bytes = stats.m_frame_output.m_allocated_bytes;
			engine.Step(Engine::StepInput{.m_elapsed_seconds = 1.0f / 60.0f});
			auto const& empty_stats = engine.LastFeatureStats();
			PR_EXPECT(empty_stats.m_articulations.m_articulation_count == 0);
			PR_EXPECT(empty_stats.m_articulations.m_resources.m_logical_bytes == 0);
			PR_EXPECT(empty_stats.m_articulations.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(empty_stats.m_frame_output.m_logical_bytes == 0);
			PR_EXPECT(empty_stats.m_frame_output.m_allocated_bytes == retained_frame_output_bytes);
			PR_EXPECT(empty_stats.m_frame_output.m_readback_count == 0);
		}

		// A fixed root with no generalized scalars still publishes its compact record while padded empty-range UAV addresses remain valid.
		PRUnitTestMethod(EngineFixedRootOnlyArticulationStep, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MidpointLink(0));
			auto articulation = builder.Build();
			articulation.ExternalForce(root, v8force{v4{0.1f, -0.2f, 0.3f, 0.0f}, v4{-0.4f, 0.5f, -0.6f, 0.0f}});
			auto forest = std::array{&articulation};
			auto const root_to_world = articulation.RootToWorld();

			auto engine = MidpointEngine(MidpointEngineConfig());
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.001f,
			});

			auto const output = PackGpuArticulations(forest);
			PR_EXPECT(output.m_positions.empty());
			PR_EXPECT(output.m_velocities.empty());
			PR_EXPECT(output.m_accelerations.empty());
			PR_EXPECT(FEql(articulation.RootToWorld(), root_to_world));
			ExpectArticulationForcesCleared(output);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Automatic sleeping evaluates the complete tree after accepted readback and omits an inactive tree's topology, proxies, dispatch, and readback.
		PRUnitTestMethod(EngineCompleteTreeSleepingSkipsGpuWork, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(MidpointLink(0));
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto config = MidpointEngineConfig();
			config.sleeping_enabled = true;
			config.sleep_velocity_threshold_lin = 0.01f;
			config.sleep_velocity_threshold_ang = 0.01f;
			config.sleep_delay_s = 0.015f;
			auto engine = MidpointEngine(config);

			// Two quiet frames cross the tree-owned delay without allowing individual links to sleep early.
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.01f,
			});
			PR_EXPECT(!articulation.Sleeping());
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.01f,
			});
			PR_EXPECT(articulation.Sleeping());

			// A sleeping-only frame performs no GPU submission or readback because the complete tree is absent from the packed forest.
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.01f,
			});
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 0);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 0);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 0);

			// A link wrench wakes and repacks the full articulation rather than a single proxy.
			articulation.ExternalForce(root, v8force{v4{}, v4{0.0f, 0.0f, 1.0f, 0.0f}});
			PR_EXPECT(!articulation.Sleeping());
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.01f,
			});
			PR_EXPECT(!articulation.Sleeping());
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);

			// Disabling sleeping wakes an explicitly sleeping tree before the next pack and leaves it awake.
			articulation.Sleep();
			config.sleeping_enabled = false;
			engine.Config(config);
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.01f,
			});
			PR_EXPECT(!articulation.Sleeping());
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
		}


		// A rigid-only maintenance frame uses its explicit empty layout instead of stale output retained by an earlier articulation step.
		PRUnitTestMethod(EngineSleepIslandRefreshAfterArticulationStep, Quick)
		{
			auto articulation = BuildMidpointArticulation(false);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto engine = MidpointEngine(MidpointEngineConfig());
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.001f,
			});

			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.Sleep();
			auto bodies = std::array<RigidBody*, 1>{&body};
			engine.UpdateSleepIslands(bodies);

			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Mixed rigid and pure-tree inputs advance across substep submissions with one final copy and completion wait.
		PRUnitTestMethod(EngineMixedRigidAndArticulationStep, Quick)
		{
			auto articulation = BuildMidpointArticulation(true);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto expected = PackGpuArticulations(forest);
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(expected, 0.0005f, 2);

			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v8motion{v4{}, v4{1.0f, 0.0f, 0.0f, 0.0f}});
			body.ForceWS(v8force{v4{}, v4{0.2f, -0.1f, 0.3f, 0.0f}});
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const position_x = body.O2W().pos.x;

			auto engine = MidpointEngine(MidpointEngineConfig());
			engine.Step(Engine::StepInput{
				.m_bodies = std::span{body_ptrs},
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = 0.001f,
				.m_substep_count = 2,
			});

			PR_EXPECT(body.O2W().pos.x > position_x);
			PR_EXPECT(FEql(body.ForceWS(), v8force{}));
			ExpectMidpointCpuNear(expected, replay.Articulations(), replay, forest, 1.0e-2f);
			ExpectArticulationForcesCleared(PackGpuArticulations(forest));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 2);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Hidden link proxies route persistent gravity and substep GPU forces into ABA without exposing links as independent rigid bodies.
		PRUnitTestMethod(EngineLinkProxyForcesMatchCpuArticulation, Quick)
		{
			auto actual = BuildProxyMidpointFixture();
			auto expected = BuildProxyMidpointFixture();
			auto const root_gravity = v4{0.7f, -1.1f, -8.9f, 0};
			auto const child_gravity = v4{-0.4f, 0.3f, -9.4f, 0};
			auto const force_ws = v4{2.3f, -1.7f, 0.9f, 0};
			actual.m_articulation.GravityWS(actual.m_root, root_gravity);
			actual.m_articulation.GravityWS(actual.m_child, child_gravity);
			expected.m_articulation.GravityWS(expected.m_root, root_gravity);
			expected.m_articulation.GravityWS(expected.m_child, child_gravity);

			auto const elapsed_seconds = 0.002f;
			auto const substep_count = 2;
			auto const dt = elapsed_seconds / substep_count;
			for (int substep_index = 0; substep_index != substep_count; ++substep_index)
			{
				expected.m_articulation.ExternalForce(
					expected.m_child,
					LinkWrenchAtCentreOfMass(expected.m_articulation, expected.m_child, force_ws));
				expected.m_articulation.Integrate(dt);
			}

			auto engine = MidpointEngine(MidpointEngineConfig());
			auto callback_count = 0;
			auto const child_force_seed_ws = actual.m_articulation.LinkDescription(actual.m_child).m_inertia.Mass() * child_gravity;
			engine.ExternalForces += [&](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				PR_EXPECT(args.m_rigid_body_count == 0);
				PR_EXPECT(args.m_body_count == actual.m_articulation.LinkCount());
				auto const proxy_index = sender.ArticulationLinkStepIndex(actual.m_articulation.Id(), actual.m_child);
				PR_EXPECT(proxy_index == 1);
				WriteBodyLinearForce(args, proxy_index, child_force_seed_ws + force_ws);
				++callback_count;
			};
			auto forest = std::array{&actual.m_articulation};
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{forest},
				.m_elapsed_seconds = elapsed_seconds,
				.m_substep_count = substep_count,
			});

			PR_EXPECT(callback_count == substep_count);
			ExpectArticulationPrimaryNear(actual.m_articulation, expected.m_articulation, 1.2e-2f);
			PR_EXPECT(FEql(actual.m_articulation.GravityWS(actual.m_root), root_gravity));
			PR_EXPECT(FEql(actual.m_articulation.GravityWS(actual.m_child), child_gravity));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == substep_count);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// One failed tree rejects the complete mixed frame, preserves all forces and state, and permits a clean subsequent upload.
		PRUnitTestMethod(EngineFailureIsTransactionalAndRecovers, Quick)
		{
			auto valid = BuildMidpointArticulation(true);
			auto singular = BuildSingularMidpointArticulation();
			ApplyMidpointForces(valid);
			auto failed_forest = std::array{&valid, &singular};
			auto const articulation_state = PackGpuArticulations(failed_forest);

			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v8motion{v4{}, v4{0.4f, -0.2f, 0.1f, 0.0f}});
			body.ForceWS(v8force{v4{0.1f, 0.2f, -0.1f, 0.0f}, v4{0.3f, -0.4f, 0.2f, 0.0f}});
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const body_to_world = body.O2W();
			auto const body_momentum = body.MomentumWS();
			auto const body_force = body.ForceWS();
			auto const body_flags = body.StateFlags();

			auto engine = MidpointEngine(MidpointEngineConfig());
			PR_THROWS(engine.Step(Engine::StepInput{
				.m_bodies = std::span{body_ptrs},
				.m_articulations = std::span{failed_forest},
				.m_elapsed_seconds = 0.001f,
				.m_substep_count = 2,
			}), std::exception);

			ExpectPackedArticulationStateExact(PackGpuArticulations(failed_forest), articulation_state);
			PR_EXPECT(FEql(body.O2W(), body_to_world));
			PR_EXPECT(FEql(body.MomentumWS(), body_momentum));
			PR_EXPECT(FEql(body.ForceWS(), body_force));
			PR_EXPECT(body.StateFlags() == body_flags);

			// Reuploading the valid tree resets sticky GPU diagnostics and consumes the force preserved by the rejected frame.
			auto valid_forest = std::array{&valid};
			auto expected = PackGpuArticulations(valid_forest);
			auto replay = ArticulationMidpointInteropRunner{};
			replay.Run(expected, 0.001f, 1);
			engine.Step(Engine::StepInput{
				.m_articulations = std::span{valid_forest},
				.m_elapsed_seconds = 0.001f,
			});
			ExpectMidpointCpuNear(expected, replay.Articulations(), replay, valid_forest, 1.0e-2f);
			ExpectArticulationForcesCleared(PackGpuArticulations(valid_forest));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
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

		// An articulation-only frame gathers compact roots, diagnostics, and generalized state through one contiguous readback.
		PRUnitTestMethod(HardwareCompactFrameOutputMatchesReplay, Extended)
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

			// Record initialization, one fused integration substep, compact gather, and the only final copy in one job.
			auto& gpu = MidpointTestGpu();
			auto aba = GpuArticulationForceAba{gpu};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto frame_output = GpuFrameOutput{gpu};
			PR_EXPECT(midpoint.Upload(gpu.m_job, upload));
			frame_output.BeginFrame(gpu.m_job, 0, 0, 1, midpoint.Output());
			midpoint.Integrate(gpu.m_job, 0.001f);
			auto readback = frame_output.GatherAndReadback(gpu.m_job, 0, nullptr, midpoint.Output());
			gpu.m_job.Run();

			auto const gathered_articulations = GpuFrameOutput::Articulations(readback);
			auto const gathered_positions = GpuFrameOutput::Positions(readback);
			auto const gathered_velocities = GpuFrameOutput::Velocities(readback);
			auto const gathered_accelerations = GpuFrameOutput::Accelerations(readback);
			PR_EXPECT(GpuFrameOutput::Bodies(readback).empty());
			PR_EXPECT(gathered_articulations.size() == replay.Articulations().size());
			PR_EXPECT(gathered_positions.size() == replay_upload.m_positions.size());
			PR_EXPECT(gathered_velocities.size() == replay_upload.m_velocities.size());
			PR_EXPECT(gathered_accelerations.size() == replay_upload.m_accelerations.size());

			// Preserve stable identity and convergence metadata while compacting the larger persistent records.
			for (int articulation_index = 0; articulation_index != isize(gathered_articulations); ++articulation_index)
			{
				auto const& actual = gathered_articulations[articulation_index];
				auto const& expected_articulation = replay.Articulations()[articulation_index];
				auto const& expected_state = replay.States()[articulation_index];
				ExpectMidpointFrameNear(actual.root_to_world, expected_articulation.root_to_world, 2.0e-4f);
				PR_EXPECT(actual.identity_low == expected_articulation.identity_low);
				PR_EXPECT(actual.identity_high == expected_articulation.identity_high);
				PR_EXPECT(actual.status == expected_state.status);
				PR_EXPECT(actual.iteration_count == expected_state.iteration_count);
				ExpectMidpointNear(actual.residual, expected_state.residual, 2.0e-4f);
			}
			for (int index = 0; index != isize(gathered_positions); ++index)
				ExpectMidpointNear(gathered_positions[index], replay_upload.m_positions[index], 2.0e-4f);
			for (int index = 0; index != isize(gathered_velocities); ++index)
				ExpectMidpointNear(gathered_velocities[index], replay_upload.m_velocities[index], 2.0e-4f);
			for (int index = 0; index != isize(gathered_accelerations); ++index)
				ExpectMidpointNear(gathered_accelerations[index], replay_upload.m_accelerations[index], 2.0e-4f);
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

		// Retain a translating centered root's actual midpoint output without prescribing an approximate map as exact physics.
		PRUnitTestMethod(HardwareTranslatingRootTransportDiagnostic, Extended)
		{
			// Exact cyclic axes and unit spatial inertia isolate moving-frame transport from geometry and mass conditioning.
			auto const root_to_world = m4x4{v4{0, 1, 0, 0}, v4{0, 0, 1, 0}, v4{1, 0, 0, 0}, v4{0, 0, 0, 1}};
			auto const timestep = (1.0f / 60.0f) / 3;
			auto builder = ArticulationBuilder{};
			builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia(m3x3::Identity(), 1.0f)}, root_to_world, v8motion{v4{0, 0, 18, 0}, v4{1, 0, 0, 0}});
			auto articulation = builder.Build();
			auto forest = std::array{&articulation};
			auto upload = PackGpuArticulations(forest);
			auto config = std::ofstream{};
			config.exceptions(std::ios::badbit | std::ios::failbit);
			config.open(temp_dir() / "translating-root-config.txt");
			config.precision(17);
			config << "timestep=" << timestep << "\nsubsteps=1\nmass=1\nCOM=0,0,0\ncentral_inertia=identity\n"
				<< "root_rotation_columns=ey,ez,ex\nroot_position=0,0,0\nroot_body_angular_velocity=0,0,18\nroot_body_origin_linear_velocity=1,0,0\n"
				<< "loads=none; no proxies/contacts/gravity/drives/damping; existing actual production midpoint Solve PSO\n"
				<< "residual=max_abs(next_midpoint_velocity-previous_candidate); not normalized\n"
				<< "residual_components=angular rad/s and linear length_units/s; scalar maximum across all six stored components\n"
				<< "stopping_rule=residual<=2e-6*(1+max_abs(next_midpoint_velocity)); iteration_limit=12\n"
				<< "diagnostic_only=no momentum/energy acceptance tolerance or exact-approximate-map contract\n"
				<< "sizes: articulation=" << sizeof(GpuArticulation) << " link=" << sizeof(GpuArticulationLink)
				<< " state=" << sizeof(GpuArticulationIntegrationState) << " spatial=" << sizeof(GpuArticulationSpatialVector) << '\n';
			config.flush();

			// Flush each exact typed stream before any GPU setup or observer can fail; empty logical streams remain empty files.
			auto retain = [&](char const* name, auto const& values)
			{
				// Evidence-write failures must fail the method rather than produce a success with missing raw inputs.
				auto stream = std::ofstream{};
				stream.exceptions(std::ios::badbit | std::ios::failbit);
				stream.open(temp_dir() / std::format("translating-root-{}.bin", name), std::ios::binary);
				if (!values.empty())
					stream.write(reinterpret_cast<char const*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(*values.data())));

				stream.flush();
			};
			retain("timestep", std::array{timestep});
			retain("authored-root", std::array{root_to_world});
			retain("input-articulations", upload.m_articulations);
			retain("input-links", upload.m_links);
			retain("input-dofs", upload.m_dofs);
			retain("input-positions", upload.m_positions);
			retain("input-velocities", upload.m_velocities);
			retain("input-accelerations", upload.m_accelerations);
			retain("input-forces", upload.m_forces);
			retain("input-external-forces", upload.m_external_forces);
			retain("input-world-forces", upload.m_world_forces);
			retain("input-children", upload.m_children);
			retain("input-levels", upload.m_levels);
			retain("input-level-links", upload.m_level_links);

			// One production dispatch and its existing explicit readback seam supply the entire characterization.
			auto aba = GpuArticulationForceAba{MidpointTestGpu()};
			auto midpoint = GpuArticulationMidpoint{aba};
			auto const result = midpoint.Solve(MidpointTestGpu().m_job, upload, timestep, 1);
			retain("output-articulations", result.m_articulations);
			retain("output-positions", result.m_positions);
			retain("output-velocities", result.m_velocities);
			retain("output-accelerations", result.m_accelerations);
			retain("output-link-accelerations", result.m_link_accelerations);
			retain("output-states", result.m_states);
			config << "dispatch_count=" << result.m_dispatch_count << "\nlogical_scratch_bytes=" << result.m_logical_scratch_bytes << '\n';
			config.flush();

			// Only valid, nontrivial native output is required here; physical and approximate-map differences remain diagnostics.
			auto const shape_ok = result.m_articulations.size() == 1 && result.m_states.size() == 1 && result.m_positions.empty() &&
				result.m_velocities.size() == 6 && result.m_accelerations.size() == 6 && result.m_link_accelerations.size() == 1;
			PR_EXPECT(shape_ok);
			if (!shape_ok)
				return;

			auto const& state = result.m_states[0];
			config << "status=" << state.status << "\niteration_count=" << state.iteration_count << "\nresidual=" << state.residual << '\n';
			config.flush();
			PR_EXPECT(result.AllSucceeded());
			if (!result.AllSucceeded())
				return;

			auto const& pose = result.m_articulations[0].root_to_world;
			auto const pose_values = std::array{pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w, pose.position.x, pose.position.y, pose.position.z, pose.position.w};
			auto finite = [](float value)
			{
				// Preserve nonfinite values in raw evidence, but never call them a valid native result.
				return std::isfinite(value);
			};
			PR_EXPECT(std::ranges::all_of(pose_values, finite) && std::ranges::all_of(result.m_velocities, finite) && std::ranges::all_of(result.m_accelerations, finite));
			PR_EXPECT(result.m_dispatch_count == 1 && state.iteration_count > 0 && std::isfinite(state.residual) && state.residual >= 0);
			PR_EXPECT(result.m_velocities != upload.m_velocities);
			printf("translating-root h=%.17g status=%d iterations=%d residual=%.17g; raw output retained, no physical-error tolerance applied\n", timestep, state.status, state.iteration_count, state.residual);
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
			log_stream() << std::format("[articulation-midpoint] shallow-24x5={}ms, chain-100={}ms\n", shallow_ms, chain_ms);
			PR_EXPECT(shallow_result.AllSucceeded());
			PR_EXPECT(chain_result.AllSucceeded());
			PR_EXPECT(shallow_ms < 15000.0);
			PR_EXPECT(chain_ms < 15000.0);
		}
	};
	// Focused controls for the coordinated Engine ownership and public-boundary migration.
	PRUnitTestClass(EngineMigrationTimingTests)
	{
		// Invalid physical time rejects before primary state, forces, pending state, or publication counters can change.
		PRUnitTestMethod(EngineInvalidTimePreservesArticulationInput, Quick)
		{
			auto articulation = BuildMidpointArticulation(true);
			ApplyMidpointForces(articulation);
			auto forest = std::array{&articulation};
			auto const accepted = PackGpuArticulations(forest);
			auto engine = MidpointEngine(MidpointEngineConfig());
			auto const before = engine.LastStepProfile();

			// Byte comparisons cover the complete packed root/link state and force inputs, not only the integration coordinates.
			auto Equal = []<typename T>(std::vector<T> const& lhs, std::vector<T> const& rhs)
			{
				static_assert(std::is_trivially_copyable_v<T>);
				return lhs.size() == rhs.size() && (lhs.empty() || std::memcmp(lhs.data(), rhs.data(), lhs.size() * sizeof(T)) == 0);
			};
			auto const invalid = std::array{
				std::pair{0.0f, 3}, std::pair{-0.01f, 1}, std::pair{std::numeric_limits<float>::quiet_NaN(), 1},
				std::pair{std::numeric_limits<float>::infinity(), 1}, std::pair{-std::numeric_limits<float>::infinity(), 1},
				std::pair{std::numeric_limits<float>::denorm_min(), 2}, std::pair{0.01f, 0}, std::pair{0.01f, -1},
			};
			for (auto const [elapsed, substeps] : invalid)
			{
				// Exercise both combined and split entry points with the original three-zero-substep case included.
				auto input = Engine::StepInput{.m_articulations = std::span{forest}, .m_elapsed_seconds = elapsed, .m_substep_count = substeps};
				if (substeps < 1)
				{
					PR_THROWS(engine.Step(input), std::runtime_error);
					PR_THROWS(engine.BeginStep(input), std::runtime_error);
				}
				else
				{
					PR_THROWS(engine.Step(input), std::invalid_argument);
					PR_THROWS(engine.BeginStep(input), std::invalid_argument);
				}
				PR_EXPECT(!engine.StepPending());
				auto const output = PackGpuArticulations(forest);
				PR_EXPECT(Equal(output.m_articulations, accepted.m_articulations));
				PR_EXPECT(Equal(output.m_links, accepted.m_links));
				PR_EXPECT(Equal(output.m_external_forces, accepted.m_external_forces));
				PR_EXPECT(output.m_positions == accepted.m_positions);
				PR_EXPECT(output.m_velocities == accepted.m_velocities);
				PR_EXPECT(output.m_accelerations == accepted.m_accelerations);
				PR_EXPECT(output.m_forces == accepted.m_forces);
				PR_EXPECT(engine.LastStepProfile().m_substep_count == before.m_substep_count);
				PR_EXPECT(engine.LastStepProfile().m_submission_count == before.m_submission_count);
				PR_EXPECT(engine.LastStepProfile().m_wait_count == before.m_wait_count);
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == before.m_readback_copy_count);
			}

			// Empty input uses the same boundary, while positive representable duration has no invented normal-float floor.
			PR_THROWS(engine.Step(Engine::StepInput{.m_elapsed_seconds = 0.0f}), std::invalid_argument);
			auto const minimum = std::numeric_limits<float>::denorm_min();
			PR_EXPECT(Engine::ValidateStepTiming(2.0f * minimum, 2) == minimum);
			engine.Step(Engine::StepInput{.m_articulations = std::span{forest}, .m_elapsed_seconds = 0.001f, .m_substep_count = 3});
			ExpectArticulationForcesCleared(PackGpuArticulations(forest));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 3);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}
	};

}
#endif
