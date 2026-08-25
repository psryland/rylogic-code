//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_force_aba_gpu.h"
#include "src/compute/articulation_impulse_aba_gpu.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/interop/articulation_mobility_runner.h"
#include "src/compute/interop/coupled_constraint_prepare_runner.h"
#include "src/compute/interop/coupled_constraint_velocity_runner.h"
#include "src/constraint/constraint_solver.h"

namespace pr::physics::tests
{
	namespace
	{
		// One mixed rigid/articulation island with nontrivial state and three active D6 rows.
		struct CoupledVelocityFixture
		{
			Articulation m_articulation;
			LinkHandle m_child;
			RigidBody m_body;
			ConstraintSet m_constraints;
			ConstraintHandle m_constraint;

			// Build a floating two-link tree before the owning fixture because Articulation has no empty state.
			static std::pair<Articulation, LinkHandle> BuildArticulation()
			{
				auto root_desc = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.33f, 0.24f, 0.18f, 0}, 2.4f, v4{0.02f, -0.03f, 0.01f, 0}),
				};
				auto child_desc = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.16f, 0.29f, 0.21f, 0}, 1.3f, v4{-0.01f, 0.02f, 0.03f, 0}),
				};
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(
					root_desc,
					m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.27f, v4{0.6f, -0.3f, 0.8f, 1}),
					v8motion{v4{0.11f, -0.07f, 0.16f, 0}, v4{-0.19f, 0.13f, 0.08f, 0}});
				auto joint = ArticulationJointDesc::Revolute(
					v4::ZAxis(),
					m4x4::Transform(v4::YAxis(), 0.21f, v4{0.28f, -0.09f, 0.17f, 1}),
					m4x4::Transform(v4::XAxis(), -0.13f, v4{-0.14f, 0.12f, 0.04f, 1}));
				joint.m_initial_position[0] = 0.23f;
				joint.m_initial_velocity[0] = -0.17f;
				auto const child = builder.AddLink(root, joint, child_desc);
				return {builder.Build(), child};
			}

			// Return the mixed D6 descriptor so multi-island tests can rebuild equivalent independent constraints.
			D6ConstraintDesc Description()
			{
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame{
					.m_body = BodyRef::Link(m_articulation, m_child),
					.m_constraint_to_body = m4x4::Transform(v4::XAxis(), 0.16f, v4{0.11f, -0.07f, 0.14f, 1}),
				};
				desc.m_frame_b = BodyFrame{
					.m_body = BodyRef::Rigid(m_body),
					.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), -0.12f, v4{-0.08f, 0.10f, -0.11f, 1}),
				};
				desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
				desc.m_linear[0].m_max_force = 160.0f;
				desc.m_linear[1].m_mode = EConstraintAxisMode::Limited;
				desc.m_linear[1].m_limits = Range<float>{-0.04f, +0.04f};
				desc.m_linear[1].m_max_force = 180.0f;
				desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
				desc.m_angular[2].m_target_velocity = 0.29f;
				desc.m_angular[2].m_max_force = 130.0f;
				return desc;
			}

			// Build a deterministic mixed island suitable for CPU/GPU one-sweep parity.
			CoupledVelocityFixture()
				: CoupledVelocityFixture(BuildArticulation())
			{
			}

		private:

			// Finish construction while retaining the generated child handle.
			explicit CoupledVelocityFixture(std::pair<Articulation, LinkHandle> tree)
				: m_articulation(std::move(tree.first))
				, m_child(tree.second)
				, m_body()
				, m_constraints()
				, m_constraint()
			{
				m_body.SetMassProperties(Inertia::Box(v4{0.22f, 0.27f, 0.31f, 0}, 1.7f, v4{0.01f, -0.02f, 0.03f, 0}));
				m_body.O2W(m4x4::Transform(Normalise(v4{-1, 1, 2, 0}), -0.17f, v4{1.0f, 0.25f, -0.2f, 1}));
				m_body.VelocityWS(v8motion{v4{-0.08f, 0.12f, 0.05f, 0}, v4{0.21f, -0.14f, 0.09f, 0}});
				m_constraint = m_constraints.Add(Description());
			}
		};

		// Prepared immutable inputs for one coupled velocity replay.
		struct CoupledVelocityInputs
		{
			GpuConstraintUpload m_constraint_upload;
			GpuArticulationUpload m_articulation_upload;
			std::vector<GpuRigidBody> m_bodies;
			std::vector<GpuConstraintBlock> m_blocks;
			std::vector<GpuConstraintRow> m_rows;
			std::vector<GpuCoupledConstraintPreconditioner> m_preconditioners;
		};

		// Compile arbitrary independent fixtures through the same final-configuration replay kernels as production.
		CoupledVelocityInputs MakeCoupledVelocityInputs(ConstraintSet const& constraints, std::span<RigidBody* const> bodies, std::span<Articulation* const> articulations)
		{
			auto remap = BodyRemap(bodies, articulations);
			auto articulation_upload = PackGpuArticulations(articulations);
			auto constraint_upload = PackGpuConstraints(constraints, remap);
			auto mobility = ArticulationMobilityInteropRunner{};
			mobility.Run(articulation_upload, constraint_upload.m_coupled_articulation_indices);
			auto link_to_world = std::vector<GpuConstraintFrame>{};
			link_to_world.reserve(articulation_upload.m_links.size());
			for (auto const* articulation : articulations)
				for (int link_idx = 0; link_idx != articulation->LinkCount(); ++link_idx)
					link_to_world.push_back(PackGpuTransform(articulation->LinkToWorld(articulation->LinkAt(link_idx))));

			auto packed_bodies = std::vector<GpuRigidBody>{};
			packed_bodies.reserve(bodies.size());
			for (int body_idx = 0; body_idx != isize(bodies); ++body_idx)
				packed_bodies.push_back(PackDynamics(*bodies[body_idx], body_idx));
			auto prepare = CoupledConstraintPrepareInteropRunner{};
			prepare.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				constraint_upload,
				packed_bodies,
				link_to_world,
				mobility.Mobilities(),
				mobility.Scratch());
			return CoupledVelocityInputs{
				.m_constraint_upload = std::move(constraint_upload),
				.m_articulation_upload = std::move(articulation_upload),
				.m_bodies = std::move(packed_bodies),
				.m_blocks = std::vector<GpuConstraintBlock>(prepare.Blocks().begin(), prepare.Blocks().end()),
				.m_rows = std::vector<GpuConstraintRow>(prepare.Rows().begin(), prepare.Rows().end()),
				.m_preconditioners = std::vector<GpuCoupledConstraintPreconditioner>(prepare.Preconditioners().begin(), prepare.Preconditioners().end()),
			};
		}

		// Compile one canonical mixed fixture through the generalized input builder.
		CoupledVelocityInputs MakeCoupledVelocityInputs(CoupledVelocityFixture& fixture)
		{
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			return MakeCoupledVelocityInputs(fixture.m_constraints, bodies, articulations);
		}

		// Return a one-row unbounded constraint whose exact scalar inverse has a known monotonicity threshold.
		D6ConstraintDesc ScalarCoupledVelocityDescription(CoupledVelocityFixture& fixture)
		{
			auto desc = fixture.Description();
			desc.m_linear[0].m_max_force = std::numeric_limits<float>::max();
			desc.m_linear[1] = {};
			desc.m_angular[2] = {};
			return desc;
		}

		// Require one scalar to agree under a relative tolerance with a stable absolute floor.
		void ExpectCoupledVelocityNear(float actual, float expected, float tolerance)
		{
			auto const scale = std::max({1.0f, std::abs(actual), std::abs(expected)});
			PR_EXPECT(std::abs(actual - expected) <= tolerance * scale);
		}

		// Pack current articulation link frames in the canonical order consumed by coupled preparation.
		std::vector<GpuConstraintFrame> CoupledVelocityLinkFrames(CoupledVelocityFixture const& fixture)
		{
			auto frames = std::vector<GpuConstraintFrame>{};
			frames.reserve(fixture.m_articulation.LinkCount());
			for (int link_idx = 0; link_idx != fixture.m_articulation.LinkCount(); ++link_idx)
				frames.push_back(PackGpuTransform(fixture.m_articulation.LinkToWorld(fixture.m_articulation.LinkAt(link_idx))));
			return frames;
		}

		// Reuse one D3D12 device and command job across coupled velocity hardware tests.
		Gpu& CoupledVelocityTestGpu()
		{
			static auto gpu = Gpu{};
			return gpu;
		}
	}

	PRUnitTestClass(CoupledConstraintVelocityGpuTests)
	{
		// Match one production CPU hybrid sweep across rigid momentum, generalized velocity, and cached link velocity.
		PRUnitTestMethod(ReplayMatchesCpuHybridSweep, Quick)
		{
			auto replay_fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(replay_fixture);
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				0);

			auto cpu_fixture = CoupledVelocityFixture{};
			auto cpu_bodies = std::array<RigidBody*, 1>{&cpu_fixture.m_body};
			auto cpu_articulations = std::array<Articulation*, 1>{&cpu_fixture.m_articulation};
			auto cpu_remap = BodyRemap(cpu_bodies, cpu_articulations);
			auto cpu_solver = CpuConstraintSolver{};
			auto config = CpuConstraintSolverConfig{
				.m_velocity_iterations = 1,
				.m_position_iterations = 0,
				.m_coupled_relaxation = 0.9f,
				.m_warm_start_factor = 0.0f,
				.m_coupled_backtrack_limit = 0,
			};
			auto const metrics = cpu_solver.Solve(
				CompileConstraints(cpu_fixture.m_constraints, cpu_remap),
				cpu_remap,
				1.0f / 60.0f,
				config);
			PR_EXPECT(metrics.m_rejected_coupled_sweeps == 0);
			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(replay.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);

			auto const expected_momentum = cpu_fixture.m_body.MomentumWS();
			PR_EXPECT(FEqlAbsolute(replay.Bodies()[0].momentum_ang, expected_momentum.ang, 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(replay.Bodies()[0].momentum_lin, expected_momentum.lin, 2.0e-3f));
			auto const cpu_upload = PackGpuArticulations(cpu_articulations);
			PR_EXPECT(replay.ArticulationVelocities().size() == cpu_upload.m_velocities.size());
			for (int velocity_idx = 0; velocity_idx != isize(cpu_upload.m_velocities); ++velocity_idx)
				ExpectCoupledVelocityNear(replay.ArticulationVelocities()[velocity_idx], cpu_upload.m_velocities[velocity_idx], 2.0e-3f);
			for (int link_idx = 0; link_idx != cpu_fixture.m_articulation.LinkCount(); ++link_idx)
			{
				auto const expected = cpu_fixture.m_articulation.LinkVelocity(cpu_fixture.m_articulation.LinkAt(link_idx));
				auto const actual = replay.ArticulationScratch()[link_idx].link_velocity;
				PR_EXPECT(FEqlAbsolute(actual.ang, expected.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual.lin, expected.lin, 2.0e-3f));
			}

			auto const row_offset = inputs.m_constraint_upload.m_coupled_island_blocks[0] * GpuConstraintRowsPerBlock;
			PR_EXPECT(std::abs(replay.Rows()[row_offset + 0].bounds.z) > 1.0e-5f);
		}

		// Halve an over-large finite step until merit decreases, and reject the same step when its retry budget is exhausted.
		PRUnitTestMethod(ReplayBacktracksAndRejectsBoundedly, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const constraint = constraints.Add(ScalarCoupledVelocityDescription(fixture));
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			auto const slot_idx = constraint.m_index;

			// Four times the exact scalar inverse overshoots at 0.9 relaxation but becomes monotone after one halving.
			inputs.m_preconditioners[slot_idx].packed[0].x *= 4.0f;
			auto accepted = CoupledConstraintVelocityInteropRunner{};
			accepted.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				1);

			PR_EXPECT(accepted.IslandStates().size() == 1);
			PR_EXPECT(accepted.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(accepted.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(FEqlAbsolute(accepted.IslandStates()[0].relaxation, 0.45f, 1.0e-6f));
			PR_EXPECT(accepted.IslandStates()[0].merit_change < 0.0f);
			PR_EXPECT(std::abs(accepted.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z) > 1.0e-5f);

			// With no retry available, the identical positive-merit candidate must leave every authoritative stream unchanged.
			auto rejected = CoupledConstraintVelocityInteropRunner{};
			rejected.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				0);

			PR_EXPECT(rejected.IslandStates().size() == 1);
			PR_EXPECT(rejected.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(rejected.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_Merit));
			PR_EXPECT(FEqlAbsolute(rejected.IslandStates()[0].relaxation, 0.9f, 1.0e-6f));
			PR_EXPECT(rejected.IslandStates()[0].merit_change > 0.0f);
			PR_EXPECT(std::memcmp(rejected.Bodies().data(), inputs.m_bodies.data(), rejected.Bodies().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(rejected.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), rejected.ArticulationVelocities().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(rejected.Rows().data(), inputs.m_rows.data(), rejected.Rows().size_bytes()) == 0);
		}

		// Let independent islands accept at different attempts without reevaluating or losing the first island's detached response.
		PRUnitTestMethod(ReplayAcceptsIndependentIslandsAtDifferentAttempts, Quick)
		{
			auto first = CoupledVelocityFixture{};
			auto second = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const first_constraint = constraints.Add(ScalarCoupledVelocityDescription(first));
			auto const second_constraint = constraints.Add(ScalarCoupledVelocityDescription(second));
			auto bodies = std::array<RigidBody*, 2>{&first.m_body, &second.m_body};
			auto articulations = std::array<Articulation*, 2>{&first.m_articulation, &second.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			inputs.m_preconditioners[second_constraint.m_index].packed[0].x *= 4.0f;

			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				1);

			auto const first_island = inputs.m_constraint_upload.m_coupled_block_topology[first_constraint.m_index].island_idx;
			auto const second_island = inputs.m_constraint_upload.m_coupled_block_topology[second_constraint.m_index].island_idx;
			PR_EXPECT(replay.IslandStates().size() == 2);
			PR_EXPECT(replay.IslandStates()[first_island].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(replay.IslandStates()[second_island].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(FEqlAbsolute(replay.IslandStates()[first_island].relaxation, 0.9f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(replay.IslandStates()[second_island].relaxation, 0.45f, 1.0e-6f));
			PR_EXPECT(replay.IslandStates()[first_island].merit_change <= 0.0f);
			PR_EXPECT(replay.IslandStates()[second_island].merit_change < 0.0f);
		}

		// Reject a non-finite block candidate without changing any rigid, generalized, cached-link, or warm-start state.
		PRUnitTestMethod(ReplayRejectsCandidateTransactionally, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x = std::numeric_limits<float>::quiet_NaN();
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies);

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(replay.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(replay.Bodies().data(), inputs.m_bodies.data(), replay.Bodies().size_bytes()) == 0);
			PR_EXPECT(std::memcmp(replay.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), replay.ArticulationVelocities().size_bytes()) == 0);
			PR_EXPECT(replay.ArticulationScratch()[0].solve_valid != 0);
			PR_EXPECT(replay.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}

		// Treat a dormant runtime limit block as a successful no-op rather than rejecting unrelated rows in its island.
		PRUnitTestMethod(ReplayAcceptsDormantLimitBlock, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_blocks[slot_idx].velocity_mask = 0;
			inputs.m_blocks[slot_idx].flags = SetFlag(inputs.m_blocks[slot_idx].flags, ConstraintBlockFlags_Active, false);
			inputs.m_blocks[slot_idx].flags = SetFlag(inputs.m_blocks[slot_idx].flags, ConstraintBlockFlags_CoupledPreconditionerValid, false);
			inputs.m_bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;
			inputs.m_bodies[0].sleep.timer_s = 2.0f;
			inputs.m_bodies[0].sleep.island_id = 7;
			auto const sleeping_body = inputs.m_bodies[0];
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies);

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(replay.IslandStates()[0].failure_flags == GpuCoupledConstraintFailure_None);
			PR_EXPECT(std::memcmp(replay.Bodies().data(), &sleeping_body, sizeof(sleeping_body)) == 0);
			PR_EXPECT(std::memcmp(replay.ArticulationVelocities().data(), inputs.m_articulation_upload.m_velocities.data(), replay.ArticulationVelocities().size_bytes()) == 0);
		}

		// Reject a finite candidate whose prospective rigid momentum would overflow before any island state commits.
		PRUnitTestMethod(ReplayRejectsRigidMomentumOverflow, Quick)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			auto& block = inputs.m_blocks[slot_idx];
			block.velocity_mask = 1u;
			block.flags |= ConstraintBlockFlags_Active | ConstraintBlockFlags_CoupledPreconditionerValid;
			block.body_idx_a = -1;
			block.body_idx_b = 0;
			auto& row = inputs.m_rows[slot_idx * GpuConstraintRowsPerBlock];
			row.jacobian_a_ang = {};
			row.jacobian_a_lin = {};
			row.jacobian_b_ang = {};
			row.jacobian_b_lin = float4{1.0f, 0.0f, 0.0f, 0.0f};
			row.solve = float4{0.0f, std::numeric_limits<float>::max(), 0.0f, 0.0f};
			row.bounds = float4{-std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0f, 0.0f};
			inputs.m_preconditioners[slot_idx] = {};
			inputs.m_preconditioners[slot_idx].packed[0].x = 1.0f;
			inputs.m_bodies[0].momentum_lin.x = 3.0e38f;
			auto const body_before = inputs.m_bodies[0];
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies);

			PR_EXPECT(replay.IslandStates().size() == 1);
			PR_EXPECT(replay.IslandStates()[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(replay.IslandStates()[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(replay.Bodies().data(), &body_before, sizeof(body_before)) == 0);
			PR_EXPECT(replay.Rows()[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}

		// Match the deterministic replay on real D3D12 in one submission and release all velocity-lane storage when idle.
		PRUnitTestMethod(HardwareMatchesReplayAndOptionalCost, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies);

			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture));

			PR_EXPECT(hardware.m_bodies.size() == replay.Bodies().size());
			PR_EXPECT(hardware.m_rows.size() == replay.Rows().size());
			PR_EXPECT(hardware.m_articulation_velocities.size() == replay.ArticulationVelocities().size());
			PR_EXPECT(hardware.m_articulation_accelerations.size() == replay.ArticulationAccelerations().size());
			PR_EXPECT(hardware.m_articulation_scratch.size() == replay.ArticulationScratch().size());
			PR_EXPECT(hardware.m_island_states.size() == replay.IslandStates().size());
			for (size_t body_idx = 0; body_idx != hardware.m_bodies.size(); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_bodies[body_idx].momentum_ang, replay.Bodies()[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(hardware.m_bodies[body_idx].momentum_lin, replay.Bodies()[body_idx].momentum_lin, 2.0e-3f));
			}
			for (size_t velocity_idx = 0; velocity_idx != hardware.m_articulation_velocities.size(); ++velocity_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_velocities[velocity_idx], replay.ArticulationVelocities()[velocity_idx], 2.0e-3f);
			for (size_t acceleration_idx = 0; acceleration_idx != hardware.m_articulation_accelerations.size(); ++acceleration_idx)
				ExpectCoupledVelocityNear(hardware.m_articulation_accelerations[acceleration_idx], replay.ArticulationAccelerations()[acceleration_idx], 2.0e-3f);
			for (size_t link_idx = 0; link_idx != hardware.m_articulation_scratch.size(); ++link_idx)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.ang, replay.ArticulationScratch()[link_idx].link_velocity.ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(hardware.m_articulation_scratch[link_idx].link_velocity.lin, replay.ArticulationScratch()[link_idx].link_velocity.lin, 2.0e-3f));
			}
			for (size_t row_idx = 0; row_idx != hardware.m_rows.size(); ++row_idx)
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[row_idx].bounds.z, replay.Rows()[row_idx].bounds.z, 2.0e-3f));
			PR_EXPECT(hardware.m_island_states[0].status == replay.IslandStates()[0].status);
			PR_EXPECT(hardware.m_island_states[0].failure_flags == replay.IslandStates()[0].failure_flags);
			PR_EXPECT(FEqlAbsolute(hardware.m_island_states[0].relaxation, replay.IslandStates()[0].relaxation, 1.0e-6f));
			ExpectCoupledVelocityNear(hardware.m_island_states[0].merit_change, replay.IslandStates()[0].merit_change, 2.0e-3f);
			PR_EXPECT(solver.Stats().m_dispatch_count == 5 * (config.constraint_coupled_backtrack_limit + 1) + 4);
			PR_EXPECT(solver.Stats().m_logical_bytes != 0);

			auto empty = GpuConstraintUpload{};
			PR_EXPECT(!solver.Upload(CoupledVelocityTestGpu().m_job, empty));
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}

		// Match successful merit backtracking and bounded transactional exhaustion on real D3D12.
		PRUnitTestMethod(HardwareBacktracksAndRejectsBoundedly, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto constraints = ConstraintSet{};
			auto const constraint = constraints.Add(ScalarCoupledVelocityDescription(fixture));
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto inputs = MakeCoupledVelocityInputs(constraints, bodies, articulations);
			auto const slot_idx = constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x *= 4.0f;

			auto replay = CoupledConstraintVelocityInteropRunner{};
			replay.Run(
				0.9f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_blocks,
				inputs.m_rows,
				inputs.m_preconditioners,
				inputs.m_bodies,
				1);

			// The one-retry hardware lane must commit the same half-relaxation candidate and exact merit as deterministic replay.
			auto accepted_config = EngineConfig{};
			accepted_config.constraint_coupled_relaxation = 0.9f;
			accepted_config.constraint_coupled_backtrack_limit = 1;
			auto accepted_constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), accepted_config};
			auto accepted_force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto accepted_mobility = GpuArticulationMobility{accepted_force_aba};
			auto accepted_impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), accepted_force_aba, accepted_mobility};
			auto accepted_prepare = GpuCoupledConstraintPrepare{accepted_constraints, accepted_config};
			auto accepted_solver = GpuCoupledConstraintVelocity{accepted_prepare, accepted_impulse_aba, accepted_config};
			auto const accepted = accepted_solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(accepted.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Committed);
			PR_EXPECT(FEqlAbsolute(accepted.m_island_states[0].relaxation, 0.45f, 1.0e-6f));
			ExpectCoupledVelocityNear(accepted.m_island_states[0].merit_change, replay.IslandStates()[0].merit_change, 2.0e-3f);
			PR_EXPECT(FEqlAbsolute(accepted.m_bodies[0].momentum_ang, replay.Bodies()[0].momentum_ang, 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(accepted.m_bodies[0].momentum_lin, replay.Bodies()[0].momentum_lin, 2.0e-3f));
			PR_EXPECT(accepted_solver.Stats().m_dispatch_count == 14);

			// A zero-retry hardware lane must report merit exhaustion and preserve all authoritative state.
			auto rejected_config = EngineConfig{};
			rejected_config.constraint_coupled_relaxation = 0.9f;
			rejected_config.constraint_coupled_backtrack_limit = 0;
			auto rejected_constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), rejected_config};
			auto rejected_force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto rejected_mobility = GpuArticulationMobility{rejected_force_aba};
			auto rejected_impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), rejected_force_aba, rejected_mobility};
			auto rejected_prepare = GpuCoupledConstraintPrepare{rejected_constraints, rejected_config};
			auto rejected_solver = GpuCoupledConstraintVelocity{rejected_prepare, rejected_impulse_aba, rejected_config};
			auto const rejected = rejected_solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(rejected.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(rejected.m_island_states[0].failure_flags, GpuCoupledConstraintFailure_Merit));
			PR_EXPECT(rejected.m_island_states[0].merit_change > 0.0f);
			PR_EXPECT(std::memcmp(rejected.m_bodies.data(), inputs.m_bodies.data(), rejected.m_bodies.size() * sizeof(rejected.m_bodies[0])) == 0);
			PR_EXPECT(std::memcmp(rejected.m_articulation_velocities.data(), inputs.m_articulation_upload.m_velocities.data(), rejected.m_articulation_velocities.size() * sizeof(rejected.m_articulation_velocities[0])) == 0);
			for (size_t row_idx = 0; row_idx != rejected.m_rows.size(); ++row_idx)
				PR_EXPECT(rejected.m_rows[row_idx].bounds.z == inputs.m_rows[row_idx].bounds.z);
			PR_EXPECT(rejected_solver.Stats().m_dispatch_count == 9);
		}

		// Reject a hardware NaN correction before HLSL clamp can select a finite bound and commit an arbitrary impulse.
		PRUnitTestMethod(HardwareRejectsNonFiniteCorrection, Extended)
		{
			auto fixture = CoupledVelocityFixture{};
			auto inputs = MakeCoupledVelocityInputs(fixture);
			auto const slot_idx = fixture.m_constraint.m_index;
			inputs.m_preconditioners[slot_idx].packed[0].x = std::numeric_limits<float>::quiet_NaN();

			auto config = EngineConfig{};
			config.constraint_coupled_relaxation = 0.9f;
			auto constraints = GpuConstraintSolver{CoupledVelocityTestGpu(), config};
			auto force_aba = GpuArticulationForceAba{CoupledVelocityTestGpu()};
			auto mobility = GpuArticulationMobility{force_aba};
			auto impulse_aba = GpuArticulationImpulseAba{CoupledVelocityTestGpu(), force_aba, mobility};
			auto prepare = GpuCoupledConstraintPrepare{constraints, config};
			auto solver = GpuCoupledConstraintVelocity{prepare, impulse_aba, config};
			auto const hardware = solver.Solve(
				CoupledVelocityTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraint_upload,
				inputs.m_articulation_upload,
				inputs.m_bodies,
				CoupledVelocityLinkFrames(fixture),
				inputs.m_preconditioners);

			PR_EXPECT(hardware.m_island_states.size() == 1);
			PR_EXPECT(hardware.m_island_states[0].status == GpuCoupledConstraintIslandStatus_Rejected);
			PR_EXPECT(AllSet(hardware.m_island_states[0].failure_flags, GpuCoupledConstraintFailure_NonFinite));
			PR_EXPECT(std::memcmp(hardware.m_bodies.data(), inputs.m_bodies.data(), hardware.m_bodies.size() * sizeof(hardware.m_bodies[0])) == 0);
			PR_EXPECT(hardware.m_rows[slot_idx * GpuConstraintRowsPerBlock].bounds.z == 0.0f);
		}
	};
}
#endif
