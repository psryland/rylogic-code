//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/interop/articulation_mobility_runner.h"
#include "src/compute/interop/coupled_constraint_prepare_runner.h"
#include "src/constraint/constraint_compiler.h"

namespace pr::physics::tests
{
	namespace
	{
		// One floating articulation, constrained child, and ordinary rigid endpoint for coupled preparation tests.
		struct CoupledPrepareFixture
		{
			Articulation m_articulation;
			LinkHandle m_child;
			RigidBody m_body;
			ConstraintSet m_constraints;
			ConstraintHandle m_constraint;

			// Build the articulation before the fixture because Articulation deliberately has no empty state.
			static std::pair<Articulation, LinkHandle> BuildArticulation()
			{
				auto root_link = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.31f, 0.24f, 0.19f, 0}, 2.3f, v4{0.03f, -0.02f, 0.01f, 0}),
				};
				auto child_link = ArticulationLinkDesc{
					.m_inertia = Inertia::Box(v4{0.17f, 0.28f, 0.22f, 0}, 1.4f, v4{-0.02f, 0.01f, 0.04f, 0}),
				};
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(
					root_link,
					m4x4::Transform(Normalise(v4{1, 2, -1, 0}), 0.31f, v4{0.7f, -0.4f, 0.9f, 1}),
					v8motion{});
				auto joint = ArticulationJointDesc::Revolute(
					v4::ZAxis(),
					m4x4::Transform(v4::YAxis(), 0.23f, v4{0.3f, -0.1f, 0.2f, 1}),
					m4x4::Transform(v4::XAxis(), -0.17f, v4{-0.2f, 0.15f, 0.05f, 1}));
				joint.m_initial_position[0] = 0.27f;
				auto const child = builder.AddLink(root, joint, child_link);
				return {builder.Build(), child};
			}

			// Build a non-axis-aligned fixture so frame conversion and mixed endpoint response are both observable.
			CoupledPrepareFixture()
				: CoupledPrepareFixture(BuildArticulation())
			{
			}

		private:

			// Complete construction from a valid tree while preserving the child's stable generational handle.
			explicit CoupledPrepareFixture(std::pair<Articulation, LinkHandle> tree)
				: m_articulation(std::move(tree.first))
				, m_child(tree.second)
				, m_body()
				, m_constraints()
				, m_constraint()
			{

				m_body.SetMassProperties(Inertia::Box(v4{0.21f, 0.26f, 0.34f, 0}, 1.8f, v4{0.02f, 0.03f, -0.01f, 0}));
				m_body.O2W(m4x4::Transform(Normalise(v4{-1, 1, 2, 0}), -0.19f, v4{1.1f, 0.2f, -0.3f, 1}));

				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame{
					BodyRef::Link(m_articulation, m_child),
					m4x4::Transform(v4::XAxis(), 0.18f, v4{0.12f, -0.08f, 0.16f, 1}),
				};
				desc.m_frame_b = BodyFrame{
					BodyRef::Rigid(m_body),
					m4x4::Transform(v4::ZAxis(), -0.14f, v4{-0.09f, 0.11f, -0.13f, 1}),
				};
				desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
				desc.m_linear[0].m_max_force = 120.0f;
				desc.m_linear[1].m_mode = EConstraintAxisMode::Limited;
				desc.m_linear[1].m_limits = Range<float>{-0.05f, +0.05f};
				desc.m_linear[1].m_max_force = 140.0f;
				desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
				desc.m_angular[2].m_target_velocity = 0.35f;
				desc.m_angular[2].m_max_force = 90.0f;
				m_constraint = m_constraints.Add(desc);
			}
		};

		// Complete packed inputs required by both replay and hardware preparation.
		struct CoupledPrepareInputs
		{
			GpuConstraintUpload m_constraints;
			std::vector<GpuRigidBody> m_bodies;
			std::vector<GpuConstraintFrame> m_link_to_world;
			std::vector<GpuArticulationSpatialMobility> m_mobilities;
			std::vector<GpuArticulationAbaScratch> m_aba_scratch;
			CompiledConstraintSet m_compiled;
			std::vector<detail::SpatialMobility> m_cpu_mobilities;
		};

		// Pack canonical final-configuration frames and mobility factors for one fixture.
		CoupledPrepareInputs MakeCoupledPrepareInputs(CoupledPrepareFixture& fixture)
		{
			auto bodies = std::array<RigidBody*, 1>{&fixture.m_body};
			auto articulations = std::array<Articulation*, 1>{&fixture.m_articulation};
			auto remap = BodyRemap(bodies, articulations);
			auto articulation_upload = PackGpuArticulations(articulations);
			auto mobility_replay = ArticulationMobilityInteropRunner{};
			auto participants = std::array{0};
			mobility_replay.Run(articulation_upload, participants);

			auto inputs = CoupledPrepareInputs{
				.m_constraints = PackGpuConstraints(fixture.m_constraints, remap),
				.m_bodies = {PackDynamics(fixture.m_body, 0)},
				.m_link_to_world = {},
				.m_mobilities = std::vector<GpuArticulationSpatialMobility>(mobility_replay.Mobilities().begin(), mobility_replay.Mobilities().end()),
				.m_aba_scratch = std::vector<GpuArticulationAbaScratch>(mobility_replay.Scratch().begin(), mobility_replay.Scratch().end()),
				.m_compiled = CompileConstraints(fixture.m_constraints, remap),
				.m_cpu_mobilities = std::vector<detail::SpatialMobility>(fixture.m_articulation.LinkCount()),
			};
			inputs.m_link_to_world.reserve(fixture.m_articulation.LinkCount());
			for (int link_index = 0; link_index != fixture.m_articulation.LinkCount(); ++link_index)
				inputs.m_link_to_world.push_back(PackGpuTransform(fixture.m_articulation.LinkToWorld(fixture.m_articulation.LinkAt(link_index))));
			detail::ComputeArticulationLinkMobilities(fixture.m_articulation, inputs.m_cpu_mobilities);
			return inputs;
		}

		// Return one scalar from a packed symmetric preconditioner.
		float CoupledPreconditionerComponent(GpuCoupledConstraintPreconditioner const& preconditioner, int row, int column)
		{
			auto const low = std::min(row, column);
			auto const high = std::max(row, column);
			auto const packed_index = low * 6 - low * (low - 1) / 2 + high - low;
			return preconditioner.packed[packed_index / 4][packed_index % 4];
		}

		// Return one exact-self response entry from the independent CPU spatial operators.
		float CoupledCpuResponse(
			CompiledConstraintRow const& lhs,
			CompiledConstraintRow const& rhs,
			CompiledConstraintBlock const& block,
			CoupledPrepareFixture const& fixture,
			std::span<detail::SpatialMobility const> mobilities)
		{
			auto response = 0.0f;
			if (block.m_endpoint_a.IsLink())
				response += Dot(lhs.m_jacobian_a, mobilities[block.m_endpoint_a.m_link_index] * rhs.m_jacobian_a);
			else if (block.m_endpoint_a.IsRigid())
				response += Dot(lhs.m_jacobian_a, fixture.m_body.InertiaInvWS() * rhs.m_jacobian_a);
			if (block.m_endpoint_b.IsLink())
				response += Dot(lhs.m_jacobian_b, mobilities[block.m_endpoint_b.m_link_index] * rhs.m_jacobian_b);
			else if (block.m_endpoint_b.IsRigid())
				response += Dot(lhs.m_jacobian_b, fixture.m_body.InertiaInvWS() * rhs.m_jacobian_b);
			return response;
		}

		// Require one runtime row to match the production CPU compiler's endpoint-coordinate Jacobians.
		void ExpectCoupledRowNear(GpuConstraintRow const& gpu, CompiledConstraintRow const& cpu, float tolerance)
		{
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_ang, cpu.m_jacobian_a.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_lin, cpu.m_jacobian_a.lin, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_ang, cpu.m_jacobian_b.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_lin, cpu.m_jacobian_b.lin, tolerance));
		}

		// Replace the rigid endpoint with a nontrivial world-space frame while retaining the coupled link endpoint.
		void SetFixtureWorldEndpoint(CoupledPrepareFixture& fixture)
		{
			auto desc = fixture.m_constraints.Get(fixture.m_constraint);
			desc.m_frame_b = BodyFrame{
				.m_body = BodyRef::World(),
				.m_constraint_to_body = m4x4::Transform(v4::ZAxis(), -0.14f, v4{1.01f, 0.31f, -0.43f, 1}),
			};
			fixture.m_constraints.Update(fixture.m_constraint, desc);
		}

		// Require hardware preparation to match deterministic replay for every shared output stream.
		void ExpectCoupledHardwareNear(GpuCoupledConstraintPrepareResult const& hardware, CoupledConstraintPrepareInteropRunner const& replay)
		{
			PR_EXPECT(hardware.m_blocks.size() == replay.Blocks().size());
			PR_EXPECT(hardware.m_rows.size() == replay.Rows().size());
			PR_EXPECT(hardware.m_preconditioners.size() == replay.Preconditioners().size());
			PR_EXPECT(std::memcmp(hardware.m_blocks.data(), replay.Blocks().data(), hardware.m_blocks.size() * sizeof(hardware.m_blocks[0])) == 0);
			for (size_t index = 0; index != hardware.m_rows.size(); ++index)
			{
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].jacobian_a_ang, replay.Rows()[index].jacobian_a_ang, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].jacobian_a_lin, replay.Rows()[index].jacobian_a_lin, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].jacobian_b_ang, replay.Rows()[index].jacobian_b_ang, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].jacobian_b_lin, replay.Rows()[index].jacobian_b_lin, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].solve, replay.Rows()[index].solve, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(hardware.m_rows[index].bounds, replay.Rows()[index].bounds, 3.0e-5f));
			}
			for (size_t index = 0; index != hardware.m_preconditioners.size(); ++index)
			for (int row = 0; row != 6; ++row)
				PR_EXPECT(FEqlAbsolute(hardware.m_preconditioners[index].packed[row], replay.Preconditioners()[index].packed[row], 3.0e-4f));
		}

		// Reuse one D3D12 device and command job across coupled preparation hardware tests.
		Gpu& CoupledPrepareTestGpu()
		{
			static auto gpu = Gpu{};
			return gpu;
		}
	}

	PRUnitTestClass(CoupledConstraintPrepareGpuTests)
	{
		// Match CPU link-coordinate row compilation and prove the packed inverse is the exact-self block inverse.
		PRUnitTestMethod(ReplayMatchesCpuCompilerAndResponse, Quick)
		{
			auto fixture = CoupledPrepareFixture{};
			auto inputs = MakeCoupledPrepareInputs(fixture);
			auto runner = CoupledConstraintPrepareInteropRunner{};
			runner.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				inputs.m_constraints,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			auto const slot_index = fixture.m_constraint.m_index;
			auto const& block = runner.Blocks()[slot_index];
			PR_EXPECT(block.velocity_mask == ((1u << 0) | (1u << 1) | (1u << 5)));
			PR_EXPECT(AllSet(block.flags, ConstraintBlockFlags_Active));
			PR_EXPECT(AllSet(block.flags, ConstraintBlockFlags_CoupledPreconditionerValid));
			PR_EXPECT(block.body_idx_a == -1);
			PR_EXPECT(block.body_idx_b == 0);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 0], inputs.m_compiled.m_rows[0], 3.0e-5f);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 1], inputs.m_compiled.m_rows[1], 3.0e-5f);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 5], inputs.m_compiled.m_rows[2], 3.0e-5f);

			// Multiplying the independent exact-self response by the stored inverse must recover identity.
			auto response = std::array<float, 9>{};
			auto const& compiled_block = inputs.m_compiled.m_blocks[0];
			for (int row = 0; row != 3; ++row)
			for (int column = 0; column != 3; ++column)
			{
				response[row * 3 + column] = CoupledCpuResponse(
					inputs.m_compiled.m_rows[row],
					inputs.m_compiled.m_rows[column],
					compiled_block,
					fixture,
					inputs.m_cpu_mobilities);
				if (row == column)
					response[row * 3 + column] += runner.Rows()[slot_index * GpuConstraintRowsPerBlock + std::array{0, 1, 5}[row]].solve.w;
			}

			auto const& preconditioner = runner.Preconditioners()[slot_index];
			for (int row = 0; row != 3; ++row)
			for (int column = 0; column != 3; ++column)
			{
				auto actual = 0.0f;
				for (int inner = 0; inner != 3; ++inner)
					actual += response[row * 3 + inner] * CoupledPreconditionerComponent(preconditioner, inner, column);
				PR_EXPECT(FEqlAbsolute(actual, row == column ? 1.0f : 0.0f, 2.0e-3f));
			}
		}

		// Match CPU compilation and suppress the impulse Jacobian for a fixed world endpoint.
		PRUnitTestMethod(WorldEndpointHasNoJacobian, Quick)
		{
			auto fixture = CoupledPrepareFixture{};
			SetFixtureWorldEndpoint(fixture);
			auto inputs = MakeCoupledPrepareInputs(fixture);
			auto runner = CoupledConstraintPrepareInteropRunner{};
			runner.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				inputs.m_constraints,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			auto const slot_index = fixture.m_constraint.m_index;
			auto const& block = runner.Blocks()[slot_index];
			PR_EXPECT(block.body_idx_a == -1);
			PR_EXPECT(block.body_idx_b == -1);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 0], inputs.m_compiled.m_rows[0], 3.0e-5f);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 1], inputs.m_compiled.m_rows[1], 3.0e-5f);
			ExpectCoupledRowNear(runner.Rows()[slot_index * GpuConstraintRowsPerBlock + 5], inputs.m_compiled.m_rows[2], 3.0e-5f);
			for (auto const axis_index : std::array{0, 1, 5})
			{
				auto const& row = runner.Rows()[slot_index * GpuConstraintRowsPerBlock + axis_index];
				PR_EXPECT(FEqlAbsolute(row.jacobian_b_ang, v4::Zero(), 1.0e-7f));
				PR_EXPECT(FEqlAbsolute(row.jacobian_b_lin, v4::Zero(), 1.0e-7f));
			}
		}

		// Disable the complete block when its participating tree did not produce valid retained ABA factors.
		PRUnitTestMethod(InvalidTreeFactorsFailClosed, Quick)
		{
			auto fixture = CoupledPrepareFixture{};
			auto inputs = MakeCoupledPrepareInputs(fixture);
			inputs.m_aba_scratch[0].solve_valid = 0;
			auto runner = CoupledConstraintPrepareInteropRunner{};
			runner.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				inputs.m_constraints,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			auto const slot_index = fixture.m_constraint.m_index;
			PR_EXPECT(!AllSet(runner.Blocks()[slot_index].flags, ConstraintBlockFlags_Active));
			PR_EXPECT(!AllSet(runner.Blocks()[slot_index].flags, ConstraintBlockFlags_CoupledPreconditionerValid));
			PR_EXPECT(runner.Blocks()[slot_index].velocity_mask == 0);
		}

		// Match real D3D12 output with replay and release every owned byte when coupled work disappears.
		PRUnitTestMethod(HardwareMatchesReplayAndOptionalCost, Extended)
		{
			auto fixture = CoupledPrepareFixture{};
			auto inputs = MakeCoupledPrepareInputs(fixture);
			auto replay_upload = inputs.m_constraints;
			for (auto& endpoint : replay_upload.m_endpoints)
				if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled))
					endpoint.flags |= GpuConstraintEndpointFlags_ResetWarmStart;

			auto replay = CoupledConstraintPrepareInteropRunner{};
			replay.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				replay_upload,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			auto config = EngineConfig{};
			auto shared = GpuConstraintSolver{CoupledPrepareTestGpu(), config};
			auto solver = GpuCoupledConstraintPrepare{shared, config};
			auto const hardware = solver.Solve(
				CoupledPrepareTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraints,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			ExpectCoupledHardwareNear(hardware, replay);

			PR_EXPECT(solver.Stats().m_dispatch_count == 1);
			PR_EXPECT(solver.Stats().m_logical_bytes == inputs.m_constraints.m_endpoints.size() * 128);
			auto empty = GpuConstraintUpload{};
			shared.Upload(CoupledPrepareTestGpu().m_job, empty);
			PR_EXPECT(!solver.Upload(CoupledPrepareTestGpu().m_job, empty));
			PR_EXPECT(solver.Stats().m_allocated_feature_bytes == 0);
		}

		// Match real D3D12 output for a link-to-world row so world Jacobian suppression cannot diverge from replay.
		PRUnitTestMethod(HardwareMatchesReplayForWorldEndpoint, Extended)
		{
			auto fixture = CoupledPrepareFixture{};
			SetFixtureWorldEndpoint(fixture);
			auto inputs = MakeCoupledPrepareInputs(fixture);
			auto replay_upload = inputs.m_constraints;
			for (auto& endpoint : replay_upload.m_endpoints)
				if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled))
					endpoint.flags |= GpuConstraintEndpointFlags_ResetWarmStart;

			auto replay = CoupledConstraintPrepareInteropRunner{};
			replay.Run(
				1.0f / 60.0f,
				1.0e-6f,
				0.0f,
				replay_upload,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			auto config = EngineConfig{};
			auto shared = GpuConstraintSolver{CoupledPrepareTestGpu(), config};
			auto solver = GpuCoupledConstraintPrepare{shared, config};
			auto const hardware = solver.Solve(
				CoupledPrepareTestGpu().m_job,
				1.0f / 60.0f,
				inputs.m_constraints,
				inputs.m_bodies,
				inputs.m_link_to_world,
				inputs.m_mobilities,
				inputs.m_aba_scratch);

			ExpectCoupledHardwareNear(hardware, replay);
		}
	};
}
#endif
