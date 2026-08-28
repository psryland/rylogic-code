//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/constraint/constraint_gpu.h"
#include "src/constraint/constraint_solver.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/interop/constraint_runner.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// Construct a finite body with controllable transform and off-origin centre of mass.
		RigidBody MakeConstraintGpuBody(float mass = 1.0f, m4x4 const& o2w = m4x4::Identity(), v4 com_os = v4::Zero())
		{
			auto body = RigidBody{};
			body.SetMassProperties(Inertia::Sphere(1.0f, mass, com_os), com_os);
			body.O2W(o2w);
			return body;
		}

		// Return a hard locked axis with a finite force bound.
		ConstraintAxisDesc MakeConstraintGpuLockedAxis(float max_force = 1000.0f)
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			axis.m_max_force = max_force;
			return axis;
		}

		// Compare one runtime Jacobian with the corresponding CPU compiler row.
		void ExpectConstraintGpuJacobian(GpuConstraintRow const& gpu, CompiledConstraintRow const& cpu, float tolerance = 2.0e-5f)
		{
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_ang, cpu.m_jacobian_a.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_a_lin, cpu.m_jacobian_a.lin, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_ang, cpu.m_jacobian_b.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(gpu.jacobian_b_lin, cpu.m_jacobian_b.lin, tolerance));
		}

		// Return true when all transform and momentum components of a packed body are finite.
		bool ConstraintGpuBodyFinite(GpuRigidBody const& body)
		{
			for (int row = 0; row != 4; ++row)
				for (int column = 0; column != 4; ++column)
					if (!std::isfinite(body.o2w[row][column]))
						return false;
			return
				IsFinite(body.momentum_ang) &&
				IsFinite(body.momentum_lin);
		}

		// Reuse one D3D12 device and command job across hardware constraint tests.
		Gpu& ConstraintTestGpu()
		{
			return SharedTestGpu();
		}

		// Build a minimal floating tree for testing the unsupported coupled-solver boundary.
		std::pair<Articulation, LinkHandle> MakeConstraintGpuArticulation()
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link);
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis()), link);
			return {builder.Build(), child};
		}
	}

	PRUnitTestClass(ConstraintGpuSolverTests)
	{
		// Match CPU world-frame compilation for rotated frames, offset CoMs, a world endpoint, and mixed canonical axis modes.
		PRUnitTestMethod(CompileMatchesCpuCanonicalRows, Quick)
		{
			auto body = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.35f, v4{1.0f, -2.0f, 0.5f, 1}),
				v4{0.2f, -0.1f, 0.3f, 0});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame{BodyRef::Rigid(body), m4x4::Transform(v4::ZAxis(), -0.2f, v4{0.4f, 0.1f, -0.3f, 1})};
			desc.m_frame_b = BodyFrame{BodyRef::World(), m4x4::Transform(v4::XAxis(), 0.15f, v4{1.3f, -1.7f, 0.8f, 1})};
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			desc.m_linear[1].m_mode = EConstraintAxisMode::Limited;
			desc.m_linear[1].m_limits = Range<float>{-0.05f, +0.05f};
			desc.m_linear[1].m_stiffness = 12.0f;
			desc.m_linear[1].m_damping = 3.0f;
			desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[2].m_target_position = 0.1f;
			desc.m_angular[2].m_target_velocity = -0.4f;
			desc.m_angular[2].m_max_force = 50.0f;

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const cpu = CompileConstraints(constraints, remap);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto runner = ConstraintInteropRunner{};
			auto buffers = ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = gpu_bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			};
			runner.Load(buffers);
			runner.CompileConstraints();

			PR_EXPECT(cpu.m_blocks.size() == 1);
			PR_EXPECT(cpu.m_rows.size() == 3);
			PR_EXPECT(runner.Blocks()[0].velocity_mask == ((1u << 0) | (1u << 1) | (1u << 5)));
			PR_EXPECT((runner.Blocks()[0].position_mask & (1u << 5)) == 0u);
			ExpectConstraintGpuJacobian(runner.Rows()[0], cpu.m_rows[0]);
			ExpectConstraintGpuJacobian(runner.Rows()[1], cpu.m_rows[1]);
			ExpectConstraintGpuJacobian(runner.Rows()[5], cpu.m_rows[2]);
			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].solve.x, cpu.m_rows[0].m_position - cpu.m_rows[0].m_target_position, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(runner.Rows()[5].solve.y, cpu.m_rows[2].m_target_velocity, 1.0e-6f));
			PR_EXPECT(runner.Rows()[1].solve.w > 0.0f);
		}

		// Match the CPU block PGS momentum result for a coupled off-centre three-axis lock.
		PRUnitTestMethod(VelocitySolveMatchesCpuReference, Quick)
		{
			auto body_a = MakeConstraintGpuBody(2.0f, m4x4::Identity(), v4{0.2f, -0.1f, 0.05f, 0});
			auto body_b = MakeConstraintGpuBody(3.0f, m4x4::Translation(0.2f, -0.1f, 0.3f), v4{-0.1f, 0.15f, -0.05f, 0});
			body_a.VelocityWS(v8motion{v4{0.3f, -0.2f, 0.1f, 0}, v4{1.0f, -0.5f, 0.7f, 0}});
			body_b.VelocityWS(v8motion{v4{-0.1f, 0.4f, -0.2f, 0}, v4{-0.6f, 0.8f, -0.3f, 0}});

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), m4x4::Translation(0.4f, 0.2f, -0.1f)};
			desc.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), m4x4::Translation(0.2f, 0.3f, -0.2f)};
			for (int axis = 0; axis != 3; ++axis)
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 4;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;

			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = gpu_bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});
			auto cpu_solver = CpuConstraintSolver{};
			cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, config);

			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, body_a.MomentumWS().ang, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, body_a.MomentumWS().lin, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_ang, body_b.MomentumWS().ang, 5.0e-4f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_lin, body_b.MomentumWS().lin, 5.0e-4f));
		}

		// Match the CPU reference across deterministic six-axis blocks with varied frames, CoMs, transforms, and momenta.
		PRUnitTestMethod(RandomizedD6BlocksMatchCpuReference, Quick)
		{
			for (int trial = 0; trial != 8; ++trial)
			{
				auto const seed = static_cast<float>(trial + 1);
				auto body_a = MakeConstraintGpuBody(
					1.0f + 0.2f * seed,
					m4x4::Transform(Normalise(v4{0.3f, 0.5f, 0.7f, 0}), 0.07f * seed, v4{0.1f * seed, -0.03f * seed, 0.02f * seed, 1}),
					v4{0.02f * seed, -0.01f * seed, 0.015f * seed, 0});
				auto body_b = MakeConstraintGpuBody(
					1.5f + 0.1f * seed,
					m4x4::Transform(Normalise(v4{0.6f, -0.2f, 0.4f, 0}), -0.05f * seed, v4{-0.04f * seed, 0.06f * seed, -0.01f * seed, 1}),
					v4{-0.01f * seed, 0.025f * seed, -0.02f * seed, 0});
				body_a.VelocityWS(v8motion{v4{0.13f * seed, -0.07f * seed, 0.04f * seed, 0}, v4{0.2f * seed, -0.11f * seed, 0.09f * seed, 0}});
				body_b.VelocityWS(v8motion{v4{-0.08f * seed, 0.05f * seed, -0.03f * seed, 0}, v4{-0.15f * seed, 0.12f * seed, -0.06f * seed, 0}});

				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), m4x4::Transform(v4::ZAxis(), 0.03f * seed, v4{0.2f, -0.1f, 0.15f, 1})};
				desc.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), m4x4::Transform(v4::YAxis(), -0.02f * seed, v4{-0.1f, 0.25f, -0.05f, 1})};
				for (int axis = 0; axis != 3; ++axis)
				{
					desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
					desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				}
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto remap = BodyRemap(body_ptrs);
				auto const upload = PackGpuConstraints(constraints, remap);
				auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 6;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 120.0f, gpu_bodies, upload.m_endpoints, upload.m_descriptors});
				auto cpu_solver = CpuConstraintSolver{};
				cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 120.0f, config);

				PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, body_a.MomentumWS().ang, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, body_a.MomentumWS().lin, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_ang, body_b.MomentumWS().ang, 1.5e-3f));
				PR_EXPECT(FEqlAbsolute(gpu_bodies[1].momentum_lin, body_b.MomentumWS().lin, 1.5e-3f));
			}
		}

		// Resolve a scalar constraint across the required mass-ratio range without losing passivity, momentum, or finite state.
		PRUnitTestMethod(ExtremeMassRatiosRemainFiniteAndPassive, Quick)
		{
			for (auto const heavy_mass : {1.0f, 1.0e2f, 1.0e4f, 1.0e6f})
			{
				auto body_a = MakeConstraintGpuBody(1.0f);
				auto body_b = MakeConstraintGpuBody(heavy_mass);
				body_a.VelocityWS(v8motion{v4::Zero(), v4::XAxis()});
				body_b.VelocityWS(v8motion{});

				// A single bilateral scalar row has a closed-form shared velocity that remains well-conditioned at 10^6:1.
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
				desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
				desc.m_linear[0] = MakeConstraintGpuLockedAxis();
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 1;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors});

				// The projected update must converge to the analytic inelastic velocity without creating kinetic energy.
				auto const velocity_a = bodies[0].momentum_lin.x * bodies[0].os_com_and_invmass.w;
				auto const velocity_b = bodies[1].momentum_lin.x * bodies[1].os_com_and_invmass.w;
				auto const expected_velocity = 1.0f / (1.0f + heavy_mass);
				auto const momentum_after = bodies[0].momentum_lin.x + bodies[1].momentum_lin.x;
				auto const energy_after = 0.5f * (
					Sqr(bodies[0].momentum_lin.x) * bodies[0].os_com_and_invmass.w +
					Sqr(bodies[1].momentum_lin.x) * bodies[1].os_com_and_invmass.w);
				PR_EXPECT(ConstraintGpuBodyFinite(bodies[0]));
				PR_EXPECT(ConstraintGpuBodyFinite(bodies[1]));
				PR_EXPECT(Abs(velocity_a - expected_velocity) < 2.0e-6f);
				PR_EXPECT(Abs(velocity_b - expected_velocity) < 2.0e-6f);
				PR_EXPECT(Abs(velocity_a - velocity_b) < 2.0e-6f);
				PR_EXPECT(Abs(momentum_after - 1.0f) < 2.0e-5f);
				PR_EXPECT(energy_after <= 0.50001f);
			}
		}

		// Preserve constraint results under arbitrary common world translations and rotations.
		PRUnitTestMethod(WorldTransformMetamorphicEquivalence, Quick)
		{
			auto const base_o2w_a = m4x4::Transform(v4::YAxis(), +0.23f, v4{-0.4f, +0.2f, -0.1f, 1});
			auto const base_o2w_b = m4x4::Transform(v4::XAxis(), -0.31f, v4{+0.5f, -0.3f, +0.2f, 1});
			auto const base_velocity_a = v8motion{v4{+0.3f, -0.2f, +0.1f, 0}, v4{+1.1f, -0.4f, +0.7f, 0}};
			auto const base_velocity_b = v8motion{v4{-0.1f, +0.4f, -0.3f, 0}, v4{-0.6f, +0.8f, -0.2f, 0}};

			// Solve the same endpoint-local problem after applying one common rigid world transform.
			auto run = [&](m4x4 const& transformed_from_base)
			{
				auto body_a = MakeConstraintGpuBody(2.0f, transformed_from_base * base_o2w_a, v4{+0.2f, -0.1f, +0.05f, 0});
				auto body_b = MakeConstraintGpuBody(3.0f, transformed_from_base * base_o2w_b, v4{-0.1f, +0.15f, -0.05f, 0});
				body_a.VelocityWS(v8motion{transformed_from_base.rot * base_velocity_a.ang, transformed_from_base.rot * base_velocity_a.lin});
				body_b.VelocityWS(v8motion{transformed_from_base.rot * base_velocity_b.ang, transformed_from_base.rot * base_velocity_b.lin});

				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), m4x4::Transform(v4::ZAxis(), +0.17f, v4{+0.3f, -0.2f, +0.1f, 1})};
				desc.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), m4x4::Transform(v4::YAxis(), -0.11f, v4{-0.2f, +0.25f, -0.15f, 1})};
				for (int axis = 0; axis != 3; ++axis)
				{
					desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
					desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				}
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 6;
				config.m_position_iterations = 4;
				config.m_warm_start_factor = 0.0f;

				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 120.0f, bodies, upload.m_endpoints, upload.m_descriptors});
				return bodies;
			};

			auto const baseline = run(m4x4::Identity());
			auto const transforms = std::array{
				m4x4::Translation(+1000.0f, -2000.0f, +3000.0f),
				m4x4::Transform(Normalise(v4{0.3f, -0.5f, 0.7f, 0}), 0.63f, v4{+7.0f, -11.0f, +13.0f, 1}),
			};

			// Momentum is translation-invariant about each centre of mass and rotates covariantly with the complete scene.
			for (auto const& transformed_from_base : transforms)
			{
				auto const transformed = run(transformed_from_base);
				for (int body_idx = 0; body_idx != isize(transformed); ++body_idx)
				{
					auto const expected_o2w = transformed_from_base * baseline[body_idx].o2w;
					auto const expected_momentum_ang = transformed_from_base.rot * baseline[body_idx].momentum_ang;
					auto const expected_momentum_lin = transformed_from_base.rot * baseline[body_idx].momentum_lin;
					PR_EXPECT(FEqlAbsolute(transformed[body_idx].o2w, expected_o2w, 2.0e-4f));
					PR_EXPECT(FEqlAbsolute(transformed[body_idx].momentum_ang, expected_momentum_ang, 2.0e-3f));
					PR_EXPECT(FEqlAbsolute(transformed[body_idx].momentum_lin, expected_momentum_lin, 2.0e-3f));
				}
			}
		}

		// Match split position correction for a hard world lock while preserving physical momentum and an off-origin CoM.
		PRUnitTestMethod(PositionSolveMatchesCpuReference, Quick)
		{
			auto body = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.12f, v4{0.3f, -0.2f, 0.1f, 1}),
				v4{0.2f, -0.1f, 0.15f, 0});
			body.MomentumWS(v8force{v4{0.2f, -0.1f, 0.3f, 0}, v4{0.4f, 0.5f, -0.2f, 0}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame{BodyRef::Rigid(body), m4x4::Translation(0.25f, -0.15f, 0.1f)};
			desc.m_frame_b = BodyFrame{BodyRef::World(), m4x4::Translation(0.7f, -0.05f, 0.2f)};
			for (int axis = 0; axis != 3; ++axis)
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const upload = PackGpuConstraints(constraints, remap);
			auto gpu_bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto const momentum_before = body.MomentumWS();
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 0;
			config.m_position_iterations = 4;
			config.m_warm_start_factor = 0.0f;

			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, gpu_bodies, upload.m_endpoints, upload.m_descriptors});
			auto cpu_solver = CpuConstraintSolver{};
			cpu_solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, config);

			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].o2w, body.O2W(), 2.0e-3f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_ang, momentum_before.ang, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(gpu_bodies[0].momentum_lin, momentum_before.lin, 1.0e-6f));
		}

		// Disabled and enabled all-free stable slots produce no block work or body mutation.
		PRUnitTestMethod(DisabledAndFreeSlotsAreNoOp, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4{0.1f, 0.2f, 0.3f, 0}, v4{1, 2, 3, 0}});
			auto before = PackDynamics(body, 0);
			auto bodies = std::vector<GpuRigidBody>{before};
			auto endpoints = std::vector<GpuConstraintEndpoint>(2);
			auto descriptors = std::vector<GpuD6ConstraintDesc>(2);
			endpoints[0].body_idx_a = 0;
			endpoints[0].body_idx_b = -1;
			endpoints[0].flags = GpuConstraintEndpointFlags_Enabled;

			auto runner = ConstraintInteropRunner{};
			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = endpoints,
				.m_descriptors = descriptors,
			});

			PR_EXPECT(runner.Blocks()[0].velocity_mask == 0u);
			PR_EXPECT(runner.Blocks()[1].velocity_mask == 0u);
			PR_EXPECT(FEql(bodies[0].momentum_ang, before.momentum_ang));
			PR_EXPECT(FEql(bodies[0].momentum_lin, before.momentum_lin));
			PR_EXPECT(FEql(bodies[0].o2w, before.o2w));
		}

		// ResetWarmStart invalidates retained impulses even when the row's active limit state is unchanged.
		PRUnitTestMethod(ResetWarmStartClearsRetainedRows, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{2, 0, 0, 0}});
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 1;
			config.m_position_iterations = 0;
			auto runner = ConstraintInteropRunner{config};
			auto buffers = ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Run(buffers);
			PR_EXPECT(std::abs(runner.Rows()[0].bounds.z) > 1.0e-5f);

			auto reset_endpoints = upload.m_endpoints;
			reset_endpoints[0].flags |= GpuConstraintEndpointFlags_ResetWarmStart;
			buffers.m_endpoints = reset_endpoints;
			runner.Load(buffers);
			runner.CompileConstraints();
			PR_EXPECT(runner.Rows()[0].bounds.z == 0.0f);
		}

		// Leave coupled runtime storage exclusively owned by its compiler even when a descriptor change requests warm-start reset.
		PRUnitTestMethod(RigidCompilerNeverMutatesCoupledSlots, Quick)
		{
			auto body = MakeConstraintGpuBody();
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto runner = ConstraintInteropRunner{};
			auto buffers = ConstraintRunnerBuffers{1.0f / 60.0f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Load(buffers);
			runner.CompileConstraints();
			auto const block_before = runner.Blocks()[0];
			auto rows_before = std::array<GpuConstraintRow, GpuConstraintRowsPerBlock>{};
			std::copy_n(runner.Rows().begin(), rows_before.size(), rows_before.begin());

			auto coupled_endpoints = upload.m_endpoints;
			coupled_endpoints[0].flags |= GpuConstraintEndpointFlags_Coupled | GpuConstraintEndpointFlags_ResetWarmStart;
			buffers.m_endpoints = coupled_endpoints;
			runner.Load(buffers);
			runner.CompileConstraints();

			PR_EXPECT(std::memcmp(&runner.Blocks()[0], &block_before, sizeof(block_before)) == 0);
			PR_EXPECT(std::memcmp(runner.Rows().data(), rows_before.data(), sizeof(rows_before)) == 0);
		}

		// Convert force caps to timestep-scaled impulse bounds and reject invalid timesteps before mutation.
		PRUnitTestMethod(TimestepAndForceBoundsRemainFinite, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{100, 0, 0, 0}});
			auto constraints = ConstraintSet{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(2.0f);
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			auto buffers = ConstraintRunnerBuffers{0.25f, bodies, upload.m_endpoints, upload.m_descriptors};
			runner.Run(buffers);

			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].bounds.x, -0.5f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(runner.Rows()[0].bounds.y, +0.5f, 1.0e-6f));
			PR_EXPECT(std::abs(runner.Rows()[0].bounds.z) <= 0.5f);
			buffers.m_dt = 0.0f;
			PR_THROWS(runner.Run(buffers), std::invalid_argument);
		}

		// Convert canonical linear impulses to force, latch the first overload, and disable later work in the same submitted frame.
		PRUnitTestMethod(BreakThresholdLatchesCanonicalLoad, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(upload.m_breakable_count == 1);
			PR_EXPECT(AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(state.generation == upload.m_endpoints[0].generation);
			PR_EXPECT(state.substep_index == 0);
			PR_EXPECT(state.peak_force > desc.m_break_force);
			PR_EXPECT(state.peak_torque == 0.0f);
			PR_EXPECT(AllSet(runner.Blocks()[0].flags, ConstraintBlockFlags_Broken));
			PR_EXPECT(runner.Blocks()[0].velocity_mask == 0u);
		}

		// Distinguish torque from force and report the canonical angular load that crossed the configured threshold.
		PRUnitTestMethod(BreakTorqueUsesAngularImpulse, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4{5.0f, 0.0f, 0.0f, 0.0f}, v4::Zero()});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_angular[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_torque = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(state.peak_force == 0.0f);
			PR_EXPECT(state.peak_torque > desc.m_break_torque);
		}

		// Respect per-axis force clamps before overload testing, making an aggregate threshold above the clamp unreachable for one active axis.
		PRUnitTestMethod(BreakForceObservesAxisForceLimit, Quick)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{100.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(2.0f);
			desc.m_break_force = 2.1f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto config = CpuConstraintSolverConfig{};
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};

			runner.Run(ConstraintRunnerBuffers{
				.m_dt = 0.25f,
				.m_bodies = bodies,
				.m_endpoints = upload.m_endpoints,
				.m_descriptors = upload.m_descriptors,
			});

			auto const& state = runner.BreakStates()[0];
			PR_EXPECT(!AllSet(state.flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(Abs(state.peak_force - desc.m_linear[0].m_max_force) < 1.0e-5f);
		}

		// Reordering independent stable slots does not change their disjoint body results.
		PRUnitTestMethod(IndependentInsertionPermutationsAgree, Quick)
		{
			auto initial_bodies = std::vector<GpuRigidBody>(4);
			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				initial_bodies[body_idx].o2w = m4x4::Identity();
				initial_bodies[body_idx].inertia_inv_diagonal = v4{1, 1, 1, 0};
				initial_bodies[body_idx].os_com_and_invmass = v4{0, 0, 0, 1};
				initial_bodies[body_idx].momentum_lin = v4{static_cast<float>(body_idx + 1), 0, 0, 0};
				initial_bodies[body_idx].sleep.island_id = -1;
			}
			auto endpoint_a = GpuConstraintEndpoint{.body_idx_a = 0, .body_idx_b = 1, .flags = GpuConstraintEndpointFlags_Enabled};
			auto endpoint_b = GpuConstraintEndpoint{.body_idx_a = 2, .body_idx_b = 3, .flags = GpuConstraintEndpointFlags_Enabled};
			auto descriptor = GpuD6ConstraintDesc{};
			descriptor.frame_a.rotation = float4{0, 0, 0, 1};
			descriptor.frame_b.rotation = float4{0, 0, 0, 1};
			descriptor.axes[0].mode = GpuConstraintAxisMode_Locked;
			descriptor.axes[0].max_force = 1000.0f;
			auto descriptors = std::vector<GpuD6ConstraintDesc>{descriptor, descriptor};
			auto endpoints_ab = std::vector<GpuConstraintEndpoint>{endpoint_a, endpoint_b};
			auto endpoints_ba = std::vector<GpuConstraintEndpoint>{endpoint_b, endpoint_a};
			auto bodies_ab = initial_bodies;
			auto bodies_ba = initial_bodies;
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner_ab = ConstraintInteropRunner{config};
			auto runner_ba = ConstraintInteropRunner{config};
			runner_ab.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies_ab, endpoints_ab, descriptors});
			runner_ba.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies_ba, endpoints_ba, descriptors});

			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(bodies_ab[body_idx].momentum_ang, bodies_ba[body_idx].momentum_ang, 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(bodies_ab[body_idx].momentum_lin, bodies_ba[body_idx].momentum_lin, 1.0e-6f));
			}
		}

		// A degree-33 dynamic hub exhausts the 32-colour mask and remains finite under coherent serial fallback.
		PRUnitTestMethod(HighDegreeHubUsesSerialFallback, Quick)
		{
			constexpr int LeafCount = 33;
			auto rigid_bodies = std::vector<RigidBody>{};
			rigid_bodies.reserve(LeafCount + 1);
			for (int body_idx = 0; body_idx != LeafCount + 1; ++body_idx)
				rigid_bodies.push_back(MakeConstraintGpuBody(1.0f, m4x4::Translation(0.01f * body_idx, 0, 0)));

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != LeafCount + 1; ++body_idx)
				bodies.push_back(PackDynamics(rigid_bodies[body_idx], body_idx));
			auto endpoints = std::vector<GpuConstraintEndpoint>(LeafCount);
			auto descriptors = std::vector<GpuD6ConstraintDesc>(LeafCount);
			for (int slot_idx = 0; slot_idx != LeafCount; ++slot_idx)
			{
				endpoints[slot_idx].body_idx_a = 0;
				endpoints[slot_idx].body_idx_b = slot_idx + 1;
				endpoints[slot_idx].flags = GpuConstraintEndpointFlags_Enabled;
				descriptors[slot_idx].frame_a.rotation = float4{0, 0, 0, 1};
				descriptors[slot_idx].frame_b.rotation = float4{0, 0, 0, 1};
				descriptors[slot_idx].axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptors[slot_idx].axes[0].max_force = 1000.0f;
			}
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 2;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			auto runner = ConstraintInteropRunner{config};
			runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, endpoints, descriptors});

			PR_EXPECT(runner.ColourOverflow());
			for (auto const& body : bodies)
				PR_EXPECT(ConstraintGpuBodyFinite(body));
		}

		// Preserve quiet-time accumulation for awake loaded bodies while still invalidating persisted sleeping state after an impulse.
		PRUnitTestMethod(ConstraintImpulsesRespectSleepState, Quick)
		{
			auto run = [](bool sleeping)
			{
				auto rigid_body = MakeConstraintGpuBody();
				rigid_body.VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});
				auto bodies = std::vector<GpuRigidBody>{PackDynamics(rigid_body, 0)};
				bodies[0].sleep.timer_s = 0.4f;
				bodies[0].sleep.island_id = -1;
				bodies[0].sleep.generation = 7;
				if (sleeping)
					bodies[0].state_flags |= ERigidBodyStateFlags_Sleeping;

				auto endpoints = std::vector<GpuConstraintEndpoint>(1);
				endpoints[0].body_idx_a = 0;
				endpoints[0].body_idx_b = -1;
				endpoints[0].flags = GpuConstraintEndpointFlags_Enabled;
				auto descriptors = std::vector<GpuD6ConstraintDesc>(1);
				descriptors[0].frame_a.rotation = float4{0, 0, 0, 1};
				descriptors[0].frame_b.rotation = float4{0, 0, 0, 1};
				descriptors[0].axes[0].mode = GpuConstraintAxisMode_Locked;
				descriptors[0].axes[0].max_force = 1000.0f;
				auto config = CpuConstraintSolverConfig{};
				config.m_velocity_iterations = 1;
				config.m_position_iterations = 0;
				config.m_warm_start_factor = 0.0f;
				auto runner = ConstraintInteropRunner{config};
				runner.Run(ConstraintRunnerBuffers{1.0f / 60.0f, bodies, endpoints, descriptors});
				return bodies[0];
			};

			auto const awake = run(false);
			PR_EXPECT(awake.sleep.generation == 7);
			PR_EXPECT(awake.sleep.timer_s == 0.4f);

			auto const sleeping = run(true);
			PR_EXPECT(sleeping.sleep.generation == 8);
			PR_EXPECT(!AllSet(sleeping.state_flags, ERigidBodyStateFlags_Sleeping));
			PR_EXPECT(sleeping.sleep.timer_s == 0.0f);
		}

		// Match the HLSL-as-C++ reference through the production D3D12 compiler, pseudo-position, and velocity passes.
		PRUnitTestMethod(HardwarePipelineMatchesInterop, Extended)
		{
			auto body_a = MakeConstraintGpuBody(
				2.0f,
				m4x4::Transform(v4::YAxis(), 0.17f, v4{0.35f, -0.15f, 0.2f, 1}),
				v4{0.2f, -0.1f, 0.15f, 0});
			auto body_b = MakeConstraintGpuBody(
				3.0f,
				m4x4::Transform(v4::XAxis(), -0.11f, v4{-0.1f, 0.25f, -0.05f, 1}),
				v4{-0.1f, 0.05f, -0.08f, 0});
			body_a.VelocityWS(v8motion{v4{0.2f, -0.3f, 0.1f, 0}, v4{0.8f, -0.4f, 0.5f, 0}});
			body_b.VelocityWS(v8motion{v4{-0.1f, 0.15f, -0.2f, 0}, v4{-0.3f, 0.6f, -0.2f, 0}});

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), m4x4::Translation(0.3f, -0.2f, 0.1f)};
			desc.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), m4x4::Translation(-0.15f, 0.25f, -0.05f)};
			for (int axis = 0; axis != 3; ++axis)
			{
				desc.m_linear[axis] = MakeConstraintGpuLockedAxis(5000.0f);
				desc.m_angular[axis] = MakeConstraintGpuLockedAxis(5000.0f);
			}

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto expected = std::vector<GpuRigidBody>{PackDynamics(body_a, 0), PackDynamics(body_b, 1)};
			auto actual = expected;
			auto config = EngineConfig{};
			config.solver_iterations = 2;
			config.push_out_iterations = 2;
			config.constraint_relaxation = 0.73f;
			config.constraint_position_relaxation = 0.61f;
			config.constraint_position_beta = 0.17f;
			config.constraint_max_position_speed = 1.3f;
			config.constraint_regularization = 3.0e-6f;
			config.constraint_warm_start_factor = 0.42f;

			auto reference = ConstraintInteropRunner{config};
			reference.Run(ConstraintRunnerBuffers{1.0f / 120.0f, expected, upload.m_endpoints, upload.m_descriptors});
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto const empty_stats = solver.Stats();
			PR_EXPECT(empty_stats.m_slot_capacity == 0);
			PR_EXPECT(empty_stats.m_body_capacity == 0);
			PR_EXPECT(empty_stats.m_break_capacity == 0);
			PR_EXPECT(empty_stats.m_dispatch_count == 0);
			PR_EXPECT(empty_stats.m_logical_bytes == 0);
			PR_EXPECT(empty_stats.m_allocated_feature_bytes == 0);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 120.0f, upload, actual);
			auto const first_stats = solver.Stats();
			PR_EXPECT(first_stats.m_slot_count == 1);
			PR_EXPECT(first_stats.m_active_count == 1);
			PR_EXPECT(first_stats.m_breakable_count == 0);
			PR_EXPECT(first_stats.m_slot_capacity >= 1);
			PR_EXPECT(first_stats.m_body_capacity >= 2);
			PR_EXPECT(first_stats.m_break_capacity == 0);
			PR_EXPECT(first_stats.m_dispatch_count > 0);
			PR_EXPECT(first_stats.m_logical_bytes > 0);
			PR_EXPECT(first_stats.m_allocated_feature_bytes >= first_stats.m_logical_bytes);

			// Report exact logical and retained-capacity costs from the published per-slot and per-body resource formula.
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow);
			auto const expected_logical_bytes =
				static_cast<size_t>(first_stats.m_slot_count) * slot_bytes +
				sizeof(uint32_t) +
				actual.size() * sizeof(GpuConstraintPseudoVelocity);
			auto const expected_allocated_bytes =
				static_cast<size_t>(first_stats.m_slot_capacity) * slot_bytes +
				sizeof(uint32_t) +
				static_cast<size_t>(first_stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(first_stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(first_stats.m_logical_bytes == expected_logical_bytes);
			PR_EXPECT(first_stats.m_allocated_feature_bytes == expected_allocated_bytes);

			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(actual[body_idx].o2w, expected[body_idx].o2w, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_ang, expected[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_lin, expected[body_idx].momentum_lin, 2.0e-3f));
			}

			// A second frame makes the configured warm-start factor observable in both persistent runtimes.
			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				expected[body_idx].momentum_ang = float4{};
				expected[body_idx].momentum_lin = float4{};
				actual[body_idx].momentum_ang = float4{};
				actual[body_idx].momentum_lin = float4{};
			}
			reference.Run(ConstraintRunnerBuffers{1.0f / 120.0f, expected, upload.m_endpoints, upload.m_descriptors});
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 120.0f, upload, actual);
			for (int body_idx = 0; body_idx != isize(actual); ++body_idx)
			{
				PR_EXPECT(FEqlAbsolute(actual[body_idx].o2w, expected[body_idx].o2w, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_ang, expected[body_idx].momentum_ang, 2.0e-3f));
				PR_EXPECT(FEqlAbsolute(actual[body_idx].momentum_lin, expected[body_idx].momentum_lin, 2.0e-3f));
			}
		}

		// Read a production compute-shader overload latch back through the solver's single diagnostic transaction.
		PRUnitTestMethod(HardwareReportsBreakState, Extended)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			auto break_states = std::vector<GpuConstraintBreakState>(upload.m_endpoints.size());
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.0f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};

			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies, {}, {}, break_states);

			PR_EXPECT(AllSet(break_states[0].flags, GpuConstraintBreakFlags_Broken));
			PR_EXPECT(break_states[0].generation == upload.m_endpoints[0].generation);
			PR_EXPECT(break_states[0].substep_index == 0);
			PR_EXPECT(break_states[0].peak_force > desc.m_break_force);

			// Breakable slots add one stable-slot latch to both logical and retained resource accounting.
			auto const stats = solver.Stats();
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow);
			auto const expected_logical_bytes =
				slot_bytes +
				sizeof(uint32_t) +
				sizeof(GpuConstraintPseudoVelocity) +
				sizeof(GpuConstraintBreakState);
			auto const expected_allocated_bytes =
				static_cast<size_t>(stats.m_slot_capacity) * slot_bytes +
				sizeof(uint32_t) +
				static_cast<size_t>(stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(stats.m_breakable_count == 1);
			PR_EXPECT(stats.m_logical_bytes == expected_logical_bytes);
			PR_EXPECT(stats.m_allocated_feature_bytes == expected_allocated_bytes);
		}

		// Measure end-to-end GPU solver scaling through at least one hundred thousand active scalar rows.
		PRUnitTestMethod(HardwareHundredThousandRowScaling, Extended)
		{
			constexpr auto SmallSlotCount = 8334;
			constexpr auto LargeSlotCount = 16667;
			constexpr auto SampleCount = 5;
			auto make_upload = [](int slot_count)
			{
				auto upload = GpuConstraintUpload{};
				upload.m_endpoints.resize(slot_count);
				upload.m_endpoint_identities.resize(slot_count);
				upload.m_descriptors.resize(slot_count);
				upload.m_rigid_active_count = slot_count;
				for (int slot_idx = 0; slot_idx != slot_count; ++slot_idx)
				{
					auto& endpoint = upload.m_endpoints[slot_idx];
					endpoint.body_idx_a = slot_idx;
					endpoint.body_idx_b = -1;
					endpoint.flags = GpuConstraintEndpointFlags_Enabled;
					endpoint.generation = 1;

					auto& descriptor = upload.m_descriptors[slot_idx];
					descriptor.frame_a.rotation = float4{0, 0, 0, 1};
					descriptor.frame_b.rotation = float4{0, 0, 0, 1};
					for (int axis_idx = 0; axis_idx != GpuConstraintRowsPerBlock; ++axis_idx)
					{
						descriptor.axes[axis_idx].mode = GpuConstraintAxisMode_Locked;
						descriptor.axes[axis_idx].max_force = 1000.0f;
					}
				}
				return upload;
			};
			auto make_bodies = [](int body_count)
			{
				auto rigid_body = MakeConstraintGpuBody();
				rigid_body.VelocityWS(v8motion{v4{0.1f, -0.2f, 0.3f, 0}, v4{0.4f, -0.5f, 0.6f, 0}});
				auto packed = PackDynamics(rigid_body, 0);
				return std::vector<GpuRigidBody>(body_count, packed);
			};
			auto const small_upload = make_upload(SmallSlotCount);
			auto const large_upload = make_upload(LargeSlotCount);
			auto config = EngineConfig{};
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_warm_start_factor = 0.0f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};

			// Allocate the maximum resources before timing so the comparison measures steady-state work rather than one-time growth.
			auto warmup_bodies = make_bodies(LargeSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, large_upload, warmup_bodies);
			auto const warmup_stats = solver.Stats();

			// Load the smaller working set against retained maximum capacity so logical and allocated accounting must diverge correctly.
			auto small_probe_bodies = make_bodies(SmallSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, small_upload, small_probe_bodies);
			auto const small_stats = solver.Stats();
			auto const slot_bytes =
				sizeof(GpuConstraintEndpoint) +
				sizeof(GpuD6ConstraintDesc) +
				sizeof(GpuConstraintBlock) +
				GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow);
			auto const expected_small_logical_bytes =
				static_cast<size_t>(SmallSlotCount) * slot_bytes +
				sizeof(uint32_t) +
				static_cast<size_t>(SmallSlotCount) * sizeof(GpuConstraintPseudoVelocity);
			PR_EXPECT(small_stats.m_logical_bytes == expected_small_logical_bytes);
			PR_EXPECT(small_stats.m_allocated_feature_bytes == warmup_stats.m_allocated_feature_bytes);
			PR_EXPECT(small_stats.m_logical_bytes < small_stats.m_allocated_feature_bytes);

			// Restore the larger workload immediately before timing so both measured sizes start from an active, saturated GPU.
			warmup_bodies = make_bodies(LargeSlotCount);
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, large_upload, warmup_bodies);
			auto measure = [&](GpuConstraintUpload const& upload)
			{
				auto bodies = make_bodies(isize(upload.m_endpoints));
				auto const begin = std::chrono::steady_clock::now();
				solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies);
				auto const elapsed = std::chrono::steady_clock::now() - begin;
				for (auto const& body : bodies)
					PR_EXPECT(ConstraintGpuBodyFinite(body));
				return std::chrono::duration<double, std::milli>(elapsed).count();
			};

			// Interleaved medians reject transient GPU clock and queue noise without concealing persistent superlinear work.
			auto small_samples = std::array<double, SampleCount>{};
			auto large_samples = std::array<double, SampleCount>{};
			for (int sample = 0; sample != SampleCount; ++sample)
			{
				small_samples[sample] = measure(small_upload);
				large_samples[sample] = measure(large_upload);
			}
			std::ranges::sort(small_samples);
			std::ranges::sort(large_samples);
			auto const small_ms = small_samples[SampleCount / 2];
			auto const large_ms = large_samples[SampleCount / 2];
			auto const small_rows = SmallSlotCount * GpuConstraintRowsPerBlock;
			auto const large_rows = LargeSlotCount * GpuConstraintRowsPerBlock;
			pr::unittests::TestFramework::out() << std::format(
				"  [benchmark] {} constraint rows: {:.3f} ms; {} rows: {:.3f} ms ({:.2f}x time for {:.2f}x rows)\n",
				small_rows,
				small_ms,
				large_rows,
				large_ms,
				large_ms / small_ms,
				static_cast<double>(large_rows) / small_rows);
			PR_EXPECT(large_rows >= 100000);
			PR_EXPECT(large_ms / small_ms < 2.2);

			// Repeated smaller submissions must reuse the maximum allocation, whose accounting follows the capacity formula exactly.
			auto const final_stats = solver.Stats();
			auto const expected_allocated_bytes =
				static_cast<size_t>(final_stats.m_slot_capacity) * slot_bytes +
				sizeof(uint32_t) +
				static_cast<size_t>(final_stats.m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(final_stats.m_break_capacity) * sizeof(GpuConstraintBreakState);
			PR_EXPECT(final_stats.m_slot_capacity == warmup_stats.m_slot_capacity);
			PR_EXPECT(final_stats.m_body_capacity == warmup_stats.m_body_capacity);
			PR_EXPECT(final_stats.m_allocated_feature_bytes == warmup_stats.m_allocated_feature_bytes);
			PR_EXPECT(final_stats.m_allocated_feature_bytes == expected_allocated_bytes);
		}

		// Preserve coherent stable-slot execution when a real GPU hub requires one hundred edge colours.
		PRUnitTestMethod(HardwareDegreeHundredUsesSerialFallback, Extended)
		{
			constexpr int LeafCount = 100;
			auto rigid_bodies = std::vector<RigidBody>{};
			rigid_bodies.reserve(LeafCount + 1);
			for (int body_idx = 0; body_idx != LeafCount + 1; ++body_idx)
				rigid_bodies.push_back(MakeConstraintGpuBody(1.0f, m4x4::Translation(0.01f * body_idx, 0, 0)));
			rigid_bodies[0].VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});

			auto body_ptrs = std::vector<RigidBody*>{};
			body_ptrs.reserve(rigid_bodies.size());
			for (auto& body : rigid_bodies)
				body_ptrs.push_back(&body);

			auto constraints = ConstraintSet{};
			for (int leaf_idx = 0; leaf_idx != LeafCount; ++leaf_idx)
			{
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(rigid_bodies[0]);
				desc.m_frame_b.m_body = BodyRef::Rigid(rigid_bodies[leaf_idx + 1]);
				desc.m_linear[0] = MakeConstraintGpuLockedAxis();
				constraints.Add(desc);
			}

			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto bodies = std::vector<GpuRigidBody>{};
			bodies.reserve(rigid_bodies.size());
			for (int body_idx = 0; body_idx != isize(rigid_bodies); ++body_idx)
				bodies.push_back(PackDynamics(rigid_bodies[body_idx], body_idx));
			auto blocks = std::vector<GpuConstraintBlock>(LeafCount);
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 0.0f;

			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, upload, bodies, blocks);

			for (auto const& block : blocks)
				PR_EXPECT(block.colour == MaxColours);
			for (auto const& body : bodies)
				PR_EXPECT(ConstraintGpuBodyFinite(body));
		}

		// Retain ordinary warm starts while rejecting identical stable slots owned by a different constraint set.
		PRUnitTestMethod(HardwareWarmStartTracksConstraintSource, Extended)
		{
			auto body = MakeConstraintGpuBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{1, 0, 0, 0}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints_a = ConstraintSet{};
			constraints_a.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 0.85f;
			auto solver = GpuConstraintSolver{ConstraintTestGpu(), config};
			auto rows = std::vector<GpuConstraintRow>(GpuConstraintRowsPerBlock);

			auto first = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_a, BodyRemap(body_ptrs)), first, {}, rows);
			PR_EXPECT(std::abs(rows[0].bounds.z) > 1.0e-4f);

			// With iterations disabled, any second-frame momentum is solely the retained warm start.
			config.solver_iterations = 0;
			body.VelocityWS(v8motion{});
			auto retained = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_a, BodyRemap(body_ptrs)), retained);
			PR_EXPECT(std::abs(retained[0].momentum_lin.x) > 1.0e-4f);

			auto constraints_b = ConstraintSet{};
			constraints_b.Add(desc);
			auto replaced = std::vector<GpuRigidBody>{PackDynamics(body, 0)};
			solver.Solve(ConstraintTestGpu().m_job, 1.0f / 60.0f, PackGpuConstraints(constraints_b, BodyRemap(body_ptrs)), replaced);
			PR_EXPECT(std::abs(replaced[0].momentum_lin.x) < 1.0e-6f);
		}

		// An empty constraint overlay must leave ordinary rigid-body output and the established frame transaction bit-identical.
		PRUnitTestMethod(EmptyConstraintSetPreservesOrdinaryStep, Extended)
		{
			auto shape = collision::ShapeBox{v4{0.4f, 0.6f, 0.8f, 0}};
			auto initial_o2w = m4x4::Transform(Normalise(v4{0.2f, 0.7f, -0.3f, 0}), 0.37f, v4{+1.0f, -2.0f, +3.0f, 1});
			auto overlay_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			auto baseline_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			auto const initial_velocity = v8motion{v4{+1.2f, -0.8f, +0.4f, 0}, v4{+0.3f, -0.2f, +0.5f, 0}};
			overlay_body.VelocityWS(initial_velocity);
			baseline_body.VelocityWS(initial_velocity);
			auto overlay_bodies = std::array<RigidBody*, 1>{&overlay_body};
			auto baseline_bodies = std::array<RigidBody*, 1>{&baseline_body};
			auto constraints = ConstraintSet{};
			auto engine = Engine{
				EngineConfig{},
				nullptr,
				SharedTestGpu().m_gpu.device(),
				SharedTestGpu().m_gpu.queue(),
			};

			// Warm the ordinary one-body path without touching optional lanes so first-frame setup cannot masquerade as overlay cost.
			auto warmup_body = RigidBody{&shape, initial_o2w, Inertia::Box(shape.m_radius, 2.5f)};
			warmup_body.VelocityWS(initial_velocity);
			auto warmup_bodies = std::array<RigidBody*, 1>{&warmup_body};
			engine.Step(Engine::StepInput{
				.m_bodies = warmup_bodies,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});

			// Submit the optional empty overlay before any feature allocation can weaken the zero-cost assertion.
			engine.Step(Engine::StepInput{
				.m_bodies = overlay_bodies,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});
			auto const overlay_stats = engine.LastFeatureStats();
			auto const overlay_profile = engine.LastStepProfile();

			// Repeat the same ordinary path without a constraint pointer and compare the complete published dynamic state.
			engine.Step(Engine::StepInput{
				.m_bodies = baseline_bodies,
				.m_elapsed_seconds = 1.0f / 120.0f,
			});
			auto const baseline_profile = engine.LastStepProfile();
			auto const overlay_o2w = overlay_body.O2W();
			auto const baseline_o2w = baseline_body.O2W();
			auto const overlay_momentum = overlay_body.MomentumWS();
			auto const baseline_momentum = baseline_body.MomentumWS();
			PR_EXPECT(std::memcmp(&overlay_o2w, &baseline_o2w, sizeof(m4x4)) == 0);
			PR_EXPECT(std::memcmp(&overlay_momentum, &baseline_momentum, sizeof(v8force)) == 0);
			PR_EXPECT(overlay_body.Sleeping() == baseline_body.Sleeping());

			// Optional lanes retain no feature storage or work, and the empty overlay adds no submission, wait, or readback.
			PR_EXPECT(overlay_stats.m_constraints.m_declared_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_active_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_constraints.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_articulations.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_dispatch_count == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_logical_bytes == 0);
			PR_EXPECT(overlay_stats.m_coupled.m_resources.m_allocated_bytes == 0);
			PR_EXPECT(overlay_profile.m_submission_count == baseline_profile.m_submission_count);
			PR_EXPECT(overlay_profile.m_wait_count == baseline_profile.m_wait_count);
			PR_EXPECT(overlay_profile.m_readback_copy_count == baseline_profile.m_readback_copy_count);
		}

		// Exercise the public constrained Engine step and prove a hard world lock corrects drift without physical momentum injection.
		PRUnitTestMethod(EngineStepAppliesWorldLock, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Translation(0.5f, 0, 0), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto const momentum_before = body.MomentumWS();
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			engine.Step(1.0f / 60.0f, bodies, constraints);

			PR_EXPECT(body.O2W().pos.x < 0.5f);
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, momentum_before.ang, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().lin, momentum_before.lin, 1.0e-6f));
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_constraints.m_declared_count == 1);
			PR_EXPECT(stats.m_constraints.m_active_count == 1);
			PR_EXPECT(stats.m_constraints.m_breakable_count == 0);
			PR_EXPECT(stats.m_constraints.m_slot_capacity >= 1);
			PR_EXPECT(stats.m_constraints.m_body_capacity >= 1);
			PR_EXPECT(stats.m_constraints.m_resources.m_dispatch_count > 0);
			PR_EXPECT(stats.m_constraints.m_resources.m_logical_bytes > 0);
			PR_EXPECT(stats.m_constraints.m_resources.m_allocated_bytes >= stats.m_constraints.m_resources.m_logical_bytes);
			PR_EXPECT(stats.m_articulations.m_articulation_count == 0);
			PR_EXPECT(stats.m_articulations.m_resources.m_dispatch_count == 0);
			PR_EXPECT(stats.m_coupled.m_constraint_count == 0);
			PR_EXPECT(stats.m_coupled.m_resources.m_dispatch_count == 0);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 1);
			PR_EXPECT(stats.m_frame_output.m_readback_bytes > 0);
			PR_EXPECT(stats.m_failure.Succeeded());

			// Removing the live descriptor leaves one reusable stable slot, but declared diagnostics report only caller-visible constraints.
			constraints.Remove(handle);
			engine.Step(1.0f / 60.0f, bodies, constraints);
			auto const& removed_stats = engine.LastFeatureStats();
			PR_EXPECT(constraints.CapacitySlots() == 1);
			PR_EXPECT(removed_stats.m_constraints.m_declared_count == 0);
			PR_EXPECT(removed_stats.m_constraints.m_active_count == 0);
		}

		// Empty simulation input still validates live constraint endpoints while allowing disabled descriptors to remain declared.
		PRUnitTestMethod(EmptyFrameValidatesLiveConstraints, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);

			PR_THROWS(engine.Step(Engine::StepInput{
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			}), std::invalid_argument);

			constraints.SetEnabled(handle, false);
			engine.Step(Engine::StepInput{
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_constraints.m_declared_count == 1);
			PR_EXPECT(stats.m_constraints.m_active_count == 0);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 0);
		}

		// Detect an early-substep overload, publish one edge event through the sole frame readback, and require explicit repair.
		PRUnitTestMethod(EngineBreaksAndRepairsConstraint, Quick)
		{
			auto shape = collision::ShapeSphere{0.25f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis(1000.0f);
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto events = std::vector<ConstraintBreakEvent>{};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.ConstraintsBroken += [&](auto&, auto broken)
			{
				events.insert(events.end(), broken.begin(), broken.end());
			};

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 1);
			PR_EXPECT(events[0].m_constraint == handle);
			PR_EXPECT(events[0].m_substep_index == 0);
			PR_EXPECT(events[0].m_force > desc.m_break_force);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);

			// Routine descriptor updates preserve the broken state and cannot emit duplicate events.
			auto updated = constraints.Get(handle);
			updated.m_linear[0].m_target_position = 0.1f;
			constraints.Update(handle, updated);
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			engine.Step(1.0f / 60.0f, body_ptrs, constraints);
			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 1);

			// Explicit repair reactivates the same stable handle and permits a later overload to generate a new edge event.
			constraints.Repair(handle);
			body.VelocityWS(v8motion{v4::Zero(), v4{5.0f, 0.0f, 0.0f, 0.0f}});
			engine.Step(1.0f / 60.0f, body_ptrs, constraints);
			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 2);
			PR_EXPECT(events[1].m_constraint == handle);

			// Repeated enable calls do not repair; a real disable-to-enable transition intentionally does.
			constraints.SetEnabled(handle, true);
			PR_EXPECT(constraints.IsBroken(handle));
			constraints.SetEnabled(handle, false);
			PR_EXPECT(constraints.IsBroken(handle));
			constraints.SetEnabled(handle, true);
			PR_EXPECT(!constraints.IsBroken(handle));
		}

		// Run articulation-only coupled work through shared stable-slot storage and the Engine's one-submission substep schedule.
		PRUnitTestMethod(CoupledConstraintsUseSharedGpuStorage, Quick)
		{
			auto [articulation, child] = MakeConstraintGpuArticulation();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, child);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0] = MakeConstraintGpuLockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto const upload = PackGpuConstraints(constraints, BodyRemap({}, articulation_ptrs));

			auto solver = GpuConstraintSolver{ConstraintTestGpu(), EngineConfig{}};
			PR_EXPECT(!solver.Upload(ConstraintTestGpu().m_job, upload));
			ConstraintTestGpu().m_job.Run();

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			articulation.Sleep();
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			auto const& profile = engine.LastStepProfile();
			PR_EXPECT(!articulation.Sleeping());
			PR_EXPECT(profile.m_submission_count == 1);
			PR_EXPECT(profile.m_wait_count == 1);
			PR_EXPECT(profile.m_readback_copy_count == 1);
			PR_EXPECT(std::ranges::all_of(articulation.JointPosition(child), [](float value) { return std::isfinite(value); }));
			PR_EXPECT(std::ranges::all_of(articulation.JointVelocity(child), [](float value) { return std::isfinite(value); }));
			PR_EXPECT(IsFinite(articulation.LinkToWorld(child).pos));
		}

		// Suppress connected-body collisions through the filtered broadphase without activating solver rows.
		PRUnitTestMethod(ConnectedBodiesCanSuppressCollision, Quick)
		{
			auto run = [](bool collide_connected)
			{
				auto shape = collision::ShapeSphere{0.5f};
				auto body_a = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
				auto body_b = RigidBody{&shape, m4x4::Translation(0.75f, 0, 0), Inertia::Sphere(shape.m_radius, 1.0f)};
				auto bodies = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
				desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
				desc.m_collide_connected = collide_connected;
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto collision_count = 0;
				auto& engine = SharedEngine();
				ResetEngineForNextTest(engine);
				engine.Collisions += [&](auto&, auto contacts)
				{
					collision_count += isize(contacts);
				};

				engine.Step(1.0f / 60.0f, bodies, constraints);
				return collision_count;
			};

			PR_EXPECT(run(false) == 0);
			PR_EXPECT(run(true) != 0);
		}
	};
}
#endif
