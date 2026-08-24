//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"
#include "src/constraint/constraint_solver.h"
#include "src/unittests/constraint_oracle.h"

namespace pr::physics::tests
{
	namespace
	{
		using namespace constraint_oracle;

		// A deterministic generator keeps randomized production-oracle comparisons reproducible.
		struct RandomSource
		{
			uint64_t m_state;

			// Return a deterministic float in [-1,+1).
			float NextSigned()
			{
				m_state ^= m_state << 13;
				m_state ^= m_state >> 7;
				m_state ^= m_state << 17;
				auto const unit = static_cast<double>(m_state >> 11) * (1.0 / 9007199254740992.0);
				return static_cast<float>(2.0 * unit - 1.0);
			}
		};

		// Construct a finite dynamic sphere with an optional world transform and CoM offset.
		RigidBody MakeBody(float mass = 1.0f, m4x4 const& o2w = m4x4::Identity(), v4 com_os = v4::Zero())
		{
			auto body = RigidBody{};
			body.SetMassProperties(Inertia::Sphere(1.0f, mass, com_os), com_os);
			body.O2W(o2w);
			return body;
		}

		// Return the current scalar velocity represented by a compiled row.
		float RowVelocity(CompiledConstraintRow const& row, CompiledConstraintBlock const& block, BodyRemap const& remap)
		{
			auto velocity = 0.0f;
			if (block.m_body_index_a >= 0)
				velocity += Dot(row.m_jacobian_a, remap.Body(block.m_body_index_a).VelocityWS());
			if (block.m_body_index_b >= 0)
				velocity += Dot(row.m_jacobian_b, remap.Body(block.m_body_index_b).VelocityWS());
			return velocity;
		}

		// Return one dense response entry without relying on the production solver implementation.
		double Response(CompiledConstraintRow const& lhs, CompiledConstraintRow const& rhs, CompiledConstraintBlock const& block, BodyRemap const& remap)
		{
			auto response = 0.0;
			if (block.m_body_index_a >= 0)
				response += Dot(lhs.m_jacobian_a, remap.Body(block.m_body_index_a).InertiaInvWS() * rhs.m_jacobian_a);
			if (block.m_body_index_b >= 0)
				response += Dot(lhs.m_jacobian_b, remap.Body(block.m_body_index_b).InertiaInvWS() * rhs.m_jacobian_b);
			return response;
		}

		// Require two spatial motions to agree component by component.
		void ExpectNear(v8motion const& actual, v8motion const& expected, float tolerance = 1.0e-5f)
		{
			PR_EXPECT(FEqlAbsolute(actual.ang, expected.ang, tolerance));
			PR_EXPECT(FEqlAbsolute(actual.lin, expected.lin, tolerance));
		}

		// Return a hard scalar axis with an optional finite force cap.
		ConstraintAxisDesc LockedAxis(float max_force = std::numeric_limits<float>::infinity())
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			axis.m_max_force = max_force;
			return axis;
		}

		// Return fixed-work settings without warm starting or positional correction.
		CpuConstraintSolverConfig VelocityOnlyConfig(int iterations = 1)
		{
			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = iterations;
			config.m_position_iterations = 0;
			config.m_warm_start_factor = 0.0f;
			return config;
		}

		// Return total linear and angular momentum about the fixed world origin.
		v8force TotalMomentum(std::span<RigidBody const* const> bodies)
		{
			auto total = v8force{};
			for (auto const* body : bodies)
			{
				auto const momentum = body->MomentumWS();
				total.lin += momentum.lin;
				total.ang += momentum.ang + Cross(body->CentreOfMassPositionWS(), momentum.lin);
			}
			return total;
		}
	}

	PRUnitTestClass(ConstraintSolverTests)
	{
		// Match an independent dense QP for a fully coupled three-row off-centre ball joint.
		PRUnitTestMethod(BlockSolveMatchesDenseOracle, Quick)
		{
			auto body_a = MakeBody(2.0f, m4x4::Identity(), v4{0.1f, -0.2f, 0.0f, 0.0f});
			auto body_b = MakeBody(3.0f, m4x4::Identity(), v4{-0.3f, +0.1f, 0.2f, 0.0f});
			body_a.VelocityWS(v8motion{v4{0.4f, -0.3f, +0.2f, 0}, v4{+1.0f, -0.5f, +0.7f, 0}});
			body_b.VelocityWS(v8motion{v4{-0.2f, +0.5f, -0.1f, 0}, v4{-0.4f, +0.8f, -0.6f, 0}});

			auto ball = BallSocketConstraintDesc{};
			ball.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), m4x4::Translation(+0.4f, +0.2f, -0.1f)};
			ball.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), m4x4::Translation(+0.4f, +0.2f, -0.1f)};
			auto constraints = ConstraintSet{};
			constraints.Add(ball);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);
			auto const& block = compiled.m_blocks[0];
			auto momentum_bodies = std::array<RigidBody const*, 2>{&body_a, &body_b};
			auto const momentum_before = TotalMomentum(momentum_bodies);

			// Form the tiny dense response independently and solve its unbounded impulse QP.
			auto response = DenseMatrix(3, 3);
			auto rhs = std::array<double, 3>{};
			for (int row = 0; row != 3; ++row)
			{
				rhs[row] = -RowVelocity(compiled.m_rows[row], block, remap);
				for (int column = 0; column <= row; ++column)
				{
					auto const symmetric_response = 0.5 * (
						Response(compiled.m_rows[row], compiled.m_rows[column], block, remap) +
						Response(compiled.m_rows[column], compiled.m_rows[row], block, remap));
					response(row, column) = symmetric_response;
					response(column, row) = symmetric_response;
				}
			}
			auto const unbounded = std::array{
				ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
				ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
				ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
			};
			auto const reference = SolveBoundedQp(response, rhs, unbounded);

			// Apply the oracle impulse to independent expected momenta.
			auto expected_momentum_a = body_a.MomentumWS();
			auto expected_momentum_b = body_b.MomentumWS();
			for (int row = 0; row != 3; ++row)
			{
				expected_momentum_a += compiled.m_rows[row].m_jacobian_a * static_cast<float>(reference.m_impulse[row]);
				expected_momentum_b += compiled.m_rows[row].m_jacobian_b * static_cast<float>(reference.m_impulse[row]);
			}
			auto const expected_velocity_a = body_a.InertiaInvWS() * expected_momentum_a;
			auto const expected_velocity_b = body_b.InertiaInvWS() * expected_momentum_b;

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 1.0f / 60.0f, VelocityOnlyConfig());

			ExpectNear(body_a.VelocityWS(), expected_velocity_a, 2.0e-5f);
			ExpectNear(body_b.VelocityWS(), expected_velocity_b, 2.0e-5f);
			auto const momentum_after = TotalMomentum(momentum_bodies);
			PR_EXPECT(FEqlAbsolute(momentum_after.lin, momentum_before.lin, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(momentum_after.ang, momentum_before.ang, 2.0e-5f));
			PR_EXPECT(metrics.m_projected_velocity_residual < 2.0e-5f);
			PR_EXPECT(metrics.m_physical_kinetic_energy_change <= 1.0e-5f);
		}

		// Recover independent dense solutions for randomized coupled six-row rigid-body blocks.
		PRUnitTestMethod(RandomBlocksMatchDenseOracle, Quick)
		{
			auto random = RandomSource{0xA0761D6478BD642Full};
			for (int trial = 0; trial != 24; ++trial)
			{
				auto const com_a = v4{0.2f * random.NextSigned(), 0.2f * random.NextSigned(), 0.2f * random.NextSigned(), 0};
				auto const com_b = v4{0.2f * random.NextSigned(), 0.2f * random.NextSigned(), 0.2f * random.NextSigned(), 0};
				auto body_a = MakeBody(1.0f + std::abs(random.NextSigned()), m4x4::Identity(), com_a);
				auto body_b = MakeBody(1.0f + std::abs(random.NextSigned()), m4x4::Identity(), com_b);
				body_a.VelocityWS(v8motion{
					v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0},
					v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0}});
				body_b.VelocityWS(v8motion{
					v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0},
					v4{random.NextSigned(), random.NextSigned(), random.NextSigned(), 0}});

				auto const rotation_vector = v4{0.4f * random.NextSigned(), 0.4f * random.NextSigned(), 0.4f * random.NextSigned(), 0};
				auto const anchor = v4{0.5f * random.NextSigned(), 0.5f * random.NextSigned(), 0.5f * random.NextSigned(), 1};
				auto const frame = m4x4{m3x3::Rotation(rotation_vector.xyz), anchor};
				auto desc = WeldConstraintDesc{};
				desc.m_frame_a = BodyFrame{BodyRef::Rigid(body_a), frame};
				desc.m_frame_b = BodyFrame{BodyRef::Rigid(body_b), frame};
				auto constraints = ConstraintSet{};
				constraints.Add(desc);
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto remap = BodyRemap(body_ptrs);
				auto const compiled = CompileConstraints(constraints, remap);
				auto const& block = compiled.m_blocks[0];

				// Symmetrise float response samples before the strict double-precision oracle validation.
				auto response = DenseMatrix(6, 6);
				auto rhs = std::array<double, 6>{};
				for (int row = 0; row != 6; ++row)
				{
					rhs[row] = -RowVelocity(compiled.m_rows[row], block, remap);
					for (int column = 0; column <= row; ++column)
					{
						auto const value = 0.5 * (
							Response(compiled.m_rows[row], compiled.m_rows[column], block, remap) +
							Response(compiled.m_rows[column], compiled.m_rows[row], block, remap));
						response(row, column) = value;
						response(column, row) = value;
					}
				}
				auto const bounds = std::array{
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
					ImpulseBounds{.m_lower = -std::numeric_limits<double>::infinity(), .m_upper = +std::numeric_limits<double>::infinity()},
				};
				auto const reference = SolveBoundedQp(response, rhs, bounds);

				// Compare the resulting twists rather than sharing any production inversion helpers.
				auto expected_momentum_a = body_a.MomentumWS();
				auto expected_momentum_b = body_b.MomentumWS();
				for (int row = 0; row != 6; ++row)
				{
					expected_momentum_a += compiled.m_rows[row].m_jacobian_a * static_cast<float>(reference.m_impulse[row]);
					expected_momentum_b += compiled.m_rows[row].m_jacobian_b * static_cast<float>(reference.m_impulse[row]);
				}
				auto const expected_velocity_a = body_a.InertiaInvWS() * expected_momentum_a;
				auto const expected_velocity_b = body_b.InertiaInvWS() * expected_momentum_b;

				auto solver = CpuConstraintSolver{};
				solver.Solve(compiled, remap, 1.0f / 120.0f, VelocityOnlyConfig());
				ExpectNear(body_a.VelocityWS(), expected_velocity_a, 2.0e-4f);
				ExpectNear(body_b.VelocityWS(), expected_velocity_b, 2.0e-4f);
			}
		}

		// Clamp an active upper limit and a motor to their exact per-step force-derived impulse bounds.
		PRUnitTestMethod(LimitsAndMotorsRespectForceBounds, Quick)
		{
			auto limited_body = MakeBody(1.0f, m4x4::Translation(+2.0f, 0.0f, 0.0f));
			auto motor_body = MakeBody(1.0f);
			limited_body.VelocityWS(v8motion{v4::Zero(), v4{+10.0f, 0.0f, 0.0f, 0}});
			motor_body.VelocityWS(v8motion{v4::Zero(), v4::Zero()});

			auto limited = D6ConstraintDesc{};
			limited.m_frame_a.m_body = BodyRef::World();
			limited.m_frame_b.m_body = BodyRef::Rigid(limited_body);
			limited.m_linear[0].m_mode = EConstraintAxisMode::Limited;
			limited.m_linear[0].m_limits = Range<float>{-1.0f, +1.0f};
			limited.m_linear[0].m_max_force = 2.0f;

			auto motor = D6ConstraintDesc{};
			motor.m_frame_a.m_body = BodyRef::World();
			motor.m_frame_b.m_body = BodyRef::Rigid(motor_body);
			motor.m_linear[1].m_mode = EConstraintAxisMode::Driven;
			motor.m_linear[1].m_target_velocity = 4.0f;
			motor.m_linear[1].m_max_force = 2.0f;

			auto constraints = ConstraintSet{};
			constraints.Add(limited);
			constraints.Add(motor);
			auto body_ptrs = std::array<RigidBody*, 2>{&limited_body, &motor_body};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 0.5f, VelocityOnlyConfig(4));

			PR_EXPECT(FEqlAbsolute(limited_body.VelocityWS().lin.x, 9.0f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(motor_body.VelocityWS().lin.y, 1.0f, 1.0e-6f));
			PR_EXPECT(metrics.m_max_impulse_bound_violation == 0.0f);
		}

		// Match the closed-form implicit spring-damper update for one translated body.
		PRUnitTestMethod(ComplianceUsesImplicitSpringDamper, Quick)
		{
			auto body = MakeBody(1.0f, m4x4::Translation(+1.0f, 0.0f, 0.0f));
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::World();
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = LockedAxis();
			desc.m_linear[0].m_stiffness = 100.0f;
			desc.m_linear[0].m_damping = 10.0f;

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);

			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled, remap, 0.1f, VelocityOnlyConfig());

			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, -10.0f / 3.0f, 1.0e-5f));
			PR_EXPECT(metrics.m_projected_velocity_residual < 1.0e-5f);
		}

		// Reduce hard positional error without changing any component of physical momentum.
		PRUnitTestMethod(SplitCorrectionPreservesMomentum, Quick)
		{
			auto body = MakeBody(2.0f, m4x4::Translation(+1.0f, 0.0f, 0.0f), v4{0.0f, 0.25f, 0.0f, 0.0f});
			body.VelocityWS(v8motion{v4{0.0f, 0.0f, 0.5f, 0}, v4{0.0f, 1.0f, 0.0f, 0}});
			auto const momentum_before = body.MomentumWS();

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::World();
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled_before = CompileConstraints(constraints, remap);

			auto config = CpuConstraintSolverConfig{};
			config.m_velocity_iterations = 0;
			config.m_position_iterations = 1;
			config.m_position_beta = 0.2f;
			config.m_max_position_speed = 10.0f;
			config.m_warm_start_factor = 0.0f;
			auto solver = CpuConstraintSolver{};
			auto const metrics = solver.Solve(compiled_before, remap, 0.1f, config);
			auto const compiled_after = CompileConstraints(constraints, remap);

			PR_EXPECT(body.MomentumWS() == momentum_before);
			PR_EXPECT(FEqlAbsolute(compiled_after.m_rows[0].m_position, 0.8f, 1.0e-4f));
			PR_EXPECT(metrics.m_initial_position_error == 1.0f);
		}

		// Treat a submitted infinite-mass endpoint as a reaction support without accumulating unusable momentum.
		PRUnitTestMethod(ImmovableEndpointRetainsZeroMomentum, Quick)
		{
			auto support = RigidBody{};
			auto body = MakeBody();
			body.VelocityWS(v8motion{v4::Zero(), v4{1.0f, 0.0f, 0.0f, 0}});

			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(support);
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&support, &body};
			auto remap = BodyRemap(body_ptrs);

			auto solver = CpuConstraintSolver{};
			solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, VelocityOnlyConfig());

			PR_EXPECT(support.MomentumWS() == v8force{});
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.0f, 1.0e-6f));
		}

		// Reuse a cached impulse across unchanged frames and invalidate it after a descriptor revision.
		PRUnitTestMethod(WarmStartScalesAndInvalidates, Quick)
		{
			auto body = MakeBody();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::World();
			desc.m_frame_b.m_body = BodyRef::Rigid(body);
			desc.m_linear[0].m_mode = EConstraintAxisMode::Driven;
			desc.m_linear[0].m_target_velocity = 2.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto remap = BodyRemap(body_ptrs);
			auto solver = CpuConstraintSolver{};

			auto populate = CpuConstraintSolverConfig{};
			populate.m_velocity_iterations = 1;
			populate.m_position_iterations = 0;
			populate.m_warm_start_factor = 1.0f;
			solver.Solve(CompileConstraints(constraints, remap), remap, 0.1f, populate);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 2.0f, 1.0e-6f));

			// With no iterations, only the retained impulse can reproduce the solved velocity.
			body.VelocityWS(v8motion{});
			auto reuse = populate;
			reuse.m_velocity_iterations = 0;
			solver.Solve(CompileConstraints(constraints, remap), remap, 0.1f, reuse);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 2.0f, 1.0e-6f));

			// A parameter revision clears the cache before the changed target can inherit a stale impulse.
			desc.m_linear[0].m_target_velocity = 3.0f;
			constraints.Update(handle, desc);
			body.VelocityWS(v8motion{});
			solver.Solve(CompileConstraints(constraints, remap), remap, 0.1f, reuse);
			PR_EXPECT(body.VelocityWS() == v8motion{});
		}

		// Match the independent oracle in the interior, apex, and sliding regions of a Coulomb cone.
		PRUnitTestMethod(FrictionConeProjectionMatchesOracle, Quick)
		{
			auto const cases = std::array{
				std::array<float, 3>{2.0f, 0.5f, 0.25f},
				std::array<float, 3>{-2.0f, 0.5f, 0.0f},
				std::array<float, 3>{0.5f, 2.0f, 0.0f},
			};
			for (auto const& input : cases)
			{
				auto const actual = pr::physics::ProjectFrictionCone(input, 0.5f);
				auto const expected = constraint_oracle::ProjectFrictionCone(
					{static_cast<double>(input[0]), static_cast<double>(input[1]), static_cast<double>(input[2])},
					0.5);
				for (int index = 0; index != 3; ++index)
					PR_EXPECT(FEqlAbsolute(actual[index], static_cast<float>(expected[index]), 1.0e-6f));
			}

			PR_THROWS(pr::physics::ProjectFrictionCone(cases[0], -0.1f), std::exception);
		}

		// Solve every named D6 projection and require all constrained row velocities to converge.
		PRUnitTestMethod(NamedJointGalleryConverges, Quick)
		{
			auto bodies = std::array{
				MakeBody(),
				MakeBody(),
				MakeBody(),
				MakeBody(),
				MakeBody(),
			};
			for (int index = 0; index != isize(bodies); ++index)
			{
				bodies[index].VelocityWS(v8motion{
					v4{0.2f * (index + 1), -0.3f, +0.4f, 0},
					v4{-0.5f, 0.1f * (index + 1), +0.3f, 0}});
			}

			auto constraints = ConstraintSet{};
			constraints.Add(BallSocketConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::World()}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[0])}});
			constraints.Add(HingeConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::World()}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[1])}});
			constraints.Add(SliderConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::World()}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[2])}});
			constraints.Add(WeldConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::World()}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[3])}});
			auto d6 = D6ConstraintDesc{};
			d6.m_frame_a.m_body = BodyRef::World();
			d6.m_frame_b.m_body = BodyRef::Rigid(bodies[4]);
			d6.m_linear[0] = LockedAxis();
			d6.m_angular[2] = LockedAxis();
			constraints.Add(d6);

			auto body_ptrs = std::array<RigidBody*, 5>{&bodies[0], &bodies[1], &bodies[2], &bodies[3], &bodies[4]};
			auto remap = BodyRemap(body_ptrs);
			auto const compiled = CompileConstraints(constraints, remap);
			auto solver = CpuConstraintSolver{};
			solver.Solve(compiled, remap, 1.0f / 60.0f, VelocityOnlyConfig(2));

			for (auto const& block : compiled.m_blocks)
				for (uint32_t index = 0; index != block.m_row_count; ++index)
					PR_EXPECT(std::abs(RowVelocity(compiled.m_rows[block.m_row_begin + index], block, remap)) < 2.0e-5f);
		}

		// Produce bit-identical momentum for repeated deterministic solves with the same ordering.
		PRUnitTestMethod(RepeatedSolveIsDeterministic, Quick)
		{
			auto Run = []()
			{
				auto bodies = std::array{MakeBody(), MakeBody(), MakeBody()};
				bodies[0].VelocityWS(v8motion{v4{0.1f, 0.2f, 0.3f, 0}, v4{+2.0f, -1.0f, 0.5f, 0}});
				bodies[1].VelocityWS(v8motion{v4{-0.2f, 0.1f, 0.4f, 0}, v4{-1.0f, +0.3f, 0.7f, 0}});
				bodies[2].VelocityWS(v8motion{v4{0.3f, -0.1f, 0.2f, 0}, v4{+0.2f, +0.8f, -0.4f, 0}});

				auto constraints = ConstraintSet{};
				constraints.Add(BallSocketConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::Rigid(bodies[0])}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[1])}});
				constraints.Add(BallSocketConstraintDesc{.m_frame_a = BodyFrame{.m_body = BodyRef::Rigid(bodies[1])}, .m_frame_b = BodyFrame{.m_body = BodyRef::Rigid(bodies[2])}});
				auto body_ptrs = std::array<RigidBody*, 3>{&bodies[0], &bodies[1], &bodies[2]};
				auto remap = BodyRemap(body_ptrs);
				auto solver = CpuConstraintSolver{};
				solver.Solve(CompileConstraints(constraints, remap), remap, 1.0f / 60.0f, VelocityOnlyConfig(12));
				return std::array{bodies[0].MomentumWS(), bodies[1].MomentumWS(), bodies[2].MomentumWS()};
			};

			auto const first = Run();
			auto const second = Run();
			PR_EXPECT(first == second);
		}
	};
}
#endif
