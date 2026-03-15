//************************************
// Physics Sandbox — GPU vs CPU Comparison Tests
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Unit tests that compare each GPU compute step against the equivalent
// CPU implementation. Each test packs known initial state, runs both
// paths, and compares results within tolerance.
//
// Tests cover:
//   1. Integration: GPU CSIntegrate vs CPU Evolve(RigidBodyDynamics&, float)
//   2. Collision resolution: GPU CSResolve vs CPU ResolveCollision()
//   3. Analytic results: free-flight, elastic collisions
//
#pragma once
#include "src/forward.h"

namespace physics_sandbox::tests
{
	namespace gpu_compare
	{
		// Tolerance for GPU vs CPU comparison (float32 precision across GPU boundary)
		inline constexpr float PosTol = 1e-4f;
		inline constexpr float RotTol = 1e-4f;
		inline constexpr float MomTol = 1e-4f;

		// Helper: run one integration step through Engine::Step (GPU path) on a single
		// dynamic body that won't collide with anything. Returns the post-integration body.
		inline physics::RigidBody RunGpuIntegrate(
			collision::Shape const* shape,
			float mass,
			m4x4 const& o2w,
			v4 ang_vel,
			v4 lin_vel,
			v4 force_ang,
			v4 force_lin,
			float dt)
		{
			// Create two bodies: the test body and a ground body far away (so no collision).
			auto ground_shape = collision::ShapeBox(v4{1, 1, 1, 0});
			physics::RigidBody bodies[2];
			bodies[0].Shape(shape, mass);
			bodies[0].O2W(o2w);
			bodies[0].VelocityWS(ang_vel, lin_vel);
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(v4{0, 0, -1000, 0})); // far away

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			// Apply the external force before stepping
			bodies[0].ZeroForces();
			bodies[0].ApplyForceWS(force_lin, force_ang, v4::Zero());
			engine.Step(dt, bodies);

			return bodies[0];
		}

		// Helper: run one integration step through CPU Evolve(RigidBodyDynamics&, float).
		// Returns the post-integration dynamics.
		inline physics::RigidBodyDynamics RunCpuIntegrate(
			collision::Shape const* shape,
			float mass,
			m4x4 const& o2w,
			v4 ang_vel,
			v4 lin_vel,
			v4 force_ang,
			v4 force_lin,
			float dt)
		{
			physics::RigidBody rb;
			rb.Shape(shape, mass);
			rb.O2W(o2w);
			rb.VelocityWS(ang_vel, lin_vel);
			rb.ZeroForces();
			rb.ApplyForceWS(force_lin, force_ang, v4::Zero());

			auto dyn = physics::PackDynamics(rb);
			physics::Evolve(dyn, dt);
			return dyn;
		}

		// Helper: compare a RigidBody (GPU output) with a RigidBodyDynamics (CPU output).
		// Returns true if they match within tolerance. Prints diagnostics on mismatch.
		inline bool CompareIntegration(
			char const* label,
			physics::RigidBody const& gpu_rb,
			physics::RigidBodyDynamics const& cpu_dyn,
			float pos_tol = PosTol,
			float rot_tol = RotTol,
			float mom_tol = MomTol)
		{
			auto gpu_dyn = physics::PackDynamics(gpu_rb);
			bool ok = true;

			auto pos_err = Length(gpu_dyn.o2w.pos - cpu_dyn.o2w.pos);
			auto rot_err = Length(gpu_dyn.o2w.x - cpu_dyn.o2w.x)
			             + Length(gpu_dyn.o2w.y - cpu_dyn.o2w.y)
			             + Length(gpu_dyn.o2w.z - cpu_dyn.o2w.z);
			auto ang_err = Length(gpu_dyn.momentum_ang - cpu_dyn.momentum_ang);
			auto lin_err = Length(gpu_dyn.momentum_lin - cpu_dyn.momentum_lin);

			if (pos_err > pos_tol || rot_err > rot_tol || ang_err > mom_tol || lin_err > mom_tol)
			{
				ok = false;
				printf("  [%s] MISMATCH:\n", label);
				printf("    pos_err=%.6f rot_err=%.6f ang_err=%.6f lin_err=%.6f\n",
					pos_err, rot_err, ang_err, lin_err);
				printf("    GPU pos=(%.6f, %.6f, %.6f)\n",
					gpu_dyn.o2w.pos.x, gpu_dyn.o2w.pos.y, gpu_dyn.o2w.pos.z);
				printf("    CPU pos=(%.6f, %.6f, %.6f)\n",
					cpu_dyn.o2w.pos.x, cpu_dyn.o2w.pos.y, cpu_dyn.o2w.pos.z);
				printf("    GPU mom_ang=(%.6f, %.6f, %.6f) mom_lin=(%.6f, %.6f, %.6f)\n",
					gpu_dyn.momentum_ang.x, gpu_dyn.momentum_ang.y, gpu_dyn.momentum_ang.z,
					gpu_dyn.momentum_lin.x, gpu_dyn.momentum_lin.y, gpu_dyn.momentum_lin.z);
				printf("    CPU mom_ang=(%.6f, %.6f, %.6f) mom_lin=(%.6f, %.6f, %.6f)\n",
					cpu_dyn.momentum_ang.x, cpu_dyn.momentum_ang.y, cpu_dyn.momentum_ang.z,
					cpu_dyn.momentum_lin.x, cpu_dyn.momentum_lin.y, cpu_dyn.momentum_lin.z);
			}
			else
			{
				printf("  [%s] OK (pos_err=%.2e rot_err=%.2e ang_err=%.2e lin_err=%.2e)\n",
					label, pos_err, rot_err, ang_err, lin_err);
			}
			return ok;
		}
	}

	// ===== GPU vs CPU Integration comparison tests =====
	PRUnitTestClass(GpuVsCpuIntegrationTests)
	{
		// Test 1: Sphere with pure linear velocity (no rotation, no forces)
		// Analytically: position should advance by v * dt exactly.
		PRUnitTestMethod(SphereLinearOnly)
		{
			auto sphere = collision::ShapeSphere(1.0f);
			auto const dt = 1.0f / 100.0f;
			auto const o2w = m4x4::Translation(v4{0, 0, 5, 0});
			auto const ang_vel = v4{0, 0, 0, 0};
			auto const lin_vel = v4{3, 0, 0, 0};

			auto gpu_rb = gpu_compare::RunGpuIntegrate(
				collision::shape_cast(&sphere), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			auto cpu_dyn = gpu_compare::RunCpuIntegrate(
				collision::shape_cast(&sphere), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			PR_EXPECT(gpu_compare::CompareIntegration("SphereLinear", gpu_rb, cpu_dyn));

			// Analytic check: position should be (3*0.01, 0, 5) = (0.03, 0, 5)
			auto expected_pos = v4{0.03f, 0, 5, 1};
			PR_EXPECT(FEqlRelative(gpu_rb.O2W().pos, expected_pos, 1e-4f));
		}

		// Test 2: Box with angular velocity only (anisotropic inertia, tests rotation path)
		PRUnitTestMethod(BoxAngularOnly)
		{
			auto box = collision::ShapeBox(v4{1, 2, 0.5f, 0});
			auto const dt = 1.0f / 100.0f;
			auto const o2w = m4x4::Translation(v4{0, 0, 5, 0});
			auto const ang_vel = v4{0.5f, 0.3f, 0.0f, 0.0f};
			auto const lin_vel = v4{0, 0, 0, 0};

			auto gpu_rb = gpu_compare::RunGpuIntegrate(
				collision::shape_cast(&box), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			auto cpu_dyn = gpu_compare::RunCpuIntegrate(
				collision::shape_cast(&box), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			PR_EXPECT(gpu_compare::CompareIntegration("BoxAngular", gpu_rb, cpu_dyn));

			// KE should be conserved (no forces)
			physics::RigidBody cpu_rb;
			cpu_rb.Shape(collision::shape_cast(&box), 10.0f);
			cpu_rb.O2W(o2w);
			cpu_rb.VelocityWS(ang_vel, lin_vel);
			auto ke0 = cpu_rb.KineticEnergy();

			auto ke_gpu = gpu_rb.KineticEnergy();
			PR_EXPECT(FEqlRelative(ke_gpu, ke0, 0.001f));
		}

		// Test 3: Polytope with off-centre CoM, combined angular and linear velocity
		PRUnitTestMethod(PolytopeOffCentreCoM)
		{
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};
			auto buf = collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<collision::ShapePolytope>();

			auto const dt = 1.0f / 100.0f;
			auto const o2w = m4x4::Translation(v4{0, 0, 10, 0});
			auto const ang_vel = v4{0.5f, 0.3f, 0.1f, 0.0f};
			auto const lin_vel = v4{2, 0, 0, 0};

			auto gpu_rb = gpu_compare::RunGpuIntegrate(
				collision::shape_cast(&poly), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			auto cpu_dyn = gpu_compare::RunCpuIntegrate(
				collision::shape_cast(&poly), 10.0f, o2w, ang_vel, lin_vel,
				v4{}, v4{}, dt);

			PR_EXPECT(gpu_compare::CompareIntegration("PolytopeOffCentre", gpu_rb, cpu_dyn));
		}

		// Test 4: Integration with external forces (gravity)
		PRUnitTestMethod(SphereWithGravity)
		{
			auto sphere = collision::ShapeSphere(1.0f);
			auto const dt = 1.0f / 100.0f;
			auto const o2w = m4x4::Translation(v4{0, 0, 10, 0});
			auto const ang_vel = v4{0, 0, 0, 0};
			auto const lin_vel = v4{0, 0, 0, 0};
			auto const mass = 10.0f;
			auto const gravity = v4{0, 0, -9.81f * mass, 0};

			auto gpu_rb = gpu_compare::RunGpuIntegrate(
				collision::shape_cast(&sphere), mass, o2w, ang_vel, lin_vel,
				v4{}, gravity, dt);

			auto cpu_dyn = gpu_compare::RunCpuIntegrate(
				collision::shape_cast(&sphere), mass, o2w, ang_vel, lin_vel,
				v4{}, gravity, dt);

			PR_EXPECT(gpu_compare::CompareIntegration("SphereGravity", gpu_rb, cpu_dyn));

			// Analytic: after one kick-drift-kick step with constant gravity
			// Störmer-Verlet: v_half = v0 + a*dt/2, x1 = x0 + v_half*dt, v1 = v_half + a*dt/2
			// a = -9.81, v0 = 0, x0 = 10
			// v_half = 0 + (-9.81)*0.005 = -0.04905
			// x1 = 10 + (-0.04905)*0.01 = 9.9995095
			// v1 = -0.04905 + (-9.81)*0.005 = -0.0981
			auto expected_z = 10.0f + (-9.81f * 0.005f) * 0.01f;
			PR_EXPECT(FEqlRelative(gpu_rb.O2W().pos.z, expected_z, 1e-4f));
		}

		// Test 5: Multi-step integration comparison (accumulates errors over 100 steps)
		PRUnitTestMethod(MultiStepBoxWithSpin)
		{
			auto box = collision::ShapeBox(v4{1, 2, 0.5f, 0});
			auto const dt = 1.0f / 100.0f;
			auto const mass = 10.0f;
			auto const ang_vel = v4{0.5f, 0.3f, 0.0f, 0.0f};
			auto const lin_vel = v4{1, 0, 0, 0};

			// GPU path: run 100 steps through Engine::Step
			auto ground_shape = collision::ShapeBox(v4{1, 1, 1, 0});
			physics::RigidBody gpu_bodies[2];
			gpu_bodies[0].Shape(collision::shape_cast(&box), mass);
			gpu_bodies[0].O2W(m4x4::Translation(v4{0, 0, 10, 0}));
			gpu_bodies[0].VelocityWS(ang_vel, lin_vel);
			gpu_bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			gpu_bodies[1].O2W(m4x4::Translation(v4{0, 0, -1000, 0}));

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			for (int step = 0; step != 100; ++step)
			{
				gpu_bodies[0].ZeroForces();
				gpu_bodies[1].ZeroForces();
				engine.Step(dt, gpu_bodies);
			}

			// CPU path: run 100 steps through Evolve(dyn, dt)
			physics::RigidBody cpu_rb;
			cpu_rb.Shape(collision::shape_cast(&box), mass);
			cpu_rb.O2W(m4x4::Translation(v4{0, 0, 10, 0}));
			cpu_rb.VelocityWS(ang_vel, lin_vel);

			auto cpu_dyn = physics::PackDynamics(cpu_rb);
			for (int step = 0; step != 100; ++step)
			{
				cpu_dyn.force_ang = v4{};
				cpu_dyn.force_lin = v4{};
				physics::Evolve(cpu_dyn, dt);
			}

			// Compare after 100 steps (allow larger tolerance for accumulated float differences)
			PR_EXPECT(gpu_compare::CompareIntegration("MultiStep100", gpu_bodies[0], cpu_dyn, 1e-3f, 1e-3f, 1e-3f));
		}
	};

	// ===== GPU vs CPU Resolve comparison tests =====
	PRUnitTestClass(GpuVsCpuResolveTests)
	{
		// Helper: run a two-body collision with both GPU and CPU resolve paths,
		// and compare the post-resolve momenta.
		struct ResolveComparison
		{
			v8force gpu_momentum_a;
			v8force gpu_momentum_b;
			v8force cpu_momentum_a;
			v8force cpu_momentum_b;
			bool gpu_collision;
			bool cpu_collision;
		};

		static ResolveComparison RunResolveComparison(
			collision::Shape const& shape_a, physics::Inertia const& inertia_a, v4 pos_a, v4 vel_a,
			collision::Shape const& shape_b, physics::Inertia const& inertia_b, v4 pos_b, v4 vel_b)
		{
			auto result = ResolveComparison{};
			auto const dt = 1.0f / 100.0f;

			// Configure perfectly elastic, frictionless collisions
			physics::MaterialMap materials;
			auto& mat = materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_friction_static = 0.0f;

			// --- GPU resolve path ---
			{
				physics::RigidBody bodies[2] = {
					physics::RigidBody{&shape_a, m4x4::Translation(pos_a), inertia_a},
					physics::RigidBody{&shape_b, m4x4::Translation(pos_b), inertia_b},
				};
				bodies[0].VelocityWS(v4::Zero(), vel_a);
				bodies[1].VelocityWS(v4::Zero(), vel_b);

				physics::Engine engine(materials);
				engine.UseGpuResolve(true);
				engine.PostCollisionDetection += [&](auto&, auto args)
				{
					if (!args.m_contacts.empty())
						result.gpu_collision = true;
				};

				for (int step = 0; step != 5000; ++step)
				{
					bodies[0].ZeroForces();
					bodies[1].ZeroForces();
					engine.Step(dt, bodies);
					if (result.gpu_collision)
					{
						result.gpu_momentum_a = bodies[0].MomentumWS();
						result.gpu_momentum_b = bodies[1].MomentumWS();
						break;
					}
				}
			}

			// --- CPU resolve path ---
			{
				physics::RigidBody bodies[2] = {
					physics::RigidBody{&shape_a, m4x4::Translation(pos_a), inertia_a},
					physics::RigidBody{&shape_b, m4x4::Translation(pos_b), inertia_b},
				};
				bodies[0].VelocityWS(v4::Zero(), vel_a);
				bodies[1].VelocityWS(v4::Zero(), vel_b);

				physics::Engine engine(materials);
				engine.UseGpuResolve(false);
				engine.PostCollisionDetection += [&](auto&, auto args)
				{
					if (!args.m_contacts.empty())
						result.cpu_collision = true;
				};

				for (int step = 0; step != 5000; ++step)
				{
					bodies[0].ZeroForces();
					bodies[1].ZeroForces();
					engine.Step(dt, bodies);
					if (result.cpu_collision)
					{
						result.cpu_momentum_a = bodies[0].MomentumWS();
						result.cpu_momentum_b = bodies[1].MomentumWS();
						break;
					}
				}
			}

			return result;
		}

		// Test: equal mass sphere head-on collision, GPU vs CPU resolve
		PRUnitTestMethod(SphereHeadOn)
		{
			auto sphere = collision::ShapeSphere(1.0f);
			auto inertia = physics::Inertia::Sphere(1.0f, 10.0f);

			auto r = RunResolveComparison(
				sphere, inertia, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere, inertia, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			printf("  GPU collision: %s, CPU collision: %s\n",
				r.gpu_collision ? "yes" : "no", r.cpu_collision ? "yes" : "no");

			PR_EXPECT(r.gpu_collision);
			PR_EXPECT(r.cpu_collision);

			if (r.gpu_collision && r.cpu_collision)
			{
				auto ang_diff_a = Length(r.gpu_momentum_a.ang - r.cpu_momentum_a.ang);
				auto lin_diff_a = Length(r.gpu_momentum_a.lin - r.cpu_momentum_a.lin);
				auto ang_diff_b = Length(r.gpu_momentum_b.ang - r.cpu_momentum_b.ang);
				auto lin_diff_b = Length(r.gpu_momentum_b.lin - r.cpu_momentum_b.lin);

				printf("  Body A: ang_diff=%.6f lin_diff=%.6f\n", ang_diff_a, lin_diff_a);
				printf("  Body B: ang_diff=%.6f lin_diff=%.6f\n", ang_diff_b, lin_diff_b);

				// GPU and CPU resolve should produce very similar results
				PR_EXPECT(ang_diff_a < 0.01f);
				PR_EXPECT(lin_diff_a < 0.01f);
				PR_EXPECT(ang_diff_b < 0.01f);
				PR_EXPECT(lin_diff_b < 0.01f);
			}
		}

		// Test: box vs ground with GPU resolve (tests off-centre impulse application)
		PRUnitTestMethod(BoxOnGround)
		{
			auto box = collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});
			auto inertia_box = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto ground = collision::ShapeBox(v4{100, 100, 0.5f, 0});
			auto inertia_ground = physics::Inertia::Infinite();

			auto r = RunResolveComparison(
				box, inertia_box, v4{0, 0, 2, 1}, v4{0, 0, -3, 0},
				ground, inertia_ground, v4{0, 0, -0.5f, 1}, v4{0, 0, 0, 0});

			printf("  GPU collision: %s, CPU collision: %s\n",
				r.gpu_collision ? "yes" : "no", r.cpu_collision ? "yes" : "no");

			PR_EXPECT(r.gpu_collision);
			PR_EXPECT(r.cpu_collision);

			if (r.gpu_collision && r.cpu_collision)
			{
				auto lin_diff = Length(r.gpu_momentum_a.lin - r.cpu_momentum_a.lin);
				printf("  Box lin_diff=%.6f\n", lin_diff);
				printf("  GPU box mom_lin=(%.4f, %.4f, %.4f)\n",
					r.gpu_momentum_a.lin.x, r.gpu_momentum_a.lin.y, r.gpu_momentum_a.lin.z);
				printf("  CPU box mom_lin=(%.4f, %.4f, %.4f)\n",
					r.cpu_momentum_a.lin.x, r.cpu_momentum_a.lin.y, r.cpu_momentum_a.lin.z);
				PR_EXPECT(lin_diff < 0.1f);
			}
		}
	};

	// ===== Analytic validation tests =====
	PRUnitTestClass(AnalyticPhysicsTests)
	{
		// Test: free-flight linear motion — verify position after N steps
		PRUnitTestMethod(FreeFlight100Steps)
		{
			auto sphere = collision::ShapeSphere(1.0f);
			auto const dt = 1.0f / 100.0f;
			auto const mass = 10.0f;
			auto const lin_vel = v4{3, -2, 1, 0};

			auto ground_shape = collision::ShapeBox(v4{1, 1, 1, 0});
			physics::RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&sphere), mass);
			bodies[0].O2W(m4x4::Translation(v4{0, 0, 100, 0}));
			bodies[0].VelocityWS(v4::Zero(), lin_vel);
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(v4{0, 0, -1000, 0}));

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			for (int step = 0; step != 100; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				engine.Step(dt, bodies);
			}

			// After 1 second at constant velocity, position should be p0 + v*t
			auto expected_pos = v4{0, 0, 100, 1} + lin_vel * 1.0f;
			expected_pos.w = 1;
			auto pos_err = Length(bodies[0].O2W().pos - expected_pos);
			printf("  Position after 100 steps: (%.6f, %.6f, %.6f)\n",
				bodies[0].O2W().pos.x, bodies[0].O2W().pos.y, bodies[0].O2W().pos.z);
			printf("  Expected: (%.6f, %.6f, %.6f) err=%.6f\n",
				expected_pos.x, expected_pos.y, expected_pos.z, pos_err);
			PR_EXPECT(pos_err < 1e-3f);
		}

		// Test: KE conservation for spinning asymmetric body (1000 steps, no forces)
		PRUnitTestMethod(AngularKEConservation)
		{
			auto box = collision::ShapeBox(v4{1, 2, 0.5f, 0});
			auto const dt = 1.0f / 100.0f;
			auto const mass = 10.0f;
			auto const ang_vel = v4{1.0f, 0.5f, 0.2f, 0.0f};

			auto ground_shape = collision::ShapeBox(v4{1, 1, 1, 0});
			physics::RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box), mass);
			bodies[0].O2W(m4x4::Translation(v4{0, 0, 100, 0}));
			bodies[0].VelocityWS(ang_vel, v4::Zero());
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(v4{0, 0, -1000, 0}));

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			auto ke0 = bodies[0].KineticEnergy();
			float max_drift = 0.0f;

			for (int step = 0; step != 1000; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				engine.Step(dt, bodies);

				auto ke = bodies[0].KineticEnergy();
				auto drift = Abs(ke - ke0) / (ke0 + 1e-10f);
				max_drift = std::max(max_drift, drift);
			}

			printf("  Initial KE: %.6f, Final KE: %.6f, Max drift: %.4f%%\n",
				ke0, bodies[0].KineticEnergy(), max_drift * 100.0f);

			// Symplectic integrator should conserve KE to within 1%
			PR_EXPECT(max_drift < 0.01f);
		}

		// Test: Broadphase AABB diagnostic — verify AABBs are correct for stationary body
		// This directly tests the failing scenario (one body stationary, one moving toward it)
		PRUnitTestMethod(BroadphaseDiagnostic)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto const dt = 1.0f / 100.0f;

			physics::RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box), 10.0f);
			bodies[0].O2W(m4x4::Translation(v4{-5, 0, 0, 1}));
			bodies[0].VelocityWS(v4::Zero(), v4{+3, 0, 0, 0});
			bodies[1].Shape(collision::shape_cast(&box), 10.0f);
			bodies[1].O2W(m4x4::Translation(v4{+5, 0, 0, 1}));
			bodies[1].VelocityWS(v4::Zero(), v4{0, 0, 0, 0});

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			// Step until the bodies should be close enough to overlap
			bool collision_detected = false;
			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				if (!args.m_contacts.empty())
					collision_detected = true;
			};

			for (int step = 0; step != 500; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				engine.Step(dt, bodies);

				if (step % 50 == 0)
				{
					printf("  Step %d: A pos=(%.2f,%.2f,%.2f) B pos=(%.2f,%.2f,%.2f)\n",
						step,
						bodies[0].O2W().pos.x, bodies[0].O2W().pos.y, bodies[0].O2W().pos.z,
						bodies[1].O2W().pos.x, bodies[1].O2W().pos.y, bodies[1].O2W().pos.z);
				}

				if (collision_detected)
				{
					printf("  Collision detected at step %d!\n", step);
					break;
				}
			}

			PR_EXPECT(collision_detected);
		}

		// Diagnostic: compare full GPU pipeline (GPU detect + GPU resolve) against
		// full CPU pipeline (CPU SAT + CPU resolve) for a sphere-sphere collision.
		// Prints the velocity differences to help diagnose GPU GJK accuracy.
		PRUnitTestMethod(FullGpuVsCpuSphereSphere)
		{
			auto sphere = collision::ShapeSphere(1.0f);
			auto inertia = physics::Inertia::Sphere(1.0f, 10.0f);
			auto const dt = 1.0f / 100.0f;

			auto run_collision = [&](bool gpu_detect, bool gpu_resolve) -> std::pair<v8motion, v8motion>
			{
				physics::RigidBody bodies[2] = {
					physics::RigidBody{&sphere, m4x4::Translation(v4{-5, 0, 0, 1}), inertia},
					physics::RigidBody{&sphere, m4x4::Translation(v4{+5, 0, 0, 1}), inertia},
				};
				bodies[0].VelocityWS(v4::Zero(), v4{+3, 0, 0, 0});
				bodies[1].VelocityWS(v4::Zero(), v4{-3, 0, 0, 0});

				physics::MaterialMap materials;
				auto& mat = materials(0);
				mat.m_elasticity_norm = 1.0f;
				mat.m_friction_static = 0.0f;

				physics::Engine engine(materials);
				engine.UseGpuDetect(gpu_detect);
				engine.UseGpuResolve(gpu_resolve);
				
				bool collision_done = false;
				engine.PostCollisionDetection += [&](auto&, auto args)
				{
					if (!args.m_contacts.empty())
						collision_done = true;
				};

				for (int step = 0; step != 5000 && !collision_done; ++step)
				{
					bodies[0].ZeroForces();
					bodies[1].ZeroForces();
					engine.Step(dt, bodies);
				}

				return {bodies[0].VelocityWS(), bodies[1].VelocityWS()};
			};

			auto [full_gpu_va, full_gpu_vb] = run_collision(true, true);
			auto [gjk_cpu_va, gjk_cpu_vb] = run_collision(true, false); // GPU detect + CPU resolve
			auto [sat_gpu_va, sat_gpu_vb] = run_collision(false, true); // CPU detect + GPU resolve
			auto [full_cpu_va, full_cpu_vb] = run_collision(false, false);

			printf("  Full GPU  (GJK+GPUres): va=(%.6f, %.6f, %.6f) vb=(%.6f, %.6f, %.6f)\n",
				full_gpu_va.lin.x, full_gpu_va.lin.y, full_gpu_va.lin.z, full_gpu_vb.lin.x, full_gpu_vb.lin.y, full_gpu_vb.lin.z);
			printf("  GJK+CPUres:             va=(%.6f, %.6f, %.6f) vb=(%.6f, %.6f, %.6f)\n",
				gjk_cpu_va.lin.x, gjk_cpu_va.lin.y, gjk_cpu_va.lin.z, gjk_cpu_vb.lin.x, gjk_cpu_vb.lin.y, gjk_cpu_vb.lin.z);
			printf("  SAT+GPUres:             va=(%.6f, %.6f, %.6f) vb=(%.6f, %.6f, %.6f)\n",
				sat_gpu_va.lin.x, sat_gpu_va.lin.y, sat_gpu_va.lin.z, sat_gpu_vb.lin.x, sat_gpu_vb.lin.y, sat_gpu_vb.lin.z);
			printf("  Full CPU  (SAT+CPUres): va=(%.6f, %.6f, %.6f) vb=(%.6f, %.6f, %.6f)\n",
				full_cpu_va.lin.x, full_cpu_va.lin.y, full_cpu_va.lin.z, full_cpu_vb.lin.x, full_cpu_vb.lin.y, full_cpu_vb.lin.z);
			printf("  Expected analytic:      va=(-3, 0, 0) vb=(+3, 0, 0)\n");

			// The key diagnostic: which combination breaks?
			auto diff_gjk_cpu = Length(gjk_cpu_va.lin - full_cpu_va.lin);
			auto diff_sat_gpu = Length(sat_gpu_va.lin - full_cpu_va.lin);
			printf("  GJK+CPUres diff from ref: %.6f (if large → GPU GJK contact data is wrong)\n", diff_gjk_cpu);
			printf("  SAT+GPUres diff from ref: %.6f (if large → GPU resolve is wrong)\n", diff_sat_gpu);

			// Both combinations should be close to the full CPU result
			PR_EXPECT(diff_gjk_cpu < 0.1f);
			PR_EXPECT(diff_sat_gpu < 0.1f);
		}
	};
}
