//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Embedded unit tests for the physics sandbox application.
// Run with the `-unittest` command line argument.
// These tests use the PR_UNITTESTS framework and run headlessly
// (no View3D, no window) — they operate directly on RigidBody objects.
//
// For a broader set of collision tests, see also:
//   projects/rylogic/physics-2/src/unittests/test_collision.h
//
#pragma once
#include "src/forward.h"

namespace physics_sandbox::tests
{
	// Common test infrastructure shared by all collision test classes.
	// Captures conserved quantities and runs a two-body elastic collision
	// to completion, recording pre/post state for validation.
	namespace collision_test
	{
		using TestEngine = physics::Engine<physics::broadphase::Brute<physics::RigidBody>, physics::MaterialMap>;

		// Snapshot of conserved system quantities (momentum, energy)
		struct SystemState
		{
			v4 total_lin_momentum;
			v4 total_ang_momentum;
			float total_ke;

			static SystemState Capture(physics::RigidBody const& a, physics::RigidBody const& b)
			{
				auto s = SystemState{};
				s.total_lin_momentum = a.MomentumWS().lin + b.MomentumWS().lin;

				// Total angular momentum about the origin:
				// L_total = (L_spin_a + r_a x p_a) + (L_spin_b + r_b x p_b)
				auto La = a.MomentumWS().ang + Cross(a.O2W().pos, a.MomentumWS().lin);
				auto Lb = b.MomentumWS().ang + Cross(b.O2W().pos, b.MomentumWS().lin);
				s.total_ang_momentum = La + Lb;
				s.total_ke = a.KineticEnergy() + b.KineticEnergy();
				return s;
			}
		};

		// Result of running a collision scenario
		struct CollisionResult
		{
			SystemState before;
			SystemState after;
			v8motion vel_a;
			v8motion vel_b;
			bool collision_occurred;
		};

		// Run a two-body elastic collision scenario to completion.
		// Bodies are created from the given shapes, positions, velocities, and inertias.
		// The engine uses perfectly elastic (restitution = 1) frictionless collisions.
		// Returns the system state before and after the first collision.
		inline CollisionResult RunScenario(
			collision::Shape const& shape_a, physics::Inertia const& inertia_a, v4 pos_a, v4 vel_a,
			collision::Shape const& shape_b, physics::Inertia const& inertia_b, v4 pos_b, v4 vel_b)
		{
			auto result = CollisionResult{};
			result.collision_occurred = false;

			physics::RigidBody bodies[2] = {
				physics::RigidBody{&shape_a, m4x4::Translation(pos_a), inertia_a},
				physics::RigidBody{&shape_b, m4x4::Translation(pos_b), inertia_b},
			};
			auto& body_a = bodies[0];
			auto& body_b = bodies[1];
			body_a.VelocityWS(v4::Zero(), vel_a);
			body_b.VelocityWS(v4::Zero(), vel_b);

			// Configure perfectly elastic, frictionless collisions
			auto engine = TestEngine{};
			auto& mat = engine.m_materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_elasticity_tang = 0.0f;
			mat.m_elasticity_tors = 0.0f;
			mat.m_friction_static = 0.0f;

			engine.m_broadphase.Add(body_a);
			engine.m_broadphase.Add(body_b);

			// Capture the "before" state on the step where collision is first detected,
			// before the impulse is applied.
			engine.PostCollisionDetection += [&](auto&, auto& collisions)
			{
				if (collisions.empty()) return;
				result.before = SystemState::Capture(body_a, body_b);
				result.collision_occurred = true;
			};

			// Step until a collision occurs (or timeout after 5000 steps)
			auto const dt = 1.0f / 100.0f;
			for (int step = 0; step != 5000; ++step)
			{
				body_a.ZeroForces();
				body_b.ZeroForces();
				engine.Step(dt, bodies);

				if (result.collision_occurred)
				{
					result.after = SystemState::Capture(body_a, body_b);
					result.vel_a = body_a.VelocityWS();
					result.vel_b = body_b.VelocityWS();
					break;
				}
			}
			return result;
		}

		// 1D elastic collision velocity formulas:
		//   v1' = ((m1-m2)*v1 + 2*m2*v2) / (m1+m2)
		//   v2' = ((m2-m1)*v2 + 2*m1*v1) / (m1+m2)
		inline float ElasticVelocity1D(float m1, float v1, float m2, float v2)
		{
			return ((m1 - m2) * v1 + 2.0f * m2 * v2) / (m1 + m2);
		}
	}

	// ===== Box vs Box collision tests =====
	PRUnitTestClass(BoxVsBoxCollisionTests)
	{
		// Helper: create a box scenario using the same shape for both bodies
		static collision_test::CollisionResult RunBoxVsBox(
			collision::ShapeBox& box,
			v4 pos_a, v4 vel_a, float mass_a,
			v4 pos_b, v4 vel_b, float mass_b)
		{
			auto inertia_a = physics::Inertia::Box(box.m_radius, mass_a);
			auto inertia_b = physics::Inertia::Box(box.m_radius, mass_b);
			return collision_test::RunScenario(box, inertia_a, pos_a, vel_a, box, inertia_b, pos_b, vel_b);
		}

		// Head-on equal mass: velocities should swap exactly
		PRUnitTestMethod(HeadOnEqualMass)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto r = RunBoxVsBox(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass head-on: velocities swap sign
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +3.0f, 0.001f));
		}

		// Head-on different mass: analytic prediction (10kg @ +3 vs 5kg @ -3)
		PRUnitTestMethod(HeadOnDiffMass)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto r = RunBoxVsBox(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0}, 5.0f);

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// v1' = ((10-5)*3 + 2*5*(-3)) / 15 = (15-30)/15 = -1
			// v2' = ((5-10)*(-3) + 2*10*3) / 15 = (15+60)/15 = +5
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -1.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +5.0f, 0.001f));
		}

		// Stationary target: moving body stops, target takes velocity
		PRUnitTestMethod(StationaryTarget)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto r = RunBoxVsBox(box,
				v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0}, 10.0f,
				v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0}, 10.0f);

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass, stationary target: projectile stops, target inherits velocity
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, 0.0f, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, 3.0f, 0.001f));
		}
	};

	// ===== Box vs Sphere collision tests =====
	// Tests collisions between a box and a sphere. The box is always body A
	// and the sphere is always body B, approaching along the X axis.
	PRUnitTestClass(BoxVsSphereCollisionTests)
	{
		// Helper: create a box-vs-sphere scenario
		static collision_test::CollisionResult RunBoxVsSphere(
			collision::ShapeBox& box, float mass_a, v4 pos_a, v4 vel_a,
			collision::ShapeSphere& sphere, float mass_b, v4 pos_b, v4 vel_b)
		{
			auto inertia_a = physics::Inertia::Box(box.m_radius, mass_a);
			auto inertia_b = physics::Inertia::Sphere(sphere.m_radius, mass_b);
			return collision_test::RunScenario(box, inertia_a, pos_a, vel_a, sphere, inertia_b, pos_b, vel_b);
		}

		// Head-on equal mass: box and sphere of equal mass, approaching head-on
		// For a centred head-on collision, the 1D elastic formulas apply
		PRUnitTestMethod(HeadOnEqualMass)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.0f);

			// Positions: box face at x=-4, sphere surface at x=+4
			// They approach at ±3 m/s along X
			auto r = RunBoxVsSphere(
				box,    10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere, 10.0f, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass head-on: velocities swap sign
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +3.0f, 0.001f));
		}

		// Head-on different mass: heavy box hits lighter sphere
		PRUnitTestMethod(HeavyBoxLightSphere)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.0f);

			auto r = RunBoxVsSphere(
				box,    10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere,  5.0f, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// v1' = ((10-5)*3 + 2*5*(-3)) / 15 = -1
			// v2' = ((5-10)*(-3) + 2*10*3) / 15 = +5
			auto expected_va = collision_test::ElasticVelocity1D(10.0f, +3.0f, 5.0f, -3.0f);
			auto expected_vb = collision_test::ElasticVelocity1D(5.0f, -3.0f, 10.0f, +3.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, expected_va, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, expected_vb, 0.001f));
		}

		// Light box hits heavy sphere (sphere barely moves, box bounces back)
		PRUnitTestMethod(LightBoxHeavySphere)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.5f);

			auto r = RunBoxVsSphere(
				box,     5.0f, v4{-5, 0, 0, 1}, v4{+4, 0, 0, 0},
				sphere, 20.0f, v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Light projectile bouncing off heavy stationary target:
			// v1' = ((5-20)*4) / 25 = -2.4, v2' = (2*5*4)/25 = +1.6
			auto expected_va = collision_test::ElasticVelocity1D(5.0f, +4.0f, 20.0f, 0.0f);
			auto expected_vb = collision_test::ElasticVelocity1D(20.0f, 0.0f, 5.0f, +4.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, expected_va, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, expected_vb, 0.001f));
		}

		// Stationary sphere: box hits a stationary sphere of equal mass
		PRUnitTestMethod(StationarySphere)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.0f);

			auto r = RunBoxVsSphere(
				box,    10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere, 10.0f, v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass, stationary target: projectile stops, target inherits velocity
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, 0.0f, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, 3.0f, 0.001f));
		}

		// Sphere hits stationary box (reverse of the typical setup)
		PRUnitTestMethod(StationaryBox)
		{
			auto box = collision::ShapeBox(v4{2, 2, 2, 0});
			auto sphere = collision::ShapeSphere(1.0f);

			auto r = RunBoxVsSphere(
				box,    10.0f, v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0},
				sphere, 10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass, stationary target: sphere stops, box inherits velocity
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, +3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, 0.0f, 0.05f));
		}
	};

	// ===== Sphere vs Sphere collision tests =====
	// Sphere-sphere collisions are the cleanest to validate because the contact
	// normal is always along the line connecting the centres, making 1D elastic
	// formulas exact for head-on configurations.
	PRUnitTestClass(SphereVsSphereCollisionTests)
	{
		// Helper: create a sphere-vs-sphere scenario
		static collision_test::CollisionResult RunSphereVsSphere(
			collision::ShapeSphere& sphere_a, float mass_a, v4 pos_a, v4 vel_a,
			collision::ShapeSphere& sphere_b, float mass_b, v4 pos_b, v4 vel_b)
		{
			auto inertia_a = physics::Inertia::Sphere(sphere_a.m_radius, mass_a);
			auto inertia_b = physics::Inertia::Sphere(sphere_b.m_radius, mass_b);
			return collision_test::RunScenario(sphere_a, inertia_a, pos_a, vel_a, sphere_b, inertia_b, pos_b, vel_b);
		}

		// Head-on equal mass, equal radius: classic Newton's cradle scenario
		PRUnitTestMethod(HeadOnEqualMassEqualRadius)
		{
			auto sphere = collision::ShapeSphere(1.0f);

			auto r = RunSphereVsSphere(
				sphere, 10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere, 10.0f, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass head-on: velocities swap sign
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +3.0f, 0.001f));
		}

		// Head-on different mass: heavy sphere hits lighter sphere
		PRUnitTestMethod(HeadOnHeavyHitsLight)
		{
			auto sphere_a = collision::ShapeSphere(1.5f);
			auto sphere_b = collision::ShapeSphere(1.0f);

			auto r = RunSphereVsSphere(
				sphere_a, 10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere_b,  5.0f, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// v1' = ((10-5)*3 + 2*5*(-3)) / 15 = -1
			// v2' = ((5-10)*(-3) + 2*10*3) / 15 = +5
			auto expected_va = collision_test::ElasticVelocity1D(10.0f, +3.0f, 5.0f, -3.0f);
			auto expected_vb = collision_test::ElasticVelocity1D(5.0f, -3.0f, 10.0f, +3.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, expected_va, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, expected_vb, 0.001f));
		}

		// Stationary target: equal mass, one sphere at rest
		PRUnitTestMethod(StationaryTarget)
		{
			auto sphere = collision::ShapeSphere(1.0f);

			auto r = RunSphereVsSphere(
				sphere, 10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere, 10.0f, v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass, stationary target: projectile stops, target inherits velocity
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, 0.0f, 0.05f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, 3.0f, 0.001f));
		}

		// Large mass ratio: light sphere bouncing off much heavier sphere
		PRUnitTestMethod(LargeMassRatio)
		{
			auto sphere_a = collision::ShapeSphere(0.5f);
			auto sphere_b = collision::ShapeSphere(2.0f);

			// Tiny sphere at 5 m/s hits a massive stationary sphere
			auto r = RunSphereVsSphere(
				sphere_a,  1.0f, v4{-5, 0, 0, 1}, v4{+5, 0, 0, 0},
				sphere_b, 50.0f, v4{+5, 0, 0, 1}, v4{ 0, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Light projectile nearly reverses; heavy target barely moves
			// v1' = ((1-50)*5) / 51 = -4.804, v2' = (2*1*5) / 51 = +0.196
			auto expected_va = collision_test::ElasticVelocity1D(1.0f, +5.0f, 50.0f, 0.0f);
			auto expected_vb = collision_test::ElasticVelocity1D(50.0f, 0.0f, 1.0f, +5.0f);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, expected_va, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, expected_vb, 0.01f));
		}

		// Both moving same direction: fast sphere catches slow sphere
		PRUnitTestMethod(SameDirectionCatchUp)
		{
			auto sphere = collision::ShapeSphere(1.0f);

			// Both moving right, but A is faster and catches B
			auto r = RunSphereVsSphere(
				sphere, 10.0f, v4{-5, 0, 0, 1}, v4{+5, 0, 0, 0},
				sphere, 10.0f, v4{+5, 0, 0, 1}, v4{+1, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass: velocities exchange → A gets +1, B gets +5
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, +1.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +5.0f, 0.001f));
		}

		// Different radii, equal mass: validates that geometry doesn't
		// affect the 1D elastic result for head-on collisions
		PRUnitTestMethod(DifferentRadiiEqualMass)
		{
			auto sphere_a = collision::ShapeSphere(0.5f);
			auto sphere_b = collision::ShapeSphere(2.0f);

			auto r = RunSphereVsSphere(
				sphere_a, 10.0f, v4{-5, 0, 0, 1}, v4{+3, 0, 0, 0},
				sphere_b, 10.0f, v4{+5, 0, 0, 1}, v4{-3, 0, 0, 0});

			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.after.total_lin_momentum, r.before.total_lin_momentum, 0.001f));
			PR_EXPECT(FEqlRelative(r.after.total_ke, r.before.total_ke, 0.001f));

			// Equal mass head-on: velocities swap regardless of sphere size
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.001f));
			PR_EXPECT(FEqlRelative(r.vel_b.lin.x, +3.0f, 0.001f));
		}
	};
}
