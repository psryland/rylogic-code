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

				// Total angular momentum about the world origin:
				// L_total = (L_spin_a + com_a x p_a) + (L_spin_b + com_b x p_b)
				// Momentum is stored at the CoM, so use CoM positions for orbital terms.
				auto La = a.MomentumWS().ang + Cross(a.CentreOfMassWS(), a.MomentumWS().lin);
				auto Lb = b.MomentumWS().ang + Cross(b.CentreOfMassWS(), b.MomentumWS().lin);
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
			physics::MaterialMap materials;
			physics::Engine engine(materials);

			auto& mat = materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_elasticity_tang = 0.0f;
			mat.m_elasticity_tors = 0.0f;
			mat.m_friction_static = 0.0f;

			engine.Broadphase().Add(body_a);
			engine.Broadphase().Add(body_b);

			// Capture the "before" state on the step where collision is first detected,
			// before the impulse is applied.
			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				if (args.m_contacts.empty())
					return;

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

	// ===== Polytope inertia and dynamics tests =====
	// Validates that polytope mass properties (inertia, CoM) are physically correct
	// and that polytope rigid bodies behave correctly during integration and collision.
	PRUnitTestClass(PolytopeInertiaTests)
	{
		// Build the tetrahedron from polytope_drop.json and validate its mass properties
		PRUnitTestMethod(TetrahedronMassProperties)
		{
			// Same vertices as polytope_drop.json
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};

			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			// Compute mass properties (density=1 to get unit inertia, then override mass)
			auto mp = physics::CalcMassProperties(poly, 1.0f);
			mp.m_mass = 10.0f;

			// Centre of mass should be at the centroid of the tetrahedron
			auto expected_com = 0.25f * (pts[0] + pts[1] + pts[2] + pts[3]);
			expected_com.w = 0;
			PR_EXPECT(FEql(mp.m_centre_of_mass, expected_com));

			// Unit inertia at origin should be valid
			auto inertia_at_origin = physics::Inertia{mp.m_os_unit_inertia, mp.m_mass};
			PR_EXPECT(inertia_at_origin.Check());

			// Translate to CoM frame
			auto inertia = inertia_at_origin;
			if (mp.m_centre_of_mass != v4{})
				inertia = physics::Translate(inertia, mp.m_centre_of_mass, physics::ETranslateInertia::TowardCoM);

			// CoM inertia should still be valid
			PR_EXPECT(inertia.Check());

			// CoM inertia diagonals should be positive
			auto Ic = inertia.Ic3x3(1);
			PR_EXPECT(Ic.x.x > 0);
			PR_EXPECT(Ic.y.y > 0);
			PR_EXPECT(Ic.z.z > 0);

			// Inverse should also be valid
			auto inv = physics::Invert(inertia);
			PR_EXPECT(inv.Check());

			// InertiaInv CoM should be zero (we're storing at CoM)
			PR_EXPECT(inv.CoM() == v4{});

			// Roundtrip: Invert(Invert(I)) ≈ I
			auto roundtrip = physics::Invert(inv);
			PR_EXPECT(FEqlRelative(roundtrip, inertia, 0.001f));

			// Print diagnostic info
			printf("  Polytope CoM: (%.4f, %.4f, %.4f)\n", mp.m_centre_of_mass.x, mp.m_centre_of_mass.y, mp.m_centre_of_mass.z);
			printf("  Volume-mass: %.4f, Override-mass: %.4f\n", physics::CalcMassProperties(poly, 1.0f).m_mass, mp.m_mass);
			printf("  Ic diag: (%.6f, %.6f, %.6f)\n", Ic.x.x, Ic.y.y, Ic.z.z);
			printf("  InvMass: %.6f\n", inv.InvMass());
		}

		// Create a RigidBody with the polytope shape and verify it's well-formed
		PRUnitTestMethod(TetrahedronRigidBody)
		{
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};

			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			// This is the exact path the sandbox takes
			physics::RigidBody rb;
			rb.Shape(collision::shape_cast(&poly), 10.0f);

			PR_EXPECT(rb.Mass() > 0.0f);
			PR_EXPECT(rb.Mass() < physics::InfiniteMass);
			PR_EXPECT(rb.InertiaInvOS().Check());

			// CoM should be non-zero for this asymmetric tetrahedron
			auto com = rb.CentreOfMassOS();
			printf("  RB CoM: (%.4f, %.4f, %.4f)\n", com.x, com.y, com.z);

			// Verify the inverse inertia produces reasonable angular velocities
			// Give it unit angular momentum and check the resulting omega isn't extreme
			rb.VelocityWS(v4{}, v4{0, 0, 0, 0});
			rb.MomentumWS(v8force{v4{1, 0, 0, 0}, v4{}});
			auto vel = rb.VelocityWS();
			printf("  h_ang=(1,0,0) -> omega=(%.4f, %.4f, %.4f)\n", vel.ang.x, vel.ang.y, vel.ang.z);
			auto omega_mag = Length(vel.ang);
			PR_EXPECT(omega_mag > 0.0f);
			PR_EXPECT(omega_mag < 100.0f); // Sanity: 1 unit momentum shouldn't give extreme omega
		}

		// Test free-flight integration: a polytope with angular velocity but no forces
		// should conserve kinetic energy and angular momentum exactly
		PRUnitTestMethod(FreeFlightEnergyConservation)
		{
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};

			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			physics::RigidBody rb;
			rb.Shape(collision::shape_cast(&poly), 10.0f);			rb.O2W(m4x4::Translation(v4{0, 0, 5, 0}));
			rb.VelocityWS(v4{0.5f, 0.3f, 0.0f, 0.0f}, v4{0, 0, 0, 0});

			auto ke0 = rb.KineticEnergy();
			auto h0 = rb.MomentumWS();
			printf("  Initial KE: %.6f\n", ke0);
			printf("  Initial h_ang: (%.4f, %.4f, %.4f)\n", h0.ang.x, h0.ang.y, h0.ang.z);
			printf("  Initial h_lin: (%.4f, %.4f, %.4f)\n", h0.lin.x, h0.lin.y, h0.lin.z);

			// Evolve for 1000 steps with no forces
			auto const dt = 1.0f / 100.0f;
			float max_ke_drift = 0.0f;
			float max_h_drift = 0.0f;
			for (int step = 0; step != 1000; ++step)
			{
				rb.ZeroForces();
				physics::Evolve(rb, dt);

				auto ke = rb.KineticEnergy();
				auto h = rb.MomentumWS();
				auto ke_drift = Abs(ke - ke0) / (ke0 + 1e-10f);
				auto h_ang_drift = Length(h.ang - h0.ang);
				auto h_lin_drift = Length(h.lin - h0.lin);
				max_ke_drift = std::max(max_ke_drift, ke_drift);
				max_h_drift = std::max(max_h_drift, h_ang_drift + h_lin_drift);

				// Check o2w stays orthonormal
				PR_EXPECT(IsOrthonormal(rb.O2W()));
			}

			printf("  After 1000 steps:\n");
			printf("  Final KE: %.6f (drift: %.6f%%)\n", rb.KineticEnergy(), max_ke_drift * 100.0f);
			printf("  Max KE drift: %.6f%%\n", max_ke_drift * 100.0f);
			printf("  Max momentum drift: %.6f\n", max_h_drift);

			// Symplectic integrator should conserve energy to within ~1%
			PR_EXPECT(max_ke_drift < 0.01f);
			// Momentum should be exactly conserved (no forces applied)
			PR_EXPECT(max_h_drift < 1e-4f);
		}

		// Helper: Drop a shape onto a ground plane with elastic collision and measure
		// total mechanical energy conservation. Returns the max drift fraction.
		// Measures energy changes separately for integration and collision phases.
		// When use_gpu is true, uses the GPU compute shader for integration.
		static float RunDropTest(
			char const* label,
			collision::Shape const* shape,
			float mass,
			v4 ang_vel,
			float drop_height = 5.0f,
			int num_steps = 2000)
		{
			auto ground_shape = pr::collision::ShapeBox(v4{100, 100, 0.5f, 0});

			physics::RigidBody bodies[2];

			// Dynamic body: at given height with given angular velocity
			bodies[0].Shape(shape, mass);
			bodies[0].O2W(m4x4::Translation(v4{0, 0, drop_height, 0}));
			bodies[0].VelocityWS(ang_vel, v4{0, 0, 0, 0});

			// Ground: infinite mass, top surface at z=0
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(v4{0, 0, -0.5f, 0}));

			physics::MaterialMap materials;
			physics::Engine engine(materials);

			auto& mat = materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_friction_static = 0.0f;

			engine.Broadphase().Add(bodies[0]);
			engine.Broadphase().Add(bodies[1]);

			auto const g = 9.81f;
			auto const gravity = v4{0, 0, -g, 0};
			auto const dt = 1.0f / 100.0f;
			auto const m = bodies[0].Mass();
			auto const com_os = bodies[0].CentreOfMassOS();

			// Total mechanical energy: KE + m*g*h (using CoM world height)
			auto TotalEnergy = [&]()
			{
				auto com_z = bodies[0].O2W().pos.z + (bodies[0].O2W().rot * com_os).z;
				return bodies[0].KineticEnergy() + m * g * com_z;
			};

			auto E0 = TotalEnergy();
			float max_energy_drift = 0.0f;
			int collision_count = 0;

			// Capture energy at the moment between integration and collision resolution.
			// PostCollisionDetection fires AFTER Evolve() but BEFORE ResolveCollision().
			float E_at_callback = 0.0f;
			bool had_collision = false;
			float sum_integration_delta = 0.0f;  // total ΔE from integration phases (signed)
			float sum_collision_delta = 0.0f;     // total ΔE from collision resolution (signed)
			float max_collision_delta = 0.0f;

			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				E_at_callback = TotalEnergy();
				if (!args.m_contacts.empty())
				{
					had_collision = true;
					++collision_count;
				}
			};

			for (int step = 0; step != num_steps; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				auto ws_com = bodies[0].O2W().rot * com_os;
				bodies[0].ApplyForceWS(gravity * m, v4::Zero(), ws_com);

				auto E_before = TotalEnergy();
				had_collision = false;
				E_at_callback = 0;

				engine.Step(dt, bodies);

				auto E_after = TotalEnergy();

				if (had_collision && E_at_callback != 0)
				{
					// Split: integration put us from E_before → E_at_callback
					//        collision moved us from E_at_callback → E_after
					auto int_delta = E_at_callback - E_before;
					auto col_delta = E_after - E_at_callback;
					sum_integration_delta += int_delta;
					sum_collision_delta += col_delta;
					max_collision_delta = std::max(max_collision_delta, Abs(col_delta));
				}
				else
				{
					// Non-collision step: all energy change is from integration
					sum_integration_delta += (E_after - E_before);
				}

				auto drift = Abs(E_after - E0) / (Abs(E0) + 1e-10f);
				max_energy_drift = std::max(max_energy_drift, drift);
			}

			auto E_final = TotalEnergy();
			printf("  %s: E0=%.4f Ef=%.4f drift=%.2f%% cols=%d "
				"int_sum=%.4f col_sum=%.4f col_max=%.6f CoM=(%.3f,%.3f,%.3f)\n",
				label, E0, E_final, max_energy_drift * 100.0f, collision_count,
				sum_integration_delta, sum_collision_delta, max_collision_delta,
				com_os.x, com_os.y, com_os.z);
			return max_energy_drift;
		}

		// Test: tetrahedron (original polytope_drop.json shape) — off-centre CoM
		PRUnitTestMethod(PolytopeDropOnGround)
		{
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("Tetra", collision::shape_cast(&poly), 10.0f,
				v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// Test: same tetrahedron but with NO angular velocity.
		// If this passes, energy injection is related to rotational coupling at off-centre CoM.
		PRUnitTestMethod(PolytopeDropNoSpin)
		{
			v4 pts[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("TetraNoSpin", collision::shape_cast(&poly), 10.0f,
				v4{0, 0, 0, 0});
			PR_EXPECT(drift < 0.05f);
		}

		// Test: centred tetrahedron — shift vertices so CoM is at origin.
		// Isolates whether the bug is specific to off-centre CoM.
		PRUnitTestMethod(CentredPolytopeDropOnGround)
		{
			// Shift original tetrahedron vertices so CoM is at origin
			v4 raw[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};

			// Compute the centroid (average of vertices) and shift
			auto centroid = (raw[0] + raw[1] + raw[2] + raw[3]) / 4.0f;
			centroid.w = 0;
			v4 pts[4];
			for (int i = 0; i != 4; ++i)
				pts[i] = raw[i] - centroid;

			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("CentredTetra", collision::shape_cast(&poly), 10.0f,
				v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// CRITICAL DIAGNOSTIC: centred tetrahedron with NO angular velocity.
		// With CoM at origin and no spin, this should behave identically to a box
		// (purely linear bounce). If this fails, the issue is in collision geometry
		// or the GJK/SAT contact computation for polytopes.
		PRUnitTestMethod(CentredPolytopeNoSpinDrop)
		{
			v4 raw[] = {
				v4{-0.8f, -0.8f, -0.5f, 1},
				v4{ 0.8f, -0.8f, -0.5f, 1},
				v4{ 0.0f,  0.8f, -0.5f, 1},
				v4{ 0.0f,  0.0f,  0.8f, 1},
			};
			auto centroid = (raw[0] + raw[1] + raw[2] + raw[3]) / 4.0f;
			centroid.w = 0;
			v4 pts[4];
			for (int i = 0; i != 4; ++i)
				pts[i] = raw[i] - centroid;

			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("CentredNoSpin", collision::shape_cast(&poly), 10.0f,
				v4{0, 0, 0, 0});
			PR_EXPECT(drift < 0.05f);
		}

		// Test: irregular hexahedron (8 vertices, non-uniform, off-centre CoM).
		// A more complex shape to stress-test collision energy conservation.
		PRUnitTestMethod(IrregularHexahedronDrop)
		{
			v4 pts[] = {
				v4{-0.6f, -0.4f, -0.3f, 1},
				v4{ 0.7f, -0.5f, -0.4f, 1},
				v4{ 0.5f,  0.6f, -0.2f, 1},
				v4{-0.4f,  0.7f, -0.5f, 1},
				v4{-0.3f, -0.2f,  0.6f, 1},
				v4{ 0.4f, -0.3f,  0.5f, 1},
				v4{ 0.3f,  0.4f,  0.7f, 1},
				v4{-0.5f,  0.3f,  0.4f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("IrregHex", collision::shape_cast(&poly), 10.0f,
				v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// Test: wedge shape (triangular prism) — long and asymmetric, large CoM offset.
		// Large lever arms should amplify any errors in the impulse calculation.
		PRUnitTestMethod(WedgeDropOnGround)
		{
			v4 pts[] = {
				v4{-1.0f, -0.3f, -0.2f, 1},
				v4{ 1.0f, -0.3f, -0.2f, 1},
				v4{ 0.0f,  0.6f, -0.2f, 1},
				v4{-1.0f, -0.3f,  0.2f, 1},
				v4{ 1.0f, -0.3f,  0.2f, 1},
				v4{ 0.0f,  0.6f,  0.2f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("Wedge", collision::shape_cast(&poly), 10.0f,
				v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// Test: flat pancake shape — extreme aspect ratio, CoM well off-centre.
		// Stresses the inertia tensor asymmetry (one principal moment much larger).
		PRUnitTestMethod(PancakeDropOnGround)
		{
			v4 pts[] = {
				v4{-1.0f,  0.0f, -0.05f, 1},
				v4{ 0.5f, -0.87f, -0.05f, 1},
				v4{ 0.5f,  0.87f, -0.05f, 1},
				v4{-1.0f,  0.0f,  0.05f, 1},
				v4{ 0.5f, -0.87f,  0.05f, 1},
				v4{ 0.5f,  0.87f,  0.05f, 1},
				v4{ 0.0f,  0.0f,  0.3f, 1},  // bump on top → off-centre CoM
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("Pancake", collision::shape_cast(&poly), 10.0f,
				v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// Control test: box (CoM at origin) should conserve energy well.
		// If this passes but polytope tests fail, the bug is specific to
		// off-centre CoM handling in the collision code.
		PRUnitTestMethod(BoxDropOnGround)
		{
			auto box_shape = pr::collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});

			auto drift = RunDropTest("Box", collision::shape_cast(&box_shape), 10.0f, v4{0.5f, 0.3f, 0.0f, 0.0f});
			PR_EXPECT(drift < 0.05f);
		}

		// ---- GPU integration tests ----
		// These repeat key energy conservation tests using the GPU compute shader.
		// The GPU shader mirrors the CPU Evolve() function — same Störmer-Verlet
		// with midpoint rotation predictor. These tests verify the HLSL implementation
		// matches the CPU path to within acceptable tolerance.

		PRUnitTestMethod(GpuBoxDrop)
		{
			auto box_shape = pr::collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});

			auto drift = RunDropTest("GPU-Box", collision::shape_cast(&box_shape), 10.0f, v4{0.5f, 0.3f, 0.0f, 0.0f}, 5.0f, 2000);
			PR_EXPECT(drift < 0.05f);
		}

		PRUnitTestMethod(GpuTetraDrop)
		{
			v4 pts[] = {
				v4{0, 0.8f, 0, 1},
				v4{0.75f, -0.4f, 0, 1},
				v4{-0.375f, -0.4f, 0.65f, 1},
				v4{-0.375f, -0.4f, -0.65f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("GPU-Tetra", collision::shape_cast(&poly), 10.0f, v4{0.0f, 0.0f, 0.5f, 0.0f}, 5.0f, 2000);
			PR_EXPECT(drift < 0.05f);
		}

		PRUnitTestMethod(GpuWedgeDrop)
		{
			v4 pts[] = {
				v4{-1.0f, -0.5f, -0.3f, 1},
				v4{ 1.0f, -0.5f, -0.3f, 1},
				v4{ 0.0f,  0.5f, -0.3f, 1},
				v4{-1.0f, -0.5f,  0.3f, 1},
				v4{ 1.0f, -0.5f,  0.3f, 1},
				v4{ 0.0f,  0.5f,  0.3f, 1},
			};
			auto buf = pr::collision::BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<pr::collision::ShapePolytope>();

			auto drift = RunDropTest("GPU-Wedge", collision::shape_cast(&poly), 10.0f, v4{0.0f, 0.0f, 0.5f, 0.0f}, 5.0f, 2000);
			PR_EXPECT(drift < 0.05f);
		}
	};
}
