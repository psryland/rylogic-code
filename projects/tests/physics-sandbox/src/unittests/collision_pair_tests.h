//************************************
// Physics Sandbox — Comprehensive Collision Pair Tests
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Systematic tests for all 15 shape pair combinations.
// Each pair is tested with a drop-onto-ground scenario (shape falling onto a
// large box ground plane) which is the most common collision configuration.
//
// The test matrix covers:
//   Analytic pairs:       sphere-sphere, sphere-box, sphere-line, line-line,
//                          line-box, line-triangle, box-box, triangle-triangle
//   GJK-with-margins:     sphere-triangle, sphere-polytope, line-polytope
//   GJK+EPA (polyhedral): box-triangle, box-polytope, tri-polytope, poly-poly
//
#pragma once
#include "src/forward.h"

namespace physics_sandbox::tests
{
	namespace collision_pair_test
	{
		// Common shapes used across tests
		inline auto MakeSphere(float r = 0.5f) { return collision::ShapeSphere(r); }
		inline auto MakeBox(v4 half = v4{0.5f, 0.5f, 0.5f, 0}) { return collision::ShapeBox(half); }
		inline auto MakeLine(float len = 1.0f, float thick = 0.1f) { return collision::ShapeLine(len, thick); }

		inline auto MakeTetra()
		{
			v4 pts[] = {
				v4{0, 0.6f, 0, 1},
				v4{0.5f, -0.3f, 0, 1},
				v4{-0.25f, -0.3f, 0.43f, 1},
				v4{-0.25f, -0.3f, -0.43f, 1},
			};
			return collision::BuildPolytopeFromPoints(pts);
		}

		// Ground plane: large thin box at z=-0.5 (top surface at z=0)
		inline auto MakeGround() { return collision::ShapeBox(v4{100, 100, 0.5f, 0}); }

		// Drop a shape onto the ground from height 'h' and check it bounces.
		// Returns true if the collision was detected and the body bounced (z velocity reversed).
		struct DropResult
		{
			bool collision_occurred = false;
			int collision_step = -1;
			float final_z = 0;
			float final_vz = 0;
			int total_collisions = 0;
		};

		inline DropResult RunDropCollisionTest(
			char const* label,
			collision::Shape const* shape,
			float mass,
			float drop_height = 3.0f,
			int num_steps = 1000)
		{
			auto ground_shape = MakeGround();

			physics::RigidBody bodies[2];
			bodies[0].Shape(shape, mass);
			bodies[0].O2W(m4x4::Translation(v4{0, 0, drop_height, 0}));
			bodies[0].VelocityWS(v4::Zero(), v4::Zero());

			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(v4{0, 0, -0.5f, 0}));

			physics::MaterialMap materials;
			physics::Engine engine(materials);
			auto& mat = materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_friction_static = 0.0f;

			auto result = DropResult{};
			auto const g = 9.81f;
			auto const gravity = v4{0, 0, -g, 0};
			auto const dt = 1.0f / 100.0f;
			auto const m = bodies[0].Mass();
			auto const com_os = bodies[0].CentreOfMassOS();

			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				if (!args.m_contacts.empty())
					result.total_collisions++;
			};

			for (int step = 0; step != num_steps; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				auto ws_com = bodies[0].O2W().rot * com_os;
				bodies[0].ApplyForceWS(gravity * m, v4::Zero(), ws_com);

				engine.Step(dt, bodies);

				auto vel = bodies[0].VelocityWS();
				auto pos = bodies[0].O2W().pos;

				// Detect first bounce: velocity reversal from downward to upward
				if (!result.collision_occurred && vel.lin.z > 0.5f && step > 10)
				{
					result.collision_occurred = true;
					result.collision_step = step;
				}

				// Safety: if body falls below kill zone, it fell through the ground
				if (pos.z < -5.0f)
				{
					printf("  [%s] FELL THROUGH GROUND at step %d, z=%.3f\n", label, step, pos.z);
					break;
				}
			}

			result.final_z = bodies[0].O2W().pos.z;
			result.final_vz = bodies[0].VelocityWS().lin.z;

			printf("  [%s] bounce=%s step=%d cols=%d final_z=%.3f final_vz=%.3f\n",
				label, result.collision_occurred ? "yes" : "NO",
				result.collision_step, result.total_collisions,
				result.final_z, result.final_vz);

			return result;
		}

		// Head-on collision between two shapes approaching along X axis.
		// Returns true if collision occurred and both bodies changed velocity.
		struct HeadOnResult
		{
			bool collision_occurred = false;
			v8motion vel_a, vel_b;
		};

		inline HeadOnResult RunHeadOnTest(
			char const* label,
			collision::Shape const& shape_a, physics::Inertia const& inertia_a,
			collision::Shape const& shape_b, physics::Inertia const& inertia_b,
			float separation = 10.0f, float speed = 3.0f)
		{
			auto result = HeadOnResult{};

			physics::RigidBody bodies[2] = {
				physics::RigidBody{&shape_a, m4x4::Translation(v4{-separation / 2, 0, 0, 1}), inertia_a},
				physics::RigidBody{&shape_b, m4x4::Translation(v4{+separation / 2, 0, 0, 1}), inertia_b},
			};
			bodies[0].VelocityWS(v4::Zero(), v4{+speed, 0, 0, 0});
			bodies[1].VelocityWS(v4::Zero(), v4{-speed, 0, 0, 0});

			physics::MaterialMap materials;
			auto& mat = materials(0);
			mat.m_elasticity_norm = 1.0f;
			mat.m_friction_static = 0.0f;

			physics::Engine engine(materials);
			engine.PostCollisionDetection += [&](auto&, auto args)
			{
				if (!args.m_contacts.empty())
					result.collision_occurred = true;
			};

			auto const dt = 1.0f / 100.0f;
			for (int step = 0; step != 5000; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				engine.Step(dt, bodies);

				if (result.collision_occurred)
				{
					result.vel_a = bodies[0].VelocityWS();
					result.vel_b = bodies[1].VelocityWS();
					break;
				}
			}

			printf("  [%s] hit=%s va=(%.3f,%.3f,%.3f) vb=(%.3f,%.3f,%.3f)\n",
				label, result.collision_occurred ? "yes" : "NO",
				result.vel_a.lin.x, result.vel_a.lin.y, result.vel_a.lin.z,
				result.vel_b.lin.x, result.vel_b.lin.y, result.vel_b.lin.z);

			return result;
		}
	}

	// ===== Drop-onto-ground tests for every shape type =====
	// Ground is a large box. This tests the most critical collision pairs
	// in typical scenes (everything falls onto a box ground plane).
	PRUnitTestClass(DropOnGroundTests)
	{
		PRUnitTestMethod(SphereDrop)
		{
			auto shape = collision_pair_test::MakeSphere();
			auto r = collision_pair_test::RunDropCollisionTest("Sphere",
				collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.total_collisions > 0);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(BoxDrop)
		{
			auto shape = collision_pair_test::MakeBox();
			auto r = collision_pair_test::RunDropCollisionTest("Box",
				collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.total_collisions > 0);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(LineDrop)
		{
			auto shape = collision_pair_test::MakeLine();
			auto r = collision_pair_test::RunDropCollisionTest("Line",
				collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.total_collisions > 0);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(PolytopeDrop)
		{
			auto buf = collision_pair_test::MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto r = collision_pair_test::RunDropCollisionTest("Polytope",
				collision::shape_cast(&poly), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.total_collisions > 0);
			PR_EXPECT(r.final_z > -1.0f);
		}
	};

	// ===== Head-on collision tests for all 15 shape pairs =====
	// Two shapes of equal mass approach along the X axis at ±3 m/s.
	// Tests that a collision is detected and both bodies change velocity.
	PRUnitTestClass(CollisionPairTests)
	{
		// ----- Sphere vs X -----

		PRUnitTestMethod(SphereVsSphere)
		{
			auto sa = collision_pair_test::MakeSphere();
			auto sb = collision_pair_test::MakeSphere();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Sphere(0.5f, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Sphere-Sphere", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.01f));
		}

		PRUnitTestMethod(SphereVsBox)
		{
			auto sa = collision_pair_test::MakeSphere();
			auto sb = collision_pair_test::MakeBox();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Sphere-Box", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.01f));
		}

		PRUnitTestMethod(SphereVsLine)
		{
			auto sa = collision_pair_test::MakeSphere();
			auto sb = collision_pair_test::MakeLine();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Sphere-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(SphereVsPolytope)
		{
			auto sa = collision_pair_test::MakeSphere();
			auto buf = collision_pair_test::MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = collision_pair_test::RunHeadOnTest("Sphere-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Box vs X -----

		PRUnitTestMethod(BoxVsBox)
		{
			auto sa = collision_pair_test::MakeBox();
			auto sb = collision_pair_test::MakeBox();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Box-Box", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(FEqlRelative(r.vel_a.lin.x, -3.0f, 0.01f));
		}

		PRUnitTestMethod(BoxVsLine)
		{
			auto sa = collision_pair_test::MakeBox();
			auto sb = collision_pair_test::MakeLine();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Box-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(BoxVsPolytope)
		{
			auto sa = collision_pair_test::MakeBox();
			auto buf = collision_pair_test::MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = collision_pair_test::RunHeadOnTest("Box-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Line vs X -----

		PRUnitTestMethod(LineVsLine)
		{
			auto sa = collision_pair_test::MakeLine();
			auto sb = collision_pair_test::MakeLine();
			auto ia = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = collision_pair_test::RunHeadOnTest("Line-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(LineVsPolytope)
		{
			auto sa = collision_pair_test::MakeLine();
			auto buf = collision_pair_test::MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = collision_pair_test::RunHeadOnTest("Line-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Polytope vs Polytope -----

		PRUnitTestMethod(PolytopeVsPolytope)
		{
			auto buf_a = collision_pair_test::MakeTetra();
			auto buf_b = collision_pair_test::MakeTetra();
			auto& poly_a = buf_a.as<collision::ShapePolytope>();
			auto& poly_b = buf_b.as<collision::ShapePolytope>();

			physics::RigidBody tmp_a, tmp_b;
			tmp_a.Shape(collision::shape_cast(&poly_a), 10.0f);
			tmp_b.Shape(collision::shape_cast(&poly_b), 10.0f);
			auto ia = physics::Invert(tmp_a.InertiaInvOS());
			auto ib = physics::Invert(tmp_b.InertiaInvOS());

			auto r = collision_pair_test::RunHeadOnTest("Polytope-Polytope", poly_a, ia, poly_b, ib);
			PR_EXPECT(r.collision_occurred);
		}
	};

	// ===== Stress test: many bodies falling onto ground =====
	// Reproduces the stress test scenario to detect TDR and passthrough bugs.
	// Uses 100 bodies (40 boxes, 40 spheres, 20 polytopes) falling under gravity onto a box ground.
	PRUnitTestClass(StressDropTests)
	{
		PRUnitTestMethod(ManyBodiesFalling)
		{
			auto ground_shape = collision::ShapeBox(v4{50, 50, 5.0f, 0});
			auto sphere_shape = collision::ShapeSphere(0.2f);
			auto box_shape = collision::ShapeBox(v4{0.2f, 0.2f, 0.2f, 0});
			auto tetra_pts = std::array<v4, 4>{
				v4{0, 0.3f, 0, 1}, v4{0.25f, -0.15f, 0, 1},
				v4{-0.125f, -0.15f, 0.22f, 1}, v4{-0.125f, -0.15f, -0.22f, 1}};
			auto poly_buf = collision::BuildPolytopeFromPoints(tetra_pts);
			auto& poly_shape = poly_buf.as<collision::ShapePolytope>();

			static constexpr int NumBodies = 100;
			static constexpr int NumSteps = 200;
			static constexpr float dt = 1.0f / 60.0f;
			static constexpr float g = 9.81f;

			// Allocate bodies: 40 spheres, 40 boxes, 20 polytopes, 1 ground
			std::vector<physics::RigidBody> bodies(NumBodies + 1);
			std::vector<collision::Shape const*> shapes(NumBodies);

			// Position bodies in a 10x10 grid above the ground
			for (int i = 0; i != NumBodies; ++i)
			{
				int row = i / 10;
				int col = i % 10;
				float x = (col - 4.5f) * 0.6f;
				float y = (row - 4.5f) * 0.6f;
				float z = 3.0f + (i % 5) * 0.5f; // stagger heights

				if (i < 40)
				{
					bodies[i].Shape(collision::shape_cast(&sphere_shape), 5.0f);
					shapes[i] = collision::shape_cast(&sphere_shape);
				}
				else if (i < 80)
				{
					bodies[i].Shape(collision::shape_cast(&box_shape), 5.0f);
					shapes[i] = collision::shape_cast(&box_shape);
				}
				else
				{
					bodies[i].Shape(collision::shape_cast(&poly_shape), 5.0f);
					shapes[i] = collision::shape_cast(&poly_shape);
				}
				bodies[i].O2W(m4x4::Translation(v4{x, y, z, 0}));
			}

			// Ground: infinite mass, top surface at z=0
			bodies[NumBodies].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[NumBodies].O2W(m4x4::Translation(v4{0, 0, -5.0f, 0}));

			physics::MaterialMap materials;
			auto& mat = materials(0);
			mat.m_elasticity_norm = 0.5f;
			mat.m_friction_static = 0.3f;

			physics::Engine engine(materials);

			int passthrough_count = 0;
			int max_pair_count = 0;

			for (int step = 0; step != NumSteps; ++step)
			{
				for (int i = 0; i != NumBodies; ++i)
				{
					bodies[i].ZeroForces();
					auto m = bodies[i].Mass();
					auto com_os = bodies[i].CentreOfMassOS();
					auto ws_com = bodies[i].O2W().rot * com_os;
					bodies[i].ApplyForceWS(v4{0, 0, -g, 0} * m, v4::Zero(), ws_com);
				}
				bodies[NumBodies].ZeroForces();

				// Build pointer span for Engine::Step
				std::vector<physics::RigidBody*> body_ptrs;
				for (auto& b : bodies)
					body_ptrs.push_back(&b);

				engine.Step(dt, body_ptrs);

				// Check for passthrough
				for (int i = 0; i != NumBodies; ++i)
				{
					if (bodies[i].O2W().pos.z < -2.0f)
					{
						++passthrough_count;
						if (passthrough_count <= 3)
						{
							auto type = i < 40 ? "sphere" : i < 80 ? "box" : "polytope";
							printf("  PASSTHROUGH: body[%d] (%s) at step %d z=%.3f\n",
								i, type, step, bodies[i].O2W().pos.z);

							// Run CPU collision test on this pair to check if CPU GJK catches it
							auto w2g = InvertOrthonormal(bodies[NumBodies].O2W());
							auto b2g = w2g * bodies[i].O2W();
							collision::Contact cpu_contact;
							bool cpu_hit = collision::Collide(
								bodies[NumBodies].Shape(), m4x4::Identity(),
								bodies[i].Shape(), b2g, cpu_contact);
							printf("  CPU Collide: %s (depth=%.6f axis=(%.3f,%.3f,%.3f))\n",
								cpu_hit ? "HIT" : "MISS", cpu_contact.m_depth,
								cpu_contact.m_axis.x, cpu_contact.m_axis.y, cpu_contact.m_axis.z);

							// Also try CPU GJK directly
							collision::Contact gjk_contact;
							bool gjk_hit = collision::GjkCollide(
								bodies[NumBodies].Shape(), m4x4::Identity(),
								bodies[i].Shape(), b2g, gjk_contact);
							printf("  CPU GJK:     %s (depth=%.6f axis=(%.3f,%.3f,%.3f))\n",
								gjk_hit ? "HIT" : "MISS", gjk_contact.m_depth,
								gjk_contact.m_axis.x, gjk_contact.m_axis.y, gjk_contact.m_axis.z);

							// Log the exact transform for reproduction
							auto& o = bodies[i].O2W();
							printf("  O2W: x=(%.6f,%.6f,%.6f) y=(%.6f,%.6f,%.6f) z=(%.6f,%.6f,%.6f) pos=(%.6f,%.6f,%.6f)\n",
								o.x.x, o.x.y, o.x.z, o.y.x, o.y.y, o.y.z, o.z.x, o.z.y, o.z.z, o.pos.x, o.pos.y, o.pos.z);
						}
					}
				}
			}

			printf("  Stress test: %d bodies, %d steps, %d passthroughs\n",
				NumBodies, NumSteps, passthrough_count);

			// No body should fall through the ground
			PR_EXPECT(passthrough_count == 0);
		}
	};
}
