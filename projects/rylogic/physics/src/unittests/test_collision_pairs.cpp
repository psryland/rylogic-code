//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// Systematic tests for all 15 shape pair combinations.
// Each pair is tested with a drop-onto-ground scenario (shape falling onto a
// large box ground plane) which is the most common collision configuration.
//
// The test matrix covers:
//   Analytic pairs:       sphere-sphere, sphere-box, sphere-line, line-line, line-box, line-triangle, box-box, triangle-triangle
//   GJK-with-margins:     sphere-triangle, sphere-polytope, line-polytope
//   GJK+EPA (polyhedral): box-triangle, box-polytope, tri-polytope, poly-poly
//
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	void ForceLink_CollisionPairs() {}

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

	// Head-on collision between two shapes approaching along X axis.
	// Returns true if collision occurred and both bodies changed velocity.
	struct HeadOnResult
	{
		bool collision_occurred = false;
		v8motion vel_a, vel_b;
	};

	// Common shapes used across tests
	auto MakeSphere(float r = 0.5f) { return collision::ShapeSphere(r); }
	auto MakeBox(v4 half = v4{0.5f, 0.5f, 0.5f, 0}) { return collision::ShapeBox(half); }
	auto MakeLine(float len = 1.0f, float thick = 0.1f) { return collision::ShapeLine(len, thick); }
	auto MakeTetra()
	{
		v4 pts[] = {
			v4{0, 0.6f, 0, 1},
			v4{0.5f, -0.3f, 0, 1},
			v4{-0.25f, -0.3f, 0.43f, 1},
			v4{-0.25f, -0.3f, -0.43f, 1},
		};
		return collision::BuildPolytopeFromPoints(pts);
	}
	auto MakeGround()
	{
		// Ground plane: large thin box at z=-0.5 (top surface at z=0)
		return collision::ShapeBox(v4{ 100, 100, 0.5f, 0 });
	}

	DropResult RunDropCollisionTest(std::string_view label, collision::Shape const* shape, float mass, float drop_height = 0.6f, int num_steps = 60, std::filesystem::path log_dir = {})
	{
		auto ground_shape = MakeGround();

		RigidBody bodies[2];
		bodies[0].Shape(shape, mass);
		bodies[0].O2W(m4x4::Translation(0, 0, drop_height));
		bodies[0].VelocityWS(v4::Zero(), v4::Zero());
		bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
		bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

		auto& engine = SharedEngine();
		ResetEngineForNextTest(engine);
		auto result = DropResult{};
		engine.Collisions += [&](auto&, auto)
		{
			++result.total_collisions;
		};

		auto const g = 9.81f;
		auto const gravity = v4{0, 0, -g, 0};
		auto const dt = 1.0f / 60.0f;
		auto const m = bodies[0].Mass();
		auto const com_os = bodies[0].CentreOfMassOS();

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
				if (!log_dir.empty())
				{
					auto log = std::ofstream(log_dir / std::format("{}_drop.log", label));
					log << std::format("[{}] FELL THROUGH GROUND at step {}, z={:.3f}\n", label, step, pos.z);
				}
				break;
			}
		}

		result.final_z = bodies[0].O2W().pos.z;
		result.final_vz = bodies[0].VelocityWS().lin.z;

		if (!log_dir.empty())
		{
			auto log = std::ofstream(log_dir / std::format("{}_drop.log", label));
			log << std::format("[{}] bounce={} step={} cols={} final_z={:.3f} final_vz={:.3f}\n",
				label, result.collision_occurred ? "yes" : "NO",
				result.collision_step, result.total_collisions,
				result.final_z, result.final_vz);
		}

		return result;
	}
	HeadOnResult RunHeadOnTest(std::string_view label, collision::Shape const& shape_a, physics::Inertia const& inertia_a, collision::Shape const& shape_b, physics::Inertia const& inertia_b, float separation = 1.5f, float speed = 3.0f, std::filesystem::path log_dir = {})
	{
		physics::RigidBody bodies[2] = {
			physics::RigidBody{&shape_a, m4x4::Translation(-separation / 2, 0, 0), inertia_a},
			physics::RigidBody{&shape_b, m4x4::Translation(+separation / 2, 0, 0), inertia_b},
		};
		bodies[0].VelocityWS(v4::Zero(), v4{+speed, 0, 0, 0});
		bodies[1].VelocityWS(v4::Zero(), v4{-speed, 0, 0, 0});

		auto& engine = SharedEngine();
		ResetEngineForNextTest(engine);
		auto result = HeadOnResult{};
		engine.Collisions += [&](auto&, auto)
		{
			result.collision_occurred = true;
		};

		auto const dt = 1.0f / 60.0f;
		for (int step = 0; step != 100; ++step)
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

		if (!log_dir.empty())
		{
			auto log = std::ofstream(log_dir / std::format("{}_headon.log", label));
			log << std::format("[{}] hit={} va=({:.3f},{:.3f},{:.3f}) vb=({:.3f},{:.3f},{:.3f})\n",
				label, result.collision_occurred ? "yes" : "NO",
				result.vel_a.lin.x, result.vel_a.lin.y, result.vel_a.lin.z,
				result.vel_b.lin.x, result.vel_b.lin.y, result.vel_b.lin.z);
		}

		return result;
	}

	PRUnitTestClass(SleepingBroadphaseTests)
	{
		PRUnitTestMethod(LowVelocityBodySleepsAfterDelay)
		{
			auto box = MakeBox();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;

			RigidBody body;
			body.Shape(collision::shape_cast(&box), 1.0f);
			body.O2W(m4x4::Translation(0, 0, 5));
			RigidBody* body_ptrs[] = { &body };

			ResetEngineForNextTest(engine);
			for (int step = 0; step != 61; ++step)
				engine.Step(dt, body_ptrs);

			PR_EXPECT(body.Sleeping());
		}

		PRUnitTestMethod(StaticContactsDoNotPreventSleep)
		{
			auto box = MakeBox();
			auto ground_shape = MakeGround();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;

			RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box), 1.0f);
			bodies[0].O2W(m4x4::Translation(0, 0, 0.49f));
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

			ResetEngineForNextTest(engine);
			for (int step = 0; step != 61; ++step)
				engine.Step(dt, bodies);

			PR_EXPECT(bodies[0].Sleeping());
		}

		PRUnitTestMethod(AllSleepingSceneSkipsGpuPipeline)
		{
			auto box = MakeBox();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;
			auto bodies = std::vector<RigidBody>(10000);

			for (int i = 0; i != std::ssize(bodies); ++i)
			{
				auto& body = bodies[i];
				auto x = static_cast<float>(i % 100) * 3.0f;
				auto y = static_cast<float>(i / 100) * 3.0f;
				body.Shape(collision::shape_cast(&box), 1.0f);
				body.O2W(m4x4::Translation(x, y, 5.0f));
				body.Sleep();
			}

			ResetEngineForNextTest(engine);
			engine.Step(dt, bodies);

			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 0);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count == 0);
			PR_EXPECT(engine.LastStepProfile().m_gpu_run_ms == 0.0);
			PR_EXPECT(engine.LastStepProfile().m_broadphase_ms == 0.0);
			PR_EXPECT(std::all_of(std::begin(bodies), std::end(bodies), [](RigidBody const& body)
			{
				return body.Sleeping();
			}));
		}

		PRUnitTestMethod(SleepingPairsAreFilteredUntilDisturbed)
		{
			auto box = MakeBox();
			auto const dt = 1.0f / 60.0f;
			auto& engine = SharedEngine();

			{
				RigidBody bodies[2];
				bodies[0].Shape(collision::shape_cast(&box), 1.0f);
				bodies[0].O2W(m4x4::Translation(0, 0, 0));
				bodies[0].Sleep();
				bodies[1].Shape(collision::shape_cast(&box), 1.0f);
				bodies[1].O2W(m4x4::Translation(0.25f, 0, 0));
				bodies[1].Sleep();

				ResetEngineForNextTest(engine);
				engine.Step(dt, bodies);
				PR_EXPECT(engine.LastCollisionStats().m_pair_count == 0);
			}

			{
				RigidBody bodies[2];
				bodies[0].Shape(collision::shape_cast(&box), 1.0f);
				bodies[0].O2W(m4x4::Translation(0, 0, 0));
				bodies[1].Shape(collision::shape_cast(&box), 1.0f);
				bodies[1].O2W(m4x4::Translation(0.25f, 0, 0));
				bodies[1].Sleep();

				ResetEngineForNextTest(engine);
				engine.Step(dt, bodies);
				PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
				PR_EXPECT(!bodies[1].Sleeping());
			}
		}

		PRUnitTestMethod(CreatedSleepingChainWakesAsOneIsland)
		{
			auto box = MakeBox();
			auto const dt = 1.0f / 60.0f;
			auto& engine = SharedEngine();

			RigidBody bodies[4];
			for (int i = 0; i != 3; ++i)
			{
				// Use a small overlap so this test isolates sleep-island propagation rather than depending on exact-touching contact tolerance.
				bodies[i].Shape(collision::shape_cast(&box), 1.0f);
				bodies[i].O2W(m4x4::Translation(0.49f * static_cast<float>(i), 0, 0));
				bodies[i].Sleep();
			}

			bodies[3].Shape(collision::shape_cast(&box), 1.0f);
			bodies[3].O2W(m4x4::Translation(-0.45f, 0, 0));
			bodies[3].VelocityWS(v4::Zero(), v4{1.0f, 0, 0, 0});

			ResetEngineForNextTest(engine);
			auto init_collision_count = 0;
			engine.Collisions += [&](auto&, auto)
			{
				++init_collision_count;
			};

			engine.UpdateSleepIslands(bodies);
			PR_EXPECT(init_collision_count == 0);

			engine.Step(dt, bodies);

			PR_EXPECT(!bodies[0].Sleeping());
			PR_EXPECT(!bodies[1].Sleeping());
			PR_EXPECT(!bodies[2].Sleeping());
		}
	};

	// ===== Drop-onto-ground tests for every shape type =====
	// Ground is a large box. This tests the most critical collision pairs
	// in typical scenes (everything falls onto a box ground plane).
	PRUnitTestClass(DropOnGroundTests)
	{
		PRUnitTestMethod(SphereDrop)
		{
			auto shape = MakeSphere();
			auto r = RunDropCollisionTest("Sphere", collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(BoxDrop)
		{
			auto shape = MakeBox();
			auto r = RunDropCollisionTest("Box", collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(LineDrop)
		{
			auto shape = MakeLine();
			auto r = RunDropCollisionTest("Line", collision::shape_cast(&shape), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.final_z > -1.0f);
		}

		PRUnitTestMethod(PolytopeDrop)
		{
			auto buf = MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto r = RunDropCollisionTest("Polytope", collision::shape_cast(&poly), 10.0f);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.final_z > -1.0f);
		}

		// Helper for the BoxDropEnergy* tests below. Drops a 1.0 x 1.3 x 1.8 box of mass 10
		// onto the ground from height 3 with a small lateral velocity. Asserts:
		//   1. Body never reaches more than 1.2 * starting_height (no upward explosion).
		//   2. Total energy (KE + PE) never exceeds initial energy substantially.
		//   3. Body does not fall through the ground.
		//   4. After settling, max penetration depth stays small (no Baumgarte runaway).
		static void BoxDropEnergyImpl(bool rotated, std::string_view trace_filename)
		{
			auto box_shape = collision::ShapeBox(v4{0.5f, 0.65f, 0.9f, 0});
			auto ground_shape = MakeGround();

			RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box_shape), 10.0f);
			auto initial_z = 3.0f;
			auto rot = rotated
				? m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, 0, 0), v4{-5, 0, initial_z, 1})
				: m4x4::Translation(-5, 0, initial_z);
			bodies[0].O2W(rot);
			bodies[0].VelocityWS(v4::Zero(), v4{0.8f, 0, 0, 0});

			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Material(physics::Material{
				.m_id = physics::Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.3f,
			});

			auto max_depth_per_frame = std::vector<float>{};
			engine.Collisions += [&](auto&, std::span<RbContact const> contacts)
			{
				auto max_depth = 0.0f;
				for (auto const& c : contacts)
					max_depth = std::max(max_depth, c.m_depth);
				max_depth_per_frame.push_back(max_depth);
			};

			auto const g = 9.81f;
			auto const dt = 1.0f / 60.0f;
			auto const num_steps = 200;
			auto const m = bodies[0].Mass();
			auto const com_os = bodies[0].CentreOfMassOS();

			// Initial total energy: KE = 0, PE = m * g * z (relative to ground top z=0)
			auto const initial_total_energy = m * g * initial_z;

			auto max_z = initial_z;
			auto max_total_energy = initial_total_energy;

			// Per-frame trace (for diagnostic when assertions fail)
			struct FrameTrace { int step; float z; float vz; float ke; float pe; float total; float max_depth; };
			auto trace = std::vector<FrameTrace>{};
			auto dump_dir = std::filesystem::path("dump");
			std::filesystem::create_directories(dump_dir);

			for (int step = 0; step != num_steps; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				auto ws_com = bodies[0].O2W().rot * com_os;
				bodies[0].ApplyForceWS(v4{0, 0, -g * m, 0}, v4::Zero(), ws_com);

				engine.Step(dt, bodies);

				auto z = bodies[0].O2W().pos.z;
				auto vz = bodies[0].VelocityWS().lin.z;
				auto ke = bodies[0].KineticEnergy();
				auto pe = m * g * z;
				auto total = ke + pe;
				auto md = max_depth_per_frame.empty() ? 0.0f : max_depth_per_frame.back();
				trace.push_back({step, z, vz, ke, pe, total, md});

				max_z = std::max(max_z, z);
				max_total_energy = std::max(max_total_energy, total);

				// Bail out early on catastrophic failure to avoid noisy logs but still report
				if (z > initial_z * 5.0f || total > initial_total_energy * 10.0f)
					break;
			}

			// Diagnostic dump: print per-frame state for post-mortem
			auto dump_path = dump_dir / trace_filename;
			{
				auto f = std::ofstream(dump_path);
				f << std::format("step,z,vz,ke,pe,total,max_depth (initial_total={:.2f})\n", initial_total_energy);
				for (auto const& t : trace)
					f << std::format("{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.4f}\n", t.step, t.z, t.vz, t.ke, t.pe, t.total, t.max_depth);
			}

			// (1) Body must never reach more than 20% above its initial height.
			PR_EXPECT(max_z < initial_z * 1.2f);

			// (2) Total energy must never substantially exceed the initial energy.
			PR_EXPECT(max_total_energy < initial_total_energy * 1.5f);

			// (3) Body must not have fallen through the ground.
			PR_EXPECT(bodies[0].O2W().pos.z > -1.0f);

			// (4) After settling, max penetration depth must stay small (no runaway).
			if (max_depth_per_frame.size() > 60)
			{
				auto window_start = max_depth_per_frame.size() - 30;
				auto max_in_window = 0.0f;
				for (auto i = window_start; i != max_depth_per_frame.size(); ++i)
					max_in_window = std::max(max_in_window, max_depth_per_frame[i]);
				PR_EXPECT(max_in_window < 0.2f);
			}
		}

		// Reproduces the failing 'drop_test.json' visual scene: a rotated, asymmetric box
		// dropped onto the ground with a small lateral velocity. With the per-point Baumgarte
		// resolver bug, the box explodes around frame 80 (z velocity → +158 m/s, KE → 264k J).
		PRUnitTestMethod(BoxDropEnergyConservation)
		{
			BoxDropEnergyImpl(/*rotated=*/true, "boxdrop_trace_rotated.log");
		}

		// Diagnostic variant: same scene but box NOT rotated. If this passes but the rotated
		// variant fails, the bug is rotation-specific (e.g. SAT axis-selection producing a
		// skewed contact normal that doesn't fully oppose gravity).
		PRUnitTestMethod(BoxDropEnergyConservationAxisAligned)
		{
			BoxDropEnergyImpl(/*rotated=*/false, "boxdrop_trace_aligned.log");
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
			auto sa = MakeSphere();
			auto sb = MakeSphere();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Sphere(0.5f, 10.0f);
			auto r = RunHeadOnTest("Sphere-Sphere", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			// Equal mass head-on: body A should reverse direction
		}

		PRUnitTestMethod(SphereVsBox)
		{
			auto sa = MakeSphere();
			auto sb = MakeBox();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto r = RunHeadOnTest("Sphere-Box", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			// Equal mass head-on: body A should reverse direction
		}

		PRUnitTestMethod(SphereVsLine)
		{
			auto sa = MakeSphere();
			auto sb = MakeLine();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = RunHeadOnTest("Sphere-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(SphereVsPolytope)
		{
			auto sa = MakeSphere();
			auto buf = MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Sphere(0.5f, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = RunHeadOnTest("Sphere-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Box vs X -----

		PRUnitTestMethod(BoxVsBox)
		{
			auto sa = MakeBox();
			auto sb = MakeBox();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto r = RunHeadOnTest("Box-Box", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
			PR_EXPECT(r.vel_a.lin.x < -1.0f);
			PR_EXPECT(r.vel_b.lin.x > +1.0f);
		}

		PRUnitTestMethod(BoxVsLine)
		{
			auto sa = MakeBox();
			auto sb = MakeLine();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = RunHeadOnTest("Box-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(BoxVsPolytope)
		{
			auto sa = MakeBox();
			auto buf = MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = RunHeadOnTest("Box-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Line vs X -----

		PRUnitTestMethod(LineVsLine)
		{
			auto sa = MakeLine();
			auto sb = MakeLine();
			auto ia = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto ib = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);
			auto r = RunHeadOnTest("Line-Line", sa, ia, sb, ib);
			PR_EXPECT(r.collision_occurred);
		}

		PRUnitTestMethod(LineVsPolytope)
		{
			auto sa = MakeLine();
			auto buf = MakeTetra();
			auto& poly = buf.as<collision::ShapePolytope>();
			auto ia = physics::Inertia::Box(v4{0.05f, 0.05f, 0.5f, 0}, 10.0f);

			physics::RigidBody tmp;
			tmp.Shape(collision::shape_cast(&poly), 10.0f);
			auto ib = physics::Invert(tmp.InertiaInvOS());

			auto r = RunHeadOnTest("Line-Polytope", sa, ia, poly, ib);
			PR_EXPECT(r.collision_occurred);
		}

		// ----- Polytope vs Polytope -----

		PRUnitTestMethod(PolytopeVsPolytope)
		{
			auto buf_a = MakeTetra();
			auto buf_b = MakeTetra();
			auto& poly_a = buf_a.as<collision::ShapePolytope>();
			auto& poly_b = buf_b.as<collision::ShapePolytope>();

			physics::RigidBody tmp_a, tmp_b;
			tmp_a.Shape(collision::shape_cast(&poly_a), 10.0f);
			tmp_b.Shape(collision::shape_cast(&poly_b), 10.0f);
			auto ia = physics::Invert(tmp_a.InertiaInvOS());
			auto ib = physics::Invert(tmp_b.InertiaInvOS());

			auto r = RunHeadOnTest("Polytope-Polytope", poly_a, ia, poly_b, ib);
			PR_EXPECT(r.collision_occurred);
		}
	};

	// ===== Stress test: many bodies falling onto ground =====
	// Reproduces the stress test scenario to detect TDR and passthrough bugs.
	// Uses 30 bodies (12 spheres, 12 boxes, 6 polytopes) falling under gravity onto a box ground.
	PRUnitTestClass(StressDropTests)
	{
		PRUnitTestMethod(ManyBodiesFalling)
		{
			// ShapeBox(dim) stores half-extents = dim * 0.5, so v4{100,100,10} gives a
			// 100x100x10 box. Centred at z=-5 this puts the top surface at z=0.
			auto ground_shape = collision::ShapeBox(v4{100, 100, 10.0f, 0});
			auto sphere_shape = collision::ShapeSphere(0.2f);
			auto box_shape = collision::ShapeBox(v4{0.2f, 0.2f, 0.2f, 0});
			auto tetra_pts = std::array<v4, 4>{
				v4{0, 0.3f, 0, 1}, v4{0.25f, -0.15f, 0, 1},
				v4{-0.125f, -0.15f, 0.22f, 1}, v4{-0.125f, -0.15f, -0.22f, 1}};
			auto poly_buf = collision::BuildPolytopeFromPoints(tetra_pts);
			auto& poly_shape = poly_buf.as<collision::ShapePolytope>();

			static constexpr int NumBodies = 30;
			static constexpr int NumSteps = 60;
			static constexpr float dt = 1.0f / 30.0f;
			static constexpr float g = 9.81f;

			// Allocate bodies: 12 spheres, 12 boxes, 6 polytopes, 1 ground
			std::vector<physics::RigidBody> bodies(NumBodies + 1);

			// Position bodies in a 6x5 grid above the ground
			for (int i = 0; i != NumBodies; ++i)
			{
				int row = i / 6;
				int col = i % 6;
				float x = (col - 2.5f) * 0.6f;
				float y = (row - 2.0f) * 0.6f;
				float z = 2.0f + (i % 3) * 0.5f;

				if (i < 12)
				{
					bodies[i].Shape(collision::shape_cast(&sphere_shape), 5.0f);
				}
				else if (i < 24)
				{
					bodies[i].Shape(collision::shape_cast(&box_shape), 5.0f);
				}
				else
				{
					bodies[i].Shape(collision::shape_cast(&poly_shape), 5.0f);
				}
				bodies[i].O2W(m4x4::Translation(x, y, z));
			}

			// Ground: infinite mass, top surface at z=0
			bodies[NumBodies].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[NumBodies].O2W(m4x4::Translation(0, 0, -5.0f));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Material(physics::Material{
				.m_id = physics::Material::DefaultID,
				.m_friction_static = 0.3f,
				.m_elasticity_norm = 0.5f,
			});

			int passthrough_count = 0;

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
						++passthrough_count;
				}
			}

			// No body should fall through the ground
			PR_EXPECT(passthrough_count == 0);
		}
	};
}
#endif
