//*********************************************
// Physics Collision Pair Tests
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
#include "pr/physics/shape/shape_builder.h"
#include "src/compute/physics_types.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
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

	// Check two bodies have matching externally visible state after stepping through different engine entry points.
	void ExpectSameRigidBodyState(RigidBody const& lhs, RigidBody const& rhs)
	{
		auto const lhs_o2w = lhs.O2W();
		auto const rhs_o2w = rhs.O2W();
		auto const pos_delta = Length(lhs_o2w.pos - rhs_o2w.pos);
		auto const x_delta = Length(lhs_o2w.x - rhs_o2w.x);
		auto const y_delta = Length(lhs_o2w.y - rhs_o2w.y);
		auto const z_delta = Length(lhs_o2w.z - rhs_o2w.z);

		auto const lhs_vel = lhs.VelocityWS();
		auto const rhs_vel = rhs.VelocityWS();
		auto const lin_delta = Length(lhs_vel.lin - rhs_vel.lin);
		auto const ang_delta = Length(lhs_vel.ang - rhs_vel.ang);

		PR_EXPECT(pos_delta < 1e-5f);
		PR_EXPECT(x_delta < 1e-5f);
		PR_EXPECT(y_delta < 1e-5f);
		PR_EXPECT(z_delta < 1e-5f);
		PR_EXPECT(lin_delta < 1e-5f);
		PR_EXPECT(ang_delta < 1e-5f);
	}

	PRUnitTestClass(SleepingBroadphaseTests)
	{
		PRUnitTestMethod(ExternalForceEventRunsBeforeIntegrate)
		{
			auto box = MakeBox();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;
			auto const time_s = 123.25;

			RigidBody body;
			body.Shape(collision::shape_cast(&box), 1.0f);
			body.O2W(m4x4::Translation(0, 0, 5));

			ResetEngineForNextTest(engine);
			auto event_count = 0;
			engine.ExternalForces += [&](Engine&, Engine::ExternalForceArgs const& args)
			{
				++event_count;
				PR_EXPECT(args.m_bodies != nullptr);
				PR_EXPECT(args.m_body_count == 1);
				PR_EXPECT(args.m_dt == dt);
				PR_EXPECT(args.m_time_s == time_s);
				PR_EXPECT(args.m_substep_index == 0);
				PR_EXPECT(args.m_substep_count == 1);

				args.m_job.m_barriers.UAV(args.m_bodies);
				args.m_job.m_barriers.Commit();
			};

			engine.Step(dt, std::span{ &body, 1 }, time_s);

			PR_EXPECT(event_count == 1);
		}

		PRUnitTestMethod(SplitStepMatchesStepWrapper)
		{
			auto box = MakeBox();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;
			auto const time_s = 12.5;

			auto step_body = RigidBody{};
			step_body.Shape(collision::shape_cast(&box), 1.0f);
			step_body.O2W(m4x4::Translation(0, 0, 5));
			step_body.VelocityWS(v4{0.2f, -0.1f, 0.05f, 0}, v4{1.0f, 0.5f, -0.25f, 0});

			auto split_body = step_body;

			ResetEngineForNextTest(engine);
			engine.Step(dt, std::span{ &step_body, 1 }, time_s);

			ResetEngineForNextTest(engine);
			engine.BeginStep(dt, std::span{ &split_body, 1 }, time_s);
			engine.CompleteStep();

			ExpectSameRigidBodyState(step_body, split_body);
		}

		PRUnitTestMethod(SplitStepGuardsPendingState)
		{
			auto box = MakeBox();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;

			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), 1.0f);
			body.O2W(m4x4::Translation(0, 0, 5));
			body.VelocityWS(v4::Zero(), v4{1.0f, 0, 0, 0});

			ResetEngineForNextTest(engine);
			engine.BeginStep(dt, std::span{ &body, 1 });

			PR_THROWS(engine.BeginStep(dt, std::span{ &body, 1 }), std::exception);
			PR_THROWS(engine.Material(physics::Material{ .m_id = physics::Material::DefaultID }), std::exception);
			PR_THROWS(engine.ResetCaches(), std::exception);

			engine.CompleteStep();
			PR_THROWS(engine.CompleteStep(), std::exception);
		}

		PRUnitTestMethod(EmptySplitStepStillRequiresComplete)
		{
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;

			ResetEngineForNextTest(engine);
			engine.BeginStep(dt, std::span<RigidBody>{});

			PR_THROWS(engine.BeginStep(dt, std::span<RigidBody>{}), std::exception);

			engine.CompleteStep();
			PR_EXPECT(engine.LastStepProfile().m_gpu_run_ms == 0.0);
		}

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

		// Check that steady support impulses do not keep resetting the sleep timer for an already-awake body.
		PRUnitTestMethod(RestSupportImpulsesDoNotPreventSleep)
		{
			auto box = MakeBox();
			auto ground_shape = MakeGround();
			auto& engine = SharedEngine();
			auto const dt = 1.0f / 60.0f;
			auto const gravity = v4{ 0, 0, -9.81f, 0 };

			RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box), 1.0f);
			bodies[0].O2W(m4x4::Translation(0, 0, 0.5f));
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

			ResetEngineForNextTest(engine);
			engine.Material(physics::Material{
				.m_id = physics::Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			// Keep gravity applied through the dedicated gravity path so the ground contact must generate support impulses while the body settles into sleep.
			for (int step = 0; step != 180; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				bodies[0].GravityWS(gravity);
				engine.Step(dt, bodies);
			}

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

	// Backing store for a compound shape, plus the mass properties produced when it was built.
	// The buffer must outlive every RigidBody that references the shape.
	struct CompoundShape
	{
		byte_data<16> m_data;
		MassProperties m_mp = {};
		v4 m_model_to_com = {};

		// The root shape, which 'ShapeBuilder' always writes at the start of the buffer.
		collision::Shape const* shape() const
		{
			return &m_data.at_byte_ofs<collision::Shape>(0);
		}
	};

	// One box child of a compound, described in the frame the compound is declared in.
	struct ChildBox
	{
		v4 m_dim;
		v4 m_pos;
		int m_mat_id;
	};

	// Build a compound of box children. The children keep their declaration order, which is the
	// order the GPU child indices are assigned in.
	void BuildCompoundBoxes(CompoundShape& out, std::span<ChildBox const> children)
	{
		ShapeBuilder sb;
		for (auto const& child : children)
			sb.AddShape(collision::ShapeBox(child.m_dim, m4x4::Translation(child.m_pos.w1()), child.m_mat_id));

		sb.BuildShape(out.m_data, out.m_mp, out.m_model_to_com);
	}

	// Build a symmetric two-box compound: child 0 at -x, child 1 at +x. Each child is a 0.5 cube, so
	// the compound spans [-half_sep - 0.25, +half_sep + 0.25] in x and [-0.25, +0.25] in y and z.
	void BuildDumbbell(CompoundShape& out, float half_sep = 0.5f, int mat0 = 0, int mat1 = 0)
	{
		ChildBox const children[] = {
			{.m_dim = v4{0.5f, 0.5f, 0.5f, 0}, .m_pos = v4{-half_sep, 0, 0, 0}, .m_mat_id = mat0},
			{.m_dim = v4{0.5f, 0.5f, 0.5f, 0}, .m_pos = v4{+half_sep, 0, 0, 0}, .m_mat_id = mat1},
		};
		BuildCompoundBoxes(out, children);
	}

	// Compound (collision::ShapeArray) bodies expand into one convex proxy per child on the GPU.
	// These tests cover the identity, transform, material and capacity behaviour of that expansion,
	// and that single-shape bodies are unaffected by it.
	PRUnitTestClass(CompoundShapeTests)
	{
		// Collect the contacts reported for a single step, so the child identity can be inspected.
		static std::vector<RbContact> StepAndCollectContacts(physics::Engine& engine, std::span<RigidBody> bodies, int steps = 1)
		{
			auto contacts = std::vector<RbContact>{};
			engine.Collisions += [&](auto&, std::span<RbContact const> step_contacts)
			{
				contacts.insert(contacts.end(), step_contacts.begin(), step_contacts.end());
			};

			auto const dt = 1.0f / 60.0f;
			for (int step = 0; step != steps; ++step)
				engine.Step(dt, bodies);

			return contacts;
		}

		// A single convex shape must pack as one entry with no child data at all.
		PRUnitTestMethod(SingleShapeFastPathHasNoChildData)
		{
			auto box = collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0});

			auto shapes = std::vector<GpuShape>{};
			auto verts = std::vector<v4>{};
			auto root = PackShapeTree(box.m_base, shapes, verts);

			PR_EXPECT(root == 0);
			PR_EXPECT(std::ssize(shapes) == 1);
			PR_EXPECT(shapes[0].type == static_cast<int>(collision::EShape::Box));
			PR_EXPECT(shapes[0].child_count == 0);
			PR_EXPECT(shapes[0].child_offset == 0);
		}

		// A compound packs as a root entry followed by a contiguous run of convex leaves.
		PRUnitTestMethod(CompoundPacksRootThenLeaves)
		{
			CompoundShape dumbbell;
			BuildDumbbell(dumbbell, 0.5f, 7, 9);

			auto shapes = std::vector<GpuShape>{};
			auto verts = std::vector<v4>{};
			auto root = PackShapeTree(*dumbbell.shape(), shapes, verts);

			PR_EXPECT(root == 0);
			PR_EXPECT(std::ssize(shapes) == 3);
			PR_EXPECT(shapes[0].type == static_cast<int>(collision::EShape::Array));
			PR_EXPECT(shapes[0].child_offset == 1);
			PR_EXPECT(shapes[0].child_count == 2);

			// The leaves keep declaration order, so the -x child is index 0 and carries the first material.
			PR_EXPECT(shapes[1].material_id == 7);
			PR_EXPECT(shapes[2].material_id == 9);
			PR_EXPECT(shapes[1].s2rb.pos.x < 0.0f);
			PR_EXPECT(shapes[2].s2rb.pos.x > 0.0f);
		}

		// Nested compounds flatten depth-first so that leaf indices follow declaration order.
		PRUnitTestMethod(NestedCompoundFlattensInDeclarationOrder)
		{
			// Layout: outer[ box(11), inner[ box(12), sphere(13) ], box(14) ]
			auto data = byte_data<16>{};
			data.push_back<collision::ShapeArray>();
			data.push_back(collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(-2.0f, 0, 0), 11));

			auto inner_ofs = data.size();
			data.push_back<collision::ShapeArray>();
			data.push_back(collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(0.0f, 0, 0), 12));
			data.push_back(collision::ShapeSphere(0.25f, m4x4::Translation(1.0f, 0, 0), false, 13));
			data.at_byte_ofs<collision::ShapeArray>(inner_ofs).Complete(2);

			data.push_back(collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(3.0f, 0, 0), 14));
			data.at_byte_ofs<collision::ShapeArray>(0).Complete(3);

			auto shapes = std::vector<GpuShape>{};
			auto verts = std::vector<v4>{};
			PackShapeTree(data.at_byte_ofs<collision::Shape>(0), shapes, verts);

			// The nested root is not emitted; only the convex leaves it contains.
			PR_EXPECT(std::ssize(shapes) == 5);
			PR_EXPECT(shapes[0].child_offset == 1);
			PR_EXPECT(shapes[0].child_count == 4);
			PR_EXPECT(shapes[1].material_id == 11);
			PR_EXPECT(shapes[2].material_id == 12);
			PR_EXPECT(shapes[3].material_id == 13);
			PR_EXPECT(shapes[4].material_id == 14);
			PR_EXPECT(shapes[3].type == static_cast<int>(collision::EShape::Sphere));
		}

		// Child identity is packed into 16 bits, so a compound larger than the supported bound must be
		// rejected rather than silently truncated to an ambiguous set of proxies.
		PRUnitTestMethod(OversizedCompoundIsRejected)
		{
			auto data = byte_data<16>{};
			data.push_back<collision::ShapeArray>();
			for (int i = 0; i != MaxCompoundChildren + 1; ++i)
				data.push_back(collision::ShapeBox(v4{0.5f, 0.5f, 0.5f, 0}, m4x4::Translation(2.0f * i, 0, 0)));

			// The largest supported compound still packs.
			data.at_byte_ofs<collision::ShapeArray>(0).Complete(MaxCompoundChildren);
			{
				auto shapes = std::vector<GpuShape>{};
				auto verts = std::vector<v4>{};
				PackShapeTree(data.at_byte_ofs<collision::Shape>(0), shapes, verts);
				PR_EXPECT(shapes[0].child_count == MaxCompoundChildren);
			}

			// One more child is over the bound and must throw.
			data.at_byte_ofs<collision::ShapeArray>(0).Complete(MaxCompoundChildren + 1);
			{
				auto shapes = std::vector<GpuShape>{};
				auto verts = std::vector<v4>{};
				PR_THROWS(PackShapeTree(data.at_byte_ofs<collision::Shape>(0), shapes, verts), std::runtime_error);
			}
		}

		// Every child of a compound that overlaps a single-shape body must produce contacts, and the
		// pair count must be exactly one per overlapping child rather than one per body.
		PRUnitTestMethod(CompoundVsSingleCollidesPerChild)
		{
			CompoundShape dumbbell;
			BuildDumbbell(dumbbell);

			// A wide, thin slab that both children of the dumbbell rest on.
			auto slab = collision::ShapeBox(v4{4.0f, 1.0f, 0.5f, 0});

			RigidBody bodies[2];
			bodies[0].Shape(dumbbell.shape(), 1.0f);
			bodies[0].O2W(m4x4::Translation(0, 0, -0.02f));
			bodies[1].Shape(collision::shape_cast(&slab), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto contacts = StepAndCollectContacts(engine, bodies);

			// Two leaves against one convex shape is exactly two broadphase pairs.
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 2);
			PR_EXPECT(!contacts.empty());

			auto seen_child0 = false;
			auto seen_child1 = false;
			for (auto const& c : contacts)
			{
				// Body 0 sorts first, so the compound is always the 'A' side of the contact.
				PR_EXPECT(c.m_objA == &bodies[0]);
				PR_EXPECT(c.m_objB == &bodies[1]);
				PR_EXPECT(c.m_child_idB == 0);
				seen_child0 |= c.m_child_idA == 0;
				seen_child1 |= c.m_child_idA == 1;
			}

			PR_EXPECT(seen_child0);
			PR_EXPECT(seen_child1);
		}

		// Compound-vs-compound must pair every child combination that overlaps and only report contacts
		// for the child pair that actually touches.
		PRUnitTestMethod(CompoundVsCompoundIdentifiesBothChildren)
		{
			CompoundShape lhs, rhs;
			BuildDumbbell(lhs);
			BuildDumbbell(rhs);

			// lhs children span x [-0.75,-0.25] and [0.25,0.75]. rhs is offset so that only its -x child
			// overlaps the +x child of lhs.
			RigidBody bodies[2];
			bodies[0].Shape(lhs.shape(), 1.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[1].Shape(rhs.shape(), 1.0f);
			bodies[1].O2W(m4x4::Translation(1.48f, 0, 0));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto contacts = StepAndCollectContacts(engine, bodies);

			PR_EXPECT(!contacts.empty());
			for (auto const& c : contacts)
			{
				PR_EXPECT(c.m_objA == &bodies[0]);
				PR_EXPECT(c.m_objB == &bodies[1]);
				PR_EXPECT(c.m_child_idA == 1);
				PR_EXPECT(c.m_child_idB == 0);
			}
		}

		// A child's shape-to-root transform and material must reach the narrowphase, so a body that can
		// only touch one child must report that child's index, material and position.
		PRUnitTestMethod(ChildTransformsAndMaterialsAreHonoured)
		{
			CompoundShape dumbbell;
			BuildDumbbell(dumbbell, 0.5f, 1, 2);

			auto ball = collision::ShapeSphere(0.24f, m4x4::Identity(), false, 3);

			RigidBody bodies[2];
			bodies[0].Shape(dumbbell.shape(), 1.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[1].Shape(collision::shape_cast(&ball), 1.0f);
			bodies[1].O2W(m4x4::Translation(0.96f, 0, 0));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			for (auto id : {1, 2, 3})
			{
				engine.Material(physics::Material{
					.m_id = id,
					.m_friction_static = 0.0f,
					.m_elasticity_norm = 1.0f,
				});
			}

			auto contacts = StepAndCollectContacts(engine, bodies);

			PR_EXPECT(!contacts.empty());
			for (auto const& c : contacts)
			{
				PR_EXPECT(c.m_child_idA == 1);
				PR_EXPECT(c.m_mat_idA == 2);
				PR_EXPECT(c.m_mat_idB == 3);

				// The contact sits out at the +x child rather than at the body origin.
				PR_EXPECT(c.Point().x > 0.5f);
			}
		}

		// Children of the same body are never paired with each other, even when they overlap.
		PRUnitTestMethod(SelfChildPairsAreExcluded)
		{
			// Two boxes whose extents overlap each other within the same compound.
			ChildBox const children[] = {
				{.m_dim = v4{0.5f, 0.5f, 0.5f, 0}, .m_pos = v4{-0.1f, 0, 0, 0}, .m_mat_id = 0},
				{.m_dim = v4{0.5f, 0.5f, 0.5f, 0}, .m_pos = v4{+0.1f, 0, 0, 0}, .m_mat_id = 0},
			};
			CompoundShape overlapping;
			BuildCompoundBoxes(overlapping, children);

			auto ground_shape = MakeGround();

			RigidBody bodies[2];
			bodies[0].Shape(overlapping.shape(), 1.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -10.0f));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto contacts = StepAndCollectContacts(engine, bodies);

			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 0);
			PR_EXPECT(contacts.empty());
		}

		// Child indices must stay bound to the same geometry for the life of the shape, because they are
		// half of the warm-start and diagnostic contact key.
		PRUnitTestMethod(ChildIdentityIsStableAcrossSteps)
		{
			CompoundShape dumbbell;
			BuildDumbbell(dumbbell);
			auto ground_shape = MakeGround();

			RigidBody bodies[2];
			bodies[0].Shape(dumbbell.shape(), 1.0f);
			bodies[0].O2W(m4x4::Translation(0, 0, -0.02f));
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Material(physics::Material{
				.m_id = physics::Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			auto seen_child0 = false;
			auto seen_child1 = false;
			auto mismatched = 0;
			engine.Collisions += [&](auto&, std::span<RbContact const> contacts)
			{
				for (auto const& c : contacts)
				{
					// Contact points are in the compound's object space, so the sign of x says which child
					// generated the contact. That must always agree with the reported child index.
					auto expected_child = c.Point().x < 0.0f ? 0 : 1;
					mismatched += static_cast<int>(c.m_child_idA != expected_child);
					seen_child0 |= c.m_child_idA == 0;
					seen_child1 |= c.m_child_idA == 1;
				}
			};

			auto const dt = 1.0f / 60.0f;
			for (int step = 0; step != 40; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				bodies[0].GravityWS(v4{0, 0, -9.81f, 0});
				engine.Step(dt, bodies);
			}

			PR_EXPECT(seen_child0);
			PR_EXPECT(seen_child1);
			PR_EXPECT(mismatched == 0);
		}

		// A resting compound must settle to sleep, and must wake again when a child is disturbed.
		PRUnitTestMethod(CompoundSleepsAtRestAndWakesOnChildContact)
		{
			CompoundShape dumbbell;
			BuildDumbbell(dumbbell);
			auto ground_shape = MakeGround();
			auto poker_shape = MakeBox();

			RigidBody bodies[3];
			bodies[0].Shape(dumbbell.shape(), 1.0f);
			bodies[0].O2W(m4x4::Translation(0, 0, 0.0f));
			bodies[1].Shape(collision::shape_cast(&ground_shape), physics::Inertia::Infinite());
			bodies[1].O2W(m4x4::Translation(0, 0, -0.5f));
			bodies[2].Shape(collision::shape_cast(&poker_shape), 1.0f);
			bodies[2].O2W(m4x4::Translation(0, 20.0f, 0));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Material(physics::Material{
				.m_id = physics::Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			auto const dt = 1.0f / 60.0f;
			for (int step = 0; step != 180; ++step)
			{
				bodies[0].ZeroForces();
				bodies[1].ZeroForces();
				bodies[2].ZeroForces();
				bodies[0].GravityWS(v4{0, 0, -9.81f, 0});
				engine.Step(dt, bodies);
			}

			PR_EXPECT(bodies[0].Sleeping());

			// Move the poker into the +x child of the sleeping compound. Waking must happen through the
			// child proxy pair, since the compound root itself is never a narrowphase operand.
			bodies[2].O2W(m4x4::Translation(0.74f, 0, 0));
			bodies[2].Sleeping(false);
			engine.Step(dt, bodies);

			PR_EXPECT(!bodies[0].Sleeping());
		}

		// Single-shape bodies must not pay for the compound path: one body pair is still one collision pair.
		PRUnitTestMethod(SingleShapeSceneKeepsOnePairPerBodyPair)
		{
			auto box = MakeBox();

			RigidBody bodies[2];
			bodies[0].Shape(collision::shape_cast(&box), 1.0f);
			bodies[0].O2W(m4x4::Identity());
			bodies[1].Shape(collision::shape_cast(&box), 1.0f);
			bodies[1].O2W(m4x4::Translation(0.25f, 0, 0));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Step(1.0f / 60.0f, bodies);

			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
		}
	};
}
#endif
