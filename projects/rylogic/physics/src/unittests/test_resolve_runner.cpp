//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/interop/resolve_runner.h"
#include <filesystem>
#include <fstream>
#include <format>

namespace pr::physics::tests
{
	void ForceLink_ResolveRunner() {}

	PRUnitTestClass(ResolveInteropRunnerTests)
	{
		PRUnitTestMethod(SplitPositionSolveSeparatesBodies)
		{
			auto config = EngineConfig{};
			config.solver_iterations = 0;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto body_a = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&box, m4x4::Translation(0.9f, 0, 0), Inertia::Box(box.m_radius, 1.0f)};
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 0),
			};

			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = v4{0.45f, 0, 0, 1},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			auto total_correction = config.position_baumgarte * (contacts[0].depth - config.position_slop);
			auto expected_shift = 0.5f * total_correction;
			PR_EXPECT(FEqlAbsolute(bodies[0].o2w.pos.x, -expected_shift, 1e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[1].o2w.pos.x, 0.9f + expected_shift, 1e-5f));
			PR_EXPECT(runner.Colours()[0] == 0);
			PR_EXPECT(runner.ContactOrder()[0] == 0);
		}

		PRUnitTestMethod(TemporalGaussSeidelRefreshesPositionCorrection)
		{
			auto config = EngineConfig{};
			config.position_iterations = 1;
			config.solver_iterations = 0;
			config.tgs_steps = 4;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto body_a = RigidBody{&box, m4x4::Identity(), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&box, m4x4::Translation(0.9f, 0, 0), Inertia::Box(box.m_radius, 1.0f)};
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 0),
			};

			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = v4{0.45f, 0, 0, 1},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			auto depth = contacts[0].depth;
			auto total_correction = 0.0f;
			for (int step = 0; step != config.tgs_steps; ++step)
			{
				auto correction = config.position_baumgarte * std::max(depth - config.position_slop, 0.0f);
				total_correction += correction;
				depth -= correction;
			}
			auto expected_shift = 0.5f * total_correction;
			PR_EXPECT(FEqlAbsolute(bodies[0].o2w.pos.x, -expected_shift, 1e-5f));
			PR_EXPECT(FEqlAbsolute(bodies[1].o2w.pos.x, 0.9f + expected_shift, 1e-5f));
		}

		PRUnitTestMethod(RejectsInvalidTemporalGaussSeidelStepCount)
		{
			auto config = EngineConfig{};
			config.tgs_steps = 0;

			auto bodies = std::vector<GpuRigidBody>{};
			auto contacts = std::vector<GpuResolveContact>{};
			auto materials = std::vector<GpuMaterial>{};
			auto runner = ResolveInteropRunner{config};

			PR_THROWS((runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			})), std::invalid_argument);
		}

		PRUnitTestMethod(SortsRestingStackContactsBottomToTop)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 0;

			auto brick_shape = collision::ShapeBox{v4{0.2f, 0.125f, 0.125f, 0}};
			auto brick0 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.125f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto brick1 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.375f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto brick2 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.625f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto gravity = v4{0, 0, -9.81f, 0};
			brick0.GravityWS(gravity);
			brick1.GravityWS(gravity);
			brick2.GravityWS(gravity);

			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(brick0, 0),
				PackDynamics(brick1, 0),
				PackDynamics(brick2, 0),
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.5f,
					.elasticity_norm = 0.0f,
				},
			};
			auto make_contact = [](RigidBody const& body_a, RigidBody const& body_b, int body_idx_a, int body_idx_b, float z)
			{
				auto w2a = InvertOrthonormal(body_a.O2W());
				auto contact_point_ws = v4{0, 0, z, 1};
				auto axis_ws = v4{0, 0, 1, 0};
				return GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = body_idx_a,
					.body_idx_b = body_idx_b,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.01f,
					.collision_time = 0,
					.feature = 1,
				};
			};

			auto contacts = std::vector<GpuResolveContact>{
				make_contact(brick1, brick2, 1, 2, 0.500f), // top contact, deliberately listed first
				make_contact(brick0, brick1, 0, 1, 0.250f), // bottom contact
				make_contact(brick0, brick2, 0, 2, 0.375f), // middle contact
			};

			auto runner = ResolveInteropRunner{config};
			runner.Load(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});
			runner.ComputeCollisionTimes();
			runner.SortContacts();

			PR_EXPECT(static_cast<int>(runner.ContactOrder()[0]) == 1);
			PR_EXPECT(static_cast<int>(runner.ContactOrder()[1]) == 2);
			PR_EXPECT(static_cast<int>(runner.ContactOrder()[2]) == 0);
		}

		PRUnitTestMethod(RotatedBodyVelocityResolveUsesContactSpace)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 8;
			config.velocity_baumgarte = 0.0f;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto ground = collision::ShapeBox{v4{10, 10, 0.5f, 0}};
			auto body_a = RigidBody{&box, m4x4::TransformDeg(10, 20, 30, v4{0, 0, 0.5f, 1}), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body_a.VelocityWS(v4::Zero(), v4{0, 0, -10.0f, 0});

			auto const w2a = InvertOrthonormal(body_a.O2W());
			auto const contact_point_ws = v4{0.15f, -0.1f, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};
			auto const ke_before = body_a.KineticEnergy();
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vel_at_contact = vel_after.LinAt(contact_point_ws - body_a.CentreOfMassWS());
			auto const closing_after = Dot3(-vel_at_contact, axis_ws);
			PR_EXPECT(vel_after.lin.z > -9.0f);
			PR_EXPECT(closing_after > -0.1f);
			PR_EXPECT(body_a.KineticEnergy() <= ke_before + 1e-4f);
		}

		PRUnitTestMethod(VelocityBiasSurvivesEnergyGuard)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0.2f;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto ground = collision::ShapeBox{v4{10, 10, 0.5f, 0}};
			auto body_a = RigidBody{&box, m4x4::Translation(0, 0, 0.5f), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};

			auto const contact_point_ws = v4{0.2f, -0.15f, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};
			auto const w2a = InvertOrthonormal(body_a.O2W());
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.1f,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			runner.Run(ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			});

			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vel_at_contact = vel_after.LinAt(contact_point_ws - body_a.CentreOfMassWS());
			auto const sep_speed = Dot3(-vel_at_contact, axis_ws);
			PR_EXPECT(sep_speed > 0.5f);
		}

		// Diagnostic: replicate the failing-frame state of BoxDropEnergyConservation around frame 100.
		// At that point the trace shows the rotated box at z=-0.32, vz=-8 m/s, with contact depth 0.55.
		// The bias should reverse vz to ~+5 m/s but it doesn't — body keeps falling.
		// This test runs ONE Resolve pass on that exact state and asserts vz separates.
		PRUnitTestMethod(BoxDropFailingFrameDiagnostic)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;     // isolate velocity solver
			config.solver_iterations = 1;       // ONE pass to see what happens
			config.velocity_baumgarte = 0.2f;
			config.penetration_slop = 0.005f;

			auto box = collision::ShapeBox{v4{0.5f, 0.65f, 0.9f, 0}};
			auto ground = collision::ShapeBox{v4{50.0f, 50.0f, 0.5f, 0}};

			// State as captured at frame ~100 of the failing trace
			auto box_o2w = m4x4::Transform(RotationRad<m3x3>(constants<float>::tau_by_8, 0, 0), v4{-3.67f, 0, -0.32f, 1});
			auto body_a = RigidBody{&box, box_o2w, Inertia::Box(box.m_radius, 10.0f)};
			body_a.VelocityWS(v4::Zero(), v4{0.8f, 0, -8.38f, 0});

			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};

			// Contact roughly matches what BoxVsBox SAT would produce: ground top normal in world.
			// Contact point = lowest box corner projected onto ground top.
			auto const contact_point_ws = v4{-3.67f, 0, 0, 1};
			auto const axis_ws = v4{0, 0, -1, 0};  // A=box → B=ground points DOWN
			auto const w2a = InvertOrthonormal(body_a.O2W());

			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.55f,
					.collision_time = 0,
					.feature = 1,
				},
			};
			contacts[0].manifold[0] = w2a * contact_point_ws;
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.3f,
				},
			};

			auto const ke_before = body_a.KineticEnergy();
			auto const vz_before = body_a.VelocityWS().lin.z;

			// Step the simulation iteration-by-iteration to see how velocity evolves
			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.SortContacts();
			runner.AssignColours();

			// Dump file
			auto dump_path = std::filesystem::path("C:/Users/paulryland/.copilot/session-state/1277a797-7dbe-4e04-9ffb-e2e396f6db2a/files/boxdrop_diag_frame.log");
			auto f = std::ofstream(dump_path);
			f << std::format("Failing-frame diagnostic: rotated box at deep penetration\n");
			f << std::format("  vz_before = {:.3f}, KE_before = {:.3f}\n", vz_before, ke_before);

			auto contact_pt_a = contacts[0].contact_point;
			auto axis_a = contacts[0].axis;
			f << std::format("  contact_pt_in_A = ({:.3f}, {:.3f}, {:.3f})\n", contact_pt_a.x, contact_pt_a.y, contact_pt_a.z);
			f << std::format("  axis_in_A      = ({:.3f}, {:.3f}, {:.3f})\n", axis_a.x, axis_a.y, axis_a.z);

			for (int iter = 0; iter != 8; ++iter)
			{
				for (int colour = 0; colour != MaxColours; ++colour)
					runner.ResolveVelocity(buffers.m_dt, colour);

				runner.Store(buffers);
				UnpackDynamics(bodies[0], body_a);
				auto v = body_a.VelocityWS();
				f << std::format("  iter {}: vz={:.3f} vx={:.3f} omega=({:.3f},{:.3f},{:.3f}) KE={:.3f}\n",
					iter, v.lin.z, v.lin.x, v.ang.x, v.ang.y, v.ang.z, body_a.KineticEnergy());
			}
			f.close();

			runner.Store(buffers);
			UnpackDynamics(bodies[0], body_a);
			auto const vel_after = body_a.VelocityWS();
			auto const vz_after = vel_after.lin.z;

			// After resolve, body should be moving UP (vz > 0) — bias should overcome gravity.
			PR_EXPECT(vz_after > 0.0f);
		}
	};

	PRUnitTestClass(ResolveGpuResolverTests)
	{
		PRUnitTestMethod(TemporalGaussSeidelPreservesAffineTransformsOnGpu)
		{
			auto config = EngineConfig{};
			config.tgs_steps = 4;

			auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
			auto ground = collision::ShapeBox{v4{10, 10, 0.5f, 0}};
			auto body_a = RigidBody{&box, m4x4::TransformDeg(5, 10, 15, v4{0, 0, 1.0f, 1}), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body_a.VelocityWS(v4{0.4f, -0.2f, 0.3f, 0}, v4{0.1f, 0.2f, -1.0f, 0});

			auto const w2a = InvertOrthonormal(body_a.O2W());
			auto const contact_point_ws = v4{0, 0, 0.5f, 1};
			auto const axis_ws = v4{0, 0, -1, 0};
			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(body_a, 0),
				PackDynamics(body_b, 1),
			};
			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = 0,
					.body_idx_b = 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.01f,
					.feature = 1,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};
			auto gpu = Gpu{};
			auto resolver = GpuResolver{gpu, config};

			resolver.Resolve(gpu.m_job, 1.0f / 60.0f, contacts, bodies, materials);

			auto dump_transform = [](GpuRigidBody const& body)
			{
				auto const& m = body.o2w;
				return std::format(
					"x=({:.9g},{:.9g},{:.9g},{:.9g}) y=({:.9g},{:.9g},{:.9g},{:.9g}) "
					"z=({:.9g},{:.9g},{:.9g},{:.9g}) w=({:.9g},{:.9g},{:.9g},{:.9g})",
					m.x.x, m.x.y, m.x.z, m.x.w,
					m.y.x, m.y.y, m.y.z, m.y.w,
					m.z.x, m.z.y, m.z.z, m.z.w,
					m.w.x, m.w.y, m.w.z, m.w.w);
			};
			if (!IsOrthonormal(bodies[0].o2w))
				throw std::runtime_error(std::format("body 0 transform is not orthonormal: {}", dump_transform(bodies[0])));
			if (!IsOrthonormal(bodies[1].o2w))
				throw std::runtime_error(std::format("body 1 transform is not orthonormal: {}", dump_transform(bodies[1])));
			if (Length(bodies[0].o2w.pos.xyz - body_a.O2W().pos.xyz) > 0.5f)
				throw std::runtime_error(std::format("body 0 translation moved implausibly: {}", dump_transform(bodies[0])));

			PR_EXPECT(IsOrthonormal(bodies[0].o2w));
			PR_EXPECT(IsOrthonormal(bodies[1].o2w));
		}

		PRUnitTestMethod(SortsRestingStackContactsBottomToTopOnGpu)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 0;

			auto brick_shape = collision::ShapeBox{v4{0.2f, 0.125f, 0.125f, 0}};
			auto brick0 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.125f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto brick1 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.375f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto brick2 = RigidBody{&brick_shape, m4x4::Translation(0, 0, 0.625f), Inertia::Box(brick_shape.m_radius, 1.0f)};
			auto gravity = v4{0, 0, -9.81f, 0};
			brick0.GravityWS(gravity);
			brick1.GravityWS(gravity);
			brick2.GravityWS(gravity);

			auto bodies = std::vector<GpuRigidBody>{
				PackDynamics(brick0, 0),
				PackDynamics(brick1, 0),
				PackDynamics(brick2, 0),
			};
			auto make_contact = [](RigidBody const& body_a, RigidBody const& body_b, int body_idx_a, int body_idx_b, float z)
			{
				auto w2a = InvertOrthonormal(body_a.O2W());
				auto contact_point_ws = v4{0, 0, z, 1};
				auto axis_ws = v4{0, 0, 1, 0};
				return GpuResolveContact{
					.axis = w2a * axis_ws,
					.contact_point = w2a * contact_point_ws,
					.b2a = w2a * body_b.O2W(),
					.body_idx_a = body_idx_a,
					.body_idx_b = body_idx_b,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.01f,
					.collision_time = 0,
					.feature = 1,
				};
			};

			auto contacts = std::vector<GpuResolveContact>{
				make_contact(brick1, brick2, 1, 2, 0.500f),
				make_contact(brick0, brick1, 0, 1, 0.250f),
				make_contact(brick0, brick2, 0, 2, 0.375f),
			};
			auto order = std::vector<uint32_t>(contacts.size());
			auto times = std::vector<float>(contacts.size());
			auto gpu = Gpu{};
			auto resolver = GpuResolver{gpu, config};

			resolver.SortContacts(gpu.m_job, 1.0f / 60.0f, contacts, bodies, order, times);

			PR_EXPECT(static_cast<int>(order[0]) == 1);
			PR_EXPECT(static_cast<int>(order[1]) == 2);
			PR_EXPECT(static_cast<int>(order[2]) == 0);
			PR_EXPECT(times[0] < times[1]);
			PR_EXPECT(times[1] < times[2]);
		}
	};
}
#endif
