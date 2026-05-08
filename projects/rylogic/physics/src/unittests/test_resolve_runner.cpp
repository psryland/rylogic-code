//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"
#include <filesystem>
#include <fstream>
#include <format>

namespace pr::physics::tests
{
	void ForceLink_ResolveRunner() {}

	PRUnitTestClass(ResolveInteropRunnerTests)
	{
		PRUnitTestMethod(ContactPriorityScoresNewtonCradleOrder)
		{
			auto settings = ContactPrioritySettings{};
			settings.m_depth_bias = 0.0f;

			auto bodies = std::vector<ContactPriorityBody>(11);
			for (int body_idx = 0; body_idx != isize(bodies); ++body_idx)
			{
				bodies[body_idx].m_position_ws = v4{static_cast<float>(body_idx), 0, 0, 1};
				bodies[body_idx].m_inv_mass = 1.0f;
			}
			bodies[0].m_velocity_ws = v4{10, 0, 0, 0};

			auto contacts = std::vector<ContactPriorityContact>{};
			for (int contact_idx = 0; contact_idx != 10; ++contact_idx)
			{
				contacts.push_back(ContactPriorityContact{
					.m_body_idx_a = contact_idx,
					.m_body_idx_b = contact_idx + 1,
					.m_axis_ws = v4{1, 0, 0, 0},
					.m_point_ws = v4{static_cast<float>(contact_idx) + 0.5f, 0, 0, 1},
				});
			}

			auto ideal_order = std::vector<int>(contacts.size());
			std::iota(ideal_order.begin(), ideal_order.end(), 0);
			auto reversed_order = ideal_order;
			std::reverse(reversed_order.begin(), reversed_order.end());

			auto const ideal = EvaluateContactPriority(settings, bodies, contacts, ideal_order);
			auto const reversed = EvaluateContactPriority(settings, bodies, contacts, reversed_order);
			auto const priority_order = ContactPriorityOrder(settings, bodies, contacts);

			PR_EXPECT(ideal.m_total_help_weight > 0.0f);
			PR_EXPECT(ideal.m_score > 0.99f);
			PR_EXPECT(reversed.m_score < 0.01f);
			PR_EXPECT(priority_order == ideal_order);
		}

		PRUnitTestMethod(ContactPriorityScoresSupportLoadOrder)
		{
			auto settings = ContactPrioritySettings{};
			settings.m_depth_bias = 0.0f;

			auto bodies = std::vector<ContactPriorityBody>(3);
			bodies[0].m_position_ws = v4{0, 0, -1, 1};
			bodies[1].m_position_ws = v4{0, 0, 0, 1};
			bodies[2].m_position_ws = v4{0, 0, 1, 1};
			bodies[0].m_inv_mass = 0.0f;
			bodies[1].m_inv_mass = 1.0f;
			bodies[2].m_inv_mass = 1.0f;
			bodies[2].m_velocity_ws = v4{0, 0, -10, 0};

			auto contacts = std::vector<ContactPriorityContact>{
				ContactPriorityContact{
					.m_body_idx_a = 1,
					.m_body_idx_b = 0,
					.m_axis_ws = v4{0, 0, -1, 0},
					.m_point_ws = v4{0, 0, -0.5f, 1},
				},
				ContactPriorityContact{
					.m_body_idx_a = 1,
					.m_body_idx_b = 2,
					.m_axis_ws = v4{0, 0, +1, 0},
					.m_point_ws = v4{0, 0, +0.5f, 1},
				},
			};

			auto load_first_order = std::vector<int>{1, 0};
			auto ground_first_order = std::vector<int>{0, 1};

			auto const load_first = EvaluateContactPriority(settings, bodies, contacts, load_first_order);
			auto const ground_first = EvaluateContactPriority(settings, bodies, contacts, ground_first_order);
			auto const priority_order = ContactPriorityOrder(settings, bodies, contacts);

			PR_EXPECT(load_first.m_total_help_weight > 0.0f);
			PR_EXPECT(load_first.m_score > 0.99f);
			PR_EXPECT(ground_first.m_score < 0.01f);
			PR_EXPECT(priority_order == load_first_order);
		}

		PRUnitTestMethod(ContactPrioritySortsInteropCradleContacts)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_shock_iterations = 16;

			auto box = collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}};
			auto rbodies = std::vector<RigidBody>{};
			for (int body_idx = 0; body_idx != 11; ++body_idx)
			{
				rbodies.push_back(RigidBody{&box, m4x4::Translation(static_cast<float>(body_idx), 0, 0), Inertia::Box(box.m_radius, 1.0f)});
				rbodies.back().VelocityWS(v4::Zero(), body_idx == 0 ? v4{10, 0, 0, 0} : v4::Zero());
			}

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != isize(rbodies); ++body_idx)
				bodies.push_back(PackDynamics(rbodies[body_idx], 0));

			auto contacts = std::vector<GpuResolveContact>{};
			for (int contact_idx = 0; contact_idx != 10; ++contact_idx)
			{
				auto const& body_a = rbodies[contact_idx];
				auto const& body_b = rbodies[contact_idx + 1];
				auto const contact_point = v4{0.5f, 0, 0, 1};
				contacts.push_back(GpuResolveContact{
					.axis = v4{1, 0, 0, 0},
					.contact_point = contact_point,
					.manifold = {contact_point},
					.b2a = InvertOrthonormal(body_a.O2W()) * body_b.O2W(),
					.body_idx_a = contact_idx,
					.body_idx_b = contact_idx + 1,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.depth = 0.0f,
					.feature = 1,
				});
			}
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.ComputeShockRanks();
			runner.SortContacts();

			for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
				PR_EXPECT(runner.ContactOrder()[contact_idx] == static_cast<uint32_t>(contact_idx));

			auto const priority = runner.ContactPriority();
			PR_EXPECT(priority.m_score > 0.99f);
		}

		PRUnitTestMethod(ContactPrioritySortsInteropSupportContactsBottomToTop)
		{
			auto config = EngineConfig{};
			config.position_iterations = 0;
			config.solver_iterations = 0;
			config.contact_sort_shock_iterations = 16;

			auto box = collision::ShapeBox{v4{0.5f, 0.5f, 0.5f, 0}};
			auto ground = collision::ShapeBox{v4{5.0f, 5.0f, 0.5f, 0}};
			auto rbodies = std::vector<RigidBody>{};
			rbodies.push_back(RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()});
			for (int body_idx = 0; body_idx != 4; ++body_idx)
			{
				rbodies.push_back(RigidBody{&box, m4x4::Translation(0, 0, 0.5f + static_cast<float>(body_idx)), Inertia::Box(box.m_radius, 1.0f)});
				rbodies.back().GravityWS(v4{0, 0, -9.81f, 0});
			}

			auto bodies = std::vector<GpuRigidBody>{};
			for (int body_idx = 0; body_idx != isize(rbodies); ++body_idx)
				bodies.push_back(PackDynamics(rbodies[body_idx], 0));

			auto contacts = std::vector<GpuResolveContact>{
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[3].O2W()) * rbodies[4].O2W(),
					.body_idx_a = 3,
					.body_idx_b = 4,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[2].O2W()) * rbodies[3].O2W(),
					.body_idx_a = 2,
					.body_idx_b = 3,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, +1, 0},
					.contact_point = v4{0, 0, +0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[1].O2W()) * rbodies[2].O2W(),
					.body_idx_a = 1,
					.body_idx_b = 2,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
				GpuResolveContact{
					.axis = v4{0, 0, -1, 0},
					.contact_point = v4{0, 0, -0.5f, 1},
					.b2a = InvertOrthonormal(rbodies[1].O2W()) * rbodies[0].O2W(),
					.body_idx_a = 1,
					.body_idx_b = 0,
					.mat_id_a = 0,
					.mat_id_b = 0,
					.feature = 1,
				},
			};
			auto materials = std::vector<GpuMaterial>{
				GpuMaterial{
					.friction_static = 0.0f,
					.elasticity_norm = 0.0f,
				},
			};

			auto runner = ResolveInteropRunner{config};
			auto buffers = ResolveRunnerBuffers{
				.m_dt = 1.0f / 60.0f,
				.m_bodies = bodies,
				.m_contacts = contacts,
				.m_materials = materials,
			};
			runner.Load(buffers);
			runner.ComputeCollisionTimes();
			runner.ComputeShockRanks();
			runner.SortContacts();

			auto const expected_order = std::array<uint32_t, 4>{3, 2, 1, 0};
			for (int order_idx = 0; order_idx != isize(expected_order); ++order_idx)
				PR_EXPECT(runner.ContactOrder()[order_idx] == expected_order[order_idx]);
		}

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
					runner.ResolveVelocity(colour);

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
}
#endif
