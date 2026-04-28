//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"

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
			auto body_b = RigidBody{&box, m4x4::Translation(v4{0.9f, 0, 0, 0}), Inertia::Box(box.m_radius, 1.0f)};
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
			auto body_b = RigidBody{&ground, m4x4::Translation(v4{0, 0, -0.5f, 1}), Inertia::Infinite()};
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
			auto body_a = RigidBody{&box, m4x4::Translation(v4{0, 0, 0.5f, 1}), Inertia::Box(box.m_radius, 1.0f)};
			auto body_b = RigidBody{&ground, m4x4::Translation(v4{0, 0, -0.5f, 1}), Inertia::Infinite()};

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
	};
}
#endif
