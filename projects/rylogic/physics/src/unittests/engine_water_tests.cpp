//*********************************************
// Physics Engine Water Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Engine-owned water buoyancy and drag. See engine_water.h.
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/buoyancy/gpu_water_forces.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	PRUnitTestClass(EngineWaterTests)
	{
		static constexpr float Gravity = 9.8f;
		static constexpr float Density = 1000.0f;

		// A still, flat water surface with optional drag.
		static WaterConfig FlatWater(double level, float linear_drag, float quadratic_drag, float angular_drag)
		{
			// Assemble a config for a level surface with the requested drag.
			return WaterConfig{
				.m_field = terrain::water::WaterField{ level },
				.m_density = Density,
				.m_linear_drag_rate = linear_drag,
				.m_quadratic_drag_coefficient = quadratic_drag,
				.m_angular_drag_rate = angular_drag,
			};
		}

		// Step one body for 'frames' frames of 1/60 s with 'substeps' internal substeps.
		static void Run(Engine& engine, RigidBody& body, int frames, int substeps)
		{
			// Gravity is re-applied each frame, as callers are required to do.
			auto* bodies = &body;
			for (int i = 0; i != frames; ++i)
			{
				body.GravityWS(v4{ 0, 0, -Gravity, 0 });
				engine.BeginStep(Engine::StepInput{
					.m_bodies = std::span<RigidBody*>{ &bodies, 1 },
					.m_elapsed_seconds = 1.0f / 60.0f,
					.m_substep_count = substeps,
					.m_time_s = i / 60.0,
				});
				engine.CompleteStep();
			}
		}

		PRUnitTestMethod(ConfigValidation, Quick)
		{
			// Density must be positive and drag values non-negative and finite.
			auto config = FlatWater(0.0, 0.5f, 0.5f, 0.5f);
			config.Validate();

			auto bad = config;
			bad.m_density = 0.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_linear_drag_rate = -1.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_quadratic_drag_coefficient = NAN;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			// Invalid configurations are rejected without changing the engine.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			bad = config;
			bad.m_angular_drag_rate = -0.1f;
			PR_THROWS(engine.Water(bad), std::invalid_argument);
			PR_EXPECT(engine.Water() == nullptr);

			engine.Water(config);
			PR_EXPECT(engine.Water() != nullptr);
			engine.Water(std::nullopt);
			PR_EXPECT(engine.Water() == nullptr);
		}

		PRUnitTestMethod(DryCullIsConservative, Quick)
		{
			// A resting body well above the water is culled, one reaching the surface within the frame is not.
			auto sphere = collision::ShapeSphere(0.5f);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 1.0f);
			body.GravityWS(v4{ 0, 0, -Gravity, 0 });

			body.O2W(m4x4::Translation(0, 0, 10));
			PR_EXPECT(!GpuWaterForces::MayBeWet(body, 0.0, 1.0f / 60.0f));
			body.O2W(m4x4::Translation(0, 0, 0.51f));
			PR_EXPECT(GpuWaterForces::MayBeWet(body, 0.0, 1.0f / 60.0f));
			body.O2W(m4x4::Translation(0, 0, 0.4f));
			PR_EXPECT(GpuWaterForces::MayBeWet(body, 0.0, 1.0f / 60.0f));

			// A fast fall can reach the water within the frame.
			body.O2W(m4x4::Translation(0, 0, 2.0f));
			body.VelocityWS(v4::Zero(), v4{ 0, 0, -120.0f, 0 });
			PR_EXPECT(GpuWaterForces::MayBeWet(body, 0.0, 1.0f / 60.0f));
		}

		PRUnitTestMethod(DryBodyMatchesNoWater, Quick)
		{
			// A body far above the water must follow exactly the same trajectory as without water.
			auto sphere = collision::ShapeSphere(0.5f);
			auto make = [&]
			{
				// Identical starting state for each run.
				auto body = RigidBody{};
				body.Shape(collision::shape_cast(&sphere), 1.0f);
				body.O2W(m4x4::Translation(0, 0, 100));
				body.VelocityWS(v4{ 0.5f, 0, 0, 0 }, v4{ 1, 0, 0, 0 });
				return body;
			};

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto dry = make();
			Run(engine, dry, 10, 4);

			engine.Water(FlatWater(0.0, 0.5f, 0.5f, 0.5f));
			auto wet_world = make();
			Run(engine, wet_world, 10, 4);
			engine.Water(std::nullopt);

			PR_EXPECT(All(dry.O2W().pos == wet_world.O2W().pos));
			PR_EXPECT(All(dry.VelocityWS().lin == wet_world.VelocityWS().lin));
		}

		PRUnitTestMethod(SubmergedBoxBuoyancy, Quick)
		{
			// Without drag, a fully submerged 1 m³ box of 2000 kg sinks at g/2.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), 2.0f * Density);
			body.O2W(m4x4::Translation(0, 0, -20));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Water(FlatWater(0.0, 0.0f, 0.0f, 0.0f));
			Run(engine, body, 60, 4);
			engine.Water(std::nullopt);

			auto const vz = body.VelocityWS().lin.z;
			PR_EXPECT(FEqlRelative(vz, -0.5f * Gravity, 0.01f));
		}

		PRUnitTestMethod(NeutralBoxStaysPut, Quick)
		{
			// A neutrally buoyant submerged box feels no net force.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), Density);
			body.O2W(m4x4::Translation(0, 0, -5));

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Water(FlatWater(0.0, 0.5f, 0.5f, 0.5f));
			Run(engine, body, 60, 4);
			engine.Water(std::nullopt);

			PR_EXPECT(FEqlAbsolute(body.O2W().pos.z, -5.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 1e-3f));
		}

		PRUnitTestMethod(SphereFloatsAtAnalyticDraft, Quick)
		{
			// A sphere of half the water's density settles with its centre on the surface.
			auto const radius = 0.5f;
			auto const volume = 2.0f / 3.0f * constants<float>::tau * radius * radius * radius;
			auto sphere = collision::ShapeSphere(radius);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 0.5f * Density * volume);

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = engine.Config();
			config.sleeping_enabled = false;
			engine.Config(config);
			engine.Water(FlatWater(2.0, 2.0f, 0.5f, 0.5f));
			body.O2W(m4x4::Translation(0, 0, 2.4f));
			Run(engine, body, 600, 4);
			engine.Water(std::nullopt);

			PR_EXPECT(FEqlAbsolute(body.O2W().pos.z, 2.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 0.01f));
		}

		PRUnitTestMethod(DragNeverReversesMotion, Quick)
		{
			// Very strong drag stops relative motion without overshooting within one frame.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), Density);
			body.O2W(m4x4::Translation(0, 0, -5));
			body.VelocityWS(v4{ 0, 0, 5, 0 }, v4{ 20, 0, 0, 0 });

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Water(FlatWater(0.0, 1000.0f, 1000.0f, 1000.0f));
			Run(engine, body, 1, 1);
			engine.Water(std::nullopt);

			auto const velocity = body.VelocityWS();
			PR_EXPECT(velocity.lin.x >= 0.0f && velocity.lin.x < 20.0f);
			PR_EXPECT(velocity.ang.z >= 0.0f && velocity.ang.z < 5.0f);
		}
	};
}
#endif
