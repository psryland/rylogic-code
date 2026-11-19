//*********************************************
// Physics Engine Recovery Tests
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/physics.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Keep device policy and shader ownership in the established class fixture.
		Engine RecoveryEngine()
		{
			auto& gpu = SharedTestGpu();
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			return Engine{config, &SharedTestShaderCache(), gpu.m_gpu.device(), gpu.m_gpu.queue()};
		}
	}

	// Exercise Engine-owned rejection and recovery, not the already reviewed primitive's terminal campaign.
	PRUnitTestClass(EngineGpuRecoveryTests)
	{
		PRUnitTestMethod(PrimaryAndRetirementFailuresRemainDistinct, Quick)
		{
			// A real moving body makes replay and premature publication observable.
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), v4::XAxis());
			auto bodies = std::array{&body};
			auto engine = RecoveryEngine();
			auto input = Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 0.001f};
			engine.Step(input);
			auto const before = body.O2W().pos;
			auto added = multicast::AutoSub{};
			auto completed = multicast::AutoSub{};
			auto installed = false;
			auto signal_count = 0;
			auto fail = true;
			engine.ExternalForces += [&](Engine&, Engine::ExternalForceArgs const& args)
			{
				// Throw after the real submission signal, then at the completed-fence observer during immediate retirement.
				if (installed)
					return;

				installed = true;
				added = args.m_job.m_gsync.SyncPointAdded += [&](auto&, auto const&)
				{
					++signal_count;
					if (fail)
						throw std::runtime_error("primary submission observer");
				};
				completed = args.m_job.m_gsync.SyncPointCompleted += [&](auto&, auto const&)
				{
					if (fail)
						throw std::runtime_error("secondary retirement observer");
				};
			};
			auto message = std::string{};
			try
			{
				engine.Step(input);
			}
			catch (std::runtime_error const& ex)
			{
				message = ex.what();
			}
			PR_EXPECT(message == "primary submission observer");
			PR_EXPECT(!engine.StepPending() && engine.RecoveryRequired());
			PR_EXPECT(All(body.O2W().pos == before));
			auto const recovery = engine.RecoveryFailure();
			PR_EXPECT(recovery != nullptr);
			try
			{
				std::rethrow_exception(recovery);
			}
			catch (std::runtime_error const& ex)
			{
				PR_EXPECT(std::string_view{ex.what()} == "secondary retirement observer");
			}

			// Guards neither submit work nor replace the retained recovery reason.
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_THROWS(engine.ResetCaches(), std::runtime_error);
			PR_THROWS(engine.Terrain(std::nullopt), std::runtime_error);
			PR_THROWS(engine.CylindricalBoundary(std::nullopt), std::runtime_error);
			PR_THROWS(engine.UpdateSleepIslands(std::span<RigidBody*>{bodies}), std::runtime_error);
			PR_EXPECT(engine.RecoveryFailure() == recovery && signal_count == 1);

			// Explicit recovery reopens the completed recording without another submission.
			fail = false;
			engine.RetireInterruptedWork();
			PR_EXPECT(!engine.RecoveryRequired() && engine.RecoveryFailure() == nullptr && signal_count == 1);
			engine.RetireInterruptedWork();
			PR_EXPECT(signal_count == 1 && All(body.O2W().pos == before));
			engine.Step(input);
			PR_EXPECT(signal_count == 2 && body.O2W().pos.x > before.x);
		}

		PRUnitTestMethod(ConsumedCompletionFailureEndsFrame, Quick)
		{
			// A post-wait observer fails after Complete consumes the run handle.
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), v4::XAxis());
			auto bodies = std::array{&body};
			auto engine = RecoveryEngine();
			auto completed = multicast::AutoSub{};
			auto installed = false;
			engine.ExternalForces += [&](Engine&, Engine::ExternalForceArgs const& args)
			{
				if (installed)
					return;

				installed = true;
				completed = args.m_job.m_gsync.SyncPointCompleted += [](auto&, auto const&)
				{
					throw std::runtime_error("post-wait observer");
				};
			};
			auto input = Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 0.001f};
			auto const before = body.O2W().pos;
			engine.BeginStep(input);
			PR_THROWS(engine.CompleteStep(), std::runtime_error);
			PR_EXPECT(!engine.StepPending() && !engine.RecoveryRequired());
			PR_EXPECT(engine.RecoveryFailure() == nullptr && All(body.O2W().pos == before));
			PR_THROWS(engine.CompleteStep(), std::runtime_error);

			// The rejected frame is over; removing the failed observer permits ordinary stepping.
			completed = {};
			engine.Step(input);
			PR_EXPECT(body.O2W().pos.x > before.x);
		}

		PRUnitTestMethod(RejectedPublicationClearsWaterResults, Quick)
		{
			// Overlapping wet bodies produce a collision callback before primary-state publication.
			auto shape = collision::ShapeSphere{0.5f};
			auto body_a = RigidBody{&shape, m4x4::Translation(0, 0, -2), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto body_b = RigidBody{&shape, m4x4::Translation(0.8f, 0, -2), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto bodies = std::array{&body_a, &body_b};
			auto engine = RecoveryEngine();
			auto water = WaterConfig{.m_field = terrain::water::WaterField{0.0}, .m_density = 1000.0f, .m_gravity = 9.8f};
			water.m_enable_diagnostics = true;
			engine.Water(water);
			auto input = Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 0.0001f};
			auto callback_ran = false;
			auto water_was_published = false;
			engine.Collisions += [&](Engine&, std::span<RbContact const>)
			{
				// Water was already read back, but the caller-visible body transaction has not committed.
				callback_ran = true;
				water_was_published = engine.WaterDiagnosticsFor(body_a) != nullptr;
				throw std::runtime_error("publication callback");
			};
			auto const before_a = body_a.O2W().pos;
			auto const before_b = body_b.O2W().pos;
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_EXPECT(callback_ran && water_was_published && !engine.StepPending() && !engine.RecoveryRequired());
			PR_EXPECT(All(body_a.O2W().pos == before_a) && All(body_b.O2W().pos == before_b));
			PR_EXPECT(engine.WaterDiagnosticsFor(body_a) == nullptr && engine.WaterDiagnosticsFor(body_b) == nullptr);
			engine.Collisions.reset();
			engine.Step(input);
			PR_EXPECT(engine.WaterDiagnosticsFor(body_a) != nullptr);
		}
	};
}
