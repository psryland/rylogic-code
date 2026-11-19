//*********************************************
// Physics Engine GPU Lifetime Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "src/unittests/forward.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	// Failed steps retain their engine-owned resources until the caller explicitly closes the engine.
	PRUnitTestClass(EngineGpuRecoveryTests, Serial)
	{
		// A failed queue signal must not publish body state, replay the step, or allow resource replacement.
		PRUnitTestMethod(FailedSignalRetainsEngineUntilExplicitClose, Quick)
		{
			// The body and queue proxy outlive the engine's cleanup.
			auto& gpu = SharedTestGpu();
			compute::tests::ForwardingQueue proxy(gpu.m_gpu.queue());
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), v4::XAxis());
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto engine = std::make_unique<Engine>(EngineConfig{ .sleeping_enabled = false }, &SharedTestShaderCache(), gpu.m_gpu.device(), &proxy);
			Scope<void> close([&]
			{
				// Restore the real queue's signal behavior before explicitly retiring retained work.
				proxy.m_fail_signals = false;
				if (engine)
					PR_EXPECT(engine->TryClose(INFINITE) == S_OK);
			});
			auto const input = Engine::StepInput{ .m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60.0f };
			engine->Step(input);

			// Interrupt a submitted step, rather than a rejected input or empty scene.
			auto const position = body.O2W().pos;
			auto const executes = proxy.m_execute_calls;
			proxy.m_fail_signals = true;
			PR_THROWS(engine->Step(input), std::runtime_error);
			PR_EXPECT(engine->Closing() && engine->StepPending());
			PR_EXPECT(All(body.O2W().pos == position));
			PR_EXPECT(proxy.m_execute_calls == executes + 1);
			PR_THROWS(engine->Step(input), std::runtime_error);
			PR_THROWS(engine->ResetCaches(), std::runtime_error);
			PR_THROWS(engine->Terrain(std::nullopt), std::runtime_error);
			PR_THROWS(engine->CylindricalBoundary(std::nullopt), std::runtime_error);
			PR_THROWS(engine->UpdateSleepIslands(bodies), std::runtime_error);
			PR_EXPECT(engine->TryClose(0) == E_OUTOFMEMORY);
			PR_EXPECT(engine->StepPending());

			// A successful retry releases pending CPU references without making this failed engine reusable.
			proxy.m_fail_signals = false;
			PR_EXPECT(engine->TryClose(INFINITE) == S_OK);
			PR_EXPECT(!engine->StepPending() && engine->Closing());
			PR_EXPECT(proxy.m_execute_calls == executes + 1);
			PR_THROWS(engine->Step(input), std::runtime_error);
			engine.reset();
			PR_EXPECT(proxy.RefCount() == 1);
		}

		// A maintenance failure retains GPU resources even though no simulation step owns a pending handle.
		PRUnitTestMethod(FailedMaintenanceRetainsResources, Quick)
		{
			// Maintenance has no PendingStep owner, but its queued resources need the same close contract.
			auto& gpu = SharedTestGpu();
			compute::tests::ForwardingQueue proxy(gpu.m_gpu.queue());
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto engine = Engine(EngineConfig{}, &SharedTestShaderCache(), gpu.m_gpu.device(), &proxy);
			Scope<void> close([&]
			{
				// Retire the injected unfenced submission before input and engine resources are destroyed.
				proxy.m_fail_signals = false;
				PR_EXPECT(engine.TryClose(INFINITE) == S_OK);
			});
			auto const executes = proxy.m_execute_calls;
			proxy.m_fail_signals = true;
			PR_THROWS(engine.UpdateSleepIslands(bodies), std::runtime_error);
			PR_EXPECT(engine.Closing() && !engine.StepPending());
			PR_EXPECT(proxy.m_execute_calls == executes + 1);
			PR_THROWS(engine.ResetCaches(), std::runtime_error);
			PR_THROWS(engine.Terrain(std::nullopt), std::runtime_error);
			PR_THROWS(engine.CylindricalBoundary(std::nullopt), std::runtime_error);
			PR_THROWS(engine.Water(std::nullopt), std::runtime_error);
			PR_THROWS(engine.ForgetShape(shape), std::runtime_error);
			PR_EXPECT(engine.TryClose(0) == E_OUTOFMEMORY);
			proxy.m_fail_signals = false;
			PR_EXPECT(engine.TryClose(INFINITE) == S_OK);
			PR_EXPECT(proxy.m_execute_calls == executes + 1);
		}

		// Failure during recording must discard the unfinished frame rather than submit it while unwinding.
		PRUnitTestMethod(RecordingFailureNeverExecutesThePartialStep, Quick)
		{
			// Keep all input owners alive until the explicit close guard has cleared pending references.
			auto& gpu = SharedTestGpu();
			compute::tests::ForwardingQueue proxy(gpu.m_gpu.queue());
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto bodies = std::array<RigidBody*, 1>{&body};
			auto engine = Engine(EngineConfig{ .sleeping_enabled = false }, &SharedTestShaderCache(), gpu.m_gpu.device(), &proxy);
			AutoSub sub = engine.ExternalForces += [](Engine&, Engine::ExternalForceArgs const&)
			{
				// Fail after recording has begun but before submission.
				throw std::runtime_error("Injected force callback failure");
			};
			Scope<void> close([&]
			{
				// The recorded frame must never reach the queue, including during cleanup.
				PR_EXPECT(engine.TryClose(INFINITE) == S_OK);
			});
			auto const executes = proxy.m_execute_calls;
			auto const input = Engine::StepInput{ .m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60.0f };
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_EXPECT(engine.Closing());
			PR_EXPECT(proxy.m_execute_calls == executes);
			PR_EXPECT(engine.TryClose(0) == S_OK);
			PR_EXPECT(proxy.m_execute_calls == executes);
		}
	};
}
#endif
