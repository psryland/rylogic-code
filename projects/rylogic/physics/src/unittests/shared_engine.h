//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared physics::Engine instance for unit tests.
//
// Constructing a physics::Engine triggers GPU shader compilation that takes tens of
// seconds (CSCollide alone is ~36s in Debug -O3). The unit-test framework instantiates
// each test fixture afresh for every PRUnitTestMethod, so an engine member would be
// re-constructed (and the shaders re-compiled) per method. To avoid that, every test
// that needs a default-configured Engine uses SharedEngine(), which lasts for the
// duration of the test class that uses it. Test classes run in parallel on separate threads,
// so each class has its own engine, and all engines share an on-disk shader cache
// so that compiled shaders are reused across classes and across test runs.
//
// All sharing tests must call ResetEngineForNextTest() at the start of each method
// (or at the start of any helper that runs a scenario) to clear the materials map,
// event handler lists, mutable configuration, and caller-owned pointer caches.
#pragma once

#if PR_UNITTESTS
#include "pr/physics/physics.h"
#include "src/unittests/shared_gpu.h"

namespace pr::physics::tests
{
	// Lazy-initialised default-config Engine for the test class running on the calling thread.
	inline physics::Engine& SharedEngine()
	{
		thread_local auto engine = std::unique_ptr<physics::Engine>{};
		return ClassFixture(engine, []
		{
			// Build on the calling thread's GPU fixture, which therefore outlives the engine.
			auto& gpu = SharedTestGpu();
			return std::make_unique<physics::Engine>(physics::EngineConfig{}, &SharedTestShaderCache(), gpu.m_gpu.device(), gpu.m_gpu.queue());
		});
	}

	// Reset per-test engine state before a test runs. Clears registered event
	// handlers, restores the default runtime configuration and material, and drops
	// the engine's internal caches so stale caller-owned pointers cannot be reused.
	inline void ResetEngineForNextTest(physics::Engine& engine)
	{
		engine.Collisions.reset();
		engine.ExternalForces.reset();
		engine.ConstraintsBroken.reset();
		engine.ConstraintRejections.reset();
		engine.Config(physics::EngineConfig{});
		engine.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = 0.0f,
			.m_elasticity_norm = 1.0f,
		});
		engine.Water(std::nullopt);
		engine.ResetCaches();
	}
}
#endif
