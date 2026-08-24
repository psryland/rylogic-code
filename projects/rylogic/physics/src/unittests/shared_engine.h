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
// that needs a default-configured Engine uses SharedEngine() — a function-local static
// that survives across every test in the executable.
//
// All sharing tests must call ResetEngineForNextTest() at the start of each method
// (or at the start of any helper that runs a scenario) to clear the materials map and
// Collisions event handler list. EngineConfig is const at construction and cannot be
// modified, so any test that needs a non-default config must construct its own Engine
// (and pay the shader-compile cost).
#pragma once

#if PR_UNITTESTS
#include "pr/physics/physics.h"

namespace pr::physics::tests
{
	// Lazy-initialised default-config Engine shared by every test that uses it.
	inline physics::Engine& SharedEngine()
	{
		static physics::Engine s_engine;
		return s_engine;
	}

	// Reset per-test engine state before a test runs. Clears registered event
	// handlers, restores the default runtime configuration and material, and drops
	// the engine's internal caches so stale caller-owned pointers cannot be reused.
	inline void ResetEngineForNextTest(physics::Engine& engine)
	{
		engine.Collisions.reset();
		engine.ExternalForces.reset();
		engine.Config(physics::EngineConfig{});
		engine.Material(physics::Material{
			.m_id = physics::Material::DefaultID,
			.m_friction_static = 0.0f,
			.m_elasticity_norm = 1.0f,
		});
		engine.ResetCaches();
	}
}
#endif
