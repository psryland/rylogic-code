//*********************************************
// Game Systems Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Shared D3D12 compute context for the game-systems unit tests.
#pragma once
#include "pr/common/unittests.h"
#include "pr/compute/shaders/shader_cache_fs.h"
#include "pr/game-systems/atmosphere/atmosphere.h"

namespace pr::game_systems::tests
{
	// Return the shader cache shared by all game-systems tests, stored in '<repo>/obj/unittests/shader-cache'.
	inline compute::shader_cache::IShaderCache& SharedTestShaderCache()
	{
		// The cache is safe to use from multiple threads and processes, and validates entries against their source dependencies.
		static auto cache = compute::shader_cache::ShaderCacheFS(std::filesystem::path(__FILE__).parent_path() / "../../../../../obj/unittests/shader-cache", "game-systems");
		return cache;
	}

	// Return the GPU fixture for the test class running on the calling thread.
	// The fixture's command lists can only be used by the thread that created them, so each test class gets its own fixture on the thread that runs it.
	// The test runner releases the fixture after the class (see TestFramework::AddClassFixture).
	inline atmosphere::Gpu& SharedTestGpu()
	{
		// Register the release on creation so fixtures built later from this one are released first.
		thread_local auto gpu = std::unique_ptr<atmosphere::Gpu>{};
		if (!gpu)
		{
			// The release may run on another thread, so it holds this thread's slot by address.
			gpu = std::make_unique<atmosphere::Gpu>(::pr::compute::TestGpuOptions);
			auto slot = &gpu;
			unittests::TestFramework::AddClassFixture([slot] { slot->reset(); });
		}
		return *gpu;
	}
}
