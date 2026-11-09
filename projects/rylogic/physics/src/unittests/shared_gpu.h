//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared D3D12 compute context for native unit tests.
#pragma once

#if PR_UNITTESTS
#include "src/utility/gpu.h"
#include "pr/compute/shaders/shader_cache_fs.h"
#include "pr/common/unittests.h"

namespace pr::physics::tests
{
	// Return the shader cache shared by all test engines, stored in '<repo>/obj/unittests/shader-cache'.
	inline compute::shader_cache::IShaderCache& SharedTestShaderCache()
	{
		// The cache is safe to use from multiple threads and processes, and validates entries against their source dependencies.
		static auto cache = compute::shader_cache::ShaderCacheFS(std::filesystem::path(__FILE__).parent_path() / "../../../../../obj/unittests/shader-cache", "physics");
		return cache;
	}

	// Return the calling thread's instance of a fixture shared by the tests of one class, creating it with 'make' on first use.
	// 'slot' must be a 'thread_local' owned by the caller. The test runner releases the fixture after the class (see TestFramework::AddClassFixture).
	template <typename T, typename Make>
	T& ClassFixture(std::unique_ptr<T>& slot, Make&& make)
	{
		// Register the release on creation so fixtures built later from this one are released first.
		if (!slot)
		{
			slot = make();
			unittests::TestFramework::AddClassFixture([&slot] { slot.reset(); });
		}
		return *slot;
	}

	// Return the GPU fixture for the test class running on the calling thread.
	// The fixture's command lists can only be used by the thread that created them, so each test class gets its own fixture on the thread that runs it.
	inline Gpu& SharedTestGpu()
	{
		thread_local auto gpu = std::unique_ptr<Gpu>{};
		return ClassFixture(gpu, [] { return std::make_unique<Gpu>(::pr::compute::TestGpuOptions); });
	}
}
#endif
