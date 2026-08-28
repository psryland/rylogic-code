//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared D3D12 compute context for sequential native unit tests.
#pragma once

#if PR_UNITTESTS
#include "src/utility/gpu.h"

namespace pr::physics::tests
{
	// Return the process-long GPU fixture used by hardware tests that never execute concurrently.
	inline Gpu& SharedTestGpu()
	{
		static auto gpu = Gpu{};
		return gpu;
	}
}
#endif
