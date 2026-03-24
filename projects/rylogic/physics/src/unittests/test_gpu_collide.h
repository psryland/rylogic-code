//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2016
//************************************
// Unit tests for collision detection
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "src/compute/collision.hlsli"

namespace pr::physics::tests
{
	PRUnitTestClass(GpuCollideTests)
	{
		PRUnitTestMethod(SphereVsSphere)
		{
		}
	};
}

#endif
