//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	struct SleepData
	{
		float m_timer_s = 0.0f;
		int m_island_id = -1;
		uint32_t m_generation = 0;
		uint32_t m_flags = 0;
	};
}
