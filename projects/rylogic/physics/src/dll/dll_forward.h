//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <unordered_set>
#include <mutex>

namespace pr::physics
{
	using LockGuard = std::lock_guard<std::recursive_mutex>;
}
