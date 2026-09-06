//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <bitset>
#include <unordered_set>
#include <mutex>
#include <memory>

namespace pr::physics
{
	using LockGuard = std::lock_guard<std::recursive_mutex>;

	struct Context;
	struct InteropState;

	// Access the process-wide DLL context after initialisation.
	// Pin the process context for the duration of an ABI operation.
	std::shared_ptr<Context> PinDll();

	// Pin the process context when it exists.
	std::shared_ptr<Context> TryPinDll();
}
