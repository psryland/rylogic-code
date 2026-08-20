//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "pr/physics/physics-dll.h"
#include "src/dll/dll_forward.h"

namespace pr::physics
{
	struct Context
	{
		using InitSet = std::unordered_set<DllHandle>;

		InitSet              m_inits;    // A unique id assigned to each Initialise call
		std::recursive_mutex m_mutex;
		ReportErrorCB        m_error_cb; // Global error callback
		std::unique_ptr<InteropState> m_interop;

		Context(ReportErrorCB error_cb);
		Context(Context&&) = delete;
		Context(Context const&) = delete;
		Context& operator=(Context&) = delete;
		Context& operator=(Context const&) = delete;
		~Context();
	};
}
