//*********************************************
// Audio Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/audio/forward.h"
#include "pr/audio/audio-dll.h"
#include "pr/audio/engine.h"

namespace pr::audio
{
	using LockGuard = std::lock_guard<std::recursive_mutex>;

	// Own one generation-aware engine slot exposed through the C ABI.
	struct EngineSlot
	{
		std::uint32_t m_generation = 1;
		std::thread::id m_owner_thread;
		std::unique_ptr<Engine> m_engine;
	};

	// Own process-wide ABI state while initialization handles remain registered.
	struct Context
	{
		using InitSet = std::unordered_set<DllHandle>;

		InitSet m_inits;
		std::recursive_mutex m_mutex;
		ReportErrorCB m_error_cb;
		std::vector<EngineSlot> m_engines;

		explicit Context(ReportErrorCB error_cb);
		Context(Context&&) = delete;
		Context(Context const&) = delete;
		Context& operator=(Context&) = delete;
		Context& operator=(Context const&) = delete;
	};

	// Pin the process context for the duration of one ABI operation.
	std::shared_ptr<Context> PinDll();

	// Pin the process context if initialization has completed.
	std::shared_ptr<Context> TryPinDll();
}