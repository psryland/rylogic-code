//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "physics/src/dll/dll_forward.h"

namespace pr::compute::shader_cache
{
	struct IShaderCache;
}

namespace pr::physics
{
	struct EngineSlot;
	struct InteropGpuBackend;

	// Owns all generation-aware ABI objects associated with the process DLL context.
	struct InteropState
	{
		std::vector<std::unique_ptr<InteropGpuBackend>> m_gpu_backends;
		std::vector<std::unique_ptr<EngineSlot>> m_engines;
		std::uint32_t m_next_cookie;

		// Compiled-kernel cache given to engines when they are created, or null for no caching.
		std::shared_ptr<compute::shader_cache::IShaderCache> m_shader_cache;

		InteropState();
		~InteropState();

		// Report whether any engine still owns simulation or device state.
		bool HasLiveEngines() const;

		// Return the context-owned compute backend shared by engines using the same explicit external-device identity.
		InteropGpuBackend& GpuBackend(void* external_device);
	};
}
