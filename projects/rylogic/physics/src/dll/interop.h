//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "physics/src/dll/dll_forward.h"

namespace pr::physics
{
	struct EngineSlot;

	// Owns all generation-aware ABI objects associated with the process DLL context.
	struct InteropState
	{
		std::vector<std::unique_ptr<EngineSlot>> m_engines;
		std::uint32_t m_next_cookie;

		InteropState();
		~InteropState();

		// Report whether any engine still owns simulation or device state.
		bool HasLiveEngines() const;
	};
}
