//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include "src/forward.h"

namespace las
{
	// Handle application-owned command modes before the GUI and renderer are initialised.
	bool TryHandleCommandLine(TCHAR const* command_line, int& exit_code);
}
