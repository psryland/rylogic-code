//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Interactive demos, shown instead of running the unit tests when '--interactive' is passed.
#pragma once
#include <string_view>

// Show the interactive demo window and run its message loop until it closes. Reads the rylogic assets path from
// 'view3d-12-tests.config.json' beside the executable. 'demo_name' selects the initial demo. If it is empty, the last used demo is shown.
// Returns the process exit code.
int RunInteractive(std::string_view demo_name);
