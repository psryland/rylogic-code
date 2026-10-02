//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Interactive 3D scene, shown instead of running the unit tests when '--interactive' is passed.
#pragma once

// Show the interactive scene window and run its message loop until it closes. Reads the rylogic assets path from
// 'view3d-12-tests.config.json' beside the executable. Returns the process exit code.
int RunInteractive();
