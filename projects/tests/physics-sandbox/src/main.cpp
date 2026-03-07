//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Entry point for the physics sandbox application.
// Two modes:
//   -unittest : Allocate a console, run embedded unit tests, and exit.
//   (default) : Launch the interactive physics sandbox window.
//

#include "src/forward.h"
#include "src/ui/sandbox_ui.h"
#include "src/unittests/sandbox_tests.h"

// Enable ComCtl32 v6 visual styles (modern themed controls)
#pragma comment(linker, "\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

using namespace pr;
using namespace pr::gui;
using namespace physics_sandbox;

// Run embedded unit tests to a console window and exit.
// Uses AllocConsole() to create a console for the WinApp process,
// then redirects stdout so PR_EXPECT output is visible.
static int RunUnitTests()
{
	// Attach to parent console (if launched from cmd) or allocate a new one
	if (!AttachConsole(DWORD(-1)))
		AllocConsole();

	// Redirect stdout/stderr to the console
	FILE* fp = nullptr;
	freopen_s(&fp, "CONOUT$", "w", stdout);
	freopen_s(&fp, "CONOUT$", "w", stderr);

	printf("Physics-2 Sandbox: Running unit tests...\n");

	// The PR_UNITTESTS framework collects tests via static initialisation.
	// RunAllTests() executes them and prints results.
	auto failed = pr::unittests::RunAllTests(true);
	return failed > 0 ? 1 : 0;
}

// Entry point
int __stdcall WinMain(HINSTANCE, HINSTANCE, LPTSTR lpCmdLine, int)
{
	// Check for -unittest mode before initialising any GUI resources
	if (lpCmdLine && strstr(lpCmdLine, "-unittest"))
		return RunUnitTests();

	// Interactive sandbox mode.
	// Must use STA (apartment-threaded) because COM UI components like IFileDialog
	// require a single-threaded apartment with a message pump on the calling thread.
	// MTA (the InitCom default) causes IFileDialog::Show() to hang indefinitely.
	pr::InitCom com(COINIT_APARTMENTTHREADED);
	pr::GdiPlus gdi;

	try
	{
		pr::win32::LoadDll<struct View3d>(L"view3d-12.dll");
		InitCtrls();

		// Create the message loop first so the form can reference it for clean shutdown
		WinGuiMsgLoop loop;

		SandboxUI sandbox;
		sandbox.cp().msg_loop(&loop);

		// Check for -scene <filepath> to load a scene file on startup
		if (lpCmdLine)
		{
			auto scene_arg = strstr(lpCmdLine, "-scene ");
			if (scene_arg != nullptr)
			{
				auto filepath = std::string(scene_arg + 7);

				// Trim leading/trailing quotes and whitespace
				while (!filepath.empty() && (filepath.front() == '"' || filepath.front() == ' '))
					filepath.erase(filepath.begin());
				while (!filepath.empty() && (filepath.back() == '"' || filepath.back() == ' '))
					filepath.pop_back();

				sandbox.LoadSceneFile(std::filesystem::path(filepath));
			}
		}

		sandbox.Show();

		// Simulation loop: fixed 60 Hz timestep for deterministic physics.
		// Render loop: variable at high target rate. The actual frame rate is
		// limited by vsync inside View3D_WindowRender→Present. Setting a high
		// target (1000 Hz) ensures the loop fires immediately after vsync
		// completes, rather than waiting for MsgWaitForMultipleObjects timeout.
		loop.AddLoop(60.0, false, [&](double dt) { sandbox.Step(dt); });
		loop.AddLoop(1000.0, true, [&](double) { sandbox.Render(); });
		loop.AddMessageFilter(sandbox);
		return loop.Run();
	}
	catch (std::exception const& ex)
	{
		auto msg = std::string("EXCEPTION: ") + ex.what() + "\n";
		OutputDebugStringA(msg.c_str());
		if (auto f = fopen("dump\\crash.log", "w")) { fputs(msg.c_str(), f); fclose(f); }
		return -1;
	}
	catch (...)
	{
		OutputDebugStringA("UNKNOWN EXCEPTION\n");
		if (auto f = fopen("dump\\crash.log", "w")) { fputs("UNKNOWN EXCEPTION\n", f); fclose(f); }
		return -2;
	}
}
