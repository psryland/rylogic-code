//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Entry point for the physics sandbox application.
// Two modes:
//   -unittest : Allocate a console, run embedded unit tests, and exit.
//   (default) : Launch the interactive physics sandbox window.
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

namespace physics_sandbox
{
	// Run embedded unit tests to a console window and exit.
	int RunUnitTests()
	{
		// Uses AllocConsole() to create a console for the WinApp process,
		// then redirects stdout so PR_EXPECT output is visible.
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

	// The user's app data directory.
	std::filesystem::path AppDataPath()
	{
		size_t size;
		getenv_s(&size, nullptr, 0, "APPDATA");

		// Get the %APPDATA% directory for the current user, and create a subdirectory for our app.
		std::string appdata(size, '\0');
		getenv_s(&size, appdata.data(), appdata.size(), "APPDATA");
		appdata.resize(size - 1); // Remove the trailing null character added by getenv_s
		
		auto path = std::filesystem::path{ appdata } / "RylogicPhysicsSandbox";
		std::filesystem::create_directories(path);
		return path;
	}
}

// Entry point
int __stdcall WinMain(HINSTANCE, HINSTANCE, LPTSTR lpCmdLine, int)
{
	// Parse the command line using pr::CmdLine which handles quoted paths,
	// tokenization, and key-value separation correctly.
	// Prepend a dummy program name because lpCmdLine doesn't include it,
	// but CmdLine's string_view constructor skips argv[0] as the exe name.
	auto cmd = pr::CmdLine("app " + std::string(lpCmdLine ? lpCmdLine : ""));

	// Check for -unittest mode before initialising any GUI resources
	if (cmd.count("unittest"))
		return physics_sandbox::RunUnitTests();

	// Interactive sandbox mode.
	// Must use STA (apartment-threaded) because COM UI components like IFileDialog
	// require a single-threaded apartment with a message pump on the calling thread.
	// MTA (the InitCom default) causes IFileDialog::Show() to hang indefinitely.
	pr::InitCom com(COINIT_APARTMENTTHREADED);
	pr::GdiPlus gdi;

	// When running headless (e.g. -autoplay from a script), suppress assert dialog boxes
	// and send them to stderr instead so the process terminates cleanly with diagnostics.
	auto autoplay = cmd.count("autoplay") > 0;
	if (autoplay)
	{
		_CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_DEBUG);
		_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
	}

	try
	{
		InitCtrls();

		// Create the message loop first so the form can reference it for clean shutdown
		WinGuiMsgLoop loop;

		SandboxUI sandbox;
		sandbox.cp().msg_loop(&loop);

		// Load a scene file on startup if specified: -scene <filepath>
		if (cmd.count("scene"))
			sandbox.LoadSceneFile(cmd("scene").as<std::filesystem::path>());

		sandbox.Show();

		// Start the simulation immediately if -autoplay was specified
		if (autoplay)
			sandbox.m_steps_remaining = -1;

		// Simulation loop: fixed 60 Hz timestep for deterministic physics.
		// Render loop: variable at high target rate. The actual frame rate is
		// limited by vsync inside View3D_WindowRender→Present. Setting a high
		// target (1000 Hz) ensures the loop fires immediately after vsync
		// completes, rather than waiting for MsgWaitForMultipleObjects timeout.
		loop.AddLoop(60.0, false, [&](double dt) { sandbox.Step(dt); });
		loop.AddLoop(1000.0, true, [&](double dt) { sandbox.Render(dt); });
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
