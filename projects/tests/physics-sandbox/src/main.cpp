//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Entry point for the physics sandbox application.
// Two modes:
//   -unittests [Class1 Class2 ...] or -unittests=Class1 : Allocate a console, run unit tests, and exit.
//     Optional class name filters are substring-matched against test class names.
//   (default) : Launch the interactive physics sandbox window.
#include "src/forward.h"
#include "src/ui/sandbox_ui.h"
#include "src/diagnostics/scene_diagnostic.h"

// Enable ComCtl32 v6 visual styles (modern themed controls)
#pragma comment(linker, "\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

using namespace pr;
using namespace pr::gui;
using namespace physics_sandbox;

namespace physics_sandbox
{
	// Route CRT assertions to places that automation can see rather than to modal dialogs.
	void ConfigureAssertReporting(bool console_output)
	{
		#if defined(_DEBUG)
		_set_error_mode(_OUT_TO_STDERR);

		auto mode = _CRTDBG_MODE_DEBUG;
		if (console_output)
			mode |= _CRTDBG_MODE_FILE;

		for (auto report_type : {_CRT_WARN, _CRT_ERROR, _CRT_ASSERT})
		{
			_CrtSetReportMode(report_type, mode);
			if (console_output)
				_CrtSetReportFile(report_type, _CRTDBG_FILE_STDERR);
		}
		#else
		(void)console_output;
		#endif
	}

	// Attach stdout/stderr to the parent console, or allocate one when launched from Explorer.
	void OpenConsoleOutput()
	{
		if (!AttachConsole(DWORD(-1)))
			AllocConsole();

		FILE* fp = nullptr;
		freopen_s(&fp, "CONOUT$", "w", stdout);
		freopen_s(&fp, "CONOUT$", "w", stderr);
		ConfigureAssertReporting(true);
	}

	// Run embedded unit tests to a console window and exit.
	// Optional class name filters are passed via 'filter' (substring match on class names).
	int RunUnitTests(std::span<std::string_view const> filter = {})
	{
		OpenConsoleOutput();

		printf("Physics Sandbox: Running unit tests...\n");

		// Optionally tee unit-test output to a file for automation capture. This app reopens CONOUT$
		// in OpenConsoleOutput(), which bypasses any inherited stdout redirection, so a console-less
		// caller cannot capture results via a pipe. Setting PR_UNITTEST_LOG to a path provides an
		// explicit file sink for the unit-test framework's output stream.
		std::ofstream unittest_log;
		if (auto const* log_path = std::getenv("PR_UNITTEST_LOG"); log_path != nullptr && *log_path != '\0')
		{
			unittest_log.open(log_path, std::ios::out | std::ios::trunc);
			if (unittest_log)
				pr::unittests::TestFramework::ostream = &unittest_log;
		}

		// The PR_UNITTESTS framework collects tests via static initialisation.
		// RunAllTests() executes them and prints results.
		auto failed = pr::unittests::RunAllTests(true, filter);
		return failed != 0 ? 1 : 0;
	}

	// Return true if unit test mode is requested and collect any optional test class filters.
	bool TryGetUnitTestFilter(pr::CmdLine const& cmd, std::vector<std::string_view>& filter)
	{
		auto found = false;
		for (auto const& arg : cmd.args)
		{
			auto key = std::string_view(arg.key);
			if (key == "unittests")
			{
				found = true;
			}
			else if (key.starts_with("unittests="))
			{
				found = true;

				auto value = key.substr(strlen("unittests="));
				if (!value.empty())
					filter.push_back(value);
			}
			else
			{
				continue;
			}

			for (auto const& value : arg.values)
				filter.push_back(value);
		}
		return found;
	}

	// Run a headless scene diagnostic and print post-step penetration metrics.
	int RunSceneDiagnostic(pr::CmdLine const& cmd)
	{
		OpenConsoleOutput();
		auto log_path = AppDataPath() / "scene_diagnostic.log";

		auto options = diag::SceneDiagnosticOptions{
			.m_scene_filepath = "projects\\tests\\physics-sandbox\\scenes\\stacked_column.json",
		};
		if (cmd.count("scene"))
			options.m_scene_filepath = cmd("scene").as<std::filesystem::path>();
		if (cmd.count("demo"))
		{
			auto const name = cmd("demo").as<std::string>();
			options.m_demo = FindDemo(name);
			if (!options.m_demo)
				throw std::runtime_error(std::format("Unknown physics demo '{}'", name));
		}
		if (cmd.count("steps"))
			options.m_steps = cmd("steps").as<int>();
		if (cmd.count("dt"))
			options.m_dt = cmd("dt").as<double>();
		if (cmd.count("report"))
			options.m_report_interval = cmd("report").as<int>();
		if (cmd.count("body"))
			options.m_trace_body = cmd("body").as<int>();
		if (cmd.count("trace_start"))
			options.m_trace_start = cmd("trace_start").as<int>();
		if (cmd.count("trace_end"))
			options.m_trace_end = cmd("trace_end").as<int>();
		if (cmd.count("ke_jump"))
			options.m_trace_ke_jump = cmd("ke_jump").as<float>();
		if (cmd.count("substeps"))
			options.m_physics_substeps = cmd("substeps").as<int>();
		if (cmd.count("solver_iterations"))
			options.m_physics_solver_iterations = cmd("solver_iterations").as<int>();
		if (cmd.count("position_iterations"))
			options.m_physics_position_iterations = cmd("position_iterations").as<int>();
		if (cmd.count("broadphase_aabb_margin"))
			options.m_physics_broadphase_aabb_margin = cmd("broadphase_aabb_margin").as<float>();
		if (cmd.count("contact_sort_propagation_scale"))
			options.m_physics_contact_sort_propagation_scale = cmd("contact_sort_propagation_scale").as<float>();
		if (cmd.count("contact_sort_shock_iterations"))
			options.m_physics_contact_sort_shock_iterations = cmd("contact_sort_shock_iterations").as<int>();
		if (cmd.count("contact_slop_scale"))
			options.m_physics_contact_slop_scale = cmd("contact_slop_scale").as<float>();
		if (cmd.count("support_contact_slop_scale"))
			options.m_physics_support_contact_slop_scale = cmd("support_contact_slop_scale").as<float>();
		if (cmd.count("warm_start_scale"))
			options.m_physics_warm_start_scale = cmd("warm_start_scale").as<float>();
		if (cmd.count("selective_refresh_passes"))
			options.m_physics_selective_refresh_passes = cmd("selective_refresh_passes").as<int>();
		if (cmd.count("selective_refresh_max_pairs"))
			options.m_physics_selective_refresh_max_pairs = cmd("selective_refresh_max_pairs").as<int>();
		if (cmd.count("selective_refresh_body_limit"))
			options.m_physics_selective_refresh_body_limit = cmd("selective_refresh_body_limit").as<int>();
		if (cmd.count("selective_refresh_contact_limit"))
			options.m_physics_selective_refresh_contact_limit = cmd("selective_refresh_contact_limit").as<int>();
		if (cmd.count("selective_refresh_solver_iterations"))
			options.m_physics_selective_refresh_solver_iterations = cmd("selective_refresh_solver_iterations").as<int>();
		if (cmd.count("selective_refresh_position_iterations"))
			options.m_physics_selective_refresh_position_iterations = cmd("selective_refresh_position_iterations").as<int>();
		if (cmd.count("selective_refresh_bias_scale"))
			options.m_physics_selective_refresh_bias_scale = cmd("selective_refresh_bias_scale").as<float>();
		if (cmd.count("selective_refresh_restitution_scale"))
			options.m_physics_selective_refresh_restitution_scale = cmd("selective_refresh_restitution_scale").as<float>();
		if (cmd.count("selective_refresh_adaptive_body_limit"))
			options.m_physics_selective_refresh_adaptive_body_limit = cmd("selective_refresh_adaptive_body_limit").as<int>();
		if (cmd.count("selective_refresh_adaptive_solver_iterations"))
			options.m_physics_selective_refresh_adaptive_solver_iterations = cmd("selective_refresh_adaptive_solver_iterations").as<int>();
		if (cmd.count("selective_refresh_support_only"))
			options.m_physics_selective_refresh_support_only = cmd("selective_refresh_support_only").as<int>();
		if (cmd.count("selective_refresh_resolve_support_only"))
			options.m_physics_selective_refresh_resolve_support_only = cmd("selective_refresh_resolve_support_only").as<int>();
		if (cmd.count("buoyancy_linear_drag_time_constant"))
			options.m_buoyancy_linear_drag_time_constant_s = cmd("buoyancy_linear_drag_time_constant").as<float>();
		if (cmd.count("buoyancy_angular_drag_time_constant"))
			options.m_buoyancy_angular_drag_time_constant_s = cmd("buoyancy_angular_drag_time_constant").as<float>();
		if (cmd.count("buoyancy_tangential_drag"))
			options.m_buoyancy_tangential_drag_coefficient = cmd("buoyancy_tangential_drag").as<float>();
		if (cmd.count("scan"))
			options.m_scan_bodies = true;
		if (cmd.count("scan_non_spheres"))
		{
			options.m_scan_bodies = true;
			options.m_scan_non_spheres = true;
		}
		if (cmd.count("column_metric"))
			options.m_column_metric = true;
		if (cmd.count("pyramid_metric"))
			options.m_pyramid_metric = true;
		if (cmd.count("cradle_metric"))
			options.m_cradle_metric = true;
		if (cmd.count("dzhanibekov_metric"))
			options.m_dzhanibekov_metric = true;
		if (cmd.count("sleep_metric"))
			options.m_sleep_metric = true;
		if (cmd.count("sleep_metric_non_spheres"))
			options.m_sleep_metric_non_spheres = true;
		if (cmd.count("engine_profile"))
			options.m_engine_profile = true;

		try
		{
			if (cmd.count("quiet"))
			{
				FILE* out = nullptr;
				freopen_s(&out, "NUL", "w", stdout);
			}

			diag::RunSceneDiagnostic(options);
			return 0;
		}
		catch (std::exception const& ex)
		{
			if (auto log = std::ofstream(log_path, std::ios::out | std::ios::app))
				log << "EXCEPTION: " << ex.what() << "\n";

			fprintf(stderr, "EXCEPTION: %s\n", ex.what());
			return -1;
		}
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

	// Check for -unittests mode before initialising any GUI resources.
	// Usage: -unittests [ClassName1 ClassName2 ...], or -unittests=ClassName.
	// Runs only matching test classes (substring match).
	auto filter = std::vector<std::string_view>{};
	if (physics_sandbox::TryGetUnitTestFilter(cmd, filter))
	{
		return physics_sandbox::RunUnitTests(filter);
	}
	if (cmd.count("scenediag"))
	{
		return physics_sandbox::RunSceneDiagnostic(cmd);
	}

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
		OpenConsoleOutput();
	else
		ConfigureAssertReporting(false);

	try
	{
		InitCtrls();

		// Create the message loop first so the form can reference it for clean shutdown
		WinGuiMsgLoop loop;

		SandboxUI sandbox(cmd.count("profile") != 0);
		sandbox.cp().msg_loop(&loop);

		// Load a named programmatic demo or a JSON scene on startup.
		if (cmd.count("demo"))
		{
			auto const name = cmd("demo").as<std::string>();
			auto const demo = FindDemo(name);
			if (!demo)
				throw std::runtime_error(std::format("Unknown physics demo '{}'", name));

			sandbox.LoadDemo(*demo);
		}
		else if (cmd.count("scene"))
			sandbox.LoadSceneFile(cmd("scene").as<std::filesystem::path>());

		sandbox.Show();

		// Start the simulation immediately if -autoplay was specified
		if (autoplay)
			sandbox.m_steps_remaining = -1;

		// SandboxUI converts wall time into fixed 60 Hz physics ticks. Variable scheduling prevents
		// the message loop from submitting several catch-up frames before yielding to the viewport.
		// Rendering targets a high rate so it resumes immediately after the previous vsync completes.
		loop.AddLoop(60.0, true, [&](double dt) { sandbox.Step(dt); });
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
