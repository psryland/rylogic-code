//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Console entry point for the view3d-12-tests project.
// With '--interactive', shows the interactive demos. '--demo <name>' selects the initial demo. Otherwise, runs the unit tests using the same
// '-verbose'/'-exclude:'/'-flags:'/positional-filter command line contract as 'projects\tests\unittests\src\main.cpp'.
#include "pr/common/unittests.h"
#include "pr/compute/utility/process_diagnostics.h"
#include "pr/view3d-12/view3d-dll.h"
#include "interactive/interactive.h"
#include <windows.h>
#include <d3d12.h>
#include <wrl/client.h>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char* argv[])
{
	// Interactive mode replaces the unit test run entirely
	auto interactive = false;
	auto demo_name = std::string_view{};
	for (auto i = 1; i != argc; ++i)
	{
		if (std::string_view{argv[i]} == "--interactive")
			interactive = true;
		else if (std::string_view{argv[i]} == "--demo" && i + 1 != argc)
			demo_name = argv[++i];
	}
	if (interactive)
		return RunInteractive(demo_name);


	// Parse startup and test choices before any device exists. GBV is explicit because it affects the entire process.
	auto args = std::vector<char*>{argv + 1, argv + argc};
	auto diagnostics = std::optional{pr::compute::TestProcessDiagnostics};
	if (!pr::compute::TakeProcessDiagnosticsOption(args, diagnostics, std::cerr))
		return 2;

	auto options = pr::unittests::UnitTestOptions{};
	if (!pr::unittests::TryParseUnitTestCommandLine(args, options, std::cerr))
		return 2;

	// Fail explicitly if requested diagnostics are unavailable rather than silently running a weaker test.
	try
	{
		if (diagnostics.has_value())
			pr::compute::ConfigureProcessDiagnostics(*diagnostics);

		pr::compute::RunnerProcessDiagnostics = diagnostics;

		// Retain the renderer tests' device diagnostics without a second process configuration call.
		auto device_debug = false;
		switch (diagnostics.value_or(pr::compute::EProcessDiagnostics::None))
		{
			case pr::compute::EProcessDiagnostics::None:
			{
				break;
			}
			case pr::compute::EProcessDiagnostics::DebugLayer:
			case pr::compute::EProcessDiagnostics::GpuValidation:
			{
				device_debug = true;
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown process diagnostics mode");
			}
		}
		auto error_cb = pr::view3d::ReportErrorCB{nullptr, [](void*, char const* message, char const*, int, std::int64_t)
		{
			std::cerr << message << '\n';
		}};
		if (!View3D_ConfigureDeviceDiagnostics(device_debug, device_debug, error_cb))
			return 2;
	}
	catch (std::exception const& ex)
	{
		std::cerr << "Process diagnostics startup failed: " << ex.what() << '\n';
		return 2;
	}
	return pr::unittests::RunAllTests(options);
}