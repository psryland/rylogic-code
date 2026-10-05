//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Console entry point for the view3d-12-tests project.
// With '--interactive', shows the interactive demos. '--demo <name>' selects the initial demo. Otherwise, runs the unit tests using the same
// '-verbose'/'-exclude:'/'-flags:'/positional-filter command line contract as 'projects\tests\unittests\src\main.cpp'.
#include "pr/common/unittests.h"
#include "interactive/interactive.h"
#include <windows.h>
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


	// Parse the unit test selection options.
	auto wordy = false;
	auto filters = std::vector<std::string_view>{};
	auto excludes = std::vector<std::string_view>{};
	auto flag_filters = std::vector<pr::unittests::EUnitTestFlags>{};
	for (auto i = 1; i != argc; ++i)
	{
		if (strcmp(argv[i], "-verbose") == 0)
			wordy = true;
		else if (auto arg = std::string_view{argv[i]}; arg.starts_with("-exclude:"))
			excludes.push_back(arg.substr(std::string_view{"-exclude:"}.size()));
		else if (arg.starts_with("-flags:"))
		{
			// Reject invalid filters before running a success-shaped empty selection.
			auto flags = pr::unittests::EUnitTestFlags::None;
			auto const expression = arg.substr(std::string_view{"-flags:"}.size());
			if (!pr::unittests::TryParseUnitTestFlags(expression, flags))
			{
				std::cerr << "Invalid unit-test flags: " << expression << std::endl;
				return 2;
			}
			flag_filters.push_back(flags);
		}
		else
			filters.push_back(argv[i]);
	}
	// The renderer tests check for D3D12 validation messages, which needs the debug layer enabled before any device exists.
	// A caller-provided value takes precedence.
	SetLastError(ERROR_SUCCESS);
	if (GetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", nullptr, 0) == 0 && GetLastError() == ERROR_ENVVAR_NOT_FOUND)
		SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", L"1");

	return pr::unittests::RunAllTests(wordy, filters, excludes, flag_filters);
}