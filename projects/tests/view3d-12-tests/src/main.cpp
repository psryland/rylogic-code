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


	// Parse the unit test selection options, rejecting invalid arguments before running anything.
	auto options = pr::unittests::UnitTestOptions{};
	if (!pr::unittests::TryParseUnitTestCommandLine(std::span{argv + 1, static_cast<std::size_t>(argc - 1)}, options, std::cerr))
		return 2;

	// The renderer tests check for D3D12 validation messages, which needs the debug layer enabled before any device exists.
	// A caller-provided value takes precedence.
	SetLastError(ERROR_SUCCESS);
	if (GetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", nullptr, 0) == 0 && GetLastError() == ERROR_ENVVAR_NOT_FOUND)
		SetEnvironmentVariableW(L"VIEW3D_DEVICE_DEBUG", L"1");

	// Enable the D3D12 debug layer for the whole process before any test creates a device. Enabling it later removes every device
	// that already exists, and tests run in parallel, so a test that enables it would break the devices of other running tests.
	// Once it is on, the later 'EnableDebugLayer' calls made by renderers do nothing.
	if (Microsoft::WRL::ComPtr<ID3D12Debug> debug; SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug))))
		debug->EnableDebugLayer();

	return pr::unittests::RunAllTests(options);
}