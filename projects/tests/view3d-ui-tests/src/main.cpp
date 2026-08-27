//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Console entry point for the view3d-ui-tests project. Mirrors 'projects\tests\unittests\src\main.cpp'
// so the same '-verbose'/'-exclude:'/'-flags:'/positional-filter command line contract works here.
#include "pr/common/unittests.h"
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char* argv[])
{
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
	return pr::unittests::RunAllTests(wordy, filters, excludes, flag_filters);
}
