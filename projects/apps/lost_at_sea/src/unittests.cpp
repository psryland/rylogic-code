//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/unittests.h"
#include "pr/common/unittests.h"

namespace las
{
	namespace
	{
		// Attach test output to the invoking terminal, or create a console for an Explorer launch.
		void OpenUnitTestConsole()
		{
			if (!AttachConsole(ATTACH_PARENT_PROCESS))
				AllocConsole();

			FILE* stream = nullptr;
			freopen_s(&stream, "CONOUT$", "w", stdout);
			freopen_s(&stream, "CONOUT$", "w", stderr);
		}
	}

	// Handle application-owned command modes before the GUI and renderer are initialised.
	bool TryHandleCommandLine(TCHAR const* command_line, int& exit_code)
	{
		auto cmd = pr::CmdLine("app " + std::string(command_line != nullptr ? command_line : ""));
		if (cmd.count("unittests") == 0)
			return false;

		OpenUnitTestConsole();

		#if defined(_DEBUG)

		// Preserve test output for console-less callers that provide an explicit log path.
		auto unittest_log = std::ofstream{};
		if (auto const* log_path = std::getenv("PR_UNITTEST_LOG"); log_path != nullptr && *log_path != '\0')
		{
			unittest_log.open(log_path, std::ios::out | std::ios::trunc);
			if (unittest_log)
				pr::unittests::TestFramework::ostream = &unittest_log;
		}

		printf("Lost at Sea: Running unit tests...\n");
		exit_code = pr::unittests::RunAllTests(true) != 0 ? 1 : 0;

		#else

		fprintf(stderr, "Lost at Sea unit tests are available only in Debug builds.\n");
		exit_code = 2;

		#endif

		return true;
	}
}
