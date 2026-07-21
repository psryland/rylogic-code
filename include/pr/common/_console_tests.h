//*****************************************************************************
// Console tests
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once

#if PR_UNITTESTS
#define private public
#include "pr/common/console.h"
#undef private

#include "pr/common/unittests.h"

namespace pr::console::tests
{
	PRUnitTestClass(ConsoleTests)
	{
		// Exercise the private helper directly without constructing a live Console instance.
		// The helper only depends on the handle value and Win32 APIs, so raw storage is enough
		// here and avoids opening or attaching a real console window during the test run.
		PRUnitTestMethod(CloseHandleUsesGlobalApi)
		{
			using Console = pr::Console<char>;
			alignas(Console) unsigned char storage[sizeof(Console)] = {};
			auto* console = reinterpret_cast<Console*>(&storage);

			// Use a disposable handle so the regression checks the recursive call path safely.
			auto handle = ::CreateEventW(nullptr, TRUE, FALSE, nullptr);
			PR_EXPECT(handle != nullptr);

			console->CloseHandle(handle);

			PR_EXPECT(handle == INVALID_HANDLE_VALUE);
		}
	};
}
#endif
