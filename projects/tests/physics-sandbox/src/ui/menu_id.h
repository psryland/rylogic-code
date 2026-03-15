#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Menu command IDs for the sandbox application
	namespace MenuID
	{
		static constexpr int OpenFile = 1001;
		static constexpr int RecentFileBase = 2000; // 2000..2000+MaxRecentFiles-1
	}
	namespace MenuItemIndex
	{
		static constexpr int OpenFile = 0;
		static constexpr int Divider0 = 1;
		static constexpr int RecentFiles = 2;
		static constexpr int Divider1 = 3;
		static constexpr int Exit = 4;
	}
}