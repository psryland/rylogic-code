//***********************************************************************
// BlitzSearch
//  Copyright (c) Rylogic Ltd 2024
//***********************************************************************
#pragma once

#include <winsdkver.h>
#include <sdkddkver.h>

#include <cstdint>
#include <stdexcept>
#include <filesystem>
#include <functional>
#include <algorithm>
#include <ranges>
#include <chrono>
#include <format>
#include <queue>
#include <span>
#include <concurrent_vector.h>

#include <windows.h>
#include <shlobj_core.h>

#include "pr/common/hresult.h"
#include "pr/common/scope.h"
#include "pr/common/profile.h"
#include "pr/container/suffix_array.h"
#include "pr/common/bit_fields.h"
#include "pr/gui/wingui.h"
#include "pr/storage/json.h"
#include "pr/threads/thread_pool.h"
#include "pr/win32/user_directories.h"
#include "pr/win32/windows_com.h"

namespace ui = pr::gui;
namespace fs = std::filesystem;

namespace blitzsearch
{
	struct FileIndex;
	struct MainUI;
}
