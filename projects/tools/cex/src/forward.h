//**********************************************
// Console Extensions
//  Copyright (c) Rylogic Ltd 2004
//**********************************************
// To add new commands add code at the NEW_COMMAND comments

#pragma once

#include <string_view>
#include <vector>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <iterator>
#include <format>

#include <sdkddkver.h>
#include <windows.h>
#include <process.h>
#include <shlobj.h>
#include <fcntl.h>
#include <io.h>
#include <shellapi.h>

//#include "pr/common/assert.h"
//#include "pr/common/fmt.h"
//#include "pr/common/cast.h"
//#include "pr/common/hresult.h"
//#include "pr/common/guid.h"
//#include "pr/common/hash.h"
#include "pr/common/command_line.h"
//#include "pr/math/math.h"
//#include "pr/str/extract.h"
//#include "pr/str/to_string.h"
//#include "pr/filesys/filesys.h"
//#include "pr/script/script_core.h"
//#include "pr/script/filter.h"
//#include "pr/algorithm/algorithm.h"
#include "pr/storage/json.h"
#include "pr/threads/process.h"
#include "pr/win32/win32.h"
#include "pr/win32/windows_com.h"

namespace cex
{
	// Show the console for this process
	void ShowConsole();

	// Add an environment variable
	void SetEnvVar(std::string_view env_var, std::string_view value);
}
