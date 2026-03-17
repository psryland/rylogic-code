#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <execution>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <atomic>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace code_sync
{
	namespace fs = std::filesystem;
}
