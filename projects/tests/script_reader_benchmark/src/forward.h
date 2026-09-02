//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Precompiled header: aggregates every foreign (STL and cross-module) include used by this
// standalone benchmark. Sibling headers in this project only include this file and each other.
#pragma once

// STL
#include <string>
#include <string_view>
#include <vector>
#include <array>
#include <chrono>
#include <random>
#include <atomic>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>
#include <utility>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <optional>

// Rylogic
#include "pr/math/math.h"
#include "pr/str/string_core.h"
#include "pr/script/reader.h"
#include "pr/script/reader2.h"
