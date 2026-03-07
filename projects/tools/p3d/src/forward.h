//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <filesystem>

#include <sdkddkver.h>
#include <windows.h>
#include <consoleapi.h>

#include "pr/common/assert.h"
#include "pr/common/fmt.h"
#include "pr/common/cast.h"
#include "pr/common/hash.h"
#include "pr/common/hresult.h"
#include "pr/common/command_line.h"
#include "pr/math/math.h"
#include "pr/math/conversion.h"
#include "pr/filesys/filesys.h"
#include "pr/script/script.h"
#include "pr/script/reader.h"

#include "pr/geometry/p3d.h"

using namespace pr;
using namespace pr::math;
using namespace pr::geometry;
using namespace std::literals;
using namespace std::filesystem;
