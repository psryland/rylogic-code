//*****************************************************************************
// Procedural animation
//  Copyright (c) Rylogic Ltd 2026
//*****************************************************************************
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "pr/math/math.h"
#include "pr/math/utils/interpolate_c2.h"
#include "pr/algorithm/fabrik.h"
#include "pr/storage/json.h"

namespace pr::animation
{
	// Notes:
	//  - The library turns control targets (body frames, limb end frames, aim points) into bone transforms. It holds no rendering or physics state.
	//  - All transforms are in model space (the space of the rig's root bones) unless stated otherwise. Bones point along their local +Y axis.
	//  - Scalars are 'float' to match the renderer.
	namespace fabrik = pr::algorithm::fabrik;
}
