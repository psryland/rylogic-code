//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#pragma once
#include <functional>
#include <cstdarg>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <span>
#include <iterator>
#include <memory>
#include <deque>
#include <filesystem>
#include <optional>
#include <format>
#include <variant>
#include <crtdbg.h>

#include "pr/common/assert.h"
#include "pr/common/command_line.h"
#include "pr/common/min_max_fix.h"
#include "pr/common/guid.h"
#include "pr/common/fmt.h"
#include "pr/common/ldraw.h"
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/gui/wingui.h"
#include "pr/gui/gdiplus.h"
#include "pr/gui/view3d_panel_static.h"
#include "pr/storage/json.h"
#include "pr/view3d-12/view3d.h"
#include "pr/win32/win32.h"
#include "pr/win32/windows_com.h"

#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_polytope.h"

#include "pr/physics-2/physics.h"

using namespace pr;
using namespace pr::math;
using namespace pr::gui;
using namespace pr::math::spatial;

namespace physics_sandbox
{
	struct SandboxUI;
	namespace scene_loader { struct SceneDesc; }

	// The user's app data directory.
	std::filesystem::path AppDataPath();
}