//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#pragma once

// std
#include <type_traits>
#include <concepts>
#include <string_view>
#include <string>
#include <span>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include <stdexcept>
#include <utility>
#include <filesystem>
#include <variant>

// Windows
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

// pr - app framework (includes windows, view3d-12, gui, camera, etc.)
#include "pr/app/forward.h"
#include "pr/app/main.h"
#include "pr/app/main_ui.h"
#include "pr/app/default_setup.h"
#include "pr/app/skybox.h"
#include "pr/common/keystate.h"
#include "pr/common/event_handler.h"
#include "pr/common/resource.h"
#include "pr/common/task_graph.h"
#include "pr/compute/compute.h"
#include "pr/geometry/p3d.h"
#include "pr/hlsl/interop.h"
#include "pr/storage/json.h"
#include "pr/algorithm/perlin_noise.h"
#include "pr/view3d-12/view3d.h"
#include "pr/physics/physics.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/shape/inertia.h"
#include "pr/collision/shape_box.h"
#include "pr/win32/win32.h"
#include "pr/view3d-12/imgui/imgui.h"

using namespace pr;

namespace las
{
	using namespace pr;
	using namespace pr::rdr12;
	using namespace pr::physics;
	using namespace pr::collision;
	using namespace pr::gui;
	using namespace pr::app;

	using ShaderCompiler = pr::compute::ShaderCompiler;
	using EHlslVersion = pr::compute::EHlslVersion;

	struct Main;
	struct MainUI;
	struct Settings;
	struct InputHandler;

	namespace input
	{
		enum class EMode;
		enum class EAction;

		struct IMode;
		struct Mode_FreeCamera;
		struct Mode_ShipControl;
		struct Mode_MenuNavigation;
	}
	namespace camera
	{
		struct ICamera;
		struct FreeCamera;
		struct ShipCamera;
	}
}
