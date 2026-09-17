#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <windows.h>
#include <d3d12.h>
#include <d3d12sdklayers.h>
#include <wrl/client.h>
#include "pr/view3d-12/view3d-dll.h"
#include "pr/view3d-12/view3d-ui-bridge.h"
#include "pr/view3d-12/scene/far_clip_fade.h"
#include "pr/view3d-12/shaders/shader.h"

// Fixture-owned bytecode exercises the custom-stage boundary without a renderer library dependency.
namespace fade_tests::compiled
{
	#include "fade_vertex.h"
	#include "unsupported_pixel.h"
}
