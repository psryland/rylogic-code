//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/hlsl/interop.h"

namespace pr::physics::surface
{
	// Maximum cell diameter in shape-local length units. See README.md for coverage and quadrature contracts.
	inline constexpr float DefaultSpacing = 0.16f;
}
