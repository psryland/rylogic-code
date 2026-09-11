//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include <array>
#include <span>
#include <vector>
#include <memory>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pr/common/assert.h"
#include "pr/math/math.h"
#include "pr/collision/shape.h"

namespace pr::physics::terrain
{
	// Terrain sampling uses double precision coordinates and derivatives end to end.
	using v2d = math::Vec2<double>;
	using v4d = math::Vec4<double>;

	// Terrain surface materials reuse the existing collision/physics material identifier.
	using MaterialId = collision::MaterialId;
}
