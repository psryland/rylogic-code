//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <type_traits>
#include <concepts>
#include <cassert>

#include "pr/math/math.h"
#include "pr/common/cast.h"
#include "pr/common/scope.h"
#include "pr/common/alloca.h"
#include "pr/container/vector.h"
#include "pr/container/tri_table.h"
#include "pr/geometry/point.h"
#include "pr/geometry/closest_point.h"
#include "pr/geometry/intersect.h"

namespace pr::collision
{
	struct Shape;
	struct ShapeSphere;
	struct ShapeBox;
	struct ShapeLine;
	struct ShapePolytope;
	struct ShapeTriangle;
	struct ShapeArray;
	struct Contact;
	struct Ray;
	struct RayCastResult;
	struct Penetration;
	struct TestPenetration;
	struct MinPenetration;
	struct ContactPenetration;
}

